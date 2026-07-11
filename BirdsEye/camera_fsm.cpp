#include "camera_fsm.h"

namespace camera_fsm {

namespace {

// Timestamps are stored non-zero so 0 can mean "timer not running".
uint32_t seedNow(uint32_t nowMs) { return nowMs == 0 ? 1 : nowMs; }

// Wraparound-safe elapsed-interval check (unsigned subtraction).
bool elapsed(uint32_t nowMs, uint32_t sinceMs, uint32_t intervalMs) {
  return (nowMs - sinceMs) >= intervalMs;
}

// Reset every timer/attempt/flag field. State/serialPresent/recordingActive
// are untouched (recordingActive is a belief flag, cleared explicitly by the
// callers that need it).
void clearTimers(Fsm& f) {
  f.rpmOnSince = 0;
  f.rpmGoneSince = 0;
  f.wakeAttemptStarted = 0;
  f.wakeAttemptsUsed = 0;
  f.subscribeAttemptsUsed = 0;
  f.connectedSince = 0;
  f.awaitGpsSince = 0;
  f.stopCondSince = 0;
  f.recordIdleSince = 0;
  f.recordRetryUsed = false;
  f.cooldownSince = 0;
  f.powerOffSentAt = 0;
  f.pairingSince = 0;
  f.entryPending = false;
}

// ---- Shared transitions -------------------------------------------------

// Land in IDLE with a clean slate: fresh RPM debounce, no live timers, and
// no belief that anything is recording.
void enterIdle(Fsm& f) {
  clearTimers(f);
  f.recordingActive = false;
  f.state = State::kIdle;
}

// Land in IDLE but KEEP the recordingActive belief. Used on the paths where
// the camera is UNREACHABLE (WAKING give-up / rpm-gone / sessionEnd, or the
// subscribe give-up): we could not send a stop, so we must not pretend the
// camera stopped. The next reconnect's stepAwaitReady reconcile corrects the
// belief against the camera's observed 0x10 record state — clearing it here
// would let that reconnect blind-toggle a still-live recording OFF (#4).
void enterIdlePreserveRecording(Fsm& f) {
  const bool wasRecording = f.recordingActive;
  clearTimers(f);
  f.recordingActive = wasRecording;
  f.state = State::kIdle;
}

// Camera is connected to our remote service; wait for its ce82 subscription
// (and a GPS fix) before recording. Reached from IDLE (stale link) or WAKING.
void enterAwaitReady(Fsm& f, uint32_t nowMs) {
  f.state = State::kAwaitReady;
  f.connectedSince = seedNow(nowMs);
  f.awaitGpsSince = seedNow(nowMs);
  f.entryPending = false;
}

// Begin a fresh wake cycle (attempt 1 of kConnectRetries).
Action enterWakingFresh(Fsm& f, uint32_t nowMs) {
  f.state = State::kWaking;
  f.wakeAttemptsUsed = 1;
  f.wakeAttemptStarted = seedNow(nowMs);
  f.rpmGoneSince = 0;
  return Action::kStartWakeBurst;
}

Action enterCooldown(Fsm& f, uint32_t nowMs, Action action) {
  f.state = State::kCooldown;
  f.cooldownSince = seedNow(nowMs);
  return action;
}

Action enterPoweringOff(Fsm& f, uint32_t nowMs) {
  f.state = State::kPoweringOff;
  f.powerOffSentAt = seedNow(nowMs);
  return Action::kSendPowerOff;
}

// ---- Per-state step logic ------------------------------------------------

Action stepIdle(Fsm& f, const Inputs& in) {
  // RPM-on debounce: any sample at/below the ON threshold resets the hold.
  if (in.rpm <= kRpmOnThreshold) {
    f.rpmOnSince = 0;
    return Action::kNone;
  }
  if (f.rpmOnSince == 0) {
    f.rpmOnSince = seedNow(in.nowMs);
    return Action::kNone;
  }
  if (!elapsed(in.nowMs, f.rpmOnSince, kRpmOnDebounceMs)) return Action::kNone;
  f.rpmOnSince = 0;

  if (in.remoteConnected) {
    // Stale remote link from a previous session: the camera is already on
    // our service, so skip the wake advert and wait for it to be ready.
    enterAwaitReady(f, in.nowMs);
    return Action::kNone;
  }
  return enterWakingFresh(f, in.nowMs);
}

Action stepWaking(Fsm& f, const Inputs& in) {
  // The camera connected to us: our advert stops automatically once it takes
  // the peripheral slot, so no explicit stop is needed here.
  if (in.remoteConnected) {
    enterAwaitReady(f, in.nowMs);
    return Action::kNone;
  }
  // Engine gone again before the camera showed up: abort the wake. Preserve
  // the recording belief — the camera is unreachable, so we can't have
  // stopped it (see enterIdlePreserveRecording).
  if (in.rpm < kRpmOffThreshold) {
    if (f.rpmGoneSince == 0) {
      f.rpmGoneSince = seedNow(in.nowMs);
    } else if (elapsed(in.nowMs, f.rpmGoneSince, kRpmGoneAbortMs)) {
      enterIdlePreserveRecording(f);
      return Action::kStopAdvertising;
    }
  } else {
    f.rpmGoneSince = 0;
  }
  // Attempt window elapsed with no connection: re-beacon (retry) or give up.
  if (elapsed(in.nowMs, f.wakeAttemptStarted, kConnectTimeoutMs)) {
    if (f.wakeAttemptsUsed < kConnectRetries) {
      f.wakeAttemptsUsed++;
      f.wakeAttemptStarted = seedNow(in.nowMs);
      f.rpmGoneSince = 0;
      return Action::kStartWakeBurst;
    }
    enterIdlePreserveRecording(f);  // never reached the camera — keep the belief
    return Action::kStopAdvertising;
  }
  if (in.sessionEndRequested) {
    enterIdlePreserveRecording(f);
    return Action::kStopAdvertising;
  }
  return Action::kNone;
}

// Enter RECORDING with fresh reconcile timers. `sendShutter` toggles the
// camera when we need to START it; a resume (camera already rolling) passes
// false so we never blind-toggle a live recording.
Action enterRecording(Fsm& f, bool sendShutter) {
  f.state = State::kRecording;
  f.stopCondSince = 0;
  f.recordIdleSince = 0;
  f.recordRetryUsed = false;
  f.subscribeAttemptsUsed = 0;  // subscribed successfully — reset the bound
  f.recordingActive = true;
  return sendShutter ? Action::kSendShutter : Action::kNone;
}

Action stepAwaitReady(Fsm& f, const Inputs& in) {
  // Camera dropped before recording started: re-wake it (fresh cycle).
  if (!in.remoteConnected) {
    return enterWakingFresh(f, in.nowMs);
  }
  if (in.sessionEndRequested) {
    // If we came in resuming a recording (belief preserved across a drop),
    // stop the camera on the way into cooldown; a fresh not-yet-recording
    // session just rides cooldown then powers off.
    if (f.recordingActive) {
      f.recordingActive = false;
      return enterCooldown(f, in.nowMs, Action::kSendShutter);  // toggle OFF
    }
    return enterCooldown(f, in.nowMs, Action::kNone);
  }
  // Connected but never subscribed to ce82: we can't send button frames, so
  // drop the link and re-wake — BOUNDED by kSubscribeRetries so a camera that
  // connects but never subscribes doesn't loop forever (#9). subscribeAttemptsUsed
  // persists across the re-wake (enterWakingFresh doesn't touch it).
  if (!in.ce82Subscribed &&
      elapsed(in.nowMs, f.connectedSince, kSubscribeTimeoutMs)) {
    f.subscribeAttemptsUsed++;
    if (f.subscribeAttemptsUsed > kSubscribeRetries) {
      // Give up: the camera connects but won't take our buttons. Preserve the
      // recording belief (we never reached it to stop it).
      enterIdlePreserveRecording(f);
      return Action::kDisconnect;
    }
    enterWakingFresh(f, in.nowMs);  // re-wake; its kStartWakeBurst is intentionally
    return Action::kDisconnect;     // dropped — drop the useless link first
  }
  // Ready to record: subscribed, and either a GPS fix or the lock wait timed
  // out (record-anyway policy).
  const bool lockTimedOut =
      kRecordWithoutGpsOnTimeout &&
      elapsed(in.nowMs, f.awaitGpsSince, kGpsLockTimeoutMs);
  if (in.ce82Subscribed && (in.gpsFixValid || lockTimedOut)) {
    // Reconcile against the camera's OBSERVED record state so we drive the
    // shutter toward the target instead of blind-toggling on belief:
    //  - kRecording: camera already rolling (resumed session) -> adopt, no shutter.
    //  - kIdle:      camera confirmed stopped -> shutter to start.
    //  - kUnknown:   no fresh 0x10 yet -> fall back to belief. If we already
    //                believe we're recording (belief preserved across a drop),
    //                adopt WITHOUT toggling — the observation confirms shortly;
    //                only a not-recording belief starts. This closes the #4
    //                inversion during the brief post-reconnect observation gap.
    if (in.recordObserved == RecordObs::kRecording) {
      return enterRecording(f, /*sendShutter=*/false);
    }
    if (in.recordObserved == RecordObs::kIdle) {
      return enterRecording(f, /*sendShutter=*/true);
    }
    return enterRecording(f, /*sendShutter=*/!f.recordingActive);
  }
  return Action::kNone;
}

Action stepRecording(Fsm& f, const Inputs& in) {
  // 1. Manual session end: stop immediately, bypassing the hold timer.
  if (in.sessionEndRequested) {
    if (f.recordingActive) {
      f.recordingActive = false;
      return enterCooldown(f, in.nowMs, Action::kSendShutter);  // toggle OFF
    }
    return enterCooldown(f, in.nowMs, Action::kNone);
  }
  // 2. Remote drop: re-wake and resume. recordingActive stays as-is so a
  //    reconnect mid-session does NOT blind-toggle the shutter. Logs untouched.
  if (!in.remoteConnected) {
    return enterWakingFresh(f, in.nowMs);
  }
  // 3. Confirm the shutter took. The camera's observed 0x10 record state is
  //    the ground truth (the shutter itself is fire-and-hope):
  //    - kRecording: confirmed -> clear the re-assert latch/timer.
  //    - kIdle sustained kRecordConfirmMs while we believe we're recording:
  //      the start never landed (dropped frame / busy camera) -> re-assert the
  //      shutter ONCE. Re-armable only after the camera next confirms recording.
  //    - kUnknown: no fresh observation -> never retry (can't runaway).
  if (in.recordObserved == RecordObs::kRecording) {
    f.recordIdleSince = 0;
    f.recordRetryUsed = false;
  } else if (in.recordObserved == RecordObs::kIdle && f.recordingActive) {
    if (!f.recordRetryUsed) {
      if (f.recordIdleSince == 0) {
        f.recordIdleSince = seedNow(in.nowMs);
      } else if (elapsed(in.nowMs, f.recordIdleSince, kRecordConfirmMs)) {
        f.recordIdleSince = 0;
        f.recordRetryUsed = true;
        return Action::kSendShutter;  // re-assert recording ON
      }
    }
  } else {
    f.recordIdleSince = 0;  // kUnknown (or belief already false): no retry in flight
  }
  // 4. Sustained stop condition.
  const bool speedStopped = in.speedMph < kSpeedStopThresholdMph;
  const bool rpmStopped = in.rpm < kRpmOffThreshold;
  const bool stopped = kStopOnEither ? (speedStopped || rpmStopped)
                                     : (speedStopped && rpmStopped);
  if (stopped) {
    if (f.stopCondSince == 0) {
      f.stopCondSince = seedNow(in.nowMs);
    } else if (elapsed(in.nowMs, f.stopCondSince, kStopRecordDelayMs)) {
      f.stopCondSince = 0;
      if (f.recordingActive) {
        f.recordingActive = false;
        return enterCooldown(f, in.nowMs, Action::kSendShutter);  // toggle OFF
      }
      return enterCooldown(f, in.nowMs, Action::kNone);
    }
  } else {
    f.stopCondSince = 0;
  }
  return Action::kNone;  // no GPS feed, no keep-alive — those are gone
}

Action stepCooldown(Fsm& f, const Inputs& in) {
  if (in.sessionEndRequested) {
    // Manual end during cooldown powers the camera off immediately.
    return enterPoweringOff(f, in.nowMs);
  }
  if (!in.remoteConnected) {
    // Camera dropped / turned itself off.
    enterIdle(f);
    return Action::kNone;
  }
  if (elapsed(in.nowMs, f.cooldownSince, kPowerOffDelayMs)) {
    return enterPoweringOff(f, in.nowMs);
  }
  if (kAutoResumeFromCooldown) {
    // DEFERRED v1 (kAutoResumeFromCooldown == false): motion/RPM return during
    // cooldown is deliberately ignored. When the flag flips, add the re-arm
    // transition back to kRecording here.
  }
  return Action::kNone;
}

Action stepPoweringOff(Fsm& f, const Inputs& in) {
  if (!in.remoteConnected) {
    enterIdle(f);
    return Action::kNone;
  }
  if (elapsed(in.nowMs, f.powerOffSentAt, kPowerOffLingerMs)) {
    // Camera never dropped the link: force it down ourselves.
    enterIdle(f);
    return Action::kDisconnect;
  }
  return Action::kNone;
}

Action stepPairing(Fsm& f, const Inputs& in) {
  if (f.entryPending) {
    f.entryPending = false;
    return Action::kStartConnectableAdvertising;
  }
  if (in.pairCancelRequested ||
      elapsed(in.nowMs, f.pairingSince, kPairingTimeoutMs)) {
    f.state = f.pairingReturnState;
    f.rpmOnSince = 0;  // fresh debounce if we land back in kIdle
    return Action::kStopAdvertising;
  }
  // Serial capture is handled by the priority rules in step().
  return Action::kNone;
}

}  // namespace

void init(Fsm& f, bool serialPresent) {
  f = Fsm{};
  f.serialPresent = serialPresent;
  f.state = serialPresent ? State::kIdle : State::kUnpaired;
}

Action step(Fsm& f, const Inputs& in) {
  // ---- Priority events, before any state logic ----

  // 1. Force-idle (transfer takeover / sleep entry) overrides everything. The
  //    caller does its own physical teardown; the returned action is
  //    belt-and-suspenders.
  if (in.forceIdleRequested) {
    const State prev = f.state;
    clearTimers(f);
    f.recordingActive = false;
    f.state = f.serialPresent ? State::kIdle : State::kUnpaired;
    if (prev == State::kWaking || prev == State::kPairing) {
      return Action::kStopAdvertising;  // we were advertising
    }
    return Action::kNone;
  }

  // 2. Unpair: only honored while no session machinery is running.
  //    kDisconnect (not kNone): a remote link can survive into kIdle — left
  //    connected, the old camera would occupy the single peripheral slot and
  //    could re-inject its serial into the next pairing attempt.
  if (in.unpairRequested &&
      (f.state == State::kUnpaired || f.state == State::kIdle)) {
    f.state = State::kUnpaired;
    f.serialPresent = false;
    return Action::kDisconnect;
  }

  // 3. Enter pairing (falls through so the state logic emits the entry action
  //    this same step).
  if (in.pairRequested &&
      (f.state == State::kUnpaired || f.state == State::kIdle)) {
    f.pairingReturnState = f.state;
    f.state = State::kPairing;
    f.pairingSince = seedNow(in.nowMs);
    f.entryPending = true;
  }

  // 4. Serial captured AND persisted by the glue.
  if (in.pairSerialCaptured) {
    f.serialPresent = true;
    if (f.state == State::kPairing) {
      enterIdle(f);
      return Action::kDisconnect;
    }
  }

  // ---- State logic ----
  switch (f.state) {
    case State::kUnpaired:
      return Action::kNone;  // only exit is pairing, handled above
    case State::kIdle:
      return stepIdle(f, in);
    case State::kWaking:
      return stepWaking(f, in);
    case State::kAwaitReady:
      return stepAwaitReady(f, in);
    case State::kRecording:
      return stepRecording(f, in);
    case State::kCooldown:
      return stepCooldown(f, in);
    case State::kPoweringOff:
      return stepPoweringOff(f, in);
    case State::kPairing:
      return stepPairing(f, in);
  }
  return Action::kNone;
}

const char* stateName(State s) {
  switch (s) {
    case State::kUnpaired:    return "UNPAIRED";
    case State::kIdle:        return "IDLE";
    case State::kWaking:      return "WAKING";
    case State::kAwaitReady:  return "READY";
    case State::kRecording:   return "RECORDING";
    case State::kCooldown:    return "COOLDOWN";
    case State::kPoweringOff: return "PWR OFF";
    case State::kPairing:     return "PAIRING";
  }
  return "?";
}

}  // namespace camera_fsm
