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
  f.recordArmSince = 0;
  f.stopCondSince = 0;
  f.recordIdleSince = 0;
  f.recordRetryUsed = false;
  f.pairingSince = 0;
  f.entryPending = false;
}

// RPM-continuously-up tracker for the record-start gate: arm on the rise
// above ON, disarm on a fall below OFF, hold through the hysteresis band.
// The record-start clock (kRecordStartDelayMs) runs from recordArmSince.
void maintainRecordArm(Fsm& f, const Inputs& in) {
  if (in.rpm > kRpmOnThreshold) {
    if (f.recordArmSince == 0) f.recordArmSince = seedNow(in.nowMs);
  } else if (in.rpm < kRpmOffThreshold) {
    f.recordArmSince = 0;  // engine off — disarm
  }
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
// before moving to WATCHING. Reached from IDLE (stale link) or WAKING.
void enterAwaitReady(Fsm& f, uint32_t nowMs) {
  f.state = State::kAwaitReady;
  f.connectedSince = seedNow(nowMs);
  f.entryPending = false;
}

// Camera ON + connected but NOT recording: the hub state. Re-records when RPM
// has held long enough, and powers off ONLY when the glue tears down on sleep.
// `resetArm` clears the record-start clock: true after a STOP so a stall-
// recovery record needs a fresh run of sustained RPM (not the stale arm from
// the session that just ended); false on the first connect so the record-start
// delay keeps measuring from the wake.
Action enterWatching(Fsm& f, Action action, bool resetArm) {
  f.state = State::kWatching;
  if (resetArm) f.recordArmSince = 0;
  f.stopCondSince = 0;
  return action;
}

// Begin a fresh wake cycle (attempt 1 of kConnectRetries).
Action enterWakingFresh(Fsm& f, uint32_t nowMs) {
  f.state = State::kWaking;
  f.wakeAttemptsUsed = 1;
  f.wakeAttemptStarted = seedNow(nowMs);
  f.rpmGoneSince = 0;
  return Action::kStartWakeBurst;
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
  // RPM is confirmed up — start the record-arm clock now so the 5 s
  // record-start delay is measured from the wake, not from when the camera
  // finally connects.
  f.recordArmSince = seedNow(in.nowMs);

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
  maintainRecordArm(f, in);  // keep the 5 s record-start clock accurate
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
  maintainRecordArm(f, in);  // keep the record-start clock accurate while connecting
  // Camera dropped before we were ready: re-wake it (fresh cycle).
  if (!in.remoteConnected) {
    return enterWakingFresh(f, in.nowMs);
  }
  if (in.sessionEndRequested) {
    // Manual end while connecting: if we were resuming a recording (belief
    // preserved across a drop), stop it; then WATCH (camera stays on).
    if (f.recordingActive && in.ce82Subscribed) {
      f.recordingActive = false;
      return enterWatching(f, Action::kSendShutter, /*resetArm=*/true);  // toggle OFF
    }
    return enterWatching(f, Action::kNone, /*resetArm=*/true);
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
  // Subscribed — the camera can take our buttons. Move to WATCHING, the hub
  // that arms recording once RPM has held long enough (no GPS-lock gate). Keep
  // the wake-set arm (resetArm=false) so the record-start delay measures from
  // the wake, not from when the camera finally connected.
  if (in.ce82Subscribed) {
    f.subscribeAttemptsUsed = 0;
    return enterWatching(f, Action::kNone, /*resetArm=*/false);
  }
  return Action::kNone;
}

// Shared record-start gate used by WATCHING: RPM held above ON for
// kRecordStartDelayMs, camera subscribed. Reconciles against the camera's
// OBSERVED record state so we drive the shutter toward the target instead of
// blind-toggling on belief (adopt-if-already-rolling; #4). Returns kNone with
// no state change when the gate isn't met.
Action tryStartRecording(Fsm& f, const Inputs& in) {
  if (!in.ce82Subscribed || f.recordArmSince == 0 ||
      !elapsed(in.nowMs, f.recordArmSince, kRecordStartDelayMs)) {
    return Action::kNone;
  }
  if (in.recordObserved == RecordObs::kRecording) {
    return enterRecording(f, /*sendShutter=*/false);  // already rolling — adopt
  }
  if (in.recordObserved == RecordObs::kIdle) {
    return enterRecording(f, /*sendShutter=*/true);   // confirmed stopped — start
  }
  return enterRecording(f, /*sendShutter=*/!f.recordingActive);  // no obs — trust belief
}

Action stepRecording(Fsm& f, const Inputs& in) {
  // 1. Manual session end (user's explicit logging-stop): stop recording, then
  //    WATCH — the camera stays on, ready to re-record; it powers off only on
  //    sleep. The main sketch already ended the log, so no auto-stop signal.
  if (in.sessionEndRequested) {
    if (f.recordingActive) {
      f.recordingActive = false;
      return enterWatching(f, Action::kSendShutter, /*resetArm=*/true);  // toggle OFF
    }
    return enterWatching(f, Action::kNone, /*resetArm=*/true);
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
  // 4. Engine-off stop: RPM below OFF for kStopRecordDelayMs (30 s) — RPM only,
  //    no speed, so a stationary-but-running grid idle keeps recording. On stop
  //    the glue sees kRecording -> kWatching (no manual sessionEnd this step) and
  //    ends the log session.
  if (in.rpm < kRpmOffThreshold) {
    if (f.stopCondSince == 0) {
      f.stopCondSince = seedNow(in.nowMs);
    } else if (elapsed(in.nowMs, f.stopCondSince, kStopRecordDelayMs)) {
      f.stopCondSince = 0;
      if (f.recordingActive) {
        f.recordingActive = false;
        return enterWatching(f, Action::kSendShutter, /*resetArm=*/true);  // toggle OFF
      }
      return enterWatching(f, Action::kNone, /*resetArm=*/true);
    }
  } else {
    f.stopCondSince = 0;
  }
  return Action::kNone;
}

Action stepWatching(Fsm& f, const Inputs& in) {
  maintainRecordArm(f, in);  // arm/disarm the record-start clock on RPM
  // Camera dropped / turned itself off: back to IDLE (re-wakes on the next
  // engine start). Preserve nothing — the camera is gone.
  if (!in.remoteConnected) {
    enterIdle(f);
    return Action::kNone;
  }
  // Manual session end while already watching: nothing to stop, stay put.
  if (in.sessionEndRequested) {
    return Action::kNone;
  }
  // Re-record once RPM has held long enough again (stall recovery / next run).
  return tryStartRecording(f, in);
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
    case State::kWatching:
      return stepWatching(f, in);
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
    case State::kWatching:    return "WATCHING";
    case State::kPairing:     return "PAIRING";
  }
  return "?";
}

}  // namespace camera_fsm
