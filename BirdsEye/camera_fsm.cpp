#include "camera_fsm.h"

namespace camera_fsm {

namespace {

// Timestamps are stored non-zero so 0 can mean "timer not running".
uint32_t seedNow(uint32_t nowMs) { return nowMs == 0 ? 1 : nowMs; }

// Wraparound-safe elapsed-interval check (unsigned subtraction).
bool elapsed(uint32_t nowMs, uint32_t sinceMs, uint32_t intervalMs) {
  return (nowMs - sinceMs) >= intervalMs;
}

// Reset every timer/attempt/flag field. State and serialPresent untouched.
void clearTimers(Fsm& f) {
  f.rpmOnSince = 0;
  f.rpmGoneSince = 0;
  f.wakeAttemptStarted = 0;
  f.wakeAttemptsUsed = 0;
  f.wakeBurstPhase = false;
  f.controlAttemptStarted = 0;
  f.controlAttemptsUsed = 0;
  f.awaitGpsSince = 0;
  f.stopCondSince = 0;
  f.lastGpsFeedAt = 0;
  f.lastKeepAliveAt = 0;
  f.cooldownSince = 0;
  f.powerOffSentAt = 0;
  f.pairingSince = 0;
  f.entryPending = false;
}

// ---- Shared transitions -------------------------------------------------

void enterIdle(Fsm& f) {
  f.state = State::kIdle;
  f.rpmOnSince = 0;  // fresh RPM debounce on every (re)entry
}

void enterConnectingFresh(Fsm& f, uint32_t nowMs) {
  f.state = State::kConnecting;
  f.controlAttemptsUsed = 1;
  f.controlAttemptStarted = seedNow(nowMs);
  f.entryPending = true;  // kStartControlConnect emitted by the state logic
}

void enterAwaitGps(Fsm& f, uint32_t nowMs) {
  f.state = State::kAwaitGps;
  f.awaitGpsSince = seedNow(nowMs);
  f.entryPending = false;
}

Action enterRecording(Fsm& f, uint32_t nowMs) {
  f.state = State::kRecording;
  f.stopCondSince = 0;
  // Back-date the feed stamp so the first GPS frame goes out immediately.
  f.lastGpsFeedAt = nowMs - kGpsFeedIntervalMs;
  f.lastKeepAliveAt = seedNow(nowMs);
  return Action::kSendStartVideo;
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

// be80 keep-alive cadence, shared by kAwaitGps / kRecording / kCooldown.
// Stamps the timer when due; the caller returns kSendKeepAlive.
bool keepAliveDue(Fsm& f, const Inputs& in) {
  if (!in.controlConnected) return false;
  if (!elapsed(in.nowMs, f.lastKeepAliveAt, kKeepAliveIntervalMs)) return false;
  f.lastKeepAliveAt = seedNow(in.nowMs);
  return true;
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

  if (in.controlConnected) {
    // Stale control link from a previous session: skip wake and connect.
    enterAwaitGps(f, in.nowMs);
    return Action::kNone;
  }
  if (in.remoteConnected) {
    // Camera already holds our remote service: skip the wake advert.
    enterConnectingFresh(f, in.nowMs);
    return Action::kNone;  // entry action emitted next step
  }
  f.state = State::kWaking;
  f.wakeAttemptsUsed = 1;
  f.wakeAttemptStarted = seedNow(in.nowMs);
  f.wakeBurstPhase = true;
  f.rpmGoneSince = 0;
  return Action::kStartWakeBurst;
}

Action stepWaking(Fsm& f, const Inputs& in) {
  if (in.remoteConnected || in.cameraAdvertSeen) {
    enterConnectingFresh(f, in.nowMs);
    return Action::kStopAdvertising;
  }
  // Engine gone again before the camera showed up: abort the wake.
  if (in.rpm < kRpmOffThreshold) {
    if (f.rpmGoneSince == 0) {
      f.rpmGoneSince = seedNow(in.nowMs);
    } else if (elapsed(in.nowMs, f.rpmGoneSince, kRpmGoneAbortMs)) {
      enterIdle(f);
      return Action::kStopAdvertising;
    }
  } else {
    f.rpmGoneSince = 0;
  }
  // Wake mfg-data burst window over: switch to connectable advertising.
  if (f.wakeBurstPhase && elapsed(in.nowMs, f.wakeAttemptStarted, kWakeBurstMs)) {
    f.wakeBurstPhase = false;
    return Action::kStartConnectableAdvertising;
  }
  if (elapsed(in.nowMs, f.wakeAttemptStarted, kConnectTimeoutMs)) {
    if (f.wakeAttemptsUsed < kConnectRetries) {
      f.wakeAttemptsUsed++;
      f.wakeAttemptStarted = seedNow(in.nowMs);
      f.wakeBurstPhase = true;
      return Action::kStartWakeBurst;
    }
    enterIdle(f);
    return Action::kStopAdvertising;
  }
  if (in.sessionEndRequested) {
    enterIdle(f);
    return Action::kStopAdvertising;
  }
  return Action::kNone;
}

Action stepConnecting(Fsm& f, const Inputs& in) {
  if (f.entryPending) {
    f.entryPending = false;
    return Action::kStartControlConnect;
  }
  if (in.controlConnected) {
    enterAwaitGps(f, in.nowMs);
    f.lastKeepAliveAt = seedNow(in.nowMs);
    return Action::kNone;
  }
  if (elapsed(in.nowMs, f.controlAttemptStarted, kControlConnectTimeoutMs)) {
    if (f.controlAttemptsUsed < kControlConnectRetries) {
      f.controlAttemptsUsed++;
      f.controlAttemptStarted = seedNow(in.nowMs);
      return Action::kStartControlConnect;
    }
    enterIdle(f);
    return Action::kStopControlConnect;
  }
  if (in.sessionEndRequested) {
    enterIdle(f);
    return Action::kStopControlConnect;
  }
  return Action::kNone;
}

Action stepAwaitGps(Fsm& f, const Inputs& in) {
  if (!in.controlConnected) {
    // Control dropped before recording started: reconnect (fresh budget).
    enterConnectingFresh(f, in.nowMs);
    return Action::kNone;
  }
  if (in.sessionEndRequested) {
    // Nothing recording yet; the camera just rides its cooldown.
    return enterCooldown(f, in.nowMs, Action::kNone);
  }
  const bool lockTimedOut =
      kRecordWithoutGpsOnTimeout &&
      elapsed(in.nowMs, f.awaitGpsSince, kGpsLockTimeoutMs);
  if (in.gpsFixValid || lockTimedOut) {
    return enterRecording(f, in.nowMs);
  }
  if (keepAliveDue(f, in)) return Action::kSendKeepAlive;
  return Action::kNone;
}

Action stepRecording(Fsm& f, const Inputs& in) {
  // 1. Manual session end: stop immediately, bypassing the hold timer.
  if (in.sessionEndRequested) {
    return enterCooldown(f, in.nowMs, Action::kSendStopVideo);
  }
  // 2. Control drop: reconnect and resume; logging untouched.
  if (!in.controlConnected) {
    enterConnectingFresh(f, in.nowMs);
    return Action::kNone;
  }
  // 3. Sustained stop condition.
  const bool speedStopped = in.speedMph < kSpeedStopThresholdMph;
  const bool rpmStopped = in.rpm < kRpmOffThreshold;
  const bool stopped = kStopOnEither ? (speedStopped || rpmStopped)
                                     : (speedStopped && rpmStopped);
  if (stopped) {
    if (f.stopCondSince == 0) {
      f.stopCondSince = seedNow(in.nowMs);
    } else if (elapsed(in.nowMs, f.stopCondSince, kStopRecordDelayMs)) {
      f.stopCondSince = 0;
      return enterCooldown(f, in.nowMs, Action::kSendStopVideo);
    }
  } else {
    f.stopCondSince = 0;
  }
  // 4. GPS overlay feed at 1 Hz. Fix lost = silently no frames.
  if (in.gpsFixValid && elapsed(in.nowMs, f.lastGpsFeedAt, kGpsFeedIntervalMs)) {
    f.lastGpsFeedAt = seedNow(in.nowMs);
    return Action::kSendGpsFrame;
  }
  // 5. Keep-alive.
  if (keepAliveDue(f, in)) return Action::kSendKeepAlive;
  return Action::kNone;
}

Action stepCooldown(Fsm& f, const Inputs& in) {
  if (in.sessionEndRequested) {
    // Manual end during cooldown powers the camera off immediately.
    return enterPoweringOff(f, in.nowMs);
  }
  if (!in.remoteConnected && !in.controlConnected) {
    // Camera turned itself off.
    enterIdle(f);
    return Action::kNone;
  }
  if (elapsed(in.nowMs, f.cooldownSince, kPowerOffDelayMs)) {
    return enterPoweringOff(f, in.nowMs);
  }
  if (kAutoResumeFromCooldown) {
    // DEFERRED v1 (kAutoResumeFromCooldown == false): motion/RPM return
    // during cooldown is deliberately ignored. When the flag flips, add
    // the re-arm transition back to kRecording here.
  }
  // Keep the be80 link alive through the power-off window so the
  // both-links-down check above stays meaningful.
  if (keepAliveDue(f, in)) return Action::kSendKeepAlive;
  return Action::kNone;
}

Action stepPoweringOff(Fsm& f, const Inputs& in) {
  if (!in.remoteConnected && !in.controlConnected) {
    enterIdle(f);
    return Action::kNone;
  }
  if (elapsed(in.nowMs, f.powerOffSentAt, kPowerOffLingerMs)) {
    // Camera never dropped the links: force them down ourselves.
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

  // 1. Force-idle (transfer takeover / sleep entry) overrides everything.
  //    The caller does its own physical teardown in force paths; the
  //    returned action is belt-and-suspenders.
  if (in.forceIdleRequested) {
    const State prev = f.state;
    clearTimers(f);
    f.state = f.serialPresent ? State::kIdle : State::kUnpaired;
    if (prev == State::kWaking || prev == State::kPairing) {
      return Action::kStopAdvertising;  // we were advertising
    }
    if (prev == State::kConnecting) return Action::kStopControlConnect;
    return Action::kNone;
  }

  // 2. Unpair: only honored while no session machinery is running.
  if (in.unpairRequested &&
      (f.state == State::kUnpaired || f.state == State::kIdle)) {
    f.state = State::kUnpaired;
    f.serialPresent = false;
    return Action::kNone;
  }

  // 3. Enter pairing (falls through so the state logic emits the entry
  //    action this same step).
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
    case State::kConnecting:
      return stepConnecting(f, in);
    case State::kAwaitGps:
      return stepAwaitGps(f, in);
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
    case State::kConnecting:  return "CONNECTING";
    case State::kAwaitGps:    return "AWAIT GPS";
    case State::kRecording:   return "RECORDING";
    case State::kCooldown:    return "COOLDOWN";
    case State::kPoweringOff: return "PWR OFF";
    case State::kPairing:     return "PAIRING";
  }
  return "?";
}

}  // namespace camera_fsm
