#include "idle_policy.h"

namespace idle_policy {

bool tachProven(int32_t rpm) {
  return rpm > kEngineProvenRpm;
}

Decision evaluate(const Inputs& in) {
  Decision d;
  d.holdMs = in.speedRuleSession ? kSpeedIdleHoldMs : kTachIdleHoldMs;
  d.stopCameraOnEnd = in.speedRuleSession;

  // Yield to an active camera recording — TACH SESSIONS ONLY. There the
  // camera owns the end (30 s engine-off), and the speed-based idle must
  // not cut the log out from under a stationary but engine-running grid
  // stint. A manual/speed session's camera CANNOT stop itself (its RPM
  // stop rule is suppressed), so its idle timer never yields — it is the
  // one and only ender.
  //
  // EXCEPTION — GPS-lock hold: no log file exists yet and the hold pins
  // the UI; if this yielded while the camera records, nothing could end a
  // fileless session with a lock that never arrives (2026-07-19 field
  // incident — the device looked bricked until a power cycle).
  if (!in.speedRuleSession && in.cameraRecording && !in.gpsLockHoldActive) {
    d.yieldToCamera = true;
    return d;
  }

  // Sprint mode: between-run queue waits are normal (engine running,
  // stationary). Idle only counts while the engine is off too, so a
  // running engine at the start line can never end the session.
  if (in.sprintEngineRunning) {
    d.resetTimer = true;
    return d;
  }

  const float threshold =
      in.speedRuleSession ? kSpeedIdleSpeedMph : kTachIdleSpeedMph;
  if (in.speedMph >= threshold) {
    d.resetTimer = true;
  }
  return d;
}

void rearmGrace(Clock& c, uint32_t nowMs) {
  c.graceStartMs = nowMs;
  c.idleRunning = false;
  c.idleStartMs = 0;
}

bool advance(Clock& c, const Decision& d, uint32_t nowMs) {
  // Camera owns the end: leave the timer exactly as it is.
  if (d.yieldToCamera) return false;

  if ((uint32_t)(nowMs - c.graceStartMs) < kSessionGraceMs) return false;

  if (d.resetTimer) {
    c.idleRunning = false;
    c.idleStartMs = 0;
    return false;
  }
  if (!c.idleRunning) {
    c.idleRunning = true;
    c.idleStartMs = nowMs;
    return false;
  }
  return (uint32_t)(nowMs - c.idleStartMs) >= d.holdMs;
}

}  // namespace idle_policy
