#pragma once

/*
 * idle_policy — the auto-idle session-end decision table (pure logic).
 *
 * checkAutoIdle() in BirdsEye.ino is cause-aware: a tach-entered session
 * keeps the original 60 s / <2 mph data-only ender (yielding to an active
 * camera recording, which owns its own 30 s engine-off stop), while a
 * manual/speed-entered session — no engine signal — is ended by a
 * 5 min / <5 mph rule that also stops the camera. Which rule applies, when
 * the timer yields, and when it resets are all policy, not I/O, so they
 * live here where tests/idle_policy_test.cpp can hold them down. The
 * sketch keeps only the clock (millis) and the side effects.
 *
 * Also here: the promotion to TACH cause. A speed-trip entry on a
 * tach-equipped kart (push/bump start: rolling above the trip speed
 * before the engine has fired) or a manual menu press (engine off at the
 * time) must not carry the no-tach rules for the whole session — once the
 * tach proves itself, the session is tach-ruled. Devices with no tach
 * never read above the threshold, so their sessions are unaffected.
 *
 * Pure logic — no Arduino headers.
 */

#include <stdint.h>

namespace idle_policy {

// ---- Tunables (single-point edits) ----
constexpr float    kTachIdleSpeedMph  = 2.0f;    // tach session: below this counts as idle
constexpr uint32_t kTachIdleHoldMs    = 60000;   // ...for this long -> end session (data only)
constexpr float    kSpeedIdleSpeedMph = 5.0f;    // manual/speed session: below this counts as idle
constexpr uint32_t kSpeedIdleHoldMs   = 300000;  // ...for this long -> end session + camera

// The tach has "proven itself" at the same threshold every other engine
// gate uses (autoRaceModeCheck, camera wake).
constexpr int32_t kEngineProvenRpm = 500;

// SPEED->TACH promotion: true when a speed-cause session should be
// reclassified as tach-ruled because real ignition is being counted.
bool tachProven(int32_t rpm);

struct Inputs {
  bool  speedRuleSession = false;   // cause is MANUAL or SPEED (after promotion)
  bool  cameraRecording = false;    // camera FSM in kRecording
  bool  gpsLockHoldActive = false;  // session still waiting for its GPS time lock
  bool  sprintEngineRunning = false;  // sprint OR drag mode AND tach reads > 0
  float speedMph = 0.0f;
};

struct Decision {
  bool     yieldToCamera = false;  // return without touching the idle timer
  bool     resetTimer = false;     // conditions say "active" — clear the idle timer
  uint32_t holdMs = 0;             // idle must persist this long to end the session
  bool     stopCameraOnEnd = false;  // on expiry, notify the camera BEFORE endRaceSession()
};

// Evaluate one iteration's policy. Exactly one of yieldToCamera /
// resetTimer / "let the timer run toward holdMs" applies.
Decision evaluate(const Inputs& in);

}  // namespace idle_policy
