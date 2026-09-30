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

// ---- The idle clock: grace window + idle timer (review fix D3) ----
//
// Grace: no auto-idle in the first kSessionGraceMs of a session (after an
// RPM wake the car is often stationary while GPS reacquires), and the
// grace RE-ARMS on activity that proves the session is alive: a
// completed sprint/drag run, a fresh drag STAGED latch.
//
// The two halves live in ONE struct because they must move together.
// The sketch used to re-arm by rewriting raceSessionStartedAt alone,
// while the grace check returned early without touching an idle timer
// that had already started. A creep under the idle speed that started
// the timer, then a re-stage, left the stale timer running under the
// new grace — so the session ended the instant the new grace expired
// instead of a full hold later. rearmGrace() is now the only way to
// restart the grace, and it always clears the timer with it.
constexpr uint32_t kSessionGraceMs = 180000;  // 3 min

struct Clock {
  uint32_t graceStartMs = 0;
  bool     idleRunning = false;
  uint32_t idleStartMs = 0;
};

// Session start, or activity that proves the session alive: restart the
// grace window AND clear any idle timer already running.
void rearmGrace(Clock& c, uint32_t nowMs);

// Advance the clock one iteration against evaluate()'s decision. Returns
// true exactly when the session should end now. Wrap-safe uint32 millis.
bool advance(Clock& c, const Decision& d, uint32_t nowMs);

}  // namespace idle_policy
