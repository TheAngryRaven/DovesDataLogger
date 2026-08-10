#include "doctest.h"
#include "idle_policy.h"

using namespace idle_policy;

namespace {

Inputs base() {
    Inputs in;
    in.speedRuleSession = false;
    in.cameraRecording = false;
    in.gpsLockHoldActive = false;
    in.sprintEngineRunning = false;
    in.speedMph = 0.0f;
    return in;
}

}  // namespace

// ---------------------------------------------------------------------------
// SPEED->TACH promotion
// ---------------------------------------------------------------------------

TEST_CASE("idle_policy - tachProven at the shared engine threshold") {
    CHECK(tachProven(0) == false);
    CHECK(tachProven(kEngineProvenRpm) == false);      // strictly above, like autoRaceModeCheck
    CHECK(tachProven(kEngineProvenRpm + 1) == true);
    CHECK(tachProven(6000) == true);
}

// ---------------------------------------------------------------------------
// Which rule applies
// ---------------------------------------------------------------------------

TEST_CASE("idle_policy - tach session keeps the original 60s/2mph data-only rule") {
    Inputs in = base();
    const Decision d = evaluate(in);
    CHECK(d.holdMs == kTachIdleHoldMs);
    CHECK(d.stopCameraOnEnd == false);
    CHECK(d.yieldToCamera == false);
    CHECK(d.resetTimer == false);  // parked at 0 mph — timer runs
}

TEST_CASE("idle_policy - manual/speed session gets 5min/5mph and owns the camera") {
    Inputs in = base();
    in.speedRuleSession = true;
    const Decision d = evaluate(in);
    CHECK(d.holdMs == kSpeedIdleHoldMs);
    CHECK(d.stopCameraOnEnd == true);
    CHECK(d.yieldToCamera == false);
}

TEST_CASE("idle_policy - thresholds bind to their own rule") {
    Inputs in = base();
    // 3 mph: above the tach threshold (resets), below the speed-rule one (idles).
    in.speedMph = 3.0f;
    CHECK(evaluate(in).resetTimer == true);
    in.speedRuleSession = true;
    CHECK(evaluate(in).resetTimer == false);
    // 5 mph: at the speed-rule threshold — moving, resets.
    in.speedMph = kSpeedIdleSpeedMph;
    CHECK(evaluate(in).resetTimer == true);
}

// ---------------------------------------------------------------------------
// The camera yield — the decision the 2026-08 review caught being cause-blind
// ---------------------------------------------------------------------------

TEST_CASE("idle_policy - tach session yields to an active camera recording") {
    Inputs in = base();
    in.cameraRecording = true;
    const Decision d = evaluate(in);
    CHECK(d.yieldToCamera == true);
}

TEST_CASE("idle_policy - manual/speed session NEVER yields to the camera") {
    // Its camera cannot stop itself (rpm stop rule suppressed): this idle
    // timer is the one and only ender, so yielding would strand the session.
    Inputs in = base();
    in.speedRuleSession = true;
    in.cameraRecording = true;
    const Decision d = evaluate(in);
    CHECK(d.yieldToCamera == false);
    CHECK(d.stopCameraOnEnd == true);
}

TEST_CASE("idle_policy - GPS-lock hold breaks the tach yield (fileless session)") {
    // 2026-07-19 incident: lock never arrives, camera recording, and the
    // yield was the only ender left — the device looked bricked.
    Inputs in = base();
    in.cameraRecording = true;
    in.gpsLockHoldActive = true;
    const Decision d = evaluate(in);
    CHECK(d.yieldToCamera == false);
}

// ---------------------------------------------------------------------------
// Sprint engine-aware reset
// ---------------------------------------------------------------------------

TEST_CASE("idle_policy - a running engine in sprint mode always resets the timer") {
    Inputs in = base();
    in.sprintEngineRunning = true;
    in.speedMph = 0.0f;  // stationary at the start line
    CHECK(evaluate(in).resetTimer == true);

    // Even for a (promoted-to-speed-rule) session — a running engine in the
    // sprint queue must never be ended by the idle timer.
    in.speedRuleSession = true;
    CHECK(evaluate(in).resetTimer == true);
}

TEST_CASE("idle_policy - sprint with engine off falls through to the speed rule") {
    Inputs in = base();
    in.sprintEngineRunning = false;  // sprint mode but tach reads 0
    in.speedMph = 0.5f;
    CHECK(evaluate(in).resetTimer == false);  // timer runs toward the end
}
