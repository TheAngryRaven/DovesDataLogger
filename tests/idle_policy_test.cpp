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

// ---------------------------------------------------------------------------
// The idle clock: grace + timer (review D3)
// ---------------------------------------------------------------------------

namespace {

// Drive the clock at 100 ms ticks with a fixed decision; returns the ms
// offset (from `from`) at which advance() first said "end", or 0.
uint32_t runUntilEnd(Clock& c, const Decision& d, uint32_t from,
                     uint32_t forMs) {
    for (uint32_t t = 0; t <= forMs; t += 100) {
        if (advance(c, d, from + t)) return t;
    }
    return 0;
}

}  // namespace

TEST_CASE("idle_policy - clock: no idle end inside the grace window") {
    Clock c;
    rearmGrace(c, 1000);
    const Decision d = evaluate(base());  // tach, stopped: idle
    CHECK(runUntilEnd(c, d, 1000, kSessionGraceMs - 100) == 0);
    CHECK_FALSE(c.idleRunning);
}

TEST_CASE("idle_policy - clock: idle holds the full hold after the grace") {
    Clock c;
    rearmGrace(c, 0);
    const Decision d = evaluate(base());
    const uint32_t end = runUntilEnd(c, d, 0, kSessionGraceMs + 2 * kTachIdleHoldMs);
    // First post-grace tick starts the timer; the end is one hold later.
    CHECK(end == kSessionGraceMs + kTachIdleHoldMs);
}

TEST_CASE("idle_policy - clock: activity resets a running timer") {
    Clock c;
    rearmGrace(c, 0);
    Inputs in = base();
    const Decision idle = evaluate(in);
    in.speedMph = 30.0f;
    const Decision moving = evaluate(in);
    uint32_t t = kSessionGraceMs;
    CHECK_FALSE(advance(c, idle, t));
    REQUIRE(c.idleRunning);
    CHECK_FALSE(advance(c, moving, t + 1000));
    CHECK_FALSE(c.idleRunning);
}

TEST_CASE("idle_policy - clock: re-arming the grace clears an idle timer already running") {
    // Regression (review D3): drag queue creep under the idle speed
    // starts the timer after the grace; the car then re-stages, which
    // re-arms the grace. The stale timer used to keep running under the
    // new grace, so the session ended the moment the NEW grace expired.
    // It must instead get a full grace AND a full hold.
    Inputs in = base();
    in.speedRuleSession = true;  // manual drag session: 5 min / 5 mph
    const Decision idle = evaluate(in);

    Clock c;
    rearmGrace(c, 0);
    uint32_t t = kSessionGraceMs;
    CHECK_FALSE(advance(c, idle, t));  // idle timer starts
    REQUIRE(c.idleRunning);

    t += kSpeedIdleHoldMs - 10000;     // 10 s short of ending...
    CHECK_FALSE(advance(c, idle, t));
    rearmGrace(c, t);                  // ...a fresh stage latch
    CHECK_FALSE(c.idleRunning);

    const uint32_t end = runUntilEnd(c, idle, t, kSessionGraceMs + 2 * kSpeedIdleHoldMs);
    CHECK(end == kSessionGraceMs + kSpeedIdleHoldMs);
}

TEST_CASE("idle_policy - clock: camera yield leaves the timer untouched") {
    Clock c;
    rearmGrace(c, 0);
    const Decision idle = evaluate(base());
    uint32_t t = kSessionGraceMs;
    CHECK_FALSE(advance(c, idle, t));
    REQUIRE(c.idleRunning);
    Inputs in = base();
    in.cameraRecording = true;
    const Decision yield = evaluate(in);
    REQUIRE(yield.yieldToCamera);
    CHECK_FALSE(advance(c, yield, t + kTachIdleHoldMs * 2));
    CHECK(c.idleRunning);
    CHECK(c.idleStartMs == t);
}

TEST_CASE("idle_policy - clock: millis wrap inside the grace and the hold") {
    Clock c;
    const uint32_t start = 0xFFFFFFFFu - 1000u;
    rearmGrace(c, start);
    const Decision d = evaluate(base());
    CHECK(runUntilEnd(c, d, start, kSessionGraceMs + 2 * kTachIdleHoldMs) ==
          kSessionGraceMs + kTachIdleHoldMs);
}
