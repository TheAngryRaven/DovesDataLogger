#include "doctest.h"
#include "gps_status_page.h"

using namespace gps_status_page;

namespace {

Inputs quiet(uint32_t nowMs) {
    return {/*fix=*/false, /*timeValid=*/false, /*buttonPressed=*/false,
            /*tachWakeBoot=*/false, /*engineRunning=*/false, nowMs};
}

Inputs locked(uint32_t nowMs) {
    Inputs in = quiet(nowMs);
    in.fix = true;
    in.timeValid = true;
    return in;
}

}  // namespace

// ---------------------------------------------------------------------------
// Hold behavior
// ---------------------------------------------------------------------------

TEST_CASE("step - holds while there is no lock, no button, no timeout") {
    State s;
    begin(s, 1000);
    for (uint32_t t = 1000; t < 60000; t += 250) {
        CHECK(step(s, quiet(t)) == Exit::kStay);
    }
}

TEST_CASE("step - fix without valid time does not arm the countdown") {
    State s;
    begin(s, 0);
    Inputs in = quiet(100);
    in.fix = true;  // position fix but time not fully resolved
    CHECK(step(s, in) == Exit::kStay);
    CHECK_FALSE(s.lockArmed);
}

// ---------------------------------------------------------------------------
// Auto-close countdown
// ---------------------------------------------------------------------------

TEST_CASE("step - stable lock auto-closes after kAutoCloseMs") {
    State s;
    begin(s, 0);
    CHECK(step(s, locked(1000)) == Exit::kStay);   // arms
    CHECK(step(s, locked(1000 + kAutoCloseMs - 1)) == Exit::kStay);
    CHECK(step(s, locked(1000 + kAutoCloseMs)) == Exit::kToMenu);
}

TEST_CASE("step - lock dropping mid-countdown restarts the full countdown") {
    State s;
    begin(s, 0);
    CHECK(step(s, locked(1000)) == Exit::kStay);
    CHECK(step(s, quiet(3000)) == Exit::kStay);    // lock flickers away
    CHECK_FALSE(s.lockArmed);
    // Re-lock: the old arm time must not count.
    CHECK(step(s, locked(3500)) == Exit::kStay);
    CHECK(step(s, locked(3500 + kAutoCloseMs - 1)) == Exit::kStay);
    CHECK(step(s, locked(3500 + kAutoCloseMs)) == Exit::kToMenu);
}

TEST_CASE("countdownSecondsLeft - reports 3..1 while armed, 0 otherwise") {
    State s;
    begin(s, 0);
    CHECK(countdownSecondsLeft(s, 0) == 0);        // not armed
    step(s, locked(1000));
    CHECK(countdownSecondsLeft(s, 1000) == 3);
    CHECK(countdownSecondsLeft(s, 2001) == 2);
    CHECK(countdownSecondsLeft(s, 3001) == 1);
    CHECK(countdownSecondsLeft(s, 1000 + kAutoCloseMs) == 0);
}

// ---------------------------------------------------------------------------
// Button skip + exit destination
// ---------------------------------------------------------------------------

TEST_CASE("step - any button exits immediately to the menu") {
    State s;
    begin(s, 0);
    Inputs in = quiet(50);
    in.buttonPressed = true;
    CHECK(step(s, in) == Exit::kToMenu);
}

TEST_CASE("step - tach-wake boot routes every exit to race mode") {
    State s;
    begin(s, 0);
    Inputs in = quiet(50);
    in.tachWakeBoot = true;
    in.buttonPressed = true;
    CHECK(step(s, in) == Exit::kToRace);

    begin(s, 0);
    Inputs lockIn = locked(1000);
    lockIn.tachWakeBoot = true;
    CHECK(step(s, lockIn) == Exit::kStay);
    lockIn.nowMs = 1000 + kAutoCloseMs;
    CHECK(step(s, lockIn) == Exit::kToRace);
}

TEST_CASE("step - live engine routes every exit to race mode") {
    State s;
    begin(s, 0);
    Inputs in = quiet(50);
    in.engineRunning = true;
    in.buttonPressed = true;
    CHECK(step(s, in) == Exit::kToRace);

    begin(s, 0);
    Inputs lockIn = locked(1000);
    lockIn.engineRunning = true;
    CHECK(step(s, lockIn) == Exit::kStay);
    lockIn.nowMs = 1000 + kAutoCloseMs;
    CHECK(step(s, lockIn) == Exit::kToRace);
}

TEST_CASE("step - engine running does NOT exit early on its own") {
    // User decision: status page first, even on tach wake — engine state
    // only picks the destination, never skips the page.
    State s;
    begin(s, 0);
    Inputs in = quiet(0);
    in.tachWakeBoot = true;
    in.engineRunning = true;
    for (uint32_t t = 0; t < 120000; t += 500) {
        in.nowMs = t;
        CHECK(step(s, in) == Exit::kStay);
    }
}

// ---------------------------------------------------------------------------
// Idle shutdown
// ---------------------------------------------------------------------------

TEST_CASE("step - idle timeout with no lock and no engine shuts down") {
    State s;
    begin(s, 0);
    CHECK(step(s, quiet(kIdleTimeoutMs - 1)) == Exit::kStay);
    CHECK(step(s, quiet(kIdleTimeoutMs)) == Exit::kToShutdown);
}

TEST_CASE("step - engine activity defers the idle shutdown") {
    // Keyed on live engine state: a spurious EMI tach wake with no
    // follow-up pulses still times out, but real pulses keep it awake.
    State s;
    begin(s, 0);
    Inputs running = quiet(kIdleTimeoutMs - 1000);
    running.engineRunning = true;
    CHECK(step(s, running) == Exit::kStay);
    // Engine stops; the clock restarts from the last activity.
    CHECK(step(s, quiet(2 * kIdleTimeoutMs - 2000)) == Exit::kStay);
    CHECK(step(s, quiet(2 * kIdleTimeoutMs - 1000)) == Exit::kToShutdown);
}

TEST_CASE("step - a tach-wake boot alone does not defer the idle shutdown") {
    State s;
    begin(s, 0);
    Inputs in = quiet(kIdleTimeoutMs);
    in.tachWakeBoot = true;  // latched boot cause, engine now silent
    CHECK(step(s, in) == Exit::kToShutdown);
}

TEST_CASE("step - an armed lock countdown is never interrupted by idle") {
    State s;
    begin(s, 0);
    // Sit quiet almost to the timeout, then lock right before it.
    CHECK(step(s, quiet(kIdleTimeoutMs - 10)) == Exit::kStay);
    CHECK(step(s, locked(kIdleTimeoutMs - 5)) == Exit::kStay);   // arms
    CHECK(step(s, locked(kIdleTimeoutMs + 100)) == Exit::kStay); // past idle, still counting
    CHECK(step(s, locked(kIdleTimeoutMs - 5 + kAutoCloseMs)) == Exit::kToMenu);
}
