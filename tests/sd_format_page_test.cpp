#include "doctest.h"
#include "sd_format_page.h"

using namespace sd_format_page;

namespace {

Inputs quiet(uint32_t nowMs) {
    return {/*selectHeld=*/false, /*otherButtonHeld=*/false,
            /*otherButtonPressed=*/false, /*engineRunning=*/false, nowMs};
}

Inputs held(uint32_t nowMs) {
    Inputs in = quiet(nowMs);
    in.selectHeld = true;
    return in;
}

Inputs otherButton(uint32_t nowMs) {
    Inputs in = quiet(nowMs);
    in.otherButtonPressed = true;
    return in;
}

Inputs comboHeld(uint32_t nowMs) {  // Select + side button, both down
    Inputs in = held(nowMs);
    in.otherButtonHeld = true;
    return in;
}

Inputs engine(uint32_t nowMs) {
    Inputs in = quiet(nowMs);
    in.engineRunning = true;
    return in;
}

// Enter the page and observe Select up once, so holds are eligible.
void beginReleased(State& s, uint32_t nowMs) {
    begin(s, nowMs);
    CHECK(step(s, quiet(nowMs)) == Exit::kStay);
}

}  // namespace

// ---------------------------------------------------------------------------
// Hold behavior
// ---------------------------------------------------------------------------

TEST_CASE("step - holds while there is no input and no timeout") {
    State s;
    begin(s, 1000);
    for (uint32_t t = 1000; t < 1000 + kIdleTimeoutMs - 1; t += 5000) {
        CHECK(step(s, quiet(t)) == Exit::kStay);
    }
}

// ---------------------------------------------------------------------------
// Confirm hold
// ---------------------------------------------------------------------------

TEST_CASE("step - continuous Select hold confirms exactly at kHoldToConfirmMs") {
    State s;
    beginReleased(s, 0);
    CHECK(step(s, held(1000)) == Exit::kStay);  // arms
    CHECK(s.holdArmed);
    CHECK(step(s, held(1000 + kHoldToConfirmMs - 1)) == Exit::kStay);
    CHECK(step(s, held(1000 + kHoldToConfirmMs)) == Exit::kFormat);
}

TEST_CASE("step - Select already down at page entry never arms until released") {
    // The wake press itself: user held Select to power on and keeps holding
    // through the dark boot. It must not count toward an erase.
    State s;
    begin(s, 0);
    for (uint32_t t = 0; t <= 3 * kHoldToConfirmMs; t += 100) {
        CHECK(step(s, held(t)) == Exit::kStay);
    }
    CHECK_FALSE(s.holdArmed);
    // Release, then a fresh hold works normally.
    CHECK(step(s, quiet(10000)) == Exit::kStay);
    CHECK(step(s, held(10100)) == Exit::kStay);
    CHECK(step(s, held(10100 + kHoldToConfirmMs)) == Exit::kFormat);
}

TEST_CASE("step - release before the threshold disarms; re-hold restarts the full window") {
    State s;
    beginReleased(s, 0);
    CHECK(step(s, held(1000)) == Exit::kStay);
    CHECK(step(s, held(3900)) == Exit::kStay);   // 2.9 s in
    CHECK(step(s, quiet(3950)) == Exit::kStay);  // bump/flicker release
    CHECK_FALSE(s.holdArmed);
    // Re-hold: the old arm time must not count toward an erase.
    CHECK(step(s, held(4000)) == Exit::kStay);
    CHECK(step(s, held(4000 + kHoldToConfirmMs - 1)) == Exit::kStay);
    CHECK(step(s, held(4000 + kHoldToConfirmMs)) == Exit::kFormat);
}

TEST_CASE("step - other buttons never confirm") {
    State s;
    beginReleased(s, 0);
    for (uint32_t t = 100; t <= 100 + 2 * kHoldToConfirmMs; t += 100) {
        CHECK(step(s, otherButton(t)) == Exit::kStay);
    }
    CHECK_FALSE(s.holdArmed);
}

TEST_CASE("step - Select+side hold never formats (global reboot combo wins)") {
    // The any-page reboot combo is Select + a side button held 5 s. Holding
    // both must never arm the 3 s format confirm, or the erase would fire
    // 2 s before the reboot the user actually asked for.
    State s;
    beginReleased(s, 0);
    for (uint32_t t = 100; t <= 100 + 3 * kHoldToConfirmMs; t += 100) {
        CHECK(step(s, comboHeld(t)) == Exit::kStay);
    }
    CHECK_FALSE(s.holdArmed);
}

TEST_CASE("step - side button joining mid-hold disarms; full window required after") {
    State s;
    beginReleased(s, 0);
    CHECK(step(s, held(1000)) == Exit::kStay);       // arms
    CHECK(step(s, comboHeld(3500)) == Exit::kStay);  // side joins at 2.5 s
    CHECK_FALSE(s.holdArmed);
    CHECK(step(s, comboHeld(1000 + kHoldToConfirmMs)) == Exit::kStay);
    // Side released, Select still down: full window restarts from here.
    CHECK(step(s, held(5000)) == Exit::kStay);
    CHECK(step(s, held(5000 + kHoldToConfirmMs - 1)) == Exit::kStay);
    CHECK(step(s, held(5000 + kHoldToConfirmMs)) == Exit::kFormat);
}

TEST_CASE("holdSecondsLeft - reports 3..1 while armed, 0 otherwise") {
    State s;
    beginReleased(s, 0);
    CHECK(holdSecondsLeft(s, 0) == 0);  // not armed
    step(s, held(1000));
    CHECK(holdSecondsLeft(s, 1000) == 3);
    CHECK(holdSecondsLeft(s, 2001) == 2);
    CHECK(holdSecondsLeft(s, 3001) == 1);
    CHECK(holdSecondsLeft(s, 1000 + kHoldToConfirmMs) == 0);
    step(s, quiet(1500));  // release disarms
    CHECK(holdSecondsLeft(s, 1500) == 0);
}

// ---------------------------------------------------------------------------
// Idle shutdown
// ---------------------------------------------------------------------------

TEST_CASE("step - idle timeout shuts down exactly at kIdleTimeoutMs") {
    State s;
    begin(s, 1000);
    CHECK(step(s, quiet(1000 + kIdleTimeoutMs - 1)) == Exit::kStay);
    CHECK(step(s, quiet(1000 + kIdleTimeoutMs)) == Exit::kToShutdown);
}

TEST_CASE("step - any button activity defers the idle timeout") {
    State s;
    begin(s, 0);
    // A B1/B3 press deep into the idle window restarts it.
    CHECK(step(s, otherButton(kIdleTimeoutMs - 1000)) == Exit::kStay);
    CHECK(step(s, quiet(kIdleTimeoutMs)) == Exit::kStay);  // old deadline passed
    CHECK(step(s, quiet(kIdleTimeoutMs - 1000 + kIdleTimeoutMs - 1)) == Exit::kStay);
    CHECK(step(s, quiet(kIdleTimeoutMs - 1000 + kIdleTimeoutMs)) == Exit::kToShutdown);
}

TEST_CASE("step - a Select hold also counts as activity") {
    State s;
    begin(s, 0);
    CHECK(step(s, held(kIdleTimeoutMs - 500)) == Exit::kStay);   // activity
    CHECK(step(s, quiet(kIdleTimeoutMs + 500)) == Exit::kStay);  // deadline moved
}

TEST_CASE("step - a running engine defers the idle shutdown") {
    // Tach-wake boot with a bad card: shutting down mid-session would just
    // tach-re-wake back into this page every 5 minutes, power-cycling the
    // device for the whole drive. RPM counts as activity.
    State s;
    begin(s, 0);
    for (uint32_t t = 0; t <= 3 * kIdleTimeoutMs; t += 60000) {
        CHECK(step(s, engine(t)) == Exit::kStay);
    }
    // Engine stops: the idle countdown starts from the last engine step.
    const uint32_t lastEngine = 3 * kIdleTimeoutMs;
    CHECK(step(s, quiet(lastEngine + kIdleTimeoutMs - 1)) == Exit::kStay);
    CHECK(step(s, quiet(lastEngine + kIdleTimeoutMs)) == Exit::kToShutdown);
}

TEST_CASE("step - an armed hold straddling the idle deadline still confirms") {
    State s;
    beginReleased(s, 0);
    // Arm just before the idle deadline; the hold refreshes activity every
    // step, so the confirm window completes instead of shutting down.
    const uint32_t armAt = kIdleTimeoutMs - 100;
    CHECK(step(s, held(armAt)) == Exit::kStay);
    CHECK(step(s, held(armAt + kHoldToConfirmMs - 1)) == Exit::kStay);
    CHECK(step(s, held(armAt + kHoldToConfirmMs)) == Exit::kFormat);
}
