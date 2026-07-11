#include "doctest.h"
#include "camera_fsm.h"

#include <algorithm>
#include <cstring>
#include <vector>

using namespace camera_fsm;

namespace {

// Small driver: holds an Inputs snapshot, advances a fake millis clock, and
// collects the actions step() emits.
struct Sim {
    Fsm f;
    Inputs in;

    explicit Sim(bool serial = true, uint32_t startMs = 1000) {
        init(f, serial);
        in.nowMs = startMs;
    }

    // Advance the clock and run one step().
    Action tick(uint32_t advanceMs = 0) {
        in.nowMs += advanceMs;  // uint32_t: wraps like millis()
        return step(f, in);
    }

    // Raise a one-shot event flag for exactly one step.
    Action pulse(bool Inputs::*flag, uint32_t advanceMs = 0) {
        in.*flag = true;
        const Action a = tick(advanceMs);
        in.*flag = false;
        return a;
    }

    // Step every stepMs for totalMs; return every non-kNone action emitted.
    std::vector<Action> run(uint32_t totalMs, uint32_t stepMs) {
        std::vector<Action> out;
        for (uint32_t t = 0; t < totalMs; t += stepMs) {
            const Action a = tick(stepMs);
            if (a != Action::kNone) out.push_back(a);
        }
        return out;
    }

    // ---- Drivers to reach each state via real input sequences ----

    void toWaking() {
        in.rpm = 1000;
        REQUIRE(tick(0) == Action::kNone);  // arms the RPM debounce
        REQUIRE(tick(kRpmOnDebounceMs) == Action::kStartWakeBurst);
        REQUIRE(f.state == State::kWaking);
    }

    // Camera connects to us (remoteConnected) → AWAIT READY.
    void toAwaitReady() {
        toWaking();
        in.remoteConnected = true;
        REQUIRE(tick(10) == Action::kNone);
        REQUIRE(f.state == State::kAwaitReady);
    }

    void toRecording() {
        toAwaitReady();
        in.ce82Subscribed = true;
        in.gpsFixValid = true;
        REQUIRE(tick(10) == Action::kSendShutter);
        REQUIRE(f.state == State::kRecording);
        REQUIRE(f.recordingActive == true);
    }

    void toCooldown() {
        toRecording();
        REQUIRE(pulse(&Inputs::sessionEndRequested, 10) == Action::kSendShutter);
        REQUIRE(f.state == State::kCooldown);
    }

    void toPoweringOff() {
        toCooldown();
        REQUIRE(pulse(&Inputs::sessionEndRequested, 10) == Action::kSendPowerOff);
        REQUIRE(f.state == State::kPoweringOff);
    }

    void toPairing() {
        REQUIRE(pulse(&Inputs::pairRequested, 10) ==
                Action::kStartConnectableAdvertising);
        REQUIRE(f.state == State::kPairing);
    }
};

long countOf(const std::vector<Action>& v, Action a) {
    return std::count(v.begin(), v.end(), a);
}

}  // namespace

// ---------------------------------------------------------------------------
// Boot / UNPAIRED
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - init boot state follows serialPresent") {
    Fsm f;
    init(f, true);
    CHECK(f.state == State::kIdle);
    CHECK(f.serialPresent == true);

    init(f, false);
    CHECK(f.state == State::kUnpaired);
    CHECK(f.serialPresent == false);
}

TEST_CASE("camera_fsm - init resets a dirty machine") {
    Fsm f;
    f.state = State::kRecording;
    f.stopCondSince = 1234;
    f.wakeAttemptsUsed = 3;
    f.recordingActive = true;
    f.entryPending = true;
    init(f, true);
    CHECK(f.state == State::kIdle);
    CHECK(f.stopCondSince == 0);
    CHECK(f.wakeAttemptsUsed == 0);
    CHECK(f.recordingActive == false);
    CHECK(f.entryPending == false);
}

TEST_CASE("camera_fsm - UNPAIRED is inert under rpm/speed/connect churn") {
    Sim s(false);
    s.in.rpm = 5000;
    s.in.speedMph = 50.0f;
    for (int i = 0; i < 20; i++) {
        s.in.remoteConnected = (i % 2) == 0;
        s.in.ce82Subscribed = (i % 3) == 0;
        s.in.gpsFixValid = true;
        CHECK(s.tick(500) == Action::kNone);
        CHECK(s.f.state == State::kUnpaired);
    }
}

// ---------------------------------------------------------------------------
// IDLE — RPM debounce & hysteresis
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - RPM debounce: 1999 ms no action, 2000 ms fires") {
    Sim s;
    s.in.rpm = 1000;
    CHECK(s.tick(0) == Action::kNone);      // arm
    CHECK(s.tick(1999) == Action::kNone);   // 1999 ms held: not yet
    CHECK(s.f.state == State::kIdle);
    CHECK(s.tick(1) == Action::kStartWakeBurst);  // 2000 ms held: fire
    CHECK(s.f.state == State::kWaking);
    CHECK(s.f.wakeAttemptsUsed == 1);
}

TEST_CASE("camera_fsm - RPM debounce: dip below threshold resets the hold") {
    Sim s;
    s.in.rpm = 1000;
    CHECK(s.tick(0) == Action::kNone);      // arm at t0
    CHECK(s.tick(1500) == Action::kNone);   // still holding at t0+1500
    s.in.rpm = 499;                         // dip resets
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.rpmOnSince == 0);
    s.in.rpm = 1000;
    CHECK(s.tick(10) == Action::kNone);     // re-arm
    CHECK(s.tick(1999) == Action::kNone);   // 1999 ms since re-arm
    CHECK(s.f.state == State::kIdle);
    CHECK(s.tick(1) == Action::kStartWakeBurst);
}

TEST_CASE("camera_fsm - hysteresis: rpm at/below 500 never arms in IDLE") {
    Sim s;
    // 350-450 oscillation inside the hysteresis band never arms.
    for (int i = 0; i < 20; i++) {
        s.in.rpm = (i % 2) == 0 ? 350 : 450;
        CHECK(s.tick(500) == Action::kNone);
    }
    CHECK(s.f.state == State::kIdle);
    CHECK(s.f.rpmOnSince == 0);

    // Exactly 500 (== threshold, not >) never arms either.
    s.in.rpm = 500;
    for (int i = 0; i < 10; i++) CHECK(s.tick(500) == Action::kNone);
    CHECK(s.f.rpmOnSince == 0);

    // 501 arms and fires.
    s.in.rpm = 501;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.tick(kRpmOnDebounceMs) == Action::kStartWakeBurst);
}

TEST_CASE("camera_fsm - hysteresis: rpm 300/400 is NOT engine-off in RECORDING") {
    Sim s;
    s.toRecording();
    s.in.gpsFixValid = false;
    s.in.speedMph = 0.0f;

    // rpm 400 >= 300: engine not off, stop condition never arms.
    s.in.rpm = 400;
    auto acts = s.run(120000, 1000);
    CHECK(s.f.state == State::kRecording);
    CHECK(s.f.stopCondSince == 0);
    CHECK(countOf(acts, Action::kSendShutter) == 0);

    // rpm exactly 300 is still not off (off is rpm < 300).
    s.in.rpm = 300;
    acts = s.run(120000, 1000);
    CHECK(s.f.state == State::kRecording);
    CHECK(s.f.stopCondSince == 0);

    // rpm 299: engine off; with speed 0 the 60 s hold runs and stops.
    s.in.rpm = 299;
    CHECK(s.tick(10) == Action::kNone);  // arm
    CHECK(s.f.stopCondSince != 0);
    CHECK(s.tick(kStopRecordDelayMs) == Action::kSendShutter);
    CHECK(s.f.state == State::kCooldown);
}

// ---------------------------------------------------------------------------
// IDLE — stale link
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - stale remote link skips the wake to AWAIT READY") {
    Sim s;
    s.in.remoteConnected = true;
    s.in.rpm = 1000;
    CHECK(s.tick(0) == Action::kNone);
    // No wake advert: straight to AWAIT READY.
    CHECK(s.tick(kRpmOnDebounceMs) == Action::kNone);
    CHECK(s.f.state == State::kAwaitReady);

    // Then subscribe + fix records.
    s.in.ce82Subscribed = true;
    s.in.gpsFixValid = true;
    CHECK(s.tick(10) == Action::kSendShutter);
    CHECK(s.f.state == State::kRecording);
}

// ---------------------------------------------------------------------------
// WAKING
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - WAKING connect advances to AWAIT READY") {
    Sim s;
    s.toWaking();
    s.in.remoteConnected = true;
    CHECK(s.tick(100) == Action::kNone);  // advert stops automatically
    CHECK(s.f.state == State::kAwaitReady);
}

TEST_CASE("camera_fsm - WAKING retry ladder: 20 s x3 then back to IDLE") {
    Sim s;
    s.toWaking();  // attempt 1 window starts at T
    CHECK(s.tick(19999) == Action::kNone);             // T+19.999s: still beaconing
    CHECK(s.tick(1) == Action::kStartWakeBurst);        // T+20s: attempt 2
    CHECK(s.f.wakeAttemptsUsed == 2);
    CHECK(s.tick(20000) == Action::kStartWakeBurst);    // T+40s: attempt 3
    CHECK(s.f.wakeAttemptsUsed == 3);
    CHECK(s.tick(20000) == Action::kStopAdvertising);   // T+60s: give up
    CHECK(s.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - WAKING silent while beaconing") {
    Sim s;
    s.toWaking();
    auto acts = s.run(19000, 500);  // through most of attempt 1
    CHECK(acts.empty());
    CHECK(s.f.state == State::kWaking);
}

TEST_CASE("camera_fsm - WAKING rpm-gone for 2 s aborts back to IDLE") {
    Sim s;
    s.toWaking();
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);      // arms rpmGoneSince
    CHECK(s.tick(1999) == Action::kNone);    // held 1999 ms: not yet
    CHECK(s.f.state == State::kWaking);
    CHECK(s.tick(1) == Action::kStopAdvertising);  // held 2000 ms
    CHECK(s.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - WAKING rpm recovery resets the abort timer") {
    Sim s;
    s.toWaking();
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.tick(1500) == Action::kNone);
    s.in.rpm = 1000;                        // engine back
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.rpmGoneSince == 0);
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);     // re-arm
    CHECK(s.tick(1999) == Action::kNone);
    CHECK(s.f.state == State::kWaking);
}

TEST_CASE("camera_fsm - WAKING sessionEnd returns to IDLE") {
    Sim s;
    s.toWaking();
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kStopAdvertising);
    CHECK(s.f.state == State::kIdle);
}

// ---------------------------------------------------------------------------
// AWAIT READY
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - AWAIT READY: subscribe + fix starts recording") {
    Sim s;
    s.toAwaitReady();
    s.in.ce82Subscribed = true;
    s.in.gpsFixValid = true;
    CHECK(s.tick(10) == Action::kSendShutter);
    CHECK(s.f.state == State::kRecording);
    CHECK(s.f.recordingActive == true);
}

TEST_CASE("camera_fsm - AWAIT READY: subscribe + no fix records anyway at 30 s") {
    Sim s;
    s.toAwaitReady();  // awaitGpsSince = T
    s.in.ce82Subscribed = true;
    CHECK(s.in.gpsFixValid == false);
    CHECK(s.tick(kGpsLockTimeoutMs) == Action::kSendShutter);
    CHECK(s.f.state == State::kRecording);
}

TEST_CASE("camera_fsm - AWAIT READY: no fix before timeout stays put") {
    Sim s;
    s.toAwaitReady();
    s.in.ce82Subscribed = true;
    CHECK(s.tick(kGpsLockTimeoutMs - 1) == Action::kNone);
    CHECK(s.f.state == State::kAwaitReady);
}

TEST_CASE("camera_fsm - AWAIT READY: subscribe timeout drops and re-wakes") {
    Sim s;
    s.toAwaitReady();  // connectedSince = T; never subscribes
    CHECK(s.in.ce82Subscribed == false);
    CHECK(s.tick(kSubscribeTimeoutMs) == Action::kDisconnect);
    CHECK(s.f.state == State::kWaking);
    CHECK(s.f.wakeAttemptsUsed == 1);  // fresh cycle
}

TEST_CASE("camera_fsm - AWAIT READY: remote drop re-wakes") {
    Sim s;
    s.toAwaitReady();
    s.in.remoteConnected = false;
    CHECK(s.tick(10) == Action::kStartWakeBurst);
    CHECK(s.f.state == State::kWaking);
    CHECK(s.f.wakeAttemptsUsed == 1);
}

TEST_CASE("camera_fsm - AWAIT READY: sessionEnd rides into COOLDOWN") {
    Sim s;
    s.toAwaitReady();
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kNone);
    CHECK(s.f.state == State::kCooldown);
}

// ---------------------------------------------------------------------------
// recording-active toggle tracking
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - entering RECORDING sends exactly one shutter") {
    Sim s;
    s.toAwaitReady();
    s.in.ce82Subscribed = true;
    s.in.gpsFixValid = true;
    CHECK(s.tick(10) == Action::kSendShutter);  // false -> true, toggle ON
    CHECK(s.f.recordingActive == true);
    // No further shutter while just recording.
    s.in.speedMph = 30.0f;
    s.in.rpm = 5000;
    auto acts = s.run(5000, 100);
    CHECK(countOf(acts, Action::kSendShutter) == 0);
}

TEST_CASE("camera_fsm - reconnect mid-session does NOT re-toggle shutter") {
    Sim s;
    s.toRecording();  // recordingActive == true
    // Remote drops: re-wake, recordingActive stays true.
    s.in.remoteConnected = false;
    CHECK(s.tick(10) == Action::kStartWakeBurst);
    CHECK(s.f.state == State::kWaking);
    CHECK(s.f.recordingActive == true);
    // Camera reconnects → AWAIT READY.
    s.in.remoteConnected = true;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kAwaitReady);
    // Ready again: recordingActive already true → NO shutter re-sent.
    s.in.ce82Subscribed = true;
    s.in.gpsFixValid = true;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kRecording);
    CHECK(s.f.recordingActive == true);
}

TEST_CASE("camera_fsm - clean stop toggles shutter OFF") {
    Sim s;
    s.toRecording();
    s.in.speedMph = 0.0f;
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);  // arm the stop hold
    CHECK(s.tick(kStopRecordDelayMs) == Action::kSendShutter);  // true -> false
    CHECK(s.f.state == State::kCooldown);
    CHECK(s.f.recordingActive == false);
}

// ---------------------------------------------------------------------------
// RECORDING — stop semantics
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - grid idle (speed 0, rpm 3000) never stops recording") {
    Sim s;
    s.toRecording();
    s.in.speedMph = 0.0f;
    s.in.rpm = 3000;
    auto acts = s.run(600000, 1000);  // 10 min
    CHECK(s.f.state == State::kRecording);
    CHECK(countOf(acts, Action::kSendShutter) == 0);
}

TEST_CASE("camera_fsm - coasting stall (30 mph, rpm 0) never stops recording") {
    Sim s;
    s.toRecording();
    s.in.speedMph = 30.0f;
    s.in.rpm = 0;
    auto acts = s.run(600000, 1000);
    CHECK(s.f.state == State::kRecording);
    CHECK(countOf(acts, Action::kSendShutter) == 0);
}

TEST_CASE("camera_fsm - stop hold: rpm blip at 59.9 s resets, clean 60 s stops") {
    Sim s;
    s.toRecording();
    s.in.gpsFixValid = false;
    s.in.speedMph = 0.0f;
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);   // arm stop hold at T
    s.tick(59890);                        // T+59.9s: still holding
    CHECK(s.f.state == State::kRecording);
    s.in.rpm = 3000;                      // blip
    s.tick(10);
    CHECK(s.f.stopCondSince == 0);        // hold reset
    s.in.rpm = 0;
    s.tick(10);                           // re-arm at T2
    CHECK(s.f.stopCondSince != 0);
    s.tick(59999);                        // T2+59.999s
    CHECK(s.f.state == State::kRecording);
    CHECK(s.tick(1) == Action::kSendShutter);  // T2+60s
    CHECK(s.f.state == State::kCooldown);
}

// ---------------------------------------------------------------------------
// Manual session end
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - manual end in RECORDING stops immediately") {
    Sim s;
    s.toRecording();
    // Speed and rpm high: the hold timer is bypassed entirely.
    s.in.speedMph = 60.0f;
    s.in.rpm = 8000;
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kSendShutter);
    CHECK(s.f.state == State::kCooldown);
    CHECK(s.f.recordingActive == false);
}

TEST_CASE("camera_fsm - manual end in COOLDOWN powers off immediately") {
    Sim s;
    s.toCooldown();
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kSendPowerOff);
    CHECK(s.f.state == State::kPoweringOff);
}

TEST_CASE("camera_fsm - manual end in AWAIT READY rides into COOLDOWN") {
    Sim s;
    s.toAwaitReady();
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kNone);
    CHECK(s.f.state == State::kCooldown);
}

TEST_CASE("camera_fsm - manual end in WAKING returns to IDLE") {
    Sim s;
    s.toWaking();
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kStopAdvertising);
    CHECK(s.f.state == State::kIdle);
}

// ---------------------------------------------------------------------------
// COOLDOWN
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - COOLDOWN powers off after 180 s") {
    Sim s;
    s.toCooldown();  // cooldownSince = T; remote still connected
    CHECK(s.tick(kPowerOffDelayMs) == Action::kSendPowerOff);
    CHECK(s.f.state == State::kPoweringOff);
}

TEST_CASE("camera_fsm - COOLDOWN remote drop means camera turned off") {
    Sim s;
    s.toCooldown();
    s.in.remoteConnected = false;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - COOLDOWN ignores motion return") {
    Sim s;
    s.toCooldown();
    s.in.rpm = 5000;
    s.in.speedMph = 40.0f;
    auto acts = s.run(10000, 500);
    CHECK(s.f.state == State::kCooldown);  // kAutoResumeFromCooldown == false
    CHECK(acts.empty());                   // nothing happens
}

// ---------------------------------------------------------------------------
// POWERING OFF
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - POWERING OFF: remote drop returns to IDLE") {
    Sim s;
    s.toPoweringOff();
    s.in.remoteConnected = false;
    CHECK(s.tick(1000) == Action::kNone);
    CHECK(s.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - POWERING OFF: 5 s linger forces the disconnect") {
    Sim s;
    s.toPoweringOff();  // powerOffSentAt = T; remote still up
    CHECK(s.tick(4999) == Action::kNone);
    CHECK(s.f.state == State::kPoweringOff);
    CHECK(s.tick(1) == Action::kDisconnect);
    CHECK(s.f.state == State::kIdle);
}

// ---------------------------------------------------------------------------
// PAIRING
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - pairRequested honored from UNPAIRED and IDLE") {
    Sim unpaired(false);
    CHECK(unpaired.pulse(&Inputs::pairRequested, 10) ==
          Action::kStartConnectableAdvertising);
    CHECK(unpaired.f.state == State::kPairing);
    CHECK(unpaired.f.pairingReturnState == State::kUnpaired);

    Sim idle(true);
    CHECK(idle.pulse(&Inputs::pairRequested, 10) ==
          Action::kStartConnectableAdvertising);
    CHECK(idle.f.state == State::kPairing);
    CHECK(idle.f.pairingReturnState == State::kIdle);
}

TEST_CASE("camera_fsm - pairRequested ignored from RECORDING and COOLDOWN") {
    Sim rec;
    rec.toRecording();
    rec.pulse(&Inputs::pairRequested, 10);
    CHECK(rec.f.state == State::kRecording);

    Sim cool;
    cool.toCooldown();
    cool.pulse(&Inputs::pairRequested, 10);
    CHECK(cool.f.state == State::kCooldown);
}

TEST_CASE("camera_fsm - PAIRING entry action emitted exactly once") {
    Sim s;
    s.toPairing();  // consumed kStartConnectableAdvertising already
    CHECK(s.tick(100) == Action::kNone);
    CHECK(s.tick(100) == Action::kNone);
    CHECK(s.f.state == State::kPairing);
}

TEST_CASE("camera_fsm - PAIRING capture binds the serial and disconnects") {
    Sim s(false);
    s.toPairing();
    CHECK(s.pulse(&Inputs::pairSerialCaptured, 100) == Action::kDisconnect);
    CHECK(s.f.state == State::kIdle);
    CHECK(s.f.serialPresent == true);
}

TEST_CASE("camera_fsm - PAIRING cancel returns to the origin state") {
    Sim fromUnpaired(false);
    fromUnpaired.toPairing();
    CHECK(fromUnpaired.pulse(&Inputs::pairCancelRequested, 100) ==
          Action::kStopAdvertising);
    CHECK(fromUnpaired.f.state == State::kUnpaired);

    Sim fromIdle(true);
    fromIdle.toPairing();
    CHECK(fromIdle.pulse(&Inputs::pairCancelRequested, 100) ==
          Action::kStopAdvertising);
    CHECK(fromIdle.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - PAIRING times out after 120 s back to the origin") {
    Sim fromUnpaired(false);
    fromUnpaired.toPairing();  // pairingSince ~= T
    CHECK(fromUnpaired.tick(kPairingTimeoutMs) == Action::kStopAdvertising);
    CHECK(fromUnpaired.f.state == State::kUnpaired);

    Sim fromIdle(true);
    fromIdle.toPairing();
    CHECK(fromIdle.tick(kPairingTimeoutMs) == Action::kStopAdvertising);
    CHECK(fromIdle.f.state == State::kIdle);
}

// ---------------------------------------------------------------------------
// forceIdle from every state
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - forceIdle from every state lands home with teardown") {
    SUBCASE("from UNPAIRED (no serial)") {
        Sim s(false);
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kNone);
        CHECK(s.f.state == State::kUnpaired);
    }
    SUBCASE("from IDLE") {
        Sim s;
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kNone);
        CHECK(s.f.state == State::kIdle);
    }
    SUBCASE("from WAKING") {
        Sim s;
        s.toWaking();
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kStopAdvertising);
        CHECK(s.f.state == State::kIdle);
    }
    SUBCASE("from AWAIT READY") {
        Sim s;
        s.toAwaitReady();
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kNone);
        CHECK(s.f.state == State::kIdle);
    }
    SUBCASE("from RECORDING") {
        Sim s;
        s.toRecording();
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kNone);
        CHECK(s.f.state == State::kIdle);
        CHECK(s.f.recordingActive == false);
    }
    SUBCASE("from COOLDOWN") {
        Sim s;
        s.toCooldown();
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kNone);
        CHECK(s.f.state == State::kIdle);
    }
    SUBCASE("from POWERING OFF") {
        Sim s;
        s.toPoweringOff();
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kNone);
        CHECK(s.f.state == State::kIdle);
    }
    SUBCASE("from PAIRING with serial") {
        Sim s;
        s.toPairing();
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kStopAdvertising);
        CHECK(s.f.state == State::kIdle);
    }
    SUBCASE("from PAIRING without serial lands UNPAIRED") {
        Sim s(false);
        s.toPairing();
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kStopAdvertising);
        CHECK(s.f.state == State::kUnpaired);
    }
}

TEST_CASE("camera_fsm - forceIdle clears all timers and recording belief") {
    Sim s;
    s.toRecording();
    s.in.speedMph = 0.0f;
    s.in.rpm = 0;
    s.tick(10);  // arm the stop hold
    CHECK(s.f.stopCondSince != 0);
    CHECK(s.f.recordingActive == true);
    s.pulse(&Inputs::forceIdleRequested, 10);
    CHECK(s.f.stopCondSince == 0);
    CHECK(s.f.rpmOnSince == 0);
    CHECK(s.f.wakeAttemptsUsed == 0);
    CHECK(s.f.recordingActive == false);
    CHECK(s.f.entryPending == false);
}

// ---------------------------------------------------------------------------
// unpair
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - unpair from IDLE clears the serial and drops links") {
    Sim s;
    // kDisconnect, not kNone: a remote link surviving into kIdle must not stay
    // connected to an unpaired (or subsequently re-paired) device.
    CHECK(s.pulse(&Inputs::unpairRequested, 10) == Action::kDisconnect);
    CHECK(s.f.state == State::kUnpaired);
    CHECK(s.f.serialPresent == false);
}

TEST_CASE("camera_fsm - unpair ignored from RECORDING") {
    Sim s;
    s.toRecording();
    s.pulse(&Inputs::unpairRequested, 10);
    CHECK(s.f.state == State::kRecording);
    CHECK(s.f.serialPresent == true);
}

// ---------------------------------------------------------------------------
// millis() wraparound
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - RPM debounce survives millis wraparound") {
    Sim s(true, 0xFFFFFF00u);  // 256 ms before the wrap
    s.in.rpm = 1000;
    CHECK(s.tick(0) == Action::kNone);      // arm at 0xFFFFFF00
    CHECK(s.tick(1999) == Action::kNone);   // now = 1743 (wrapped), held 1999 ms
    CHECK(s.in.nowMs < 0x10000u);           // clock really did wrap
    CHECK(s.f.state == State::kIdle);
    CHECK(s.tick(1) == Action::kStartWakeBurst);  // held 2000 ms across the wrap
    CHECK(s.f.state == State::kWaking);
}

TEST_CASE("camera_fsm - 60 s stop hold survives millis wraparound") {
    Sim s(true, 0xFFFFF000u);  // ~4 s before the wrap
    s.toRecording();           // consumes some of that headroom
    s.in.gpsFixValid = false;
    s.in.speedMph = 0.0f;
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);  // arm the hold pre-wrap
    const uint32_t armedAt = s.f.stopCondSince;
    CHECK(armedAt > 0xFFFFF000u);        // armed before the wrap
    s.tick(59999);                       // 59.999 s held
    CHECK(s.in.nowMs < 0x10000u);        // clock wrapped during the hold
    CHECK(s.f.state == State::kRecording);
    CHECK(s.tick(1) == Action::kSendShutter);  // fires exactly at 60 s
    CHECK(s.f.state == State::kCooldown);
}

// ---------------------------------------------------------------------------
// One-shot event consumption
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - sessionEnd is consumed in a single step") {
    Sim s;
    s.toRecording();
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kSendShutter);
    CHECK(s.f.state == State::kCooldown);
    // Flag cleared by the caller: the next step must NOT act on it again (a
    // stale sessionEnd in COOLDOWN would emit kSendPowerOff).
    CHECK(s.tick(10) != Action::kSendPowerOff);
    CHECK(s.f.state == State::kCooldown);
}

// ---------------------------------------------------------------------------
// stateName
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - stateName labels every state and never returns null") {
    CHECK(strcmp(stateName(State::kUnpaired), "UNPAIRED") == 0);
    CHECK(strcmp(stateName(State::kIdle), "IDLE") == 0);
    CHECK(strcmp(stateName(State::kWaking), "WAKING") == 0);
    CHECK(strcmp(stateName(State::kAwaitReady), "READY") == 0);
    CHECK(strcmp(stateName(State::kRecording), "RECORDING") == 0);
    CHECK(strcmp(stateName(State::kCooldown), "COOLDOWN") == 0);
    CHECK(strcmp(stateName(State::kPoweringOff), "PWR OFF") == 0);
    CHECK(strcmp(stateName(State::kPairing), "PAIRING") == 0);
    // Out-of-range value (corrupted state) still yields a printable label.
    CHECK(strcmp(stateName(static_cast<State>(200)), "?") == 0);
}
