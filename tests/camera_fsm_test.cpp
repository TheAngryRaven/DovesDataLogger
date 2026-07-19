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
        // Above BOTH the wake threshold (500) and the record threshold (1500)
        // so the shared drivers can reach RECORDING; tests that need the band
        // between them set their own rpm.
        in.rpm = 3000;
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

    // Subscribed → WATCHING (the hub state; not yet recording).
    void toWatching() {
        toAwaitReady();
        in.ce82Subscribed = true;
        REQUIRE(tick(10) == Action::kNone);
        REQUIRE(f.state == State::kWatching);
    }

    // Watching + RPM held kRecordStartDelayMs → one shutter → RECORDING.
    void toRecording() {
        toWatching();
        REQUIRE(tick(kRecordStartDelayMs) == Action::kSendShutter);
        REQUIRE(f.state == State::kRecording);
        REQUIRE(f.recordingActive == true);
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
    f.recordArmSince = 999;
    f.recordingActive = true;
    f.entryPending = true;
    init(f, true);
    CHECK(f.state == State::kIdle);
    CHECK(f.stopCondSince == 0);
    CHECK(f.wakeAttemptsUsed == 0);
    CHECK(f.recordArmSince == 0);
    CHECK(f.recordingActive == false);
    CHECK(f.entryPending == false);
}

TEST_CASE("camera_fsm - UNPAIRED is inert under rpm/connect churn") {
    Sim s(false);
    s.in.rpm = 5000;
    for (int i = 0; i < 20; i++) {
        s.in.remoteConnected = (i % 2) == 0;
        s.in.ce82Subscribed = (i % 3) == 0;
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
    // Wake-band RPM (above ON, below kRecordRpmThreshold) wakes the camera
    // but must NOT arm the record clock — pull-start cranking blips live here.
    CHECK(s.f.recordArmSince == 0);
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
    for (int i = 0; i < 20; i++) {
        s.in.rpm = (i % 2) == 0 ? 350 : 450;
        CHECK(s.tick(500) == Action::kNone);
    }
    CHECK(s.f.state == State::kIdle);
    CHECK(s.f.rpmOnSince == 0);

    s.in.rpm = 500;  // == threshold, not > : never arms
    for (int i = 0; i < 10; i++) CHECK(s.tick(500) == Action::kNone);
    CHECK(s.f.rpmOnSince == 0);

    s.in.rpm = 501;  // 501 arms and fires
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.tick(kRpmOnDebounceMs) == Action::kStartWakeBurst);
}

// ---------------------------------------------------------------------------
// IDLE — stale link
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - stale remote link skips the wake to AWAIT READY then WATCHING") {
    Sim s;
    s.in.remoteConnected = true;
    s.in.rpm = 3000;  // record band, so the flow can reach RECORDING below
    CHECK(s.tick(0) == Action::kNone);
    CHECK(s.tick(kRpmOnDebounceMs) == Action::kNone);  // no wake advert
    CHECK(s.f.state == State::kAwaitReady);

    s.in.ce82Subscribed = true;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kWatching);
    CHECK(s.tick(kRecordStartDelayMs) == Action::kSendShutter);
    CHECK(s.f.state == State::kRecording);
}

// ---------------------------------------------------------------------------
// WAKING
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - WAKING connect advances to AWAIT READY") {
    Sim s;
    s.toWaking();
    s.in.remoteConnected = true;
    CHECK(s.tick(100) == Action::kNone);
    CHECK(s.f.state == State::kAwaitReady);
}

TEST_CASE("camera_fsm - WAKING retry ladder: 20 s x3 then back to IDLE") {
    Sim s;
    s.toWaking();
    CHECK(s.tick(19999) == Action::kNone);
    CHECK(s.tick(1) == Action::kStartWakeBurst);        // attempt 2
    CHECK(s.f.wakeAttemptsUsed == 2);
    CHECK(s.tick(20000) == Action::kStartWakeBurst);    // attempt 3
    CHECK(s.f.wakeAttemptsUsed == 3);
    CHECK(s.tick(20000) == Action::kStopAdvertising);   // give up
    CHECK(s.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - WAKING rpm-gone for 2 s aborts back to IDLE") {
    Sim s;
    s.toWaking();
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.tick(1999) == Action::kNone);
    CHECK(s.f.state == State::kWaking);
    CHECK(s.tick(1) == Action::kStopAdvertising);
    CHECK(s.f.state == State::kIdle);
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

TEST_CASE("camera_fsm - AWAIT READY: subscribe moves to WATCHING") {
    Sim s;
    s.toAwaitReady();
    s.in.ce82Subscribed = true;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kWatching);
    CHECK(s.f.subscribeAttemptsUsed == 0);
}

TEST_CASE("camera_fsm - AWAIT READY: subscribe timeout re-wakes, bounded to IDLE") {
    Sim s;
    s.toAwaitReady();
    CHECK(s.in.ce82Subscribed == false);
    // Each cycle: timeout -> kDisconnect -> kWaking, then (link still up in the
    // sim) bounce back to AWAIT READY. subscribeAttemptsUsed bounds the loop.
    for (uint8_t cycle = 1; cycle <= kSubscribeRetries; ++cycle) {
        CHECK(s.tick(kSubscribeTimeoutMs) == Action::kDisconnect);
        CHECK(s.f.state == State::kWaking);
        CHECK(s.f.subscribeAttemptsUsed == cycle);
        CHECK(s.tick(10) == Action::kNone);  // remote still up -> AWAIT READY
        CHECK(s.f.state == State::kAwaitReady);
    }
    CHECK(s.tick(kSubscribeTimeoutMs) == Action::kDisconnect);  // budget exceeded
    CHECK(s.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - AWAIT READY: remote drop re-wakes") {
    Sim s;
    s.toAwaitReady();
    s.in.remoteConnected = false;
    CHECK(s.tick(10) == Action::kStartWakeBurst);
    CHECK(s.f.state == State::kWaking);
    CHECK(s.f.wakeAttemptsUsed == 1);
}

TEST_CASE("camera_fsm - AWAIT READY: sessionEnd rides into WATCHING") {
    Sim s;
    s.toAwaitReady();
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kNone);
    CHECK(s.f.state == State::kWatching);
}

// ---------------------------------------------------------------------------
// WATCHING — record start (5 s RPM), re-record, no power-off
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - WATCHING records one shutter after 5 s of RPM") {
    Sim s;
    s.toWatching();
    // The record-start clock is armed at the wake, so re-arm from a known point
    // (drop then raise RPM) to measure the 5 s precisely.
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.recordArmSince == 0);
    s.in.rpm = 3000;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.recordArmSince != 0);
    CHECK(s.tick(kRecordStartDelayMs - 1) == Action::kNone);   // not yet
    CHECK(s.f.state == State::kWatching);
    CHECK(s.tick(1) == Action::kSendShutter);                  // fire at 5 s
    CHECK(s.f.state == State::kRecording);
    CHECK(s.f.recordingActive == true);
    // No further shutter while just recording.
    s.in.rpm = 5000;
    auto acts = s.run(5000, 100);
    CHECK(countOf(acts, Action::kSendShutter) == 0);
}

TEST_CASE("camera_fsm - WATCHING: RPM dropping during the 5 s resets the arm") {
    Sim s;
    s.toWatching();
    CHECK(s.tick(3000) == Action::kNone);   // 3 s into the arm window
    s.in.rpm = 0;                           // engine off — disarm
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.recordArmSince == 0);
    CHECK(s.f.state == State::kWatching);
    s.in.rpm = 3000;                        // back on — re-arm
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.recordArmSince != 0);
    CHECK(s.tick(kRecordStartDelayMs - 1) == Action::kNone);
    CHECK(s.tick(1) == Action::kSendShutter);   // 5 s from the re-arm
    CHECK(s.f.state == State::kRecording);
}

TEST_CASE("camera_fsm - cranking-band RPM wakes but NEVER records (pull-start)") {
    // Field incident (2026-07-19): pull-starting registers real ignition
    // pulses well above the wake threshold, which woke the camera AND
    // started a recording before the engine was actually running. RPM in
    // the band (kRpmOnThreshold, kRecordRpmThreshold) may drive the whole
    // wake flow but must never fire the shutter, no matter how long it holds.
    Sim s;
    s.toWatching();
    // Force a fresh arm window, then sit in the cranking band indefinitely.
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);
    s.in.rpm = 1000;  // above ON (500), below record (1500)
    auto acts = s.run(60000, 250);  // a full minute of "cranking"
    CHECK(countOf(acts, Action::kSendShutter) == 0);
    CHECK(s.f.state == State::kWatching);
    CHECK(s.f.recordArmSince == 0);
}

TEST_CASE("camera_fsm - record threshold boundary: 1499 never arms, 1500 does") {
    Sim s;
    s.toWatching();
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);
    s.in.rpm = kRecordRpmThreshold - 1;
    auto acts = s.run(3 * kRecordStartDelayMs, 100);
    CHECK(countOf(acts, Action::kSendShutter) == 0);
    CHECK(s.f.recordArmSince == 0);
    s.in.rpm = kRecordRpmThreshold;  // at threshold — arms
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.recordArmSince != 0);
    CHECK(s.tick(kRecordStartDelayMs) == Action::kSendShutter);
}

TEST_CASE("camera_fsm - dip below record threshold restarts the 5 s clock") {
    // "1500+ RPM for 5 seconds" is strict: sustained means sustained. A dip
    // to idle (above OFF, below record) must restart the record-start clock,
    // not coast through a hysteresis band.
    Sim s;
    s.toWatching();
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);
    s.in.rpm = 3000;
    CHECK(s.tick(kRecordStartDelayMs - 500) == Action::kNone);  // 4.5 s up
    s.in.rpm = 1000;  // sag below the record band (engine still "on")
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.recordArmSince == 0);
    s.in.rpm = 3000;
    CHECK(s.tick(10) == Action::kNone);  // re-arms — fresh clock from here
    CHECK(s.tick(kRecordStartDelayMs - 1) == Action::kNone);
    CHECK(s.tick(1) == Action::kSendShutter);
    CHECK(s.f.state == State::kRecording);
}

TEST_CASE("camera_fsm - WATCHING never powers off (only sleep does)") {
    Sim s;
    s.toWatching();
    s.in.rpm = 0;  // engine off, sitting in watching
    auto acts = s.run(600000, 1000);  // 10 min
    CHECK(s.f.state == State::kWatching);
    CHECK(countOf(acts, Action::kSendPowerOff) == 0);
    CHECK(countOf(acts, Action::kSendShutter) == 0);
}

TEST_CASE("camera_fsm - WATCHING: remote drop returns to IDLE") {
    Sim s;
    s.toWatching();
    s.in.remoteConnected = false;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kIdle);
}

// ---------------------------------------------------------------------------
// RECORDING — engine-off stop (30 s, RPM only)
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - RECORDING stops after 30 s engine-off, one shutter, to WATCHING") {
    Sim s;
    s.toRecording();
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);   // arm the stop hold
    CHECK(s.f.stopCondSince != 0);
    CHECK(s.tick(kStopRecordDelayMs) == Action::kSendShutter);  // toggle OFF
    CHECK(s.f.state == State::kWatching);
    CHECK(s.f.recordingActive == false);
}

TEST_CASE("camera_fsm - RECORDING: engine on (rpm 3000) never stops, speed irrelevant") {
    Sim s;
    s.toRecording();
    s.in.rpm = 3000;  // stationary grid idle would be speed 0 — but we ignore speed
    auto acts = s.run(600000, 1000);
    CHECK(s.f.state == State::kRecording);
    CHECK(countOf(acts, Action::kSendShutter) == 0);
}

TEST_CASE("camera_fsm - RECORDING: rpm blip at 29.9 s resets, clean 30 s stops") {
    Sim s;
    s.toRecording();
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);  // arm at T
    s.tick(29890);                       // T+29.9 s
    CHECK(s.f.state == State::kRecording);
    s.in.rpm = 3000;                     // blip
    s.tick(10);
    CHECK(s.f.stopCondSince == 0);       // reset
    s.in.rpm = 0;
    s.tick(10);                          // re-arm at T2
    CHECK(s.f.stopCondSince != 0);
    s.tick(kStopRecordDelayMs - 1);
    CHECK(s.f.state == State::kRecording);
    CHECK(s.tick(1) == Action::kSendShutter);  // T2+30 s
    CHECK(s.f.state == State::kWatching);
}

TEST_CASE("camera_fsm - manual end in RECORDING stops immediately to WATCHING") {
    Sim s;
    s.toRecording();
    s.in.rpm = 8000;  // engine high: the 30 s hold is bypassed
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kSendShutter);
    CHECK(s.f.state == State::kWatching);
    CHECK(s.f.recordingActive == false);
}

// ---------------------------------------------------------------------------
// Stall recovery: stop -> WATCHING -> RPM returns -> record again
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - stall recovery re-records from WATCHING on RPM return") {
    Sim s;
    s.toRecording();
    // Engine off 30 s -> stop -> WATCHING.
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.tick(kStopRecordDelayMs) == Action::kSendShutter);
    CHECK(s.f.state == State::kWatching);
    CHECK(s.f.recordingActive == false);
    // Sit a while, still off — no re-record, no power-off.
    auto acts = s.run(20000, 500);
    CHECK(acts.empty());
    CHECK(s.f.state == State::kWatching);
    // Engine restarts: re-arm, and after 5 s record again (one shutter).
    s.in.rpm = 3000;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.recordArmSince != 0);
    CHECK(s.tick(kRecordStartDelayMs) == Action::kSendShutter);
    CHECK(s.f.state == State::kRecording);
    CHECK(s.f.recordingActive == true);
}

// ---------------------------------------------------------------------------
// Observed-record-state reconcile (0x10 timer)
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - WATCHING adopts an already-recording camera without a shutter") {
    Sim s;
    s.toWatching();
    s.in.recordObserved = RecordObs::kRecording;  // camera already rolling
    CHECK(s.tick(kRecordStartDelayMs) == Action::kNone);  // adopt, no toggle
    CHECK(s.f.state == State::kRecording);
    CHECK(s.f.recordingActive == true);
}

TEST_CASE("camera_fsm - WATCHING with camera IDLE observed sends the start shutter") {
    Sim s;
    s.toWatching();
    s.in.recordObserved = RecordObs::kIdle;
    CHECK(s.tick(kRecordStartDelayMs) == Action::kSendShutter);
    CHECK(s.f.recordingActive == true);
}

TEST_CASE("camera_fsm - reconnect mid-record does NOT blind-toggle; adopts on observation") {
    Sim s;
    s.toRecording();                 // recordingActive == true
    s.in.remoteConnected = false;    // link drops
    CHECK(s.tick(10) == Action::kStartWakeBurst);
    CHECK(s.f.state == State::kWaking);
    CHECK(s.f.recordingActive == true);      // preserved
    s.in.remoteConnected = true;
    CHECK(s.tick(10) == Action::kNone);      // AWAIT READY
    s.in.ce82Subscribed = true;
    CHECK(s.tick(10) == Action::kNone);      // WATCHING
    s.in.recordObserved = RecordObs::kRecording;  // camera still rolling
    CHECK(s.tick(kRecordStartDelayMs) == Action::kNone);  // adopt, NO shutter
    CHECK(s.f.state == State::kRecording);
    CHECK(s.f.recordingActive == true);
}

TEST_CASE("camera_fsm - RECORDING re-asserts the shutter once if the camera reports idle") {
    Sim s;
    s.toRecording();
    s.in.rpm = 5000;  // no stop condition
    s.in.recordObserved = RecordObs::kIdle;   // start never took
    CHECK(s.tick(10) == Action::kNone);       // arm the confirm timer
    CHECK(s.f.recordIdleSince != 0);
    CHECK(s.tick(kRecordConfirmMs) == Action::kSendShutter);  // re-assert
    CHECK(s.f.recordRetryUsed == true);
    auto acts = s.run(10000, 500);            // no second retry while idle
    CHECK(countOf(acts, Action::kSendShutter) == 0);
    s.in.recordObserved = RecordObs::kRecording;  // confirmed -> latch re-armed
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.recordRetryUsed == false);
}

TEST_CASE("camera_fsm - RECORDING never retries without a fresh observation") {
    Sim s;
    s.toRecording();
    s.in.rpm = 5000;
    s.in.recordObserved = RecordObs::kUnknown;
    auto acts = s.run(30000, 500);
    CHECK(countOf(acts, Action::kSendShutter) == 0);
    CHECK(s.f.recordIdleSince == 0);
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

TEST_CASE("camera_fsm - pairRequested ignored from RECORDING and WATCHING") {
    Sim rec;
    rec.toRecording();
    rec.pulse(&Inputs::pairRequested, 10);
    CHECK(rec.f.state == State::kRecording);

    Sim watch;
    watch.toWatching();
    watch.pulse(&Inputs::pairRequested, 10);
    CHECK(watch.f.state == State::kWatching);
}

TEST_CASE("camera_fsm - PAIRING capture binds the serial and disconnects") {
    Sim s(false);
    s.toPairing();
    CHECK(s.pulse(&Inputs::pairSerialCaptured, 100) == Action::kDisconnect);
    CHECK(s.f.state == State::kIdle);
    CHECK(s.f.serialPresent == true);
}

TEST_CASE("camera_fsm - PAIRING times out after 120 s back to the origin") {
    Sim fromIdle(true);
    fromIdle.toPairing();
    CHECK(fromIdle.tick(kPairingTimeoutMs) == Action::kStopAdvertising);
    CHECK(fromIdle.f.state == State::kIdle);
}

// ---------------------------------------------------------------------------
// forceIdle from every state
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - forceIdle from every state lands home with teardown") {
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
    SUBCASE("from WATCHING") {
        Sim s;
        s.toWatching();
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
    s.in.rpm = 0;
    s.tick(10);  // arm the stop hold
    CHECK(s.f.stopCondSince != 0);
    CHECK(s.f.recordingActive == true);
    s.pulse(&Inputs::forceIdleRequested, 10);
    CHECK(s.f.stopCondSince == 0);
    CHECK(s.f.rpmOnSince == 0);
    CHECK(s.f.recordArmSince == 0);
    CHECK(s.f.wakeAttemptsUsed == 0);
    CHECK(s.f.recordingActive == false);
    CHECK(s.f.entryPending == false);
}

// ---------------------------------------------------------------------------
// unpair
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - unpair from IDLE clears the serial and drops links") {
    Sim s;
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
    CHECK(s.tick(1999) == Action::kNone);   // wrapped, held 1999 ms
    CHECK(s.in.nowMs < 0x10000u);           // clock really did wrap
    CHECK(s.f.state == State::kIdle);
    CHECK(s.tick(1) == Action::kStartWakeBurst);
    CHECK(s.f.state == State::kWaking);
}

TEST_CASE("camera_fsm - 30 s stop hold survives millis wraparound") {
    Sim s(true, 0xFFFF8000u);  // positioned so the 30 s hold crosses the wrap
    s.toRecording();           // consumes some headroom
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);  // arm pre-wrap
    const uint32_t armedAt = s.f.stopCondSince;
    CHECK(armedAt > 0xFFFF0000u);
    s.tick(kStopRecordDelayMs - 1);
    CHECK(s.in.nowMs < 0x10000u);        // clock wrapped during the hold
    CHECK(s.f.state == State::kRecording);
    CHECK(s.tick(1) == Action::kSendShutter);  // fires exactly at 30 s
    CHECK(s.f.state == State::kWatching);
}

// ---------------------------------------------------------------------------
// One-shot event consumption
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - sessionEnd is consumed in a single step") {
    Sim s;
    s.toRecording();
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kSendShutter);
    CHECK(s.f.state == State::kWatching);
    // Flag cleared by the caller: the next step must NOT act on it again.
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kWatching);
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
    CHECK(strcmp(stateName(State::kWatching), "WATCHING") == 0);
    CHECK(strcmp(stateName(State::kPairing), "PAIRING") == 0);
    CHECK(strcmp(stateName(static_cast<State>(200)), "?") == 0);
}
