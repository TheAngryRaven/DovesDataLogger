#include "doctest.h"
#include "camera_fsm.h"

#include <algorithm>
#include <cstring>
#include <vector>

using namespace camera_fsm;

namespace {

// Small driver: holds an Inputs snapshot, advances a fake millis clock,
// and collects the actions step() emits.
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
        // Entry step 2: the be80 scanner comes up alongside the wake beacon.
        REQUIRE(tick(0) == Action::kStartControlConnect);
    }

    void toConnecting() {
        // Real path: the concurrent WAKING scanner spots the camera's be80.
        // entryPending stays false, so CONNECTING does not re-scan. The
        // connectable remote advert stays UP (kNone) — the woken camera
        // reconnects to it.
        toWaking();
        in.cameraAdvertSeen = true;
        REQUIRE(tick(10) == Action::kNone);
        in.cameraAdvertSeen = false;
        REQUIRE(f.state == State::kConnecting);
    }

    // Alternate entry: reach CONNECTING via a stale remote link (camera
    // already on our ce80), which arms the CONNECTING entry action
    // (kStartControlConnect). Used by the CONNECTING-behavior tests.
    void toConnectingViaRemote() {
        in.remoteConnected = true;
        in.rpm = 1000;
        REQUIRE(tick(0) == Action::kNone);
        REQUIRE(tick(kRpmOnDebounceMs) == Action::kNone);  // enterConnectingFresh
        REQUIRE(f.state == State::kConnecting);
    }

    void toAwaitGps() {
        toConnecting();  // scanner already running; no CONNECTING re-scan
        in.controlConnected = true;
        REQUIRE(tick(10) == Action::kNone);
        REQUIRE(f.state == State::kAwaitGps);
    }

    void toRecording() {
        toAwaitGps();
        in.gpsFixValid = true;
        REQUIRE(tick(10) == Action::kSendStartVideo);
        REQUIRE(f.state == State::kRecording);
    }

    void toCooldown() {
        toRecording();
        REQUIRE(pulse(&Inputs::sessionEndRequested, 10) == Action::kSendStopVideo);
        REQUIRE(f.state == State::kCooldown);
    }

    void toPoweringOff() {
        toCooldown();
        REQUIRE(pulse(&Inputs::sessionEndRequested, 10) == Action::kSendPowerOff);
        REQUIRE(f.state == State::kPoweringOff);
    }

    void toPairing() {
        REQUIRE(pulse(&Inputs::pairRequested, 10) == Action::kStartConnectableAdvertising);
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
    f.entryPending = true;
    init(f, true);
    CHECK(f.state == State::kIdle);
    CHECK(f.stopCondSince == 0);
    CHECK(f.wakeAttemptsUsed == 0);
    CHECK(f.entryPending == false);
}

TEST_CASE("camera_fsm - UNPAIRED is inert under rpm/speed/connect churn") {
    Sim s(false);
    s.in.rpm = 5000;
    s.in.speedMph = 50.0f;
    for (int i = 0; i < 20; i++) {
        s.in.controlConnected = (i % 2) == 0;
        s.in.remoteConnected = (i % 3) == 0;
        s.in.cameraAdvertSeen = (i % 4) == 0;
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
    // Entry step 2 brings up the be80 scanner concurrently with the beacon.
    CHECK(s.tick(0) == Action::kStartControlConnect);
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
    s.in.gpsFixValid = false;  // quiet the GPS feed for this test
    s.in.speedMph = 0.0f;

    // rpm 400 >= 300: engine not off, stop condition never arms.
    s.in.rpm = 400;
    auto acts = s.run(120000, 1000);
    CHECK(s.f.state == State::kRecording);
    CHECK(s.f.stopCondSince == 0);
    CHECK(countOf(acts, Action::kSendStopVideo) == 0);

    // rpm exactly 300 is still not off (off is rpm < 300).
    s.in.rpm = 300;
    acts = s.run(120000, 1000);
    CHECK(s.f.state == State::kRecording);
    CHECK(s.f.stopCondSince == 0);

    // rpm 299: engine off; with speed 0 the 60 s hold runs and stops.
    s.in.rpm = 299;
    CHECK(s.tick(10) == Action::kNone);  // arm (keep-alive already stamped)
    CHECK(s.f.stopCondSince != 0);
    CHECK(s.tick(kStopRecordDelayMs) == Action::kSendStopVideo);
    CHECK(s.f.state == State::kCooldown);
}

// ---------------------------------------------------------------------------
// IDLE — stale links
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - stale control link skips straight to AWAIT GPS") {
    Sim s;
    s.in.controlConnected = true;
    s.in.rpm = 1000;
    CHECK(s.tick(0) == Action::kNone);
    CHECK(s.tick(kRpmOnDebounceMs) == Action::kNone);
    CHECK(s.f.state == State::kAwaitGps);

    // No advertising action ever, straight through to recording.
    s.in.gpsFixValid = true;
    CHECK(s.tick(10) == Action::kSendStartVideo);
    CHECK(s.f.state == State::kRecording);
}

TEST_CASE("camera_fsm - stale remote link skips the wake to CONNECTING") {
    Sim s;
    s.in.remoteConnected = true;
    s.in.rpm = 1000;
    CHECK(s.tick(0) == Action::kNone);
    CHECK(s.tick(kRpmOnDebounceMs) == Action::kNone);  // entry action next step
    CHECK(s.f.state == State::kConnecting);
    CHECK(s.f.controlAttemptsUsed == 1);
    CHECK(s.tick(10) == Action::kStartControlConnect);
}

// ---------------------------------------------------------------------------
// WAKING
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - WAKING keeps beaconing (no connectable switch)") {
    Sim s;
    s.toWaking();  // beacon + scanner both up; entry step consumed
    // No 5 s connectable-advert switch any more: it just keeps beaconing.
    auto acts = s.run(19000, 500);  // through most of attempt 1
    CHECK(countOf(acts, Action::kStartConnectableAdvertising) == 0);
    CHECK(acts.empty());  // silent while beaconing + scanning
    CHECK(s.f.state == State::kWaking);
}

TEST_CASE("camera_fsm - WAKING retry ladder: 20 s x3 then back to IDLE") {
    Sim s;
    s.toWaking();  // attempt 1 window starts at T (scanner already running)
    CHECK(s.tick(19999) == Action::kNone);            // T+19.999s: still beaconing
    CHECK(s.tick(1) == Action::kStartWakeBurst);       // T+20s: attempt 2 (re-beacon)
    CHECK(s.f.wakeAttemptsUsed == 2);
    CHECK(s.tick(20000) == Action::kStartWakeBurst);   // T+40s: attempt 3
    CHECK(s.f.wakeAttemptsUsed == 3);
    CHECK(s.tick(20000) == Action::kStopAdvertising);  // T+60s: give up
    CHECK(s.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - WAKING connect during attempt 3 goes to CONNECTING") {
    Sim s;
    s.toWaking();
    CHECK(s.tick(20000) == Action::kStartWakeBurst);   // attempt 2
    CHECK(s.tick(20000) == Action::kStartWakeBurst);   // attempt 3
    s.in.cameraAdvertSeen = true;                      // scanner spots be80
    // Remote advert deliberately stays up (camera reconnects to it).
    CHECK(s.tick(1000) == Action::kNone);
    CHECK(s.f.state == State::kConnecting);
    CHECK(s.f.controlAttemptsUsed == 1);
    // Scanner already running from WAKING: CONNECTING does NOT re-scan.
    CHECK(s.tick(10) == Action::kNone);
}

TEST_CASE("camera_fsm - WAKING controlConnected advances straight to AWAIT GPS") {
    Sim s;
    s.toWaking();
    // The scan callback can connect the control link before we even register
    // the advert sighting — WAKING then goes straight to AWAIT GPS. The
    // remote advert stays up for the camera's reconnect.
    s.in.controlConnected = true;
    CHECK(s.tick(100) == Action::kNone);
    CHECK(s.f.state == State::kAwaitGps);
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
// CONNECTING
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - CONNECTING entry action emitted exactly once") {
    Sim s;
    s.toConnectingViaRemote();
    CHECK(s.tick(10) == Action::kStartControlConnect);
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kConnecting);
}

TEST_CASE("camera_fsm - CONNECTING retry ladder: 15 s x3 then back to IDLE") {
    Sim s;
    s.toConnectingViaRemote();  // attempt 1 window starts at T
    CHECK(s.tick(10) == Action::kStartControlConnect);           // entry
    CHECK(s.tick(14989) == Action::kNone);                       // T+14.999s
    CHECK(s.tick(1) == Action::kStartControlConnect);            // T+15s: attempt 2
    CHECK(s.f.controlAttemptsUsed == 2);
    CHECK(s.tick(15000) == Action::kStartControlConnect);        // attempt 3
    CHECK(s.f.controlAttemptsUsed == 3);
    CHECK(s.tick(15000) == Action::kStopControlConnect);         // give up
    CHECK(s.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - CONNECTING success goes to AWAIT GPS") {
    Sim s;
    s.toConnectingViaRemote();
    CHECK(s.tick(10) == Action::kStartControlConnect);
    s.in.controlConnected = true;
    CHECK(s.tick(500) == Action::kNone);
    CHECK(s.f.state == State::kAwaitGps);
}

TEST_CASE("camera_fsm - CONNECTING sessionEnd returns to IDLE") {
    Sim s;
    s.toConnectingViaRemote();
    CHECK(s.tick(10) == Action::kStartControlConnect);
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kStopControlConnect);
    CHECK(s.f.state == State::kIdle);
}

// ---------------------------------------------------------------------------
// AWAIT GPS
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - AWAIT GPS: fix starts the recording") {
    Sim s;
    s.toAwaitGps();
    s.in.gpsFixValid = true;
    CHECK(s.tick(10) == Action::kSendStartVideo);
    CHECK(s.f.state == State::kRecording);
}

TEST_CASE("camera_fsm - AWAIT GPS: 30 s without fix records anyway") {
    Sim s;
    s.toAwaitGps();  // awaitGpsSince = T
    CHECK(s.in.gpsFixValid == false);
    CHECK(s.tick(kGpsLockTimeoutMs) == Action::kSendStartVideo);
    CHECK(s.f.state == State::kRecording);
}

TEST_CASE("camera_fsm - AWAIT GPS: control drop reconnects with fresh budget") {
    Sim s;
    s.toAwaitGps();
    s.in.controlConnected = false;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kConnecting);
    CHECK(s.f.controlAttemptsUsed == 1);  // fresh budget

    // All three attempts available again.
    CHECK(s.tick(10) == Action::kStartControlConnect);   // entry (attempt 1)
    CHECK(s.tick(15000) == Action::kStartControlConnect);  // attempt 2
    CHECK(s.tick(15000) == Action::kStartControlConnect);  // attempt 3
    CHECK(s.tick(15000) == Action::kStopControlConnect);
    CHECK(s.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - AWAIT GPS: keep-alive at 2 s cadence") {
    Sim s;
    s.toAwaitGps();  // lastKeepAliveAt = T
    auto acts = s.run(10000, 500);  // (T, T+10s], no fix
    CHECK(countOf(acts, Action::kSendKeepAlive) == 5);  // T+2,4,6,8,10 s
    CHECK(s.f.state == State::kAwaitGps);
}

TEST_CASE("camera_fsm - AWAIT GPS: sessionEnd rides into COOLDOWN") {
    Sim s;
    s.toAwaitGps();
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kNone);
    CHECK(s.f.state == State::kCooldown);
}

// ---------------------------------------------------------------------------
// RECORDING — GPS feed decimation
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - RECORDING GPS feed: 1 Hz decimation at 250 Hz stepping") {
    Sim s;
    s.toRecording();  // entered at T; fix valid; rpm 1000, speed 0
    auto acts = s.run(5000, 4);  // 1250 steps over 5 s
    // First frame goes immediately on entry.
    REQUIRE(!acts.empty());
    CHECK(acts.front() == Action::kSendGpsFrame);
    // Exactly one frame per 1000 ms window.
    CHECK(countOf(acts, Action::kSendGpsFrame) == 5);
    CHECK(s.f.state == State::kRecording);
}

TEST_CASE("camera_fsm - RECORDING GPS feed pauses on fix loss and resumes") {
    Sim s;
    s.toRecording();
    CHECK(s.tick(4) == Action::kSendGpsFrame);  // first frame

    s.in.gpsFixValid = false;
    auto acts = s.run(3000, 100);
    CHECK(countOf(acts, Action::kSendGpsFrame) == 0);  // silently no frames
    CHECK(s.f.state == State::kRecording);

    s.in.gpsFixValid = true;
    CHECK(s.tick(100) == Action::kSendGpsFrame);  // resumes immediately
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
    CHECK(countOf(acts, Action::kSendStopVideo) == 0);
}

TEST_CASE("camera_fsm - coasting stall (30 mph, rpm 0) never stops recording") {
    Sim s;
    s.toRecording();
    s.in.speedMph = 30.0f;
    s.in.rpm = 0;
    auto acts = s.run(600000, 1000);
    CHECK(s.f.state == State::kRecording);
    CHECK(countOf(acts, Action::kSendStopVideo) == 0);
}

TEST_CASE("camera_fsm - stop hold: rpm blip at 59.9 s resets, clean 60 s stops") {
    Sim s;
    s.toRecording();
    s.in.gpsFixValid = false;  // quiet the GPS feed
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
    CHECK(s.tick(1) == Action::kSendStopVideo);  // T2+60s
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
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kSendStopVideo);
    CHECK(s.f.state == State::kCooldown);
}

TEST_CASE("camera_fsm - manual end in COOLDOWN powers off immediately") {
    Sim s;
    s.toCooldown();
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kSendPowerOff);
    CHECK(s.f.state == State::kPoweringOff);
}

// ---------------------------------------------------------------------------
// RECORDING — control drop + resume cycle
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - control drop mid-RECORDING reconnects and resumes") {
    Sim s;
    s.toRecording();
    s.in.controlConnected = false;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kConnecting);
    CHECK(s.f.controlAttemptsUsed == 1);  // fresh budget

    CHECK(s.tick(10) == Action::kStartControlConnect);
    s.in.controlConnected = true;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kAwaitGps);
    CHECK(s.tick(10) == Action::kSendStartVideo);  // fix still valid
    CHECK(s.f.state == State::kRecording);
}

// ---------------------------------------------------------------------------
// COOLDOWN
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - COOLDOWN powers off after 180 s") {
    Sim s;
    s.toCooldown();  // cooldownSince = T; control still connected
    CHECK(s.tick(kPowerOffDelayMs) == Action::kSendPowerOff);
    CHECK(s.f.state == State::kPoweringOff);
}

TEST_CASE("camera_fsm - COOLDOWN both links down means camera turned off") {
    Sim s;
    s.toCooldown();
    s.in.controlConnected = false;
    s.in.remoteConnected = false;
    CHECK(s.tick(10) == Action::kNone);
    CHECK(s.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - COOLDOWN ignores motion return; keep-alive continues") {
    Sim s;
    s.toCooldown();
    s.in.rpm = 5000;
    s.in.speedMph = 40.0f;
    auto acts = s.run(10000, 500);
    CHECK(s.f.state == State::kCooldown);  // kAutoResumeFromCooldown == false
    // Keep-alive keeps the be80 link alive through the window...
    CHECK(countOf(acts, Action::kSendKeepAlive) >= 4);
    // ...and nothing else happens.
    CHECK(countOf(acts, Action::kSendKeepAlive) == (long)acts.size());
}

// ---------------------------------------------------------------------------
// POWERING OFF
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - POWERING OFF: links drop returns to IDLE") {
    Sim s;
    s.toPoweringOff();
    s.in.controlConnected = false;
    s.in.remoteConnected = false;
    CHECK(s.tick(1000) == Action::kNone);
    CHECK(s.f.state == State::kIdle);
}

TEST_CASE("camera_fsm - POWERING OFF: 5 s linger forces the disconnect") {
    Sim s;
    s.toPoweringOff();  // powerOffSentAt = T; control still up
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
    CHECK(unpaired.pulse(&Inputs::pairRequested, 10) == Action::kStartConnectableAdvertising);
    CHECK(unpaired.f.state == State::kPairing);
    CHECK(unpaired.f.pairingReturnState == State::kUnpaired);

    Sim idle(true);
    CHECK(idle.pulse(&Inputs::pairRequested, 10) == Action::kStartConnectableAdvertising);
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
    CHECK(fromUnpaired.pulse(&Inputs::pairCancelRequested, 100) == Action::kStopAdvertising);
    CHECK(fromUnpaired.f.state == State::kUnpaired);

    Sim fromIdle(true);
    fromIdle.toPairing();
    CHECK(fromIdle.pulse(&Inputs::pairCancelRequested, 100) == Action::kStopAdvertising);
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
    SUBCASE("from CONNECTING") {
        Sim s;
        s.toConnecting();
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kStopControlConnect);
        CHECK(s.f.state == State::kIdle);
    }
    SUBCASE("from AWAIT GPS") {
        Sim s;
        s.toAwaitGps();
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kNone);
        CHECK(s.f.state == State::kIdle);
    }
    SUBCASE("from RECORDING") {
        Sim s;
        s.toRecording();
        CHECK(s.pulse(&Inputs::forceIdleRequested, 10) == Action::kNone);
        CHECK(s.f.state == State::kIdle);
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

TEST_CASE("camera_fsm - forceIdle clears all timers") {
    Sim s;
    s.toRecording();
    s.in.speedMph = 0.0f;
    s.in.rpm = 0;
    s.tick(10);  // arm the stop hold
    CHECK(s.f.stopCondSince != 0);
    s.pulse(&Inputs::forceIdleRequested, 10);
    CHECK(s.f.stopCondSince == 0);
    CHECK(s.f.rpmOnSince == 0);
    CHECK(s.f.wakeAttemptsUsed == 0);
    CHECK(s.f.controlAttemptsUsed == 0);
    CHECK(s.f.entryPending == false);
}

// ---------------------------------------------------------------------------
// unpair
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - unpair from IDLE clears the serial and drops links") {
    Sim s;
    // kDisconnect, not kNone: a remote link surviving into kIdle must not
    // stay connected to an unpaired (or subsequently re-paired) device.
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
    s.toRecording();           // consumes ~2 s of that headroom
    s.in.gpsFixValid = false;
    s.in.speedMph = 0.0f;
    s.in.rpm = 0;
    CHECK(s.tick(10) == Action::kNone);  // arm the hold pre-wrap
    const uint32_t armedAt = s.f.stopCondSince;
    CHECK(armedAt > 0xFFFFF000u);        // armed before the wrap
    s.tick(59999);                       // 59.999 s held (keep-alives interleave)
    CHECK(s.in.nowMs < 0x10000u);        // clock wrapped during the hold
    CHECK(s.f.state == State::kRecording);
    CHECK(s.tick(1) == Action::kSendStopVideo);  // fires exactly at 60 s
    CHECK(s.f.state == State::kCooldown);
}

// ---------------------------------------------------------------------------
// One-shot event consumption
// ---------------------------------------------------------------------------

TEST_CASE("camera_fsm - sessionEnd is consumed in a single step") {
    Sim s;
    s.toRecording();
    CHECK(s.pulse(&Inputs::sessionEndRequested, 10) == Action::kSendStopVideo);
    CHECK(s.f.state == State::kCooldown);
    // Flag cleared by the caller: the next step must NOT act on it again
    // (a stale sessionEnd in COOLDOWN would emit kSendPowerOff).
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
    CHECK(strcmp(stateName(State::kConnecting), "CONNECTING") == 0);
    CHECK(strcmp(stateName(State::kAwaitGps), "AWAIT GPS") == 0);
    CHECK(strcmp(stateName(State::kRecording), "RECORDING") == 0);
    CHECK(strcmp(stateName(State::kCooldown), "COOLDOWN") == 0);
    CHECK(strcmp(stateName(State::kPoweringOff), "PWR OFF") == 0);
    CHECK(strcmp(stateName(State::kPairing), "PAIRING") == 0);
    // Out-of-range value (corrupted state) still yields a printable label.
    CHECK(strcmp(stateName(static_cast<State>(200)), "?") == 0);
}
