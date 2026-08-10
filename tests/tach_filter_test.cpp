#include "doctest.h"
#include "tach_filter.h"

#include <cmath>

using namespace tach_filter;

// ---------------------------------------------------------------------------
// rpmFromMeanPeriodUs — period → RPM conversion
// ---------------------------------------------------------------------------

TEST_CASE("rpmFromMeanPeriodUs - real engine speeds at wasted spark") {
    // 12 ms per rev = 5000 RPM (the comment in tachometer.ino's header)
    CHECK(rpmFromMeanPeriodUs(12000.0f, 1.0f) == doctest::Approx(5000.0f));
    // 3 ms = the debounce floor = 20k RPM ceiling
    CHECK(rpmFromMeanPeriodUs(3000.0f, 1.0f) == doctest::Approx(20000.0f));
    // 2 s = the sanity-bound ceiling = 30 RPM floor
    CHECK(rpmFromMeanPeriodUs(2000000.0f, 1.0f) == doctest::Approx(30.0f));
}

TEST_CASE("rpmFromMeanPeriodUs - revsPerPulse scales linearly") {
    // Half a rev per pulse (e.g. 2 pulses/rev pickup) halves the RPM.
    CHECK(rpmFromMeanPeriodUs(12000.0f, 0.5f) == doctest::Approx(2500.0f));
    CHECK(rpmFromMeanPeriodUs(12000.0f, 2.0f) == doctest::Approx(10000.0f));
}

TEST_CASE("rpmFromMeanPeriodUs - non-positive inputs return 0") {
    CHECK(rpmFromMeanPeriodUs(0.0f, 1.0f) == 0.0f);
    CHECK(rpmFromMeanPeriodUs(-100.0f, 1.0f) == 0.0f);
    CHECK(rpmFromMeanPeriodUs(12000.0f, 0.0f) == 0.0f);
}

// ---------------------------------------------------------------------------
// Kalman update behavior
// ---------------------------------------------------------------------------

TEST_CASE("Kalman - reset state is rest with high uncertainty") {
    Kalman k;
    k.x = 4321.0f;
    k.p = 5.0f;
    reset(k);
    CHECK(k.x == 0.0f);
    CHECK(k.p == kInitialUncertaintyP);
}

TEST_CASE("Kalman - first update after reset jumps most of the way") {
    // High post-reset uncertainty means the first measurement dominates:
    // gain = (10000+800)/(10000+800+2500) ≈ 0.81.
    Kalman k;
    update(k, 5000.0f, 1);
    CHECK(k.x > 4000.0f);
    CHECK(k.x < 5000.0f);
}

TEST_CASE("Kalman - converges to a constant measurement") {
    Kalman k;
    for (int i = 0; i < 50; i++) {
        update(k, 5000.0f, 3);
    }
    CHECK(k.x == doctest::Approx(5000.0f).epsilon(0.001));
}

TEST_CASE("Kalman - approach to a constant input is monotonic (no overshoot)") {
    Kalman k;
    float prev = k.x;
    for (int i = 0; i < 20; i++) {
        update(k, 6000.0f, 2);
        CHECK(k.x > prev);        // climbing toward the measurement…
        CHECK(k.x <= 6000.0f);    // …without ever passing it
        prev = k.x;
    }
}

TEST_CASE("Kalman - more periods per measurement means faster convergence") {
    // R scales as R_BASE/periodCount, so a batch of 8 periods pulls the
    // estimate harder than a single period does.
    Kalman one, eight;
    update(one, 5000.0f, 1);
    update(eight, 5000.0f, 8);
    CHECK(eight.x > one.x);
}

TEST_CASE("Kalman - uncertainty never collapses below the floor") {
    Kalman k;
    for (int i = 0; i < 1000; i++) {
        update(k, 5000.0f, 8);
    }
    CHECK(k.p >= kUncertaintyFloorP);
}

TEST_CASE("Kalman - non-positive period count is a no-op") {
    Kalman k;
    update(k, 5000.0f, 3);
    const float x = k.x;
    const float p = k.p;
    update(k, 9999.0f, 0);
    update(k, 9999.0f, -1);
    CHECK(k.x == x);
    CHECK(k.p == p);
}

TEST_CASE("Kalman - tracks a ramp like a real engine pull") {
    // Feed an accelerating input (25 Hz batches, +100 RPM per batch) and
    // require the estimate to lag but follow within the process noise's
    // ability to track crankshaft inertia.
    Kalman k;
    float rpm = 3000.0f;
    for (int i = 0; i < 40; i++) {
        update(k, rpm, 3);
        rpm += 100.0f;
    }
    // input is now 6900 (last fed 6900-100); estimate must be close behind
    CHECK(k.x > 6000.0f);
    CHECK(k.x < rpm);
}

// ---------------------------------------------------------------------------
// revsPerPulse — engine geometry (plan 0003)
// ---------------------------------------------------------------------------

TEST_CASE("revsPerPulse - the defaults reproduce the old hardcoded behaviour") {
    // The single most important case: a device that has never been configured
    // must read exactly as it did before these settings existed.
    CHECK(revsPerPulse(1, true) == doctest::Approx(1.0f));
}

TEST_CASE("revsPerPulse - more cylinders means fewer revs per pulse") {
    CHECK(revsPerPulse(2, true) == doctest::Approx(0.5f));
    CHECK(revsPerPulse(4, true) == doctest::Approx(0.25f));
}

TEST_CASE("revsPerPulse - single-fire sees one spark per two revolutions") {
    // A 4-stroke without wasted spark fires half as often, so each pulse
    // accounts for two revolutions rather than one.
    CHECK(revsPerPulse(1, false) == doctest::Approx(2.0f));
    CHECK(revsPerPulse(2, false) == doctest::Approx(1.0f));
    CHECK(revsPerPulse(4, false) == doctest::Approx(0.5f));
}

TEST_CASE("revsPerPulse - a nonsensical cylinder count degrades, never divides by zero") {
    // A corrupt or hand-edited SETTINGS.json must not produce inf/NaN RPM.
    CHECK(revsPerPulse(0, true) == doctest::Approx(1.0f));
    CHECK(revsPerPulse(-3, true) == doctest::Approx(1.0f));
    CHECK(revsPerPulse(9999, true) == doctest::Approx(1.0f / (float)kMaxCylinders));
    CHECK(std::isfinite(revsPerPulse(0, false)));
}

TEST_CASE("revsPerPulse - end to end, a twin reads half a single's RPM") {
    // 6000 pulse-RPM measured at the pickup: on a single that IS 6000 rev/min,
    // on a wasted-spark twin the crank is only turning 3000.
    const float periodUs = 60.0e6f / 6000.0f;
    CHECK(rpmFromMeanPeriodUs(periodUs, revsPerPulse(1, true)) == doctest::Approx(6000.0f));
    CHECK(rpmFromMeanPeriodUs(periodUs, revsPerPulse(2, true)) == doctest::Approx(3000.0f));
    // …and a 4-stroke single-fire is turning twice as fast as the pulses suggest.
    CHECK(rpmFromMeanPeriodUs(periodUs, revsPerPulse(1, false)) == doctest::Approx(12000.0f));
}

// ---------------------------------------------------------------------------
// minPulseGapUs — debounce that keeps the true-RPM ceiling constant
// ---------------------------------------------------------------------------

TEST_CASE("minPulseGapUs - unchanged for the historical single-cylinder case") {
    CHECK(minPulseGapUs(1, true) == kBasePulseGapUs);
}

TEST_CASE("minPulseGapUs - tightens as pulses per rev rise") {
    // A twin firing every rev produces twice the pulses, so the gap has to
    // halve or the debounce itself becomes the RPM ceiling.
    CHECK(minPulseGapUs(2, true) == 1500u);
    CHECK(minPulseGapUs(3, true) == 1000u);
    CHECK(minPulseGapUs(4, true) == kMinPulseGapFloorUs);
}

TEST_CASE("minPulseGapUs - never goes below the floor") {
    CHECK(minPulseGapUs(8, true) == kMinPulseGapFloorUs);
    CHECK(minPulseGapUs(16, true) == kMinPulseGapFloorUs);
}

TEST_CASE("minPulseGapUs - widens when the engine fires less often") {
    // Fewer edges to catch, so the extra margin is free ringing rejection.
    CHECK(minPulseGapUs(1, false) == 6000u);
}

TEST_CASE("minPulseGapUs - holds the old true-RPM ceiling up to four cylinders") {
    // The old fixed 3 ms allowed 20,000 pulses/min, which on a single IS
    // 20,000 RPM. Deriving the gap is what keeps that ceiling meaningful on
    // every engine instead of quietly halving it per added cylinder.
    const float oldCeilingRpm = 60.0e6f / (float)kBasePulseGapUs;  // 20,000
    struct { int cyl; bool wasted; } cases[] = {
        {1, true}, {2, true}, {3, true}, {4, true},
        {1, false}, {2, false}, {4, false}, {8, false},
    };
    for (const auto& c : cases) {
        const float gap = (float)minPulseGapUs(c.cyl, c.wasted);
        const float ceiling = rpmFromMeanPeriodUs(gap, revsPerPulse(c.cyl, c.wasted));
        CHECK(ceiling >= oldCeilingRpm);
    }
}

TEST_CASE("minPulseGapUs - past four cylinders the floor binds, and that is fine") {
    // Documented consequence, asserted so it can't drift silently: the gap
    // stops shrinking, so the ceiling falls. It stays far above anything this
    // logger is pointed at.
    const float ceiling8 =
        rpmFromMeanPeriodUs((float)minPulseGapUs(8, true), revsPerPulse(8, true));
    CHECK(ceiling8 == doctest::Approx(10000.0f));
    CHECK(ceiling8 < 60.0e6f / (float)kBasePulseGapUs);
}

TEST_CASE("minPulseGapUs - a clamped cylinder count still yields a usable gap") {
    CHECK(minPulseGapUs(0, true) == kBasePulseGapUs);
    CHECK(minPulseGapUs(-1, true) == kBasePulseGapUs);
    CHECK(minPulseGapUs(9999, true) >= kMinPulseGapFloorUs);
}
