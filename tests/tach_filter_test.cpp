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
