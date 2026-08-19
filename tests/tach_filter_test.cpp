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
// Mode parsing — the tach_filter setting
// ---------------------------------------------------------------------------

TEST_CASE("modeFromSetting - the three tokens, case-insensitively") {
    CHECK(modeFromSetting("smooth") == Mode::kSmooth);
    CHECK(modeFromSetting("legacy") == Mode::kLegacy);
    CHECK(modeFromSetting("raw") == Mode::kRaw);
    CHECK(modeFromSetting("RAW") == Mode::kRaw);
    CHECK(modeFromSetting("Legacy") == Mode::kLegacy);
}

TEST_CASE("modeFromSetting - anything unrecognised degrades to the default") {
    // House idiom: a blank, garbled or future settings value must never be
    // the reason a device logs worse data than the shipped default.
    CHECK(modeFromSetting("") == Mode::kSmooth);
    CHECK(modeFromSetting(nullptr) == Mode::kSmooth);
    CHECK(modeFromSetting("rawr") == Mode::kSmooth);      // prefix is not a match
    CHECK(modeFromSetting("ra") == Mode::kSmooth);        // nor is a truncation
    CHECK(modeFromSetting("off") == Mode::kSmooth);
}

TEST_CASE("modeName / modeTag - round trip and stay distinct") {
    const Mode all[] = {Mode::kSmooth, Mode::kLegacy, Mode::kRaw};
    for (Mode m : all) {
        CHECK(modeFromSetting(modeName(m)) == m);
    }
    CHECK(modeTag(Mode::kSmooth) != modeTag(Mode::kLegacy));
    CHECK(modeTag(Mode::kSmooth) != modeTag(Mode::kRaw));
    CHECK(modeTag(Mode::kLegacy) != modeTag(Mode::kRaw));
}

// ---------------------------------------------------------------------------
// Noise models
// ---------------------------------------------------------------------------

TEST_CASE("processNoise - legacy is flat per update, smooth is per second") {
    CHECK(processNoise(Mode::kLegacy, 0.001f) == kProcessNoiseQ);
    CHECK(processNoise(Mode::kLegacy, 10.0f) == kProcessNoiseQ);
    // The rate is chosen so smooth matches legacy at a 6000 RPM single's
    // 10 ms period — the point the old per-update tuning was implicitly
    // calibrated for.
    CHECK(processNoise(Mode::kSmooth, 0.010f) == doctest::Approx(kProcessNoiseQ));
    // …and, unlike legacy, it halves when the period does.
    CHECK(processNoise(Mode::kSmooth, 0.005f) == doctest::Approx(kProcessNoiseQ / 2));
}

TEST_CASE("processNoise - clamped at both ends") {
    // A long coast must not open the uncertainty so wide the gate stops
    // meaning anything; a degenerate dt must not freeze the filter by
    // driving the Kalman gain to zero.
    CHECK(processNoise(Mode::kSmooth, 100.0f) ==
          doctest::Approx(kProcessNoiseRate * kMaxPredictSeconds));
    CHECK(processNoise(Mode::kSmooth, 0.0f) == kProcessNoiseFloorQ);
    CHECK(processNoise(Mode::kSmooth, -5.0f) == kProcessNoiseFloorQ);
}

TEST_CASE("measurementNoise - legacy is RPM-blind, smooth is not") {
    CHECK(measurementNoise(Mode::kLegacy, 2000.0f, 1.0f, 1) == kMeasurementNoiseRBase);
    CHECK(measurementNoise(Mode::kLegacy, 14000.0f, 1.0f, 1) == kMeasurementNoiseRBase);
    // THE bug this fixes: RPM = K/period, so a fixed timing error costs
    // RPM^2/K of RPM error. A flat R was far too confident at the top of
    // the rev range, which is exactly where the plotted trace got noisy.
    CHECK(measurementNoise(Mode::kSmooth, 14000.0f, 1.0f, 1) >
          10.0f * measurementNoise(Mode::kSmooth, 3000.0f, 1.0f, 1));
}

TEST_CASE("measurementNoise - grows monotonically with RPM") {
    float prev = 0.0f;
    for (float rpm = 500.0f; rpm <= 20000.0f; rpm += 500.0f) {
        const float r = measurementNoise(Mode::kSmooth, rpm, 1.0f, 1);
        CHECK(r > prev);
        prev = r;
    }
}

TEST_CASE("measurementNoise - a shorter period per pulse is noisier") {
    // A twin at 6000 RPM fires twice as often, so each period is half as
    // long and the same timing jitter is worth twice the RPM error.
    CHECK(measurementNoise(Mode::kSmooth, 6000.0f, 0.5f, 1) >
          measurementNoise(Mode::kSmooth, 6000.0f, 1.0f, 1));
}

TEST_CASE("measurementNoise - averaging more periods lowers it, in both modes") {
    CHECK(measurementNoise(Mode::kSmooth, 6000.0f, 1.0f, 4) ==
          doctest::Approx(measurementNoise(Mode::kSmooth, 6000.0f, 1.0f, 1) / 4));
    CHECK(measurementNoise(Mode::kLegacy, 6000.0f, 1.0f, 4) ==
          doctest::Approx(kMeasurementNoiseRBase / 4));
}

TEST_CASE("measurementNoise - degenerate inputs stay finite and positive") {
    CHECK(measurementNoise(Mode::kSmooth, -100.0f, 1.0f, 1) == kMeasurementNoiseRBase);
    CHECK(std::isfinite(measurementNoise(Mode::kSmooth, 6000.0f, 0.0f, 1)));
    CHECK(measurementNoise(Mode::kSmooth, 6000.0f, 1.0f, 0) > 0.0f);
}

// ---------------------------------------------------------------------------
// Kalman update behavior
// ---------------------------------------------------------------------------

namespace {

// A steady stream of identical measurements at one engine speed: the
// period each pulse spans is the dt the predict step gets.
void feed(Kalman& k, Mode m, float rpm, int count, float revsPerPulse = 1.0f) {
    const float dt = 60.0f / rpm * revsPerPulse;  // seconds per pulse
    for (int i = 0; i < count; i++) {
        update(k, m, rpm, 1, dt, revsPerPulse);
    }
}

}  // namespace

TEST_CASE("Kalman - reset state is rest with high uncertainty") {
    Kalman k;
    k.x = 4321.0f;
    k.p = 5.0f;
    k.updates = 9;
    k.strikes = 2;
    reset(k);
    CHECK(k.x == 0.0f);
    CHECK(k.p == kInitialUncertaintyP);
    CHECK(k.updates == 0);
    CHECK(k.strikes == 0);
}

TEST_CASE("Kalman - reset keeps the reject count") {
    // reset() runs on every engine stop. Clearing the diagnostic there
    // would zero it at every corner and make the pickup-health number on
    // the tach debug line meaningless.
    Kalman k;
    k.rejected = 37;
    reset(k);
    CHECK(k.rejected == 37);
}

TEST_CASE("Kalman - the first measurement after a reset is adopted whole") {
    // With no prior there is nothing to filter against, and a slow climb
    // out of 0 RPM would arm the gate partway up and get the engine's own
    // speed rejected as an outlier.
    Kalman k;
    CHECK(update(k, Mode::kSmooth, 5000.0f, 1, 0.012f, 1.0f));
    CHECK(k.x == doctest::Approx(5000.0f));
    CHECK(k.p == kInitialUncertaintyP);
}

TEST_CASE("Kalman - legacy keeps its partial first jump") {
    // Not adopted: kLegacy exists to reproduce the shipped filter, whose
    // flat R put the first update's gain at 0.81.
    Kalman k;
    CHECK(update(k, Mode::kLegacy, 5000.0f, 1, 0.012f, 1.0f));
    CHECK(k.x > 4000.0f);
    CHECK(k.x < 5000.0f);
}

TEST_CASE("Kalman - converges to a constant measurement") {
    Kalman k;
    feed(k, Mode::kSmooth, 5000.0f, 200);
    CHECK(k.x == doctest::Approx(5000.0f).epsilon(0.001));
}

TEST_CASE("Kalman - approach to a constant input is monotonic (no overshoot)") {
    // Started from a settled estimate, not a fresh reset: the first
    // measurement after a reset is adopted outright, so there is no climb
    // to watch there. The step is well inside the gate.
    Kalman k;
    feed(k, Mode::kSmooth, 5800.0f, 40);
    float prev = k.x;
    for (int i = 0; i < 20; i++) {
        update(k, Mode::kSmooth, 6000.0f, 1, 0.010f, 1.0f);
        CHECK(k.x > prev);        // climbing toward the measurement…
        CHECK(k.x <= 6000.0f);    // …without ever passing it
        prev = k.x;
    }
}

TEST_CASE("Kalman - more periods per measurement means faster convergence") {
    // R scales as R/periodCount, so a batch of 8 periods pulls the
    // estimate harder than a single period does.
    Kalman one, eight;
    feed(one, Mode::kSmooth, 5800.0f, 40);
    feed(eight, Mode::kSmooth, 5800.0f, 40);
    update(one, Mode::kSmooth, 6000.0f, 1, 0.010f, 1.0f);
    update(eight, Mode::kSmooth, 6000.0f, 8, 0.080f, 1.0f);
    CHECK(eight.x > one.x);
}

TEST_CASE("Kalman - uncertainty never collapses below the floor") {
    Kalman k;
    for (int i = 0; i < 1000; i++) {
        update(k, Mode::kSmooth, 5000.0f, 8, 0.096f, 1.0f);
    }
    CHECK(k.p >= kUncertaintyFloorP);
}

TEST_CASE("Kalman - non-positive period count is a no-op in every mode") {
    const Mode all[] = {Mode::kSmooth, Mode::kLegacy, Mode::kRaw};
    for (Mode m : all) {
        Kalman k;
        update(k, m, 5000.0f, 3, 0.036f, 1.0f);
        const float x = k.x;
        const float p = k.p;
        CHECK_FALSE(update(k, m, 9999.0f, 0, 0.012f, 1.0f));
        CHECK_FALSE(update(k, m, 9999.0f, -1, 0.012f, 1.0f));
        CHECK(k.x == x);
        CHECK(k.p == p);
    }
}

TEST_CASE("Kalman - tracks a ramp like a real engine pull") {
    // Feed an accelerating input and require the estimate to lag but
    // follow within the process noise's ability to track crank inertia.
    Kalman k;
    float rpm = 3000.0f;
    for (int i = 0; i < 120; i++) {
        update(k, Mode::kSmooth, rpm, 1, 60.0f / rpm, 1.0f);
        rpm += 40.0f;
    }
    CHECK(k.x > 6000.0f);
    CHECK(k.x < rpm);
}

// ---------------------------------------------------------------------------
// The outlier gate (plan 0009) — why the plotted RPM used to spike
// ---------------------------------------------------------------------------

TEST_CASE("gate - one spurious edge does not move the estimate") {
    // THE bug. A ringing edge 4 ms after a real one at 3000 RPM reads as
    // 15,000 RPM; the ungated filter folded ~43% of that straight into
    // the output, so a single bad microsecond reading became a 5000 RPM
    // spike on the graph.
    Kalman k;
    feed(k, Mode::kSmooth, 3000.0f, 40);
    const float settled = k.x;

    CHECK_FALSE(update(k, Mode::kSmooth, 15000.0f, 1, 0.004f, 1.0f));
    CHECK(k.x == doctest::Approx(settled));   // estimate coasted, untouched
    CHECK(k.rejected == 1);
}

TEST_CASE("gate - one missed spark does not halve the estimate either") {
    // The other signature: the pickup drops a spark, the period doubles,
    // and the measurement reads half the real engine speed.
    Kalman k;
    feed(k, Mode::kSmooth, 6000.0f, 40);
    const float settled = k.x;

    CHECK_FALSE(update(k, Mode::kSmooth, 3000.0f, 1, 0.020f, 1.0f));
    CHECK(k.x == doctest::Approx(settled));
    CHECK(k.rejected == 1);
}

TEST_CASE("gate - the ungated filter really did pass those spikes through") {
    // Same insult, legacy mode: the contrast IS the bug report, so assert
    // it rather than leaving it to the plan document.
    Kalman k;
    feed(k, Mode::kLegacy, 3000.0f, 40);
    const float settled = k.x;
    CHECK(update(k, Mode::kLegacy, 15000.0f, 1, 0.004f, 1.0f));
    CHECK(k.x > settled + 4000.0f);
    CHECK(k.rejected == 0);   // legacy has no gate to count with
}

TEST_CASE("gate - three consecutive outliers are a real step change") {
    // A clutch dump or a spin is not three coincident bad edges. The
    // third measurement is adopted outright, not filtered toward.
    Kalman k;
    feed(k, Mode::kSmooth, 11000.0f, 60);

    CHECK_FALSE(update(k, Mode::kSmooth, 4000.0f, 1, 0.015f, 1.0f));
    CHECK_FALSE(update(k, Mode::kSmooth, 4000.0f, 1, 0.015f, 1.0f));
    CHECK(update(k, Mode::kSmooth, 4000.0f, 1, 0.015f, 1.0f));
    CHECK(k.x == doctest::Approx(4000.0f));
    CHECK(k.strikes == 0);    // strike run cleared on adoption
}

TEST_CASE("gate - an isolated outlier does not accumulate strikes") {
    // Rejections have to be CONSECUTIVE. Otherwise a pickup emitting one
    // bad edge every few seconds would eventually trip the escape hatch
    // on three unrelated spikes and adopt one of them.
    Kalman k;
    feed(k, Mode::kSmooth, 6000.0f, 40);
    for (int i = 0; i < 3; i++) {
        CHECK_FALSE(update(k, Mode::kSmooth, 15000.0f, 1, 0.004f, 1.0f));
        feed(k, Mode::kSmooth, 6000.0f, 1);   // a good period in between
    }
    CHECK(k.x == doctest::Approx(6000.0f).epsilon(0.02));
    CHECK(k.rejected == 3);
}

TEST_CASE("gate - stays shut until the estimate means something") {
    // Straight out of reset the estimate is 0 RPM. Gating against that
    // would reject the engine starting, which is the one measurement the
    // filter absolutely must accept.
    Kalman k;
    CHECK(update(k, Mode::kSmooth, 4000.0f, 1, 0.015f, 1.0f));
    CHECK(k.rejected == 0);
    CHECK(k.x > 0.0f);
}

TEST_CASE("gate - a whole engine start is never rejected") {
    Kalman k;
    for (int i = 0; i < 20; i++) {
        CHECK(update(k, Mode::kSmooth, 4000.0f, 1, 0.015f, 1.0f));
    }
    CHECK(k.rejected == 0);
    CHECK(k.x == doctest::Approx(4000.0f).epsilon(0.01));
}

TEST_CASE("gate - real acceleration never trips it") {
    // A 5500 RPM/s pull moves the crank only ~25 RPM between pulses —
    // orders of magnitude inside the gate. If this ever fails, the gate
    // has been tightened into something that fights the engine.
    Kalman k;
    float rpm = 2000.0f;
    feed(k, Mode::kSmooth, rpm, 20);
    for (int i = 0; i < 400 && rpm < 13000.0f; i++) {
        const float dt = 60.0f / rpm;
        rpm += 5500.0f * dt;
        update(k, Mode::kSmooth, rpm, 1, dt, 1.0f);
    }
    CHECK(k.rejected == 0);
    CHECK(k.x > 11000.0f);
}

TEST_CASE("gate - hard deceleration is not held up either") {
    // 10,000 RPM/s off the throttle. What is left is ordinary tracking
    // lag, bounded here at 600 RPM — 60 ms at that rate — not the gate
    // fighting the engine, which would look like the estimate parking
    // near 13,000 while the measurements sat at 3000.
    Kalman k;
    float rpm = 13000.0f;
    feed(k, Mode::kSmooth, rpm, 40);
    for (int i = 0; i < 400 && rpm > 3000.0f; i++) {
        const float dt = 60.0f / rpm;
        rpm -= 10000.0f * dt;
        update(k, Mode::kSmooth, rpm, 1, dt, 1.0f);
    }
    CHECK(k.x > rpm - 600.0f);
    CHECK(k.x < rpm + 600.0f);
}

// ---------------------------------------------------------------------------
// Raw mode — the estimator switched off for a track A/B
// ---------------------------------------------------------------------------

TEST_CASE("raw - publishes the measurement verbatim, spikes included") {
    Kalman k;
    feed(k, Mode::kRaw, 6000.0f, 10);
    CHECK(k.x == doctest::Approx(6000.0f));
    // The whole point: no gate, no smoothing, so what the pickup delivers
    // is what the DOVEX column gets.
    CHECK(update(k, Mode::kRaw, 15000.0f, 1, 0.004f, 1.0f));
    CHECK(k.x == doctest::Approx(15000.0f));
    CHECK(k.rejected == 0);
}

TEST_CASE("raw - leaves the uncertainty wide open") {
    // So the first filtered update after a mode change (which means a
    // reboot) re-acquires immediately instead of inheriting false
    // confidence from a run of unfiltered values.
    Kalman k;
    feed(k, Mode::kRaw, 6000.0f, 50);
    CHECK(k.p == kInitialUncertaintyP);
}

// ---------------------------------------------------------------------------
// Legacy mode — must reproduce the pre-0009 filter exactly, or the
// track-side A/B against existing logs is comparing against a fiction.
// ---------------------------------------------------------------------------

TEST_CASE("legacy - is the old fixed-Q, fixed-R, ungated arithmetic") {
    Kalman k;
    // Hand-rolled reference: the shipped filter, verbatim.
    float x = 0.0f, p = kInitialUncertaintyP;
    for (int i = 0; i < 25; i++) {
        const float rpm = 5000.0f + 100.0f * (float)i;
        p += kProcessNoiseQ;
        const float r = kMeasurementNoiseRBase / 3.0f;
        const float gain = p / (p + r);
        x += gain * (rpm - x);
        p *= (1.0f - gain);
        if (p < kUncertaintyFloorP) p = kUncertaintyFloorP;

        CHECK(update(k, Mode::kLegacy, rpm, 3, 0.036f, 1.0f));
        CHECK(k.x == doctest::Approx(x));
        CHECK(k.p == doctest::Approx(p));
    }
}

TEST_CASE("legacy - ignores dt entirely") {
    Kalman fast, slow;
    for (int i = 0; i < 20; i++) {
        update(fast, Mode::kLegacy, 6000.0f, 1, 0.001f, 1.0f);
        update(slow, Mode::kLegacy, 6000.0f, 1, 0.400f, 1.0f);
    }
    CHECK(fast.x == doctest::Approx(slow.x));
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
