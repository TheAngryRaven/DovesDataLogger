#pragma once

///////////////////////////////////////////
// TACHOMETER KALMAN FILTER
// The 1-D Kalman filter that turns mean inter-pulse periods into a
// smoothed RPM estimate, extracted from tachometer.ino so the math is
// host-testable. The ISR/ring-buffer plumbing stays in the sketch; this
// unit owns the predict/update equations and the tuning constants.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

namespace tach_filter {

// Process noise Q: how much RPM^2 the true value can change between
// updates. A kart engine with a light flywheel can shift ~200 RPM per
// pulse at 5k RPM. 800 is conservative-smooth; raise to 1500–2000 if
// tracking feels sluggish.
constexpr float kProcessNoiseQ = 800.0f;

// Measurement noise R base: variance of a single-period RPM measurement
// (~50 RPM std dev from combustion variation + ISR latency jitter).
// Scales inversely with the number of periods in the measurement —
// more pulses, more confidence.
constexpr float kMeasurementNoiseRBase = 2500.0f;

// Uncertainty assigned at reset (engine stop / sleep / boot): high, so
// the first real measurement dominates the stale estimate.
constexpr float kInitialUncertaintyP = 10000.0f;

// Floor that keeps the uncertainty from collapsing numerically to zero
// (which would make the filter stop tracking).
constexpr float kUncertaintyFloorP = 1.0f;

struct Kalman {
  float x = 0.0f;                    // RPM estimate
  float p = kInitialUncertaintyP;    // estimate uncertainty (RPM^2)
};

// Return the estimate to the rest state (RPM 0, high uncertainty).
void reset(Kalman& k);

// Fold one measurement into the estimate: `rpmMeasured` is the RPM
// implied by the mean of `periodCount` inter-pulse periods. periodCount
// <= 0 is a no-op.
void update(Kalman& k, float rpmMeasured, int periodCount);

// Convert a mean inter-pulse period (microseconds) to RPM for a pickup
// producing `revsPerPulse` revolutions per pulse (wasted spark = 1.0).
// Non-positive inputs return 0.
float rpmFromMeanPeriodUs(float meanPeriodUs, float revsPerPulse);

}  // namespace tach_filter
