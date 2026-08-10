#pragma once

#include <stdint.h>

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

///////////////////////////////////////////
// ENGINE GEOMETRY
//
// The pickup counts IGNITION PULSES; the tach reports REVOLUTIONS. Those
// are only the same thing on a single-cylinder engine that fires every
// revolution — the common kart case, which is why one pulse per rev was
// assumed for so long. Anything else skews RPM by a fixed factor:
//
//   pulses per rev = cylinders x (wasted spark ? 1.0 : 0.5)
//
// Both settings default to the values that reproduce the old behaviour
// exactly (1 cylinder, wasted spark => 1.0), so a device that has never
// been configured reads identically before and after.
//
// NOTE ON PICKUP PLACEMENT: `cylinderCount` is the cylinders the pickup
// SEES, not the engine's. A clamp around one plug wire of a twin sees one.
///////////////////////////////////////////

// Pulses per revolution contributed by each cylinder.
constexpr float kPulsesPerRevWasted = 1.0f;  // 2-stroke, or 4-stroke wasted spark
constexpr float kPulsesPerRevSingle = 0.5f;  // 4-stroke single-fire: one spark per two revs

// Clamp for a nonsensical stored value — a bad setting must not divide RPM
// by zero or by something absurd, it must fall back to sane behaviour.
constexpr int kMinCylinders = 1;
constexpr int kMaxCylinders = 16;

// Debounce gap for a single-cylinder wasted-spark engine: the historical
// 3 ms, which rejects ignition ringing and caps at ~20 000 pulses/min.
constexpr uint32_t kBasePulseGapUs = 3000;

// Never debounce tighter than this, however many cylinders are configured.
//
// Chosen so the full ~20 000 true-RPM ceiling survives up to FOUR cylinders
// (3000 / 4 = 750); past that the floor binds and the ceiling drops — 10 000
// true RPM at eight cylinders — which is well clear of anything this logger
// is pointed at.
//
// Not a CPU limit: the ISR body is <1 us, so even the floor's worst case
// (~1300 interrupts/s) is negligible. It is a RINGING limit, and the margin
// holds from both ends — the tach input is RC-filtered (~100 us), and the
// documented pickup circuits emit pulses MILLISECONDS wide (see
// TACHOMETER/README.md: circuit 1's 5 ms pulse is itself the ~9800 RPM
// limit on that hardware), so a spurious edge inside 750 us is not a shape
// either circuit produces.
constexpr uint32_t kMinPulseGapFloorUs = 750;

// Revolutions per pulse for an engine, ready for `rpmFromMeanPeriodUs`.
// Out-of-range cylinder counts are clamped rather than rejected.
float revsPerPulse(int cylinderCount, bool wastedSpark);

// The debounce gap that preserves the same TRUE-RPM ceiling on every
// engine. A fixed 3 ms caps ~20 000 pulses/min, which on a twin firing
// every rev is only ~10 000 real RPM — a screaming 2-cyl 2T would hit the
// debounce and read low. Scaling the gap by pulses-per-rev keeps the
// ceiling where it has always been.
uint32_t minPulseGapUs(int cylinderCount, bool wastedSpark);

}  // namespace tach_filter
