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
//
// WHERE THE SPIKES CAME FROM (plan 0009). The estimator only ever saw
// ONE number per pulse — the RPM implied by that period — and folded it
// in with a steady-state gain of ~0.43. So a single bad edge (ignition
// ringing sneaking past the debounce, or a spark the pickup missed) is
// not a small error: at 3000 RPM a spurious edge 4 ms after a real one
// reads as 15 000 RPM, and 43% of that lands in the output. That is a
// 5000 RPM spike from ONE bad microsecond reading, and nothing in the
// pipeline could tell it apart from a real engine speed. Three changes
// fix it, all in this unit:
//
//   1. An OUTLIER GATE. A measurement further than kGateSigmas from the
//      estimate is not folded in — the estimate coasts instead. Three
//      consecutive rejections mean the estimate, not the measurement, is
//      wrong, so the third one is adopted outright.
//   2. MEASUREMENT NOISE THAT KNOWS THE RPM. RPM = K / period, so a
//      fixed timing error is worth RPM^2/K of RPM error — quadratic. A
//      flat 2500 RPM^2 was roughly right at 4000 RPM and far too
//      confident above 8000, which is why the trace got noisier the
//      harder the engine was working.
//   3. PROCESS NOISE PER SECOND, not per update. Updates arrive at the
//      pulse rate, so a per-update Q made the filter four times looser
//      at 12 000 RPM than at 3000 — and looser again whenever an SD
//      stall batched several pulses into one update.
///////////////////////////////////////////

namespace tach_filter {

///////////////////////////////////////////
// FILTER MODE (plan 0009)
//
// Exposed as the `tach_filter` setting so the filter can be A/B'd against
// a live engine at the track instead of argued about from a plotted log.
///////////////////////////////////////////
enum class Mode : uint8_t {
  kSmooth = 0,  // default: time-based Q, RPM-aware R, outlier gate
  kLegacy = 1,  // the pre-0009 filter exactly: fixed Q/R, no gate
  kRaw    = 2,  // no estimator at all — publish what the periods say
};

// Parse the `tach_filter` setting. Unknown/blank/garbled degrades to
// kSmooth, the house idiom: a bad settings value must never be the
// reason a device logs worse data than the shipped default.
Mode modeFromSetting(const char* value);

// Lowercase setting-token name ("smooth" / "legacy" / "raw") — debug logs.
const char* modeName(Mode m);

// One-character tag ('S' / 'L' / 'R') for the 21-column display line.
char modeTag(Mode m);

// Process noise Q for kLegacy: how much RPM^2 the true value can change
// between UPDATES. Kept only so kLegacy reproduces the shipped filter
// bit for bit; kSmooth uses the rate below instead.
constexpr float kProcessNoiseQ = 800.0f;

// Process noise as a RATE (RPM^2 per second), which is what a random-walk
// model actually needs: updates arrive at the pulse rate, so charging a
// fixed Q per update silently retunes the filter with engine speed. The
// value is the legacy Q divided by the 10 ms period of a 6000 RPM single
// — i.e. kSmooth behaves like the old filter at 6000 RPM and, unlike it,
// keeps behaving that way everywhere else.
constexpr float kProcessNoiseRate = 80000.0f;

// Never predict more than this far ahead in one step. A batch that spans
// a long coast (or a sanity-bounded 2 s period) must not open the
// uncertainty so wide that the outlier gate stops meaning anything.
constexpr float kMaxPredictSeconds = 0.5f;

// Floor under the per-update process noise, so a degenerate dt can't
// freeze the filter by driving the Kalman gain to zero.
constexpr float kProcessNoiseFloorQ = 25.0f;

// Measurement noise R base: the RPM-independent floor (~50 RPM std dev).
// Scales inversely with the number of periods in the measurement —
// more pulses, more confidence.
constexpr float kMeasurementNoiseRBase = 2500.0f;

// Fixed per-edge timing uncertainty, microseconds: GPIOTE ISR latency
// (the handler is deferred by SoftDevice radio ISRs like everything else
// at app priority) plus edge-shape variation in the pickup. It costs
// sigma_T * RPM^2 / K of RPM error — 26 RPM at 6000, 143 at 14 000.
constexpr float kTimingJitterUs = 80.0f;

// Genuine crank speed variation between firings, as a fraction of the
// period. A single-cylinder engine really does speed up and slow down
// within one revolution; that is not measurement error, but it is not
// something anyone wants plotted either. Costs frac * RPM — linear.
constexpr float kCycleVariationFrac = 0.010f;

///////////////////////////////////////////
// OUTLIER GATE
///////////////////////////////////////////

// How many standard deviations of combined (estimate + process +
// measurement) uncertainty a measurement may sit from the estimate
// before it is treated as a bad edge rather than a new engine speed.
// 5 is deliberately loose: the gate exists to catch the half-period and
// double-period signatures of a spurious/missed spark, which are tens of
// sigma out, NOT to trim honest noise. Real acceleration never trips it —
// a 5500 RPM/s pull moves the crank ~25 RPM between pulses.
constexpr float kGateSigmas = 5.0f;

// Consecutive rejections after which the measurement wins. Three
// independent periods agreeing that the estimate is wrong is a real step
// change (clutch dump, spin, a missed batch), not three coincident bad
// edges — so the third is ADOPTED outright rather than filtered toward,
// which is what makes a hard deceleration settle in ~80 ms.
constexpr uint8_t kGateMaxRejects = 3;

// Accepted updates after a reset before the gate arms. Straight out of
// reset the estimate is 0 RPM and meaningless; gating against it would
// reject the engine starting.
constexpr uint16_t kGatePrimeUpdates = 4;

// Uncertainty assigned at reset (engine stop / sleep / boot): high, so
// the first real measurement dominates the stale estimate.
constexpr float kInitialUncertaintyP = 10000.0f;

// Floor that keeps the uncertainty from collapsing numerically to zero
// (which would make the filter stop tracking).
constexpr float kUncertaintyFloorP = 1.0f;

struct Kalman {
  float x = 0.0f;                    // RPM estimate
  float p = kInitialUncertaintyP;    // estimate uncertainty (RPM^2)
  uint16_t updates = 0;              // accepted updates since reset (arms the gate)
  uint8_t  strikes = 0;              // consecutive gate rejections
  uint16_t rejected = 0;             // lifetime rejections — diagnostic only
};

// Return the estimate to the rest state (RPM 0, high uncertainty).
// `rejected` deliberately SURVIVES: reset() runs on every engine stop, so
// clearing it there would zero the counter between every corner and make
// the pickup-health number on the debug line useless.
void reset(Kalman& k);

// Process noise for one predict step of `dtSeconds`. kLegacy ignores the
// argument and returns the flat per-update Q.
float processNoise(Mode m, float dtSeconds);

// Measurement noise for an `rpmMeasured` reading averaged over
// `periodCount` periods on a pickup producing `revsPerPulse` revolutions
// per pulse. kLegacy ignores the RPM and returns kMeasurementNoiseRBase
// scaled by the count.
float measurementNoise(Mode m, float rpmMeasured, float revsPerPulse,
                       int periodCount);

// Fold one measurement into the estimate: `rpmMeasured` is the RPM
// implied by the mean of `periodCount` inter-pulse periods, which span
// `dtSeconds` of engine time. periodCount <= 0 is a no-op.
//
// Returns true when the estimate moved. False means the outlier gate
// rejected the measurement and the estimate coasted — the caller does
// nothing about that (coasting IS the handling); the count is available
// as `k.rejected` for the debug line.
bool update(Kalman& k, Mode m, float rpmMeasured, int periodCount,
            float dtSeconds, float revsPerPulse);

// Convert a mean inter-pulse period (microseconds) to RPM for a pickup
// producing `revsPerPulse` revolutions per pulse (wasted spark = 1.0).
// Non-positive inputs return 0.
float rpmFromMeanPeriodUs(float meanPeriodUs, float revsPerPulse);

///////////////////////////////////////////
// ENGINE GEOMETRY
//
// The pickup counts IGNITION PULSES on ONE wire. There is one sense wire
// and one clamp, so what it sees is one cylinder's ignition, whatever the
// engine has behind it:
//
//   pulses per rev = (wasted spark ? 1.0 : 0.5)
//
// 2-stroke and 4-stroke wasted-spark plugs fire every revolution; a
// 4-stroke single-fire (traditional distributor/magneto) plug fires once
// every two revolutions. That is the WHOLE geometry.
//
// CYLINDER COUNT IS DELIBERATELY NOT IN THIS MATH. It used to be —
// `pulses_per_rev = cylinders x sparkFactor` — which silently assumed the
// pickup saw EVERY cylinder, true only of a clamp on a shared coil/king
// lead. On the hardware this firmware actually ships with (one clamp, one
// plug wire) it divided RPM by the cylinder count: a V8 on a traditional
// magneto, configured honestly as 8 cylinders + single fire, read an
// EIGHTH of its real crank speed. The setting stayed (it drives the
// multi-cylinder warning in the settings UI and the docs); the divider
// did not.
//
// WHAT THIS COSTS, and why it is accepted: on anything but a single, the
// crank speed is INFERRED from one cylinder's firing rate. Between
// firings the RPM is an assumption, and a cylinder that stops firing
// reads as an engine that stopped. That is exactly how every clamp-on
// inductive tach works, and it is the known and accepted behaviour for
// this class of pickup.
///////////////////////////////////////////

// Revolutions per ignition pulse on the clamped wire.
constexpr float kRevsPerPulseWasted = 1.0f;  // 2-stroke, or 4-stroke wasted spark
constexpr float kRevsPerPulseSingle = 2.0f;  // 4-stroke single-fire: one spark per two revs

// Accepted range for the `cylinder_count` setting. It no longer enters the
// RPM math (see above) — it is the engine descriptor that decides whether
// the "RPM is inferred from ignition pulses" warning applies — but the
// bounds still exist so a corrupt value can be clamped rather than trusted.
constexpr int kMinCylinders = 1;
constexpr int kMaxCylinders = 16;

// Debounce gap for a plug firing every revolution: the historical 3 ms,
// which rejects ignition ringing and caps at ~20 000 RPM.
//
// Not a CPU limit: the ISR body is <1 us. It is a RINGING limit, and the
// margin holds from both ends — the tach input is RC-filtered (~100 us),
// and the documented pickup circuits emit pulses MILLISECONDS wide (see
// TACHOMETER/README.md: circuit 1's 5 ms pulse is itself the ~9800 RPM
// limit on that hardware).
constexpr uint32_t kBasePulseGapUs = 3000;

// Revolutions per pulse for an engine, ready for `rpmFromMeanPeriodUs`.
float revsPerPulse(bool wastedSpark);

// The debounce gap, which holds the same ~20 000 RPM ceiling in both spark
// modes: a single-fire plug fires half as often, so it gets twice the gap.
// Cylinder count is not a term here either — one clamped wire never
// delivers pulses faster than the cylinder it is wrapped around fires.
uint32_t minPulseGapUs(bool wastedSpark);

// Clamp a stored `cylinder_count` into range. A nonsensical value must not
// be able to reach the warning logic (or a future consumer) unbounded.
int clampCylinderCount(int cylinderCount);

// Whether a cylinder count means the crank speed is being INFERRED from
// one cylinder's ignition pulses — i.e. anything but a single. Drives the
// user-facing warning; nothing in the RPM path branches on it.
bool rpmIsInferred(int cylinderCount);

}  // namespace tach_filter
