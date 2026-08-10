#include "tach_filter.h"

namespace tach_filter {

void reset(Kalman& k) {
  k.x = 0.0f;
  k.p = kInitialUncertaintyP;
}

void update(Kalman& k, float rpmMeasured, int periodCount) {
  if (periodCount <= 0) return;

  // Predict step: constant-RPM model, uncertainty grows
  k.p += kProcessNoiseQ;

  // Measurement noise scales inversely with number of periods
  const float r = kMeasurementNoiseRBase / (float)periodCount;

  // Update step
  const float gain = k.p / (k.p + r);
  k.x += gain * (rpmMeasured - k.x);
  k.p *= (1.0f - gain);

  // Uncertainty floor to prevent numerical collapse
  if (k.p < kUncertaintyFloorP) k.p = kUncertaintyFloorP;
}

float rpmFromMeanPeriodUs(float meanPeriodUs, float revsPerPulse) {
  if (meanPeriodUs <= 0.0f || revsPerPulse <= 0.0f) return 0.0f;
  return (60.0e6f * revsPerPulse) / meanPeriodUs;
}

namespace {

// Shared by both public helpers so they can never disagree about the
// geometry — a mismatch would scale RPM by one factor and the debounce by
// another.
int clampCylinders(int cylinderCount) {
  if (cylinderCount < kMinCylinders) return kMinCylinders;
  if (cylinderCount > kMaxCylinders) return kMaxCylinders;
  return cylinderCount;
}

}  // namespace

float revsPerPulse(int cylinderCount, bool wastedSpark) {
  const float pulsesPerRev = (float)clampCylinders(cylinderCount) *
                             (wastedSpark ? kPulsesPerRevWasted : kPulsesPerRevSingle);
  return 1.0f / pulsesPerRev;
}

uint32_t minPulseGapUs(int cylinderCount, bool wastedSpark) {
  const uint32_t cyl = (uint32_t)clampCylinders(cylinderCount);

  // Integer division, deliberately: pulses-per-rev is either `cyl` (wasted
  // spark) or `cyl / 2` (single-fire), so dividing the base gap by it is
  // exact in both cases once the single-fire case is expressed as a
  // doubled numerator. Keeps the float rounding — and the rounding-cast
  // bug class that goes with it — out of a value the ISR compares against.
  const uint32_t gap = wastedSpark ? (kBasePulseGapUs / cyl)
                                   : ((kBasePulseGapUs * 2u) / cyl);

  // Fewer pulses per rev (4-stroke single-fire) widens the gap, which is
  // free: there are genuinely fewer edges to catch, so the extra margin
  // only buys more ringing rejection at the same true-RPM ceiling.
  return gap < kMinPulseGapFloorUs ? kMinPulseGapFloorUs : gap;
}

}  // namespace tach_filter
