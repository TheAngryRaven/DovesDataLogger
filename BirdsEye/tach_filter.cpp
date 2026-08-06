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
float pulsesPerRev(int cylinderCount, bool wastedSpark) {
  if (cylinderCount < kMinCylinders) cylinderCount = kMinCylinders;
  if (cylinderCount > kMaxCylinders) cylinderCount = kMaxCylinders;
  return (float)cylinderCount * (wastedSpark ? kPulsesPerRevWasted : kPulsesPerRevSingle);
}

}  // namespace

float revsPerPulse(int cylinderCount, bool wastedSpark) {
  return 1.0f / pulsesPerRev(cylinderCount, wastedSpark);
}

uint32_t minPulseGapUs(int cylinderCount, bool wastedSpark) {
  const float ppr = pulsesPerRev(cylinderCount, wastedSpark);
  const float gap = (float)kBasePulseGapUs / ppr;
  if (gap < (float)kMinPulseGapFloorUs) return kMinPulseGapFloorUs;
  // Fewer pulses per rev (4-stroke single-fire) widens the gap, which is
  // free: there are genuinely fewer edges to catch, so the extra margin
  // only buys more ringing rejection at the same true-RPM ceiling.
  return (uint32_t)(gap + 0.5f);
}

}  // namespace tach_filter
