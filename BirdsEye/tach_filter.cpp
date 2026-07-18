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

}  // namespace tach_filter
