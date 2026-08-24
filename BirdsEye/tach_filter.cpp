#include "tach_filter.h"

#include <math.h>

namespace tach_filter {

namespace {

// Case-insensitive compare over ASCII. Hand-rolled rather than pulling in
// strcasecmp, which is a POSIX extension the host and the AVR-flavoured
// embedded toolchains disagree about — this unit stays dependency-free.
bool equalsIgnoreCase(const char* a, const char* b) {
  if (a == nullptr || b == nullptr) return false;
  for (; *a != '\0' && *b != '\0'; ++a, ++b) {
    char ca = *a, cb = *b;
    if (ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
    if (cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
    if (ca != cb) return false;
  }
  return *a == *b;
}

}  // namespace

Mode modeFromSetting(const char* value) {
  if (equalsIgnoreCase(value, "raw")) return Mode::kRaw;
  if (equalsIgnoreCase(value, "legacy")) return Mode::kLegacy;
  return Mode::kSmooth;  // blank, unknown, or a future token
}

const char* modeName(Mode m) {
  switch (m) {
    case Mode::kRaw:    return "raw";
    case Mode::kLegacy: return "legacy";
    default:            return "smooth";
  }
}

char modeTag(Mode m) {
  switch (m) {
    case Mode::kRaw:    return 'R';
    case Mode::kLegacy: return 'L';
    default:            return 'S';
  }
}

void reset(Kalman& k) {
  k.x = 0.0f;
  k.p = kInitialUncertaintyP;
  k.updates = 0;
  k.strikes = 0;
  // k.rejected survives on purpose — see the header.
}

float processNoise(Mode m, float dtSeconds) {
  if (m == Mode::kLegacy) return kProcessNoiseQ;

  float dt = dtSeconds;
  if (!(dt > 0.0f)) dt = 0.0f;              // also catches NaN
  if (dt > kMaxPredictSeconds) dt = kMaxPredictSeconds;

  const float q = kProcessNoiseRate * dt;
  return q < kProcessNoiseFloorQ ? kProcessNoiseFloorQ : q;
}

float measurementNoise(Mode m, float rpmMeasured, float revsPerPulse,
                       int periodCount) {
  const float n = (float)(periodCount > 0 ? periodCount : 1);
  if (m == Mode::kLegacy) return kMeasurementNoiseRBase / n;

  // A period measurement carries two errors, and BOTH grow with RPM:
  //   * fixed timing jitter on a period of K/rpm microseconds, so the RPM
  //     error is sigma_T * rpm^2 / K — quadratic, and the reason a flat R
  //     under-reports the noise at the top of the rev range;
  //   * real crank speed variation between firings, a fraction of the
  //     period, so frac * rpm — linear.
  // They are added as standard deviations rather than variances: the
  // conservative direction, and the gate wants to be loose, not tight.
  const float rpm = rpmMeasured > 0.0f ? rpmMeasured : 0.0f;
  const float rpp = revsPerPulse > 0.0f ? revsPerPulse : 1.0f;
  const float sigma = kTimingJitterUs * rpm * rpm / (60.0e6f * rpp) +
                      kCycleVariationFrac * rpm;
  return (kMeasurementNoiseRBase + sigma * sigma) / n;
}

bool update(Kalman& k, Mode m, float rpmMeasured, int periodCount,
            float dtSeconds, float revsPerPulse) {
  if (periodCount <= 0) return false;

  if (m == Mode::kRaw) {
    // No estimator at all: publish what the periods say, spikes included.
    // The uncertainty is parked high so the very next filtered update
    // (after a settings change and reboot) re-acquires immediately.
    k.x = rpmMeasured;
    k.p = kInitialUncertaintyP;
    k.strikes = 0;
    if (k.updates < 0xFFFFu) k.updates++;
    return true;
  }

  if (m == Mode::kSmooth && k.updates == 0) {
    // Nothing is known yet: after an engine-stop reset the estimate is
    // literally 0 RPM. Filtering toward the first measurement through an
    // RPM-aware R takes several pulses to arrive anywhere true — and the
    // gate arms partway up that climb and starts rejecting the engine's
    // real speed as an outlier. With no prior, the first measurement IS
    // the estimate. (kLegacy deliberately does not do this: its whole job
    // is to reproduce the shipped filter, whose flat R made the first
    // update's gain 0.81 and made the climb short enough not to matter.)
    k.x = rpmMeasured;
    k.p = kInitialUncertaintyP;
    k.updates = 1;
    return true;
  }

  const float q = processNoise(m, dtSeconds);
  const float r = measurementNoise(m, rpmMeasured, revsPerPulse, periodCount);

  // ---- Outlier gate ----
  // Only kSmooth gates, and only once the estimate means something: out
  // of reset it is 0 RPM, and gating against that would reject the engine
  // starting.
  if (m == Mode::kSmooth && k.updates >= kGatePrimeUpdates) {
    const float sigma = sqrtf(k.p + q + r);
    if (fabsf(rpmMeasured - k.x) > kGateSigmas * sigma) {
      if (k.rejected < 0xFFFFu) k.rejected++;
      if (k.strikes < 0xFFu) k.strikes++;
      if (k.strikes < kGateMaxRejects) {
        return false;  // one bad edge is not a new engine speed — coast
      }
      // Three in a row is not three coincident bad edges: the estimate is
      // stale. ADOPT the measurement rather than filtering toward it —
      // filtering toward it with the huge R that high RPM implies would
      // take the better part of a second to catch a clutch dump.
      k.x = rpmMeasured;
      k.p = kInitialUncertaintyP;
      k.strikes = 0;
      if (k.updates < 0xFFFFu) k.updates++;
      return true;
    }
  }
  k.strikes = 0;

  // Predict step: constant-RPM model, uncertainty grows
  k.p += q;

  // Update step
  const float gain = k.p / (k.p + r);
  k.x += gain * (rpmMeasured - k.x);
  k.p *= (1.0f - gain);

  // Uncertainty floor to prevent numerical collapse
  if (k.p < kUncertaintyFloorP) k.p = kUncertaintyFloorP;

  if (k.updates < 0xFFFFu) k.updates++;
  return true;
}

float rpmFromMeanPeriodUs(float meanPeriodUs, float revsPerPulse) {
  if (meanPeriodUs <= 0.0f || revsPerPulse <= 0.0f) return 0.0f;
  return (60.0e6f * revsPerPulse) / meanPeriodUs;
}

int clampCylinderCount(int cylinderCount) {
  if (cylinderCount < kMinCylinders) return kMinCylinders;
  if (cylinderCount > kMaxCylinders) return kMaxCylinders;
  return cylinderCount;
}

bool rpmIsInferred(int cylinderCount) {
  return clampCylinderCount(cylinderCount) > 1;
}

float revsPerPulse(bool wastedSpark) {
  return wastedSpark ? kRevsPerPulseWasted : kRevsPerPulseSingle;
}

uint32_t minPulseGapUs(bool wastedSpark) {
  // A single-fire plug fires half as often, so doubling its gap keeps the
  // same true-RPM ceiling in both modes. Integer math, deliberately: this
  // value is compared against in the ISR, so the rounding-cast bug class
  // stays out of it.
  return wastedSpark ? kBasePulseGapUs : (kBasePulseGapUs * 2u);
}

}  // namespace tach_filter
