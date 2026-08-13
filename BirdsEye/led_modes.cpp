#include "led_modes.h"

#include <cmath>

namespace led_modes {

using led_frame::kStripCenter;
using led_frame::kStripCount;
using led_frame::Rgb;

PacePip pacePip(float paceMsPerM) {
  float const mag = paceMsPerM < 0 ? -paceMsPerM : paceMsPerM;
  if (mag <= kPaceDeadbandMsPerM) {
    return PacePip{kStripCenter, led_frame::kWhite};
  }
  // Steps away from center: ceil(mag / stepSize), clamped to the 4
  // pixels available per side. Ceil (not round) so the pip leaves the
  // center the moment the deadband is exceeded.
  const float stepSize = kPaceFullScaleMsPerM / (float)kStripCenter;
  int steps = (int)(mag / stepSize);
  if ((float)steps * stepSize < mag) {
    steps++;
  }
  if (steps < 1) {
    steps = 1;
  }
  if (steps > kStripCenter) {
    steps = kStripCenter;
  }
  // Positive pace = slower than best = LEFT of center, red.
  if (paceMsPerM > 0) {
    return PacePip{kStripCenter - steps, led_frame::kRed};
  }
  return PacePip{kStripCenter + steps, led_frame::kGreen};
}

void renderPace(float paceMsPerM, Rgb out[kStripCount]) {
  for (int i = 0; i < kStripCount; i++) {
    out[i] = led_frame::kOff;
  }
  out[kStripCenter] = led_frame::scale(led_frame::kWhite, kPaceCenterLevel);
  PacePip const pip = pacePip(paceMsPerM);
  out[pip.stripIndex] = pip.color;
}

void renderScale(float value, const ScaleSpec& spec, Rgb out[kStripCount]) {
  float const span = spec.max - spec.min;
  float frac = span > 0 ? (value - spec.min) / span : 0.0f;
  if (frac < 0) {
    frac = 0;
  }
  if (frac > 1) {
    frac = 1;
  }
  int const lit = (int)lroundf(frac * (float)kStripCount);
  // A pixel is "past" the red fraction when its fill position crosses
  // it: with redFrac 0.5 on 9 px that is indices 5..8 — red past the
  // halfway mark, the center pixel itself still low-color.
  int const redFrom = (int)lroundf(spec.redFrac * (float)kStripCount);
  for (int i = 0; i < kStripCount; i++) {
    if (i >= lit) {
      out[i] = led_frame::kOff;
    } else {
      out[i] = i >= redFrom ? spec.highColor : spec.lowColor;
    }
  }
}

led_frame::Rgb evalStatus(const StatusAction& a, StatusState& s, float value,
                          bool valid, uint32_t nowMs) {
  if (a.source == Source::kNone || !valid) {
    s.active = false;
    return led_frame::kOff;
  }
  if (!s.active && value >= a.threshold) {
    s.active = true;
  } else if (s.active && value < a.clearBelow) {
    s.active = false;
  }
  if (!s.active) {
    return led_frame::kOff;
  }
  uint16_t const half = a.flashHalfPeriodMs;
  if (half == 0) {
    return a.color;  // no flash configured: solid
  }
  return ((nowMs / half) & 1U) == 0 ? a.color : led_frame::kOff;
}

}  // namespace led_modes
