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
  if (a.source == Source::kNone) {
    s.active = false;
    return led_frame::kOff;
  }
  if (!valid) {
    // Release the latch (never latch stale data) and show the action's
    // no-signal color — solid, so it can't be misread as an alert flash.
    s.active = false;
    return a.invalidColor;
  }
  // A caller can hand us clearBelow ABOVE threshold — overrev_limit and
  // target_rpm clamp independently, so overrev_limit <= target_rpm *
  // kRevClearFrac makes the overrev action's release point sit above its
  // own trip point. Left alone, a value in that inverted band sets the
  // latch on one frame and clears it on the next: a 15 Hz strobe of the
  // whole chain instead of the intended 100 ms flash. A release point
  // above the trip point is never meaningful, so collapse it — the action
  // degrades to a plain threshold with no hysteresis, which is right.
  const float clearBelow =
      a.clearBelow > a.threshold ? a.threshold : a.clearBelow;
  if (!s.active && value >= a.threshold) {
    s.active = true;
  } else if (s.active && value < clearBelow) {
    s.active = false;
  }
  if (!s.active) {
    return led_frame::kOff;
  }
  return flashOn(nowMs, a.flashHalfPeriodMs) ? a.color : led_frame::kOff;
}

bool flashOn(uint32_t nowMs, uint16_t halfPeriodMs) {
  if (halfPeriodMs == 0) {
    return true;  // no flash configured: solid
  }
  return ((nowMs / halfPeriodMs) & 1U) == 0;
}

void renderSearchPip(uint32_t tMs, Rgb out[kStripCount]) {
  for (int i = 0; i < kStripCount; i++) {
    out[i] = led_frame::kOff;
  }
  // Triangle wave over the round trip: 0..8..0 across
  // kSearchBouncePeriodMs. The sweep spans (kStripCount - 1) steps each
  // way so both end pixels are reached (and briefly held, since the
  // integer position dwells one slot at each extreme).
  uint32_t const phase = tMs % kSearchBouncePeriodMs;
  uint32_t const halfPeriod = kSearchBouncePeriodMs / 2;
  uint32_t const span = (uint32_t)(kStripCount - 1);
  uint32_t pos;
  if (phase < halfPeriod) {
    pos = (phase * span + halfPeriod / 2) / halfPeriod;
  } else {
    uint32_t const back = phase - halfPeriod;
    pos = span - (back * span + halfPeriod / 2) / halfPeriod;
  }
  out[pos] = led_frame::kGreen;
}

}  // namespace led_modes
