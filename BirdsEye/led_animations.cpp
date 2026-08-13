#include "led_animations.h"

namespace led_animations {

using led_frame::Frame;
using led_frame::kPixelCount;
using led_frame::Rgb;

uint32_t mix(uint32_t x) {
  // xorshift-multiply avalanche (fmix32 shape) with a golden-ratio
  // pre-xor so 0 doesn't map to 0. Cheap, stateless, and spreads
  // consecutive inputs across the word.
  x ^= 0x9E3779B9U;
  x ^= x >> 16;
  x *= 0x7FEB352DU;
  x ^= x >> 15;
  x *= 0x846CA68BU;
  x ^= x >> 16;
  return x;
}

Rgb hueToRgb(uint8_t hue) {
  // 3-segment wheel: 0..84 r->g, 85..169 g->b, 170..255 b->r.
  if (hue < 85) {
    uint8_t const ramp = (uint8_t)(hue * 3);
    return Rgb{(uint8_t)(255 - ramp), ramp, 0};
  }
  if (hue < 170) {
    uint8_t const ramp = (uint8_t)((hue - 85) * 3);
    return Rgb{0, (uint8_t)(255 - ramp), ramp};
  }
  uint8_t const ramp = (uint8_t)((hue - 170) * 3);
  return Rgb{ramp, 0, (uint8_t)(255 - ramp)};
}

// Layer white sparkle glints over the frame for one time slot. chance is
// per-pixel out of 256; the glint's brightness envelope is a triangle
// over the slot so pops swell and die instead of blinking.
static void addSparkles(Frame& f, uint32_t tMs, uint32_t seed,
                        uint8_t chance) {
  uint32_t const slot = tMs / kSparkleSlotMs;
  uint32_t const phase = tMs % kSparkleSlotMs;
  // Triangle envelope 0..255..0 across the slot.
  uint32_t const half = kSparkleSlotMs / 2;
  uint32_t const env = phase < half ? (phase * 255) / half
                              : ((kSparkleSlotMs - phase) * 255) / half;
  for (int i = 0; i < kPixelCount; i++) {
    uint32_t const h = mix(seed ^ (slot * (uint32_t)kPixelCount + (uint32_t)i));
    if ((h & 0xFFU) < chance) {
      // Peak brightness varies per glint (128..255).
      uint8_t const peak = (uint8_t)(128 + ((h >> 8) & 0x7FU));
      uint8_t const level = (uint8_t)((env * peak) / 255);
      f.px[i] = led_frame::add(f.px[i], led_frame::scale(led_frame::kWhite, level));
    }
  }
}

bool renderBoot(uint32_t tMs, uint32_t seed, Frame& out) {
  led_frame::clear(out);
  if (tMs >= kBootDurationMs) {
    return false;
  }

  // Global fade level: full through the comet phase, ramping to 0 at
  // kBootFadeEndMs, then only sparkles remain.
  uint32_t fade = 255;
  if (tMs >= kBootFadeEndMs) {
    fade = 0;
  } else if (tMs >= kBootCometMs) {
    fade = ((kBootFadeEndMs - tMs) * 255) / (kBootFadeEndMs - kBootCometMs);
  }

  if (fade > 0) {
    // Comet head position in 1/256ths of a pixel around the 11-px ring.
    // While the comet phase runs it advances kBootCometRevs revolutions;
    // during the fade it keeps drifting at the same rate (looks better
    // than freezing) — position simply keeps accumulating.
    uint32_t const headPos256 =
        (tMs * (uint32_t)(kBootCometRevs * kPixelCount) * 256U) / kBootCometMs;
    uint32_t const headPx = (headPos256 / 256U) % (uint32_t)kPixelCount;
    // Hue advances a full wheel per revolution.
    uint8_t const headHue = (uint8_t)(headPos256 / kPixelCount);
    for (int back = 0; back < kBootCometTail; back++) {
      int const px = (int)((headPx + (uint32_t)(kPixelCount - back)) %
                     (uint32_t)kPixelCount);
      // Quadratic tail falloff: bright head, fast decay.
      uint32_t const lin = (uint32_t)(kBootCometTail - back) * 255U /
                     (uint32_t)kBootCometTail;
      uint8_t const level = (uint8_t)((lin * lin) / 255U);
      Rgb const c = hueToRgb((uint8_t)(headHue + (uint8_t)(back * 12)));
      out.px[px] = led_frame::add(
          out.px[px],
          led_frame::scale(led_frame::scale(c, level), (uint8_t)fade));
    }
  }

  if (tMs >= kBootSparkleStartMs) {
    addSparkles(out, tMs, seed, 40);
  }
  return true;
}

bool renderPurple(uint32_t tMs, uint32_t seed, Frame& out) {
  led_frame::clear(out);
  if (tMs >= kPurpleDurationMs) {
    return false;
  }

  // Chain-absolute center of the whole 11-px run (strip centerline).
  const int center = led_frame::kStripFirst + led_frame::kStripCenter;

  if (tMs < kPurpleWaveMs) {
    // Expanding wave: radius sweeps 0..full-chain over the wave phase;
    // the wavefront pixel gets a white kick so the edge reads as motion.
    const int maxRadius = kPixelCount - 1 - center + 1;  // reaches px 10
    int const radius = (int)((tMs * (uint32_t)(maxRadius + 1)) / kPurpleWaveMs);
    for (int i = 0; i < kPixelCount; i++) {
      int const d = i > center ? i - center : center - i;
      if (d < radius) {
        out.px[i] = led_frame::kPurple;
      } else if (d == radius) {
        out.px[i] = led_frame::add(led_frame::kPurple,
                                   led_frame::scale(led_frame::kWhite, 96));
      }
    }
    return true;
  }

  // Hold solid, then fade.
  uint32_t level = 255;
  if (tMs >= kPurpleFadeStartMs) {
    level = ((kPurpleDurationMs - tMs) * 255) /
            (kPurpleDurationMs - kPurpleFadeStartMs);
  }
  for (int i = 0; i < kPixelCount; i++) {
    out.px[i] = led_frame::scale(led_frame::kPurple, (uint8_t)level);
  }
  if (tMs < kPurpleFadeStartMs) {
    addSparkles(out, tMs, seed, 48);
  }
  return true;
}

}  // namespace led_animations
