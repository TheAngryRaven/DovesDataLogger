#include "led_frame.h"

namespace led_frame {

void clear(Frame& f) {
  for (int i = 0; i < kPixelCount; i++) {
    f.px[i] = kOff;
  }
}

static uint8_t scaleChannel(uint8_t v, uint8_t level) {
  // Round-to-nearest v*level/255. v<=255 so the result is <= level.
  return (uint8_t)(((uint16_t)v * level + 127) / 255);
}

Rgb scale(Rgb c, uint8_t level) {
  return Rgb{scaleChannel(c.r, level), scaleChannel(c.g, level),
             scaleChannel(c.b, level)};
}

void applyCap(Frame& f, uint8_t cap) {
  for (int i = 0; i < kPixelCount; i++) {
    f.px[i] = scale(f.px[i], cap);
  }
}

static uint8_t addChannel(uint8_t a, uint8_t b) {
  uint16_t s = (uint16_t)a + b;
  return s > 255 ? 255 : (uint8_t)s;
}

Rgb add(Rgb a, Rgb b) {
  return Rgb{addChannel(a.r, b.r), addChannel(a.g, b.g),
             addChannel(a.b, b.b)};
}

}  // namespace led_frame
