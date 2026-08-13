#pragma once

#include <stdint.h>

#include "led_frame.h"

///////////////////////////////////////////
// LED GLOBAL ANIMATIONS
// The boot flourish and the purple-sector celebration, rendered as PURE
// functions of (time since start, seed): no rand(), no millis(), no
// retained state. Sparkle placement hashes (seed, time slot, pixel), so
// a given (tMs, seed) pair always produces the identical frame — host
// tests golden-lock exact output and the glue stays trivially
// restartable (re-arm = new start stamp + fresh seed).
//
// Boot (2600 ms): a hue comet circles the 11 px as a ring (2 revolutions
// with a trailing fade), the whole thing fades out, and white sparkle
// glints overlap the tail end.
// Purple (1600 ms): a purple wave expands from the strip center outward
// across all 11 px (status LEDs included — it's a celebration), holds
// solid with sparkles, then fades.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

namespace led_animations {

constexpr uint32_t kBootDurationMs = 2600;
// Boot phases: comet circles until kBootCometMs, global fade completes
// at kBootFadeEndMs, sparkles run kBootSparkleStartMs..end.
constexpr uint32_t kBootCometMs = 1500;
constexpr uint32_t kBootFadeEndMs = 2200;
constexpr uint32_t kBootSparkleStartMs = 900;
constexpr int kBootCometRevs = 2;
constexpr int kBootCometTail = 5;  // pixels of trailing fade

constexpr uint32_t kPurpleDurationMs = 1600;
constexpr uint32_t kPurpleWaveMs = 400;      // center-out expansion
constexpr uint32_t kPurpleFadeStartMs = 1200;

// Sparkle time-slot length: each (slot, pixel) pair rolls the hash once,
// so glints pop and die at a readable rate instead of shimmering at the
// frame rate.
constexpr uint32_t kSparkleSlotMs = 80;

// Deterministic integer mixer (xorshift-style avalanche). Exposed for
// tests; also handy anywhere else that needs cheap stateless hashing.
uint32_t mix(uint32_t x);

// 0..255 color wheel (r->g->b->r), full saturation.
led_frame::Rgb hueToRgb(uint8_t hue);

// Render the frame at tMs since the animation started. Returns false
// once tMs >= duration (frame is cleared) — the caller drops back to
// normal composition.
bool renderBoot(uint32_t tMs, uint32_t seed, led_frame::Frame& out);
bool renderPurple(uint32_t tMs, uint32_t seed, led_frame::Frame& out);

}  // namespace led_animations
