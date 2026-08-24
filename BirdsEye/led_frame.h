#pragma once

#include <stdint.h>

///////////////////////////////////////////
// LED FRAME MODEL
// The NeoPixel subsystem's pixel layout and the ONE brightness rule.
// The physical chain is 11 WS2812 pixels: pixel 0 and pixel 10 are the
// two status indicators, pixels 1..9 are the strip with a centerline
// pixel in the middle. Modes and animations author colors in full
// 0..255 and never think about brightness; applyCap() is the single
// choke point that enforces the global cap — after it runs, no channel
// of any pixel exceeds the cap, ever. (Deliberately NOT
// Adafruit_NeoPixel::setBrightness(), which rewrites the pixel buffer
// lossily and would spread the invariant across call sites.)
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

namespace led_frame {

// LOGICAL chain layout, always left-to-right as the driver sees it:
// logical px 0 is the LEFT status LED, 10 the RIGHT one, 1..9 the strip
// (addressed strip-relative 0..8 by the mode renderers; kStripCenter is
// strip-relative). Every renderer and animation authors in this space.
constexpr int kPixelCount = 11;
constexpr int kStatusLeft = 0;    // logical left status LED
constexpr int kStatusRight = 10;  // logical right status LED
constexpr int kStripFirst = 1;    // strip = logical px 1..9
constexpr int kStripCount = 9;
constexpr int kStripCenter = 4;  // strip-relative centerline (logical px 5)

// The hardware is wired data-in at the PHYSICAL RIGHT end: chain pixel
// 0 (first on the wire) is the rightmost LED, so the whole chain —
// status LEDs included — is mirrored relative to logical space.
// physicalIndex() does that mapping once, at push time; nothing that
// renders ever thinks about it. Set false if a future build wires
// data-in on the left.
constexpr bool kChainReversed = true;

// Logical (left-to-right) index -> position on the physical wire.
int physicalIndex(int logical);

struct Rgb {
  uint8_t r;
  uint8_t g;
  uint8_t b;
};

// One full frame of the chain, chain-ordered (status, strip, status).
struct Frame {
  Rgb px[kPixelCount];
};

constexpr Rgb kOff{0, 0, 0};
constexpr Rgb kRed{255, 0, 0};
constexpr Rgb kGreen{0, 255, 0};
constexpr Rgb kWhite{255, 255, 255};
// Purple with a blue lean — pure 50/50 red+blue reads pink on WS2812.
constexpr Rgb kPurple{160, 0, 255};
constexpr Rgb kBlue{0, 0, 255};
// Orange needs the green held well down or it reads yellow.
constexpr Rgb kOrange{255, 64, 0};
// Yellow sits between kOrange and a true 255/255/0, which on a WS2812
// reads as a washed-out white rather than yellow. 150 green is the
// point where it is unmistakably yellow next to kOrange and kGreen.
constexpr Rgb kYellow{255, 150, 0};

// All pixels off.
void clear(Frame& f);

// Scale a color by level/255, round-to-nearest integer math. level 255
// is identity, 0 is off.
Rgb scale(Rgb c, uint8_t level);

// THE global brightness invariant. Scales every channel of every pixel
// by cap/255. Post-condition: every channel <= cap (input channels are
// <= 255, so the proportional scale can never exceed the cap). Called
// exactly once per frame, at push time.
void applyCap(Frame& f, uint8_t cap);

// Saturating add, for layering sparkle highlights over a base frame.
Rgb add(Rgb a, Rgb b);

}  // namespace led_frame
