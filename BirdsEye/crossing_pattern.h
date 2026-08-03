#pragma once

///////////////////////////////////////////
// CROSSING ANIMATION PATTERN (pure, host-tested)
//
// The two-frame "calculating" animation shown while the timer is inside a
// crossing zone used to be two hand-stored 1 KB PROGMEM bitmaps
// (image_data_calculating1/2). Both were pure block patterns: 16x16 px
// cells, filled only in the ODD 16 px row bands, with the two frames
// offset by one cell horizontally so alternating them scrolls the blocks
// sideways.
//
// Storing 2048 bytes of flash to say that is a bad trade — the same
// output is eight fillRect() calls. This unit emits those rectangles;
// crossing_pattern_test.cpp rasterizes them and asserts the result is
// byte-identical to the original bitmaps, so the animation is provably
// unchanged. (The bird splash stays a real bitmap — it is actual art.)
//
// No Arduino headers: compiled into both the firmware and the host tests.
///////////////////////////////////////////

namespace crossing_pattern {

// Display + cell geometry (the original bitmaps were 128x64, 16 px cells).
constexpr int kWidth  = 128;
constexpr int kHeight = 64;
constexpr int kCell   = 16;

// Filled cells only ever occupy the odd row bands, 4 per band, 2 bands.
constexpr int kMaxRects = 8;

struct Rect {
  int x, y, w, h;
};

/**
 * @brief Emit the filled blocks for one animation frame.
 *
 * @param flip Which of the two frames: false = the even column phase
 *   (original image_data_calculating2), true = the odd column phase
 *   (original image_data_calculating1).
 * @param out Caller's array, at least kMaxRects entries.
 * @param maxOut Capacity of out; emission stops if it would overflow.
 * @return Number of rectangles written.
 */
int frameRects(bool flip, Rect* out, int maxOut);

}  // namespace crossing_pattern
