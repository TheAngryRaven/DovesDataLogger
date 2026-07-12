#pragma once

#include <stdint.h>

///////////////////////////////////////////
// SATELLITE SIGNAL BAR MATH
// The GPS status boot page draws MyChron-style vertical signal bars —
// one per satellite, height proportional to carrier-to-noise (CNO) —
// across the bottom half of the 128x64 OLED. This unit owns the two
// pure pieces: picking which satellites to show (used-in-nav first,
// strongest first) and laying the bars out in the pixel area. The
// UBX-NAV-SAT plumbing and fillRect calls stay in the sketch.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

namespace sat_bars {

// Display cap: more bars than this get too thin to read on 128 px.
constexpr int kMaxSats = 16;

// CNO (dB-Hz) mapped to a full-height bar. Open-sky strong signals run
// ~45-50; anything at/above the ceiling clamps to full height.
constexpr int kCnoCeiling = 50;

// One satellite as observed in UBX-NAV-SAT.
struct SatObs {
  uint8_t cno;  // dB-Hz, 0 = not tracked
  bool used;    // flags.bits.svUsed — participating in the nav solution
};

// Select up to `cap` CNO values from `obs[0..n)`: satellites used in
// the nav solution first, then the rest, each group strongest-first.
// Zero-CNO (untracked) entries are skipped. Returns the count written
// to outCno.
int selectCnos(const SatObs* obs, int n, uint8_t* outCno, int cap);

// One bar's geometry. y is derived at draw time (areaBottom - h).
struct Bar {
  int16_t x;
  int16_t w;
  int16_t h;
};

// Lay out `count` bars across `areaW` px: equal widths (>= 2 px, 1 px
// gap), heights scaled cno/kCnoCeiling into areaH and clamped. Returns
// the number of bars written to `out` (<= maxBars).
int layout(const uint8_t* cno, int count, int areaW, int areaH, Bar* out,
           int maxBars);

}  // namespace sat_bars
