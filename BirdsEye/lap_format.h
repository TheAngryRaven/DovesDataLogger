#pragma once

///////////////////////////////////////////
// LAP TIME FORMATTING
// Single source of truth for rendering a lap time in milliseconds as
// "M:SS.mmm". The display pages used to carry six divergent inline
// copies of this math — three of which appended the millisecond
// zero-padding AFTER the value (7 ms rendered as ".700", a 693 ms error
// on the number the whole product exists to display) and one of which
// did no padding at all.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

#include <stddef.h>

namespace lap_format {

// Worst case is "71582:47.295" (the full unsigned 32-bit ms range) —
// 12 chars + NUL. 16 leaves headroom.
constexpr size_t kLapTimeStrLen = 16;

// How to render the minutes field when the time is under one minute.
// When minutes > 0 every style renders identically: "M:SS.mmm" with
// zero-padded seconds.
enum ZeroMinutesStyle : unsigned char {
  kOmit,   // "23.007"  — compact (replay results)
  kShow,   // "0:23.007" — fixed field (lap history list)
  kSpace,  // " 23.007"  — column-stable for big-font live pages: the
           //              minutes slot becomes a space and seconds are
           //              space-padded to two chars (" 3.007" never
           //              shifts the decimal point as the lap rolls past
           //              10 s or 1 min)
};

// Format `ms` into `buf` (size `bufSize`, recommend kLapTimeStrLen).
// Milliseconds are always exactly three zero-padded digits. Output is
// always NUL-terminated (truncated if the buffer is too small).
void formatLapTime(unsigned long ms, ZeroMinutesStyle style,
                   char* buf, size_t bufSize);

}  // namespace lap_format
