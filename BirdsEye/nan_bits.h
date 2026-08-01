#pragma once

///////////////////////////////////////////
// NaN DETECTION THAT SURVIVES -Ofast
//
// The Seeed nRF52 platform compiles sketches with -Ofast, which
// includes -ffinite-math-only: GCC constant-folds isnan()/isinf() to
// FALSE and assumes NaN never exists in comparisons. Field-confirmed
// on the DovesSensorEgg (2026-07-27), and this repo has the same
// exposure: the Temp1 page's isnan() gates fold out, so a stale egg
// link rendered lroundf(NaN) garbage ("-214748") instead of '---'.
// (The DOVEX temp columns survived by luck alone - dtostrf(NaN) emits
// "nan", which the isNumericString guard then rejects into the same
// literal "nan" the isnan branch would have written.) Host builds
// don't use -Ofast, so the host test suite cannot see any of this.
//
// This helper inspects the IEEE-754 bit pattern via memcpy, which the
// optimizer cannot fold away: exponent all-ones + nonzero mantissa.
// RULE: device-compiled code (the .ino and every module it links) must
// use isNanF() and must never compare against a possibly-NaN value
// without checking it first. Plain isnan() is reserved for host-only
// code.
///////////////////////////////////////////

#include <stdint.h>
#include <string.h>

static inline bool isNanF(float f) {
  uint32_t u;
  memcpy(&u, &f, sizeof u);
  return (u & 0x7F800000u) == 0x7F800000u && (u & 0x007FFFFFu) != 0u;
}
