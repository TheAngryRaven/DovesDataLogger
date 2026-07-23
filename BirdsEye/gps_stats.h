#pragma once

#include <stdint.h>

///////////////////////////////////////////
// GPS PIPELINE DROP ACCOUNTING (pure logic, host-tested)
//
// Turns the 1 s PVT frame-rate window (calculateGPSFrameRate) into a
// monotonic dropped-frame counter. The module streams PVT at a known
// nav rate, so each window's expected frame count is rate * elapsed;
// any sustained deficit is data loss somewhere in the serial pipeline
// (core UART ring overflow, 4 KB ring overflow, or a UBX checksum
// failure from a corrupted byte).
//
// Design notes:
// - A fractional expected-frame remainder carries across windows, so
//   the long-run expectation is exact even though window lengths are
//   not exactly 1000 ms.
// - A frame that slips across a window boundary shows up as -1 in one
//   window and +1 in the next. One frame of slack absorbs that jitter
//   so a nonzero droppedTotal always means real loss; the cost is that
//   the very first real drop of a session is not counted (undercount
//   bounded at kSlackFrames for the whole session).
// - Surplus credit is capped so clock skew between millis() and the
//   GPS-disciplined stream cannot bank enough credit to hide later
//   real losses.
// - A window with zero frames is a dead stream — that regime belongs
//   to the PVT-arrival watchdog, and counting hundreds of "drops" per
//   silent second would drown the sub-1% signal this counter exists
//   for. Dead windows reset the accounting instead.
///////////////////////////////////////////

namespace gps_stats {

// Boundary-jitter slack: deficits up to this many frames are held as
// negative balance instead of being counted as drops.
constexpr int32_t kSlackFrames = 1;

// Surplus-credit cap (frames). Bounds how much a fast millis() clock
// can offset future real losses.
constexpr int32_t kMaxCreditFrames = 2;

struct DropMonitor {
  uint32_t droppedTotal = 0;        // monotonic; the display counter
  int32_t balance = 0;              // received-minus-expected carry,
                                    // clamped to [-kSlackFrames, kMaxCreditFrames]
  uint32_t remainderMilliFrames = 0;  // fractional expected-frame carry
  bool suppressNext = false;        // set by noteRateChange()
};

// Expected whole frames for this window at rateHz, consuming and
// updating the fractional remainder (exact over the long run).
uint32_t expectedFrames(DropMonitor& m, uint32_t elapsedMs, uint16_t rateHz);

// Call whenever the nav rate target changes (5 Hz <-> 25 Hz) or the
// stream restarts (wake / reconfigure / baud recovery): the window in
// progress mixes rates, so its expectation is meaningless and the next
// windowUpdate() is discarded.
void noteRateChange(DropMonitor& m);

// Feed one completed frame-rate window. framesReceived is the PVT
// count observed over elapsedMs while the target rate was rateHz.
void windowUpdate(DropMonitor& m, uint32_t framesReceived,
                  uint32_t elapsedMs, uint16_t rateHz);

}  // namespace gps_stats
