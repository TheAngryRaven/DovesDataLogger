#include "gps_stats.h"

namespace gps_stats {

uint32_t expectedFrames(DropMonitor& m, uint32_t elapsedMs, uint16_t rateHz) {
  // Work in milli-frames so the sub-frame remainder carries exactly.
  const uint32_t milliFrames =
      m.remainderMilliFrames + elapsedMs * (uint32_t)rateHz;
  m.remainderMilliFrames = milliFrames % 1000U;
  return milliFrames / 1000U;
}

void noteRateChange(DropMonitor& m) {
  m.suppressNext = true;
}

void windowUpdate(DropMonitor& m, uint32_t framesReceived,
                  uint32_t elapsedMs, uint16_t rateHz) {
  if (m.suppressNext) {
    // The window spanned a rate switch or stream restart — its
    // expectation mixes rates. Discard it and restart the accounting.
    m.suppressNext = false;
    m.remainderMilliFrames = 0;
    m.balance = 0;
    return;
  }
  if (framesReceived == 0) {
    // Dead stream (module asleep, not yet detected, recovery pending):
    // the PVT watchdog owns this regime.
    m.remainderMilliFrames = 0;
    m.balance = 0;
    return;
  }

  const uint32_t expected = expectedFrames(m, elapsedMs, rateHz);
  m.balance += (int32_t)framesReceived - (int32_t)expected;
  if (m.balance < -kSlackFrames) {
    m.droppedTotal += (uint32_t)(-kSlackFrames - m.balance);
    m.balance = -kSlackFrames;
  } else if (m.balance > kMaxCreditFrames) {
    m.balance = kMaxCreditFrames;
  }
}

}  // namespace gps_stats
