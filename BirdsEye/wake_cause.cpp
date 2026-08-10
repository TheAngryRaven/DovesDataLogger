#include "wake_cause.h"

namespace wake_cause {

Cause decode(const Regs& regs, const PinMasks& pins) {
  if (regs.resetreas & kReasOff) {
    // GPIO DETECT wake from System OFF. LATCH says which pin(s); tach
    // wins over buttons so an engine start with a coincidental button
    // bounce still boots toward race mode.
    const bool tachLatched = (regs.latch0 & pins.tach0) || (regs.latch1 & pins.tach1);
    const bool buttonLatched =
        (regs.latch0 & pins.buttons0) || (regs.latch1 & pins.buttons1);
    if (tachLatched) return Cause::kTachWake;
    if (buttonLatched) return Cause::kButtonWake;
    return Cause::kOffWakeUnknown;
  }
  if (regs.resetreas & kReasVbus) return Cause::kUsbWake;
  if (regs.resetreas & kReasDog) return Cause::kWatchdog;
  if (regs.resetreas & kReasSreq) return Cause::kSoftReset;
  return Cause::kColdBoot;  // reset pin or power-on (RESETREAS empty)
}

bool tachIdleIsHigh(unsigned highSamples, unsigned totalSamples) {
  // Majority vote; ties (including 0 samples) fall to idle-high so a
  // floating input under the pull-up keeps the original SENSE-LOW arm.
  return highSamples * 2 >= totalSamples;
}

}  // namespace wake_cause
