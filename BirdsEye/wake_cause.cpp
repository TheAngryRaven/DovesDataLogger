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

const char* shortName(Cause c) {
  switch (c) {
    case Cause::kColdBoot:       return "COLD";
    case Cause::kTachWake:       return "TACH";
    case Cause::kButtonWake:     return "BTN";
    case Cause::kUsbWake:        return "USB";
    case Cause::kWatchdog:       return "WDT";
    case Cause::kSoftReset:      return "SRQ";
    case Cause::kOffWakeUnknown: return "OFF?";
  }
  return "?";
}

PinArm armTach(bool restsHigh) {
  return restsHigh ? PinArm::kSenseLowPullUp : PinArm::kSenseHighPullDown;
}

PinArm armButton(bool restsHigh) {
  return restsHigh ? PinArm::kSenseLowPullUp : PinArm::kSkip;
}

bool armHoldsDetect(PinArm arm, bool pinIsHigh) {
  switch (arm) {
    case PinArm::kSenseLowPullUp:    return !pinIsHigh;
    case PinArm::kSenseHighPullDown: return pinIsHigh;
    case PinArm::kSkip:              break;
  }
  return false;
}

}  // namespace wake_cause
