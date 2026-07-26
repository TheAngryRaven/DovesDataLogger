#pragma once

#include <stdint.h>

///////////////////////////////////////////
// BOOT WAKE-CAUSE DECODER
// Every boot is now a cold start (sleep = nRF52 System OFF, wake = full
// reset), so the only record of *why* we booted is what the hardware
// left behind: NRF_POWER->RESETREAS plus the per-pin GPIO LATCH
// registers (both sticky across System OFF, write-1-to-clear). This
// unit turns those raw register snapshots into a wake cause the sketch
// can route on (tach wake -> race exit from the GPS status page, USB
// wake -> charging loop, everything else -> normal boot).
//
// GPREGRET is deliberately NOT used: register 0 is owned by the
// firmware-OTA/bootloader handoff (0xA8 = enter BLE DFU).
//
// Pure logic — no Arduino/MDK headers — so it is exercised by host
// tests. Register bit positions are mirrored here from the nRF52840
// product spec for the same reason.
///////////////////////////////////////////

namespace wake_cause {

// NRF_POWER->RESETREAS bits (nRF52840 PS v1.8, POWER chapter).
constexpr uint32_t kReasResetPin = 1u << 0;   // reset pin
constexpr uint32_t kReasDog      = 1u << 1;   // watchdog
constexpr uint32_t kReasSreq     = 1u << 2;   // soft reset (NVIC_SystemReset)
constexpr uint32_t kReasOff      = 1u << 16;  // GPIO DETECT wake from System OFF
constexpr uint32_t kReasVbus     = 1u << 20;  // VBUS wake from System OFF

enum class Cause : uint8_t {
  kColdBoot,        // power applied / reset pin / nothing recorded
  kTachWake,        // System OFF wake, tach pin latched (engine started)
  kButtonWake,      // System OFF wake, a button pin latched
  kUsbWake,         // System OFF wake by VBUS (cable plugged in)
  kWatchdog,        // WDT expiry
  kSoftReset,       // NVIC_SystemReset (reboot combo, BLE teardown, OTA)
  kOffWakeUnknown,  // System OFF wake but no known pin latched
};

// Snapshot of the sticky boot registers, read (then cleared) first
// thing in setup().
struct Regs {
  uint32_t resetreas;
  uint32_t latch0;  // NRF_P0->LATCH
  uint32_t latch1;  // NRF_P1->LATCH
};

// Which physical port bits belong to the wake pins. Built at runtime
// from the board variant's pin map so raw P-numbers never get
// hardcoded.
struct PinMasks {
  uint32_t tach0;     // tach pin bit if on port 0, else 0
  uint32_t tach1;     // tach pin bit if on port 1, else 0
  uint32_t buttons0;  // OR of button pin bits on port 0
  uint32_t buttons1;  // OR of button pin bits on port 1
};

// Decode priority: a System OFF GPIO wake outranks everything (the OFF
// bit can only be set by the wake we armed); within it the tach outranks
// buttons (an engine start must never be misread as a button press).
// VBUS is next, then watchdog, then soft reset, then cold boot.
Cause decode(const Regs& regs, const PinMasks& pins);

// Short (<= 4 char) label for the boot screen. "Why did I just boot?" is
// otherwise invisible, and the answer separates the failure modes that
// all look identical from the outside — a shutdown that woke itself back
// up (kTachWake / kButtonWake / kOffWakeUnknown), a watchdog reset
// (kWatchdog), and a deliberate reboot (kSoftReset).
const char* shortName(Cause c);

///////////////////////////////////////////
// SYSTEM OFF WAKE-PIN ARMING (the mirror of decode(), above)
//
// DETECT is the OR of every SENSE-enabled pin's sense condition, and the
// nRF52840 turns "DETECT already high when System OFF is entered" into an
// immediate wake — which is a chip reset. So arming a wake pin has to
// follow the level the pin actually rests at; it can never assume a
// polarity from how the pin is wired.
//
// This is what broke sleep in the field. The shipped tachometer board
// (TACHOMETER/paid_schematic_1.PDF) ends in an SN74LVC1G14 Schmitt
// INVERTER whose input is held high by a 10K pull-up, so its PULSE_RPM
// output rests LOW and pulses HIGH on each spark — the firmware's
// falling-edge ISR counts the trailing edge. Arming that pin SENSE_LOW
// (right for an open-collector pickup, wrong for this one) left DETECT
// permanently asserted, so every shutdown became an instant wake-reset:
// the logger "rebooted itself" instead of sleeping, every time, on any
// unit with the tach board plugged in. A bench unit with nothing on the
// tach header rests high on the internal pull-up and slept fine, which
// is why it never showed up off-kart.
///////////////////////////////////////////

enum class PinArm : uint8_t {
  kSkip,               // leave SENSE off — this pin is not a wake source
  kSenseLowPullUp,     // rests high -> wake on the fall
  kSenseHighPullDown,  // rests low  -> wake on the rise
};

// Tachometer: the polarity belongs to whichever pickup board is wired up,
// so wake on a change AWAY from the observed resting level. Both families
// work — open-collector/optocoupler (rests high) and the push-pull
// Schmitt output (rests low).
PinArm armTach(bool restsHigh);

// Buttons: always active-low (INPUT_PULLUP + switch to ground), so a pin
// resting low is a stuck or still-held button, never inverted wiring.
// Arming it would hold DETECT high and cost us System OFF altogether;
// skipping it costs only that one button as a wake source for this sleep.
PinArm armButton(bool restsHigh);

// Would this arm assert DETECT with the pin at the given level? Callers
// re-read every pin after arming — RC filters need time to follow a pull
// change, and a line can move — and disarm whatever answers true.
bool armHoldsDetect(PinArm arm, bool pinIsHigh);

}  // namespace wake_cause
