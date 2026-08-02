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

// Shutdown-side companion to the boot decode: decide the tach pin's
// parked idle level from a burst of samples taken right before System
// OFF entry, so SENSE can be armed for the OPPOSITE level. The tach
// pickup's output stage (Schmitt inverter + optocoupler) idles high or
// low depending on the circuit build, and runtime RPM counting cannot
// tell the two apart — a spark pulse yields exactly one falling edge
// either way. But arming SENSE toward the idle level satisfies DETECT
// immediately and System OFF wake-resets within a second (the
// battery-sleep instant-reboot bug). Majority vote over the samples;
// a tie or an empty burst resolves to idle-high — the pull-up's level
// on a floating/disconnected tach input, and the historical hard-coded
// assumption.
bool tachIdleIsHigh(unsigned highSamples, unsigned totalSamples);

}  // namespace wake_cause
