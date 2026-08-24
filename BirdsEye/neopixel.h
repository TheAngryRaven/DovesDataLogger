#pragma once

#include <stdint.h>

// Arduino's auto-generated prototypes land near the top of the
// concatenated TU, and this module's internal npxPushFrame() takes a
// led_frame::Frame& — the type must therefore be visible from
// BirdsEye.ino's include block (which pulls this header), or the
// generated prototype fails to parse. Pure stdint header, safe
// everywhere. (See CLAUDE.md "Development Conventions" on the
// auto-prototype include-order trap.)
#include "led_frame.h"

///////////////////////////////////////////
// NEOPIXEL STRIP MODULE (plan 0006)
// 11 WS2812 pixels on the NFC pads (converted to GPIO): pixel 0 and
// pixel 10 are status indicators, pixels 1..9 are the strip with a
// centerline. A 5 V boost converter feeds the strip; its EN pin is
// driven so the rail is truly off in sleep (GPIO state is retained in
// System OFF — the "blue LED stays on" precedent).
//
// All decision math lives in the host-tested pure units (led_frame,
// led_modes, led_animations, sector_purple); this module is glue: it
// snapshots inputs each frame, composes by priority (boot animation >
// purple animation > parked/menu off > race mode + status actions),
// applies the ONE global brightness cap, and pushes the frame.
//
// Compiled out entirely unless BIRDSEYE_ENABLE_NEOPIXEL (project.h,
// beta channel) — a flag-off build never writes UICR and never drives
// these pins. First flag-on boot programs UICR->NFCPINS (one-way) and
// self-resets once; see NEOPIXEL_SETUP() and plan 0006.
//
// Adafruit_NeoPixel's nRF52 show() claims a free PWM instance
// (EasyDMA, interrupts ON, ~0.4 ms for 11 px) — this sketch uses no
// tone()/analogWrite(), so PWM0-2 are always free. If all PWMs were
// ever occupied the library falls back to a bit-bang WITH INTERRUPTS
// OFF — never let a future PWM user create that path. show() also
// mallocs/frees a ~560 B pattern buffer per call: same-size alloc/free
// is fragmentation-benign here, it is NOT a leak.
///////////////////////////////////////////

// Pins (Arduino numbering; the Seeed XIAO variant maps the NFC pads:
// 30 = P0.09 / NFC1, 31 = P0.10 / NFC2). The split between the two is
// arbitrary — swap these to match the actual wiring.
#define NEOPIXEL_PIN_BOOST_EN 30  // boost EN: HIGH = 5 V rail on, LOW = off
#define NEOPIXEL_PIN_DATA     31  // WS2812 data in (GRB, 800 kHz)

// BOOST EN IS NOT ALWAYS OURS. A profiling build
// (BIRDSEYE_ENABLE_PROFILING, plan 0011 — the beta channel) claims pin 30
// as its scope output, so this module stops driving EN entirely and the
// regulator runs at its hardware default (pulled up = rail on). The
// strip still works — only the ability to switch the rail is gone,
// including the driven-LOW that holds it down through System OFF. See
// the "BOOST EN OWNERSHIP" block in neopixel.ino and profiling.h.

// Runtime settings, read at boot in BirdsEye.ino's settings block.
extern uint8_t settingLedBrightness;  // global cap 0-255; 0 = LEDs disabled
extern int settingRevLimit;           // true RPM: scale ceiling + rev flasher
// Day/night swap (plan 0010). settingUtcOffsetMin is device-wide rather
// than LED-specific — it is extern'd here because the LED strip is its
// only consumer today; move it if a second one appears. The two hours
// are LOCAL wall clock; equal hours disable the swap.
extern int16_t settingUtcOffsetMin;
extern uint8_t settingLedBrightnessNight;
extern uint8_t settingLedDayStartHour;
extern uint8_t settingLedNightStartHour;

// One-time UICR NFC->GPIO ensure (may self-reset ONCE on the first
// flag-on boot — must run before the SoftDevice is enabled and before
// wdtSetup()), then boost rail up, strip init, boot animation armed.
void NEOPIXEL_SETUP();

// Self-throttled ~30 Hz frame: gather -> compose -> cap -> show. Called
// from the main loop AND from the BLE/USB parked branches (there it
// renders all-off — a frozen mid-pattern strip looks crashed).
void NEOPIXEL_LOOP();

// Shutdown hook (enterShutdown, next to the IMU rail-off): blank the
// strip while 5 V is still up, then data LOW, then boost EN LOW.
// Unconditional — driven-LOW is retained through System OFF.
void NEOPIXEL_SLEEP();

// Charging-loop soft resume: boost EN back up, strip re-initialized.
void NEOPIXEL_WAKE();

// Fire (or restart) the purple-sector celebration. The module also
// self-detects via sector_purple; this is the external hook.
void neopixelNotifyPurpleSector();
