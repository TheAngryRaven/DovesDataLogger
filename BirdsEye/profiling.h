#pragma once

#include <stdint.h>

#include "loop_profile.h"
#include "project.h"

///////////////////////////////////////////
// LOOP PROFILING (plan 0011) — BETA CHANNEL ONLY
//
// Answers one question: where does a main-loop iteration's time go, and
// how much of it is left? That is the input to the nRF52840-vs-nRF5340
// decision for the commercial board, and to the "what would porting off
// the Arduino core buy us" question — both need a baseline measured on
// the hardware that exists, not an estimate.
//
// Compiled out entirely unless BIRDSEYE_ENABLE_PROFILING (project.h).
// The beta workflow passes it; master and release never do.
//
// TWO INSTRUMENTS, ONE SUBSYSTEM:
//
//  1. THE PIN. Pin 30 — the NeoPixel boost converter's EN line in a
//     normal build — is driven HIGH for the span being profiled and LOW
//     outside it. A scope or logic analyser then reads the loop period
//     off the rising edges and the span's cost off the high time, with
//     no software in the measurement path. PROFILING_PIN_SECTION picks
//     the span: the whole loop body by default, or any single section
//     (-DPROFILING_PIN_SECTION=PROF_SEC_GPS and friends, below).
//
//     THE PIN COSTS THE 5 V RAIL. A profiling build never drives EN —
//     not at setup, not at sleep — so the boost converter sits at its
//     hardware default (EN pulled up = rail ON) and the firmware can no
//     longer switch it. Consequences, both deliberate:
//       - The strip stays powered through System OFF, so a profiling
//         unit left asleep on a battery drains it. Bench tool; do not
//         ship one to a driver.
//       - If pin 30 is still physically wired to EN on the rig, the
//         toggling chops the rail at loop rate and the LEDs will
//         misbehave. Pull that jumper (or tie EN high) before profiling
//         — the whole premise here is that the regulator is on by
//         default and only *use*, not *testing*, needs firmware control
//         of it.
//
//  2. THE ROLLUP. Every section is also timed in software and rolled up
//     once a second into a loop_profile::Report, rendered on the
//     LOOP PROFILE race page (first page of the rotation on a profiling
//     build). That is the instrument that says which subsystem owns the
//     iteration; the pin is the one that says the software is telling
//     the truth.
//
// TIMEBASE: the DWT cycle counter (64 ticks/us at 64 MHz) when it will
// run, micros() when it will not — most sections are well under a
// microsecond, so micros() alone would quantise half of them to zero.
// The page shows which one is live; trust sub-microsecond figures only
// under DWT.
//
// OVERHEAD: a section bracket is two counter reads plus a saturating
// add — order 30 cycles, ~0.5 us per iteration across all 13 sections,
// under 0.1% of a 4 ms loop. It is inside the numbers it reports (it
// lands in the bracketed section, not in OTH), which is the right place
// for it: what you read is what the instrumented firmware actually
// costs.
///////////////////////////////////////////

// Sentinel for PROFILING_PIN_SECTION: mark the whole loop body rather
// than one section. Chosen so it can never collide with a Section id.
#define PROFILING_PIN_WHOLE_LOOP 255

// Preprocessor mirror of loop_profile::Section. It has to be macros, not
// the enum: PROFILING_PIN_SECTION is tested with #if (so the pin edges
// compile out of the hot path when the pin is marking the whole loop),
// and the preprocessor cannot evaluate a scoped C++ name. The
// static_asserts below make the mirror impossible to get wrong.
#define PROF_SEC_GPS 0
#define PROF_SEC_TACH 1
#define PROF_SEC_ACCEL 2
#define PROF_SEC_BLE 3
#define PROF_SEC_EGG 4
#define PROF_SEC_TRACK 5
#define PROF_SEC_LAP 6
#define PROF_SEC_IDLE 7
#define PROF_SEC_CAMERA 8
#define PROF_SEC_LED 9
#define PROF_SEC_BUTTONS 10
#define PROF_SEC_PAGES 11
#define PROF_SEC_DISPLAY 12

static_assert(PROF_SEC_GPS == loop_profile::kGps, "section mirror drift");
static_assert(PROF_SEC_TACH == loop_profile::kTach, "section mirror drift");
static_assert(PROF_SEC_ACCEL == loop_profile::kAccel, "section mirror drift");
static_assert(PROF_SEC_BLE == loop_profile::kBle, "section mirror drift");
static_assert(PROF_SEC_EGG == loop_profile::kEgg, "section mirror drift");
static_assert(PROF_SEC_TRACK == loop_profile::kTrack, "section mirror drift");
static_assert(PROF_SEC_LAP == loop_profile::kLap, "section mirror drift");
static_assert(PROF_SEC_IDLE == loop_profile::kIdle, "section mirror drift");
static_assert(PROF_SEC_CAMERA == loop_profile::kCamera, "section mirror drift");
static_assert(PROF_SEC_LED == loop_profile::kLed, "section mirror drift");
static_assert(PROF_SEC_BUTTONS == loop_profile::kButtons,
              "section mirror drift");
static_assert(PROF_SEC_PAGES == loop_profile::kPages, "section mirror drift");
static_assert(PROF_SEC_DISPLAY == loop_profile::kDisplay,
              "section mirror drift");
static_assert(PROFILING_PIN_WHOLE_LOOP > loop_profile::kSectionCount,
              "the whole-loop sentinel must not collide with a section id");

// Which span the pin marks. Default is the whole iteration: rising-edge
// period = loop period, and the (tiny) low time is the Arduino core's
// loop dispatch. Re-point it at a single subsystem with e.g.
// -DPROFILING_PIN_SECTION=PROF_SEC_GPS.
#ifndef PROFILING_PIN_SECTION
  #define PROFILING_PIN_SECTION PROFILING_PIN_WHOLE_LOOP
#endif

// The pin itself. Defaults to the boost EN pad (see the warning above);
// override if the rig has it wired elsewhere. NFC pads 30/31 only work
// as GPIO once the one-way UICR conversion has happened — that is
// NEOPIXEL_SETUP's job, so on a board that has never run a NeoPixel
// build the pin is refused at setup and only the rollup runs.
#ifndef PROFILING_PIN
  #define PROFILING_PIN 30
#endif

// Rollup window. One second matches the GPS frame-rate window and is
// slow enough that the page is readable while driving.
#ifndef PROFILING_WINDOW_MS
  #define PROFILING_WINDOW_MS 1000UL
#endif

#if BIRDSEYE_ENABLE_PROFILING

// Probe the timebase, claim the pin (if the pads are GPIO), anchor the
// first window. Call AFTER NEOPIXEL_SETUP() — that is what converts the
// NFC pads and it may self-reset the chip on the way.
void PROFILING_SETUP();

// Park the pin LOW. Called from enterShutdown() before System OFF: the
// loop scope guard's destructor never runs on that path, so without
// this the pin would be latched HIGH for the whole power-down.
void PROFILING_SLEEP();

// Section brackets. Not re-entrant per section (each keeps one start
// stamp), which is all loop() needs — nothing here nests.
void profSectionBegin(uint8_t section);
void profSectionEnd(uint8_t section);

// Last completed window. `.valid` is false until the first one lands.
const loop_profile::Report& profilingReport();

// "DWT" or "us" — which timebase produced that report.
const char* profilingTimebaseTag();

// False when the pin was refused (pads still NFC): the rollup is live
// but the scope will see nothing.
bool profilingPinLive();

// Whole-iteration timing + the once-a-second rollup, as a scope guard so
// every early return out of loop() (BLE parked, USB parked, shutdown)
// is still counted. Declared at the top of loop() via the macro below.
class ProfLoopScope {
 public:
  ProfLoopScope();
  ~ProfLoopScope();

 private:
  uint32_t startTicks_;
};

#define PROFILE_LOOP_SCOPE() ProfLoopScope profLoopScope_
#define PROFILE_SECTION(sec, ...)  \
  do {                             \
    profSectionBegin(sec);         \
    __VA_ARGS__;                   \
    profSectionEnd(sec);           \
  } while (0)

#else  // !BIRDSEYE_ENABLE_PROFILING

// Real functions rather than empty macros so the call sites in setup()
// and enterShutdown() read the same on every channel (the neopixel
// precedent). The section brackets DO vanish — they are in the hottest
// path in the firmware and must cost literally nothing when off.
void PROFILING_SETUP();
void PROFILING_SLEEP();

#define PROFILE_LOOP_SCOPE() do { } while (0)
#define PROFILE_SECTION(sec, ...) \
  do {                            \
    __VA_ARGS__;                  \
  } while (0)

#endif  // BIRDSEYE_ENABLE_PROFILING
