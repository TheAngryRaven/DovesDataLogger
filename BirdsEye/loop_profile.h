#pragma once

#include <stdint.h>

///////////////////////////////////////////
// MAIN-LOOP CPU PROFILE (pure logic, host-tested)
//
// Accounting for where one main-loop iteration's time goes. The glue
// (profiling.ino) brackets each subsystem call in loop() and hands the
// elapsed span here; every window (1 s) this rolls the accumulators up
// into a Report the debug page renders and the profiling pin is timed
// against.
//
// WHY THIS EXISTS: the firmware is a superloop with no idle task, so
// "CPU usage" is not a duty cycle — the CPU is busy 100% of the time by
// construction. The number that actually answers "how much CPU are we
// using" is therefore the SHAPE of an iteration: how long it takes, how
// that time divides between subsystems, and how bad the worst iteration
// is. That is what feeds the nRF52840-vs-nRF5340 decision (plan 0011).
//
// Design notes:
// - TICKS, not microseconds. The caller picks the timebase and passes
//   its ticksPerUs at rollup; ratios are computed from raw ticks so a
//   sub-microsecond section is not quantised to 0 before it is summed.
//   profiling.ino prefers the DWT cycle counter (64 ticks/us) and falls
//   back to micros() (1 tick/us) when DWT will not run.
// - Shares are permille (tenths of a percent), integer math throughout.
//   A float here would be fine on this M4F, but the exactness of the
//   "everything adds to 1000" property is worth more than the syntax.
// - Accumulators SATURATE rather than wrap. At 64 MHz a uint32 of ticks
//   is only ~67 s, and a rollup can be arbitrarily late if the loop
//   stalls (SD garbage collection can block 100 ms-2 s). A saturated
//   window reads as "pegged", which is true; a wrapped one reads as
//   near-zero, which is a lie.
// - The window's unaccounted remainder is reported (kOther) instead of
//   being hidden. It is the honesty check on the instrumentation: if
//   OTH is large, time is going somewhere loop() is not bracketing.
///////////////////////////////////////////

namespace loop_profile {

// The instrumented spans of one loop() iteration, in call order. Adding
// one means adding its tag to sectionTag() and bracketing it in loop().
enum Section : uint8_t {
  kGps = 0,    // GPS_LOOP: UBX drain, PVT callbacks, DOVEX row write
  kTach,       // TACH_LOOP: ring drain + Kalman
  kAccel,      // ACCEL_LOOP: LSM6DS3 I2C reads
  kBle,        // BLUETOOTH_LOOP: deferred SD commands, transfer streaming
  kEgg,        // SENSOREGG_LOOP: scan buffer drain + parse
  kTrack,      // trackDetectionLoop: haversine manifest scan / JSON parse
  kLap,        // checkForNewLapData
  kIdle,       // checkAutoIdle + autoRaceModeCheck + updateGpsLockHold
  kCamera,     // CAMERA_LOOP: Insta360 FSM step + GPS overlay stream
  kLed,        // NEOPIXEL_LOOP: compose + cap + show
  kButtons,    // readButtons + updateButtonHoldState (multi-sample
               // debounce; near-free unless a button is actually held)
  kPages,      // gpsStatusPageLoop + sdFormatPageLoop + courseCreatorLoop
  kDisplay,    // displayLoop: page render + I2C framebuffer push
  kSectionCount
};

// Report slot for loop time not inside any bracketed section. Not a
// Section — nothing accumulates into it; rollup() derives it.
constexpr uint8_t kOther = kSectionCount;
constexpr uint8_t kReportSlots = kSectionCount + 1;

// Three-character tag for the debug page's two-column grid. Returns
// "???" out of range so a mismatched enum shows up on screen instead
// of reading past the table.
const char* sectionTag(uint8_t section);

struct SectionStat {
  uint32_t totalTicks;  // saturating
  uint32_t maxTicks;    // worst single call this window
  uint32_t calls;       // saturating
};

// Live accumulators. Zero-initialised is a valid empty window; the glue
// still calls reset() at setup so windowStartTicks is anchored.
struct State {
  SectionStat sec[kSectionCount];
  uint32_t loops;           // completed iterations this window
  uint32_t loopTotalTicks;  // sum of whole-iteration spans
  uint32_t loopMaxTicks;    // worst iteration this window
  uint32_t windowStartTicks;
};

// One window's rolled-up answer. All *Permille are of the WINDOW's wall
// time, so section shares plus kOther plus outsideLoopPermille sum to
// ~1000 (integer truncation loses a few).
struct Report {
  bool valid;                      // false until the first rollup lands
  uint16_t permille[kReportSlots];  // per section, plus kOther
  uint32_t maxUs[kSectionCount];    // worst single call, microseconds
  uint16_t outsideLoopPermille;    // window time between iterations
  uint32_t loops;                  // iterations in the window
  uint32_t loopRateHz;             // rounded iterations per second
  uint32_t loopMeanUs;             // mean iteration length
  uint32_t loopMaxUs;              // worst iteration
  uint32_t windowUs;               // the window actually measured
  bool saturated;                  // an accumulator pegged (see header)
};

// Wrap-safe tick difference. uint32 subtraction is already modular; this
// names the intent (and is where a narrower timebase would be handled).
inline uint32_t ticksSince(uint32_t from, uint32_t to) { return to - from; }

// part/whole as tenths of a percent, saturated at 1000 and safe on a
// zero whole.
uint16_t permilleOf(uint32_t part, uint32_t whole);

// Clear the accumulators and anchor a fresh window at nowTicks.
void reset(State& s, uint32_t nowTicks);

// Fold one bracketed section span in.
void addSection(State& s, uint8_t section, uint32_t elapsedTicks);

// Fold one whole-iteration span in.
void addLoop(State& s, uint32_t elapsedTicks);

// If the window is complete, fill `out`, restart the window at nowTicks
// and return true; otherwise leave `out` untouched and return false.
// ticksPerUs converts the tick totals to the microsecond fields (0 is
// treated as 1 so a mis-probed timebase still reports ratios).
bool rollup(State& s, uint32_t nowTicks, uint32_t windowTicks,
            uint32_t ticksPerUs, Report& out);

}  // namespace loop_profile
