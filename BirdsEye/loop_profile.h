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
// - TWO CLOCKS, and mixing them up was the first real bug in this unit.
//   Durations are measured in TICKS from whatever counter profiling.ino
//   probed (the DWT cycle counter at 64 ticks/us, or micros()), because
//   a sub-microsecond section must not be quantised to 0 before it is
//   summed. But the WINDOW is measured in milliseconds off millis(),
//   and every share is computed against that wall time.
//
//   The DWT counter counts CPU CYCLES, not time: if the core ever halts
//   (WFE/WFI in the FreeRTOS idle task, sd_app_evt_wait) it stops while
//   the world keeps going. Using it to close the window meant the
//   "one second" window was really one second of CPU-awake time, so the
//   loop rate came out multiplied by the sleep factor and every share
//   was a fraction of awake time wearing a wall-time label. The first
//   hardware run reported a rate pinned at the display clamp; this is
//   why. (The micros() fallback never had the bug — it is a real clock.)
//
//   The upside of fixing it properly: wall time minus loop() execution
//   time is now a MEASURED number (kSleep), so the question "does this
//   firmware have any CPU headroom at all" gets an answer instead of an
//   assumption.
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
// - Every reported slot is a share of the same wall-clock window, so
//   the sections plus kOther plus kSleep sum to ~1000 permille (integer
//   truncation loses a few). That is checkable on the display page at a
//   glance, and it is the property that makes a wrong reading obvious.
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
  kDisplay,    // displayLoop: page render + I2C framebuffer push, plus
               // the boot-page state machines (gpsStatusPageLoop,
               // sdFormatPageLoop, courseCreatorLoop) — same family of
               // work, and folding them freed the grid slot kSleep needed
  kSectionCount
};

// Derived report slots. Neither is a Section — nothing accumulates into
// them; rollup() computes both.
//   kOther: loop() time no section bracketed.
//   kSleep: wall time not inside loop() at all — scheduler dispatch,
//           other FreeRTOS tasks, and CPU sleep. THE headroom number.
constexpr uint8_t kOther = kSectionCount;
constexpr uint8_t kSleep = kSectionCount + 1;
constexpr uint8_t kReportSlots = kSectionCount + 2;

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
// still calls reset() at setup so both window anchors are set.
struct State {
  SectionStat sec[kSectionCount];
  uint32_t loops;           // completed iterations this window
  uint32_t loopTotalTicks;  // sum of whole-iteration spans
  uint32_t loopMaxTicks;    // worst iteration this window
  uint32_t windowStartTicks;  // duration clock (may stop when asleep)
  uint32_t windowStartMs;     // wall clock — this is what closes a window
};

// One window's rolled-up answer. Every permille is a share of the
// WALL-CLOCK window, so the sections plus kOther plus kSleep sum to
// ~1000 (integer truncation loses a few).
struct Report {
  bool valid;                       // false until the first rollup lands
  uint16_t permille[kReportSlots];  // per section, plus kOther and kSleep
  uint32_t maxUs[kSectionCount];    // worst single call, microseconds
  uint16_t busyPermille;            // wall time spent executing loop()
  uint32_t loops;                   // iterations in the window
  uint32_t loopRateHz;              // rounded iterations per WALL second
  uint32_t loopMeanUs;              // mean iteration execution time
  uint32_t loopMaxUs;               // worst iteration
  uint32_t windowUs;                // wall time the window covered
  uint32_t awakeUs;                 // duration-clock time in the same
                                    // window; well under windowUs means
                                    // the CPU was asleep (or the tick
                                    // counter stopped) for the balance
  bool saturated;                   // an accumulator pegged (see header)
};

// Wrap-safe tick difference. uint32 subtraction is already modular; this
// names the intent (and is where a narrower timebase would be handled).
inline uint32_t ticksSince(uint32_t from, uint32_t to) { return to - from; }

// part/whole as tenths of a percent, saturated at 1000 and safe on a
// zero whole.
uint16_t permilleOf(uint32_t part, uint32_t whole);

// Clear the accumulators and anchor a fresh window at both clocks.
void reset(State& s, uint32_t nowTicks, uint32_t nowMs);

// Fold one bracketed section span in.
void addSection(State& s, uint8_t section, uint32_t elapsedTicks);

// Fold one whole-iteration span in.
void addLoop(State& s, uint32_t elapsedTicks);

// If windowMs of WALL time has passed, fill `out`, restart the window
// and return true; otherwise leave `out` untouched and return false.
// The window is closed on nowMs specifically so a stopped duration clock
// (a sleeping CPU) cannot stretch it — see the header. ticksPerUs
// converts the tick totals to microseconds (0 is treated as 1 so a
// mis-probed timebase still reports ratios).
bool rollup(State& s, uint32_t nowTicks, uint32_t nowMs, uint32_t windowMs,
            uint32_t ticksPerUs, Report& out);

}  // namespace loop_profile
