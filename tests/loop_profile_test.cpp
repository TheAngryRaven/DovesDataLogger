#include "doctest.h"

#include <cstdint>
#include <cstring>

#include "loop_profile.h"

using namespace loop_profile;

namespace {

// The nRF52840's DWT cycle counter: 64 ticks per microsecond at a
// 64 MHz core clock. This is a DURATION clock, not a wall clock — the
// distinction is the point of half these tests.
constexpr uint32_t kTicksPerUs = 64;
constexpr uint32_t kWindowMs = 1000;
// Ticks that elapse in one wall second IF the CPU never halts.
constexpr uint32_t kWindow = 1000000u * kTicksPerUs;

// Drive `loops` identical iterations: each spends `sectionTicks` in
// `section` and `loopTicks` in the iteration as a whole.
void feed(State& s, uint32_t loops, uint8_t section, uint32_t sectionTicks,
          uint32_t loopTicks) {
    for (uint32_t i = 0; i < loops; i++) {
        addSection(s, section, sectionTicks);
        addLoop(s, loopTicks);
    }
}

// Sum of every reported slot. The unit's headline invariant: all
// fourteen are shares of the same wall-clock window.
uint32_t slotSum(const Report& r) {
    uint32_t total = 0;
    for (uint8_t i = 0; i < kReportSlots; i++) total += r.permille[i];
    return total;
}

}  // namespace

// ---------------------------------------------------------------------------
// permilleOf
// ---------------------------------------------------------------------------

TEST_CASE("loop_profile - permilleOf is tenths of a percent") {
    CHECK(permilleOf(1, 2) == 500);
    CHECK(permilleOf(412, 1000) == 412);
    CHECK(permilleOf(0, 1000) == 0);
}

TEST_CASE("loop_profile - permilleOf survives a zero window and saturates") {
    CHECK(permilleOf(100, 0) == 0);   // no divide by zero
    CHECK(permilleOf(2000, 1000) == 1000);  // never reports >100%
}

TEST_CASE("loop_profile - permilleOf does not overflow on full-scale ticks") {
    // part * 1000 overflows uint32 above ~4.3e6 ticks (67 ms at 64 MHz),
    // which every real window exceeds. 64-bit intermediate required.
    CHECK(permilleOf(kWindow / 2, kWindow) == 500);
    CHECK(permilleOf(UINT32_MAX, UINT32_MAX) == 1000);
}

// ---------------------------------------------------------------------------
// Accumulation
// ---------------------------------------------------------------------------

TEST_CASE("loop_profile - addSection accumulates total, max and calls") {
    State s;
    reset(s, 0, 0);
    addSection(s, kGps, 100);
    addSection(s, kGps, 900);
    addSection(s, kGps, 50);
    CHECK(s.sec[kGps].totalTicks == 1050);
    CHECK(s.sec[kGps].maxTicks == 900);
    CHECK(s.sec[kGps].calls == 3);
    CHECK(s.sec[kDisplay].calls == 0);
}

TEST_CASE("loop_profile - addSection ignores an out-of-range section") {
    State s;
    reset(s, 0, 0);
    addSection(s, kSectionCount, 1000);  // kOther is a report slot only
    addSection(s, 200, 1000);
    for (uint8_t i = 0; i < kSectionCount; i++) {
        CHECK(s.sec[i].totalTicks == 0);
    }
}

TEST_CASE("loop_profile - accumulators saturate instead of wrapping") {
    State s;
    reset(s, 0, 0);
    addSection(s, kGps, UINT32_MAX - 10);
    addSection(s, kGps, 1000);
    CHECK(s.sec[kGps].totalTicks == UINT32_MAX);
    addLoop(s, UINT32_MAX);
    addLoop(s, 5);
    CHECK(s.loopTotalTicks == UINT32_MAX);
}

// ---------------------------------------------------------------------------
// rollup — windowing
// ---------------------------------------------------------------------------

TEST_CASE("loop_profile - rollup withholds an incomplete window") {
    State s;
    Report r;
    r.valid = false;
    reset(s, 1000, 0);
    feed(s, 10, kGps, 100, 200);
    CHECK(rollup(s, 1000 + kWindow - 1, kWindowMs - 1, kWindowMs, kTicksPerUs, r) == false);
    CHECK(r.valid == false);       // untouched
    CHECK(s.sec[kGps].calls == 10);  // accumulators kept accruing
}

TEST_CASE("loop_profile - rollup restarts the window and clears state") {
    State s;
    Report r;
    reset(s, 0, 0);
    feed(s, 250, kGps, 1000, 4000);
    CHECK(rollup(s, kWindow, kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(s.windowStartTicks == kWindow);
    CHECK(s.windowStartMs == kWindowMs);
    CHECK(s.loops == 0);
    CHECK(s.sec[kGps].totalTicks == 0);
    CHECK(s.sec[kGps].maxTicks == 0);
}

TEST_CASE("loop_profile - rollup is wrap-safe across the tick counter") {
    // DWT->CYCCNT wraps every ~67 s at 64 MHz, mid-session every time.
    State s;
    Report r;
    const uint32_t start = UINT32_MAX - (kWindow / 2);
    reset(s, start, 0);
    feed(s, 100, kGps, kWindow / 1000, kWindow / 500);
    const uint32_t now = start + kWindow;  // wraps
    CHECK(now < start);
    CHECK(rollup(s, now, kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.windowUs == 1000000u);
    CHECK(r.loops == 100);
}

TEST_CASE("loop_profile - rollup is wrap-safe across the millis() clock") {
    // millis() wraps every ~49 days. Rare, but a wrapped window would
    // either never close or close instantly forever.
    State s;
    Report r;
    const uint32_t startMs = UINT32_MAX - 400u;
    reset(s, 0, startMs);
    feed(s, 250, kGps, 1600 * kTicksPerUs, 4000 * kTicksPerUs);
    const uint32_t nowMs = startMs + kWindowMs;  // wraps
    CHECK(nowMs < startMs);
    REQUIRE(rollup(s, kWindow, nowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.windowUs == 1000000u);
    CHECK(r.loopRateHz == 250);
}

// ---------------------------------------------------------------------------
// The two clocks — the bug this unit was rebuilt around
// ---------------------------------------------------------------------------

TEST_CASE("loop_profile - a stopped duration clock cannot stretch the window") {
    // THE REGRESSION TEST. The DWT cycle counter stops when the core
    // halts, so if it were the thing closing the window, a CPU that is
    // awake 25% of the time would take FOUR wall seconds to accumulate
    // "one second" of ticks — and 1000 iterations over those 4 s would
    // be reported as 1000 Hz instead of 250 Hz. The window must close on
    // wall time regardless of what the tick counter did.
    State s;
    Report r;
    reset(s, 0, 0);
    for (int i = 0; i < 1000; i++) {
        addSection(s, kGps, 100 * kTicksPerUs);
        addLoop(s, 250 * kTicksPerUs);  // 250 us of execution per iteration
    }
    // One wall second elapsed; only 250 ms of it was spent awake.
    const uint32_t awakeTicks = 250000u * kTicksPerUs;
    REQUIRE(rollup(s, awakeTicks, kWindowMs, kWindowMs, kTicksPerUs, r) == true);

    CHECK(r.windowUs == 1000000u);       // wall, not ticks
    CHECK(r.awakeUs == 250000u);         // and the discrepancy is reported
    CHECK(r.loopRateHz == 1000);         // 1000 iterations per WALL second
    CHECK(r.loopMeanUs == 250);          // execution time per iteration
    CHECK(r.busyPermille == 250);        // the CPU ran loop() 25% of the time
    CHECK(r.permille[kSleep] == 750);    // ...and did something else for 75%
    CHECK(r.permille[kGps] == 100);      // 100 ms of 1000 ms, not of 250 ms
}

TEST_CASE("loop_profile - awakeUs equals the window when nothing sleeps") {
    State s;
    Report r;
    reset(s, 0, 0);
    feed(s, 250, kGps, 1600 * kTicksPerUs, 4000 * kTicksPerUs);
    REQUIRE(rollup(s, kWindow, kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.awakeUs == r.windowUs);
    CHECK(r.busyPermille == 1000);
    CHECK(r.permille[kSleep] == 0);
}

TEST_CASE("loop_profile - every reported slot sums to the whole window") {
    // The invariant a reader can check on the display page at a glance.
    State s;
    Report r;
    reset(s, 0, 0);
    for (int i = 0; i < 500; i++) {
        addSection(s, kGps, 200 * kTicksPerUs);
        addSection(s, kDisplay, 150 * kTicksPerUs);
        addSection(s, kButtons, 80 * kTicksPerUs);
        addLoop(s, 600 * kTicksPerUs);  // 170 us of unbracketed glue
    }
    REQUIRE(rollup(s, kWindow, kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.permille[kGps] == 100);
    CHECK(r.permille[kDisplay] == 75);
    CHECK(r.permille[kButtons] == 40);
    CHECK(r.permille[kOther] == 85);   // 170 us x 500 = 85 ms
    CHECK(r.permille[kSleep] == 700);  // busy 300, so 700 elsewhere
    CHECK(slotSum(r) == 1000);
}

TEST_CASE("loop_profile - busy over 100% cannot make kSleep negative") {
    // Preemption and separate clock reads can push the loop total a hair
    // past the window. kSleep is unsigned; it must clamp, not wrap.
    State s;
    Report r;
    reset(s, 0, 0);
    addLoop(s, kWindow + (1000u * kTicksPerUs));
    REQUIRE(rollup(s, kWindow, kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.busyPermille == 1000);
    CHECK(r.permille[kSleep] == 0);
}

// ---------------------------------------------------------------------------
// rollup — the reported numbers
// ---------------------------------------------------------------------------

TEST_CASE("loop_profile - shares are of window wall time") {
    State s;
    Report r;
    reset(s, 0, 0);
    // 250 iterations of 4 ms; 40% of each iteration in GPS, 10% in LED.
    for (int i = 0; i < 250; i++) {
        addSection(s, kGps, 1600 * kTicksPerUs);
        addSection(s, kLed, 400 * kTicksPerUs);
        addLoop(s, 4000 * kTicksPerUs);
    }
    REQUIRE(rollup(s, kWindow, kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.permille[kGps] == 400);
    CHECK(r.permille[kLed] == 100);
    CHECK(r.permille[kTach] == 0);
    CHECK(r.loops == 250);
    CHECK(r.loopRateHz == 250);
    CHECK(r.loopMeanUs == 4000);
    CHECK(r.loopMaxUs == 4000);
    CHECK(r.windowUs == 1000000u);
    CHECK(r.saturated == false);
}

TEST_CASE("loop_profile - unbracketed loop time lands in kOther") {
    State s;
    Report r;
    reset(s, 0, 0);
    // Half of each iteration is inside GPS; the rest is loop glue.
    for (int i = 0; i < 100; i++) {
        addSection(s, kGps, 5000 * kTicksPerUs);
        addLoop(s, 10000 * kTicksPerUs);
    }
    REQUIRE(rollup(s, kWindow, kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.permille[kGps] == 500);
    CHECK(r.permille[kOther] == 500);
    CHECK(r.permille[kSleep] == 0);
}

TEST_CASE("loop_profile - kOther cannot go negative when spans disagree") {
    // The whole-iteration span and the section spans are separate reads
    // and can disagree by a tick or two; that must not underflow.
    State s;
    Report r;
    reset(s, 0, 0);
    addSection(s, kGps, 1000 * kTicksPerUs);
    addSection(s, kLed, 1000 * kTicksPerUs);
    addLoop(s, 1990 * kTicksPerUs);  // less than the sum of its parts
    REQUIRE(rollup(s, kWindow, kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.permille[kOther] == 0);
}

TEST_CASE("loop_profile - time outside loop() is reported, not hidden") {
    State s;
    Report r;
    reset(s, 0, 0);
    // Only a quarter of the window is spent inside loop() at all.
    for (int i = 0; i < 250; i++) {
        addSection(s, kGps, 500 * kTicksPerUs);
        addLoop(s, 1000 * kTicksPerUs);
    }
    REQUIRE(rollup(s, kWindow, kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.permille[kGps] == 125);
    CHECK(r.permille[kOther] == 125);
    CHECK(r.permille[kSleep] == 750);
}

TEST_CASE("loop_profile - per-section max survives as microseconds") {
    State s;
    Report r;
    reset(s, 0, 0);
    // One SD garbage-collection stall inside the GPS section.
    addSection(s, kGps, 300 * kTicksPerUs);
    addSection(s, kGps, 1800000u * kTicksPerUs);  // 1.8 s
    addLoop(s, 1800400u * kTicksPerUs);
    REQUIRE(rollup(s, 2u * kWindow, 2u * kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.maxUs[kGps] == 1800000u);
    CHECK(r.loopMaxUs == 1800400u);
    CHECK(r.windowUs == 2000000u);
    CHECK(r.loopRateHz == 1);  // 1 iteration in a 2 s window rounds to 1
}

TEST_CASE("loop_profile - a pegged accumulator is flagged, not silently wrong") {
    State s;
    Report r;
    reset(s, 0, 0);
    addSection(s, kGps, UINT32_MAX);
    addSection(s, kGps, 1);
    addLoop(s, UINT32_MAX);
    REQUIRE(rollup(s, kWindow, kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.saturated == true);
    CHECK(r.permille[kGps] == 1000);
}

TEST_CASE("loop_profile - an empty window reports zeros, not garbage") {
    State s;
    Report r;
    reset(s, 0, 0);
    REQUIRE(rollup(s, kWindow, kWindowMs, kWindowMs, kTicksPerUs, r) == true);
    CHECK(r.valid == true);
    CHECK(r.loops == 0);
    CHECK(r.loopRateHz == 0);
    CHECK(r.loopMeanUs == 0);
    CHECK(r.permille[kGps] == 0);
    CHECK(r.permille[kSleep] == 1000);
}

TEST_CASE("loop_profile - a micros() timebase reports the same ratios") {
    // ticksPerUs 1 is the fallback when DWT will not run. Shares must be
    // identical; only the resolution of the microsecond fields changes.
    State s;
    Report r;
    reset(s, 0, 0);
    for (int i = 0; i < 250; i++) {
        addSection(s, kGps, 1600);
        addLoop(s, 4000);
    }
    REQUIRE(rollup(s, 1000000u, kWindowMs, kWindowMs, 1, r) == true);
    CHECK(r.permille[kGps] == 400);
    CHECK(r.loopMeanUs == 4000);
    CHECK(r.loopRateHz == 250);
}

TEST_CASE("loop_profile - ticksPerUs 0 still yields ratios") {
    // A mis-probed timebase is treated as 1 tick/us rather than dividing
    // by zero, so the ticks are read as microseconds and the shares stay
    // meaningful even though the resolution is gone.
    State s;
    Report r;
    reset(s, 0, 0);
    addSection(s, kGps, 500000);
    addLoop(s, 1000000);
    REQUIRE(rollup(s, 1000000, kWindowMs, kWindowMs, 0, r) == true);
    CHECK(r.permille[kGps] == 500);
    CHECK(r.busyPermille == 1000);
}

// ---------------------------------------------------------------------------
// Tags
// ---------------------------------------------------------------------------

TEST_CASE("loop_profile - every section has a distinct three-char tag") {
    for (uint8_t i = 0; i < kSectionCount; i++) {
        const char* tag = sectionTag(i);
        CHECK(std::strlen(tag) == 3);
        CHECK(std::strcmp(tag, "???") != 0);
        for (uint8_t j = 0; j < i; j++) {
            CHECK(std::strcmp(tag, sectionTag(j)) != 0);
        }
    }
    CHECK(std::strcmp(sectionTag(kOther), "OTH") == 0);
    CHECK(std::strcmp(sectionTag(kSleep), "SLP") == 0);
    CHECK(std::strcmp(sectionTag(200), "???") == 0);
}
