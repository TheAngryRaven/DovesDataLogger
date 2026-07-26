#include "doctest.h"

#include <cstdint>

#include "gps_stats.h"

using namespace gps_stats;

// ---------------------------------------------------------------------------
// expectedFrames — fractional remainder carry
// ---------------------------------------------------------------------------

TEST_CASE("gps_stats - expectedFrames exact at whole-second windows") {
    DropMonitor m;
    CHECK(expectedFrames(m, 1000, 25) == 25);
    CHECK(m.remainderMilliFrames == 0);
    CHECK(expectedFrames(m, 1000, 5) == 5);
    CHECK(m.remainderMilliFrames == 0);
}

TEST_CASE("gps_stats - expectedFrames carries sub-frame remainder exactly") {
    DropMonitor m;
    // 1013 ms at 25 Hz = 25.325 frames per window. Over 4 windows the
    // remainders must add up to exactly one extra frame (25,25,25,26).
    CHECK(expectedFrames(m, 1013, 25) == 25);
    CHECK(m.remainderMilliFrames == 325);
    CHECK(expectedFrames(m, 1013, 25) == 25);
    CHECK(m.remainderMilliFrames == 650);
    CHECK(expectedFrames(m, 1013, 25) == 25);
    CHECK(m.remainderMilliFrames == 975);
    CHECK(expectedFrames(m, 1013, 25) == 26);
    CHECK(m.remainderMilliFrames == 300);
}

TEST_CASE("gps_stats - expectedFrames long-run total is exact") {
    DropMonitor m;
    // 100 windows of oddly-sized elapsed times must sum to exactly
    // rate * totalMs / 1000 with the remainder holding the fraction.
    uint32_t total = 0;
    uint32_t totalMs = 0;
    for (int i = 0; i < 100; i++) {
        uint32_t elapsed = 990 + (i % 21);  // 990..1010 ms
        totalMs += elapsed;
        total += expectedFrames(m, elapsed, 25);
    }
    CHECK(total == (25 * totalMs) / 1000);
    CHECK(m.remainderMilliFrames == (25 * totalMs) % 1000);
}

// ---------------------------------------------------------------------------
// windowUpdate — drop counting
// ---------------------------------------------------------------------------

TEST_CASE("gps_stats - steady exact stream counts zero drops") {
    DropMonitor m;
    for (int i = 0; i < 60; i++) {
        windowUpdate(m, 25, 1000, 25);
    }
    CHECK(m.droppedTotal == 0);
    CHECK(m.balance == 0);
}

TEST_CASE("gps_stats - boundary slip (24 then 26) is not a drop") {
    DropMonitor m;
    windowUpdate(m, 24, 1000, 25);
    CHECK(m.droppedTotal == 0);  // absorbed by slack
    CHECK(m.balance == -1);
    windowUpdate(m, 26, 1000, 25);
    CHECK(m.droppedTotal == 0);
    CHECK(m.balance == 0);
}

TEST_CASE("gps_stats - sustained one-frame-per-window loss counts") {
    DropMonitor m;
    // First deficit lands in the slack; every following one is real.
    for (int i = 0; i < 10; i++) {
        windowUpdate(m, 24, 1000, 25);
    }
    CHECK(m.droppedTotal == 9);
    CHECK(m.balance == -kSlackFrames);
}

TEST_CASE("gps_stats - burst loss counts immediately past the slack") {
    DropMonitor m;
    windowUpdate(m, 20, 1000, 25);  // 5 missing, 1 slack
    CHECK(m.droppedTotal == 4);
    CHECK(m.balance == -kSlackFrames);
    // Recovery to exact rate holds the total steady.
    for (int i = 0; i < 5; i++) {
        windowUpdate(m, 25, 1000, 25);
    }
    CHECK(m.droppedTotal == 4);
}

TEST_CASE("gps_stats - surplus credit is capped") {
    DropMonitor m;
    // A fast clock (or boundary phase) can't bank unlimited credit.
    for (int i = 0; i < 10; i++) {
        windowUpdate(m, 26, 1000, 25);
    }
    CHECK(m.balance == kMaxCreditFrames);
    CHECK(m.droppedTotal == 0);
    // A real burst is offset by at most the cap (+ slack).
    windowUpdate(m, 15, 1000, 25);  // 10 missing vs credit 2 + slack 1
    CHECK(m.droppedTotal == 7);
}

TEST_CASE("gps_stats - rate change suppresses one window and resets state") {
    DropMonitor m;
    windowUpdate(m, 24, 1000, 25);  // park some slack state
    CHECK(m.balance == -1);
    noteRateChange(m);
    // Mid-switch window would look like a huge deficit — discarded.
    windowUpdate(m, 13, 1000, 25);
    CHECK(m.droppedTotal == 0);
    CHECK(m.balance == 0);
    CHECK(m.remainderMilliFrames == 0);
    // Accounting resumes cleanly at the new rate.
    windowUpdate(m, 5, 1000, 5);
    CHECK(m.droppedTotal == 0);
    windowUpdate(m, 3, 1000, 5);  // 2 missing, 1 slack
    CHECK(m.droppedTotal == 1);
}

TEST_CASE("gps_stats - dead stream windows are not drops") {
    DropMonitor m;
    for (int i = 0; i < 30; i++) {
        windowUpdate(m, 0, 1000, 25);
    }
    CHECK(m.droppedTotal == 0);
    CHECK(m.balance == 0);
    CHECK(m.remainderMilliFrames == 0);
}

TEST_CASE("gps_stats - droppedTotal is monotonic") {
    DropMonitor m;
    uint32_t last = 0;
    // Mixed traffic: losses, surpluses, dead windows, rate changes.
    const uint32_t received[] = {25, 22, 26, 0, 25, 19, 25, 27, 24, 25};
    for (int pass = 0; pass < 3; pass++) {
        for (uint32_t r : received) {
            windowUpdate(m, r, 1000, 25);
            CHECK(m.droppedTotal >= last);
            last = m.droppedTotal;
        }
        noteRateChange(m);
        windowUpdate(m, 11, 1000, 25);
        CHECK(m.droppedTotal == last);  // suppressed window never counts
    }
}
