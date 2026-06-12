#include "doctest.h"
#include "lap_format.h"

#include <string>

using lap_format::formatLapTime;
using lap_format::kLapTimeStrLen;
using lap_format::kOmit;
using lap_format::kShow;
using lap_format::kSpace;

static std::string fmt(unsigned long ms, lap_format::ZeroMinutesStyle style) {
    char buf[kLapTimeStrLen];
    formatLapTime(ms, style, buf, sizeof(buf));
    return std::string(buf);
}

// ---------------------------------------------------------------------------
// The H-1 regression: milliseconds must be zero-padded BEFORE the value,
// never after. 1:23.007 used to render as 1:23.700 — a 693 ms error.
// ---------------------------------------------------------------------------

TEST_CASE("formatLapTime - small millisecond values are left-zero-padded") {
    CHECK(fmt(83007, kOmit) == "1:23.007");   // the review's exact case
    CHECK(fmt(83007, kShow) == "1:23.007");
    CHECK(fmt(83007, kSpace) == "1:23.007");

    CHECK(fmt(83070, kOmit) == "1:23.070");
    CHECK(fmt(83700, kOmit) == "1:23.700");   // .700 is only ever 700 ms

    CHECK(fmt(7, kOmit) == "0.007");
    CHECK(fmt(70, kOmit) == "0.070");
    CHECK(fmt(700, kOmit) == "0.700");
}

// ---------------------------------------------------------------------------
// Whole-field behavior with minutes present (identical across styles)
// ---------------------------------------------------------------------------

TEST_CASE("formatLapTime - minutes present: M:SS.mmm with zero-padded seconds") {
    CHECK(fmt(60000, kOmit) == "1:00.000");
    CHECK(fmt(65007, kOmit) == "1:05.007");   // seconds zero-padded, not " 5"
    CHECK(fmt(65007, kSpace) == "1:05.007");
    CHECK(fmt(65007, kShow) == "1:05.007");
    CHECK(fmt(599999, kOmit) == "9:59.999");
    CHECK(fmt(600000, kOmit) == "10:00.000");
}

// ---------------------------------------------------------------------------
// Sub-minute styles
// ---------------------------------------------------------------------------

TEST_CASE("formatLapTime - kOmit drops the minutes field entirely") {
    CHECK(fmt(23007, kOmit) == "23.007");
    CHECK(fmt(5123, kOmit) == "5.123");
    CHECK(fmt(0, kOmit) == "0.000");
}

TEST_CASE("formatLapTime - kShow renders a fixed 0: minutes field") {
    CHECK(fmt(23007, kShow) == "0:23.007");
    CHECK(fmt(5123, kShow) == "0:05.123");
    CHECK(fmt(0, kShow) == "0:00.000");
}

TEST_CASE("formatLapTime - kSpace keeps the decimal point column-stable") {
    // The minutes slot becomes a space and seconds are space-padded to two
    // chars, so the '.' never moves while the lap is under a minute
    // (matching the original live-page alignment behavior).
    CHECK(fmt(23007, kSpace) == " 23.007");
    CHECK(fmt(5123, kSpace) == "  5.123");
    CHECK(fmt(0, kSpace) == "  0.000");
    CHECK(fmt(59999, kSpace) == " 59.999");
    CHECK(fmt(5123, kSpace).find('.') == fmt(59999, kSpace).find('.'));
}

// ---------------------------------------------------------------------------
// Bounds and safety
// ---------------------------------------------------------------------------

TEST_CASE("formatLapTime - full 32-bit range fits kLapTimeStrLen") {
    CHECK(fmt(4294967295UL, kOmit) == "71582:47.295");
}

TEST_CASE("formatLapTime - truncates safely into a small buffer") {
    char buf[5];
    formatLapTime(83007, kOmit, buf, sizeof(buf));
    CHECK(std::string(buf) == "1:23");  // truncated but NUL-terminated

    // Degenerate buffers must not crash or write.
    formatLapTime(83007, kOmit, nullptr, 16);
    char one[1];
    formatLapTime(83007, kOmit, one, 0);
}
