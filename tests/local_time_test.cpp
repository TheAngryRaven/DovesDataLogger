#include "doctest.h"
#include "local_time.h"

#include <cstdint>

using namespace local_time;

namespace {

DateTime dt(uint16_t y, uint8_t mo, uint8_t d, uint8_t h, uint8_t mi) {
    return DateTime{y, mo, d, h, mi};
}

void checkEq(const DateTime& a, const DateTime& b) {
    CHECK((int)a.year   == (int)b.year);
    CHECK((int)a.month  == (int)b.month);
    CHECK((int)a.day    == (int)b.day);
    CHECK((int)a.hour   == (int)b.hour);
    CHECK((int)a.minute == (int)b.minute);
}

}  // namespace

// ---------------------------------------------------------------------------
// isValidOffsetMinutes
// ---------------------------------------------------------------------------

TEST_CASE("isValidOffsetMinutes - real-world extremes are inside the band") {
    CHECK(isValidOffsetMinutes(0));
    CHECK(isValidOffsetMinutes(-720));  // Baker Island, UTC-12
    CHECK(isValidOffsetMinutes(840));   // Kiribati, UTC+14
    CHECK(isValidOffsetMinutes(330));   // India, UTC+5:30
    CHECK(isValidOffsetMinutes(-210));  // Newfoundland, UTC-3:30
    CHECK(isValidOffsetMinutes(765));   // Chatham Islands, UTC+12:45
}

TEST_CASE("isValidOffsetMinutes - outside ±14h is rejected") {
    CHECK_FALSE(isValidOffsetMinutes(841));
    CHECK_FALSE(isValidOffsetMinutes(-841));
    CHECK_FALSE(isValidOffsetMinutes(100000));
    CHECK_FALSE(isValidOffsetMinutes(-100000));
}

// ---------------------------------------------------------------------------
// daysInMonth
// ---------------------------------------------------------------------------

TEST_CASE("daysInMonth - ordinary months") {
    CHECK((int)daysInMonth(2026, 1)  == 31);
    CHECK((int)daysInMonth(2026, 4)  == 30);
    CHECK((int)daysInMonth(2026, 12) == 31);
}

TEST_CASE("daysInMonth - February follows the Gregorian leap rule") {
    CHECK((int)daysInMonth(2026, 2) == 28);
    CHECK((int)daysInMonth(2024, 2) == 29);  // divisible by 4
    CHECK((int)daysInMonth(1900, 2) == 28);  // century, not by 400
    CHECK((int)daysInMonth(2000, 2) == 29);  // divisible by 400
}

TEST_CASE("daysInMonth - out-of-range month returns 0, never indexes off the table") {
    CHECK((int)daysInMonth(2026, 0)   == 0);
    CHECK((int)daysInMonth(2026, 13)  == 0);
    CHECK((int)daysInMonth(2026, 255) == 0);
}

// ---------------------------------------------------------------------------
// applyOffset
// ---------------------------------------------------------------------------

TEST_CASE("applyOffset - zero offset is identity") {
    checkEq(applyOffset(dt(2026, 8, 21, 14, 30), 0), dt(2026, 8, 21, 14, 30));
}

TEST_CASE("applyOffset - whole-hour shift inside the same day") {
    // 14:30 UTC in US Central (UTC-5) is 09:30 local.
    checkEq(applyOffset(dt(2026, 8, 21, 14, 30), -300), dt(2026, 8, 21, 9, 30));
    // ...and in Tokyo (UTC+9) it is 23:30 the same day.
    checkEq(applyOffset(dt(2026, 8, 21, 14, 30), 540), dt(2026, 8, 21, 23, 30));
}

TEST_CASE("applyOffset - half-hour and quarter-hour zones") {
    // India, UTC+5:30.
    checkEq(applyOffset(dt(2026, 8, 21, 14, 45), 330), dt(2026, 8, 21, 20, 15));
    // Newfoundland, UTC-3:30.
    checkEq(applyOffset(dt(2026, 8, 21, 2, 15), -210), dt(2026, 8, 20, 22, 45));
    // Chatham Islands, UTC+12:45.
    checkEq(applyOffset(dt(2026, 8, 21, 0, 0), 765), dt(2026, 8, 21, 12, 45));
}

TEST_CASE("applyOffset - negative offset rolls back to the previous day") {
    // 02:00 UTC on the 21st is 21:00 on the 20th in UTC-5. This is the
    // case the day/night gate actually cares about: an evening session
    // is already "tomorrow" in UTC.
    checkEq(applyOffset(dt(2026, 8, 21, 2, 0), -300), dt(2026, 8, 20, 21, 0));
}

TEST_CASE("applyOffset - positive offset rolls forward to the next day") {
    checkEq(applyOffset(dt(2026, 8, 21, 23, 0), 120), dt(2026, 8, 22, 1, 0));
}

TEST_CASE("applyOffset - rollback crosses a month boundary") {
    // 1st at 02:00 UTC, UTC-5 -> last day of the previous month.
    checkEq(applyOffset(dt(2026, 8, 1, 2, 0), -300), dt(2026, 7, 31, 21, 0));
    // Previous month with 30 days.
    checkEq(applyOffset(dt(2026, 7, 1, 2, 0), -300), dt(2026, 6, 30, 21, 0));
}

TEST_CASE("applyOffset - rollforward crosses a month boundary") {
    checkEq(applyOffset(dt(2026, 6, 30, 23, 0), 120), dt(2026, 7, 1, 1, 0));
    checkEq(applyOffset(dt(2026, 7, 31, 23, 0), 120), dt(2026, 8, 1, 1, 0));
}

TEST_CASE("applyOffset - rollback lands on the leap day, and skips it in a common year") {
    // 2024 IS a leap year: 1 Mar 02:00 UTC in UTC-5 is 29 Feb 21:00.
    checkEq(applyOffset(dt(2024, 3, 1, 2, 0), -300), dt(2024, 2, 29, 21, 0));
    // 2026 is not: the same instant is 28 Feb.
    checkEq(applyOffset(dt(2026, 3, 1, 2, 0), -300), dt(2026, 2, 28, 21, 0));
}

TEST_CASE("applyOffset - rollforward over the leap day") {
    checkEq(applyOffset(dt(2024, 2, 28, 23, 0), 120), dt(2024, 2, 29, 1, 0));
    checkEq(applyOffset(dt(2026, 2, 28, 23, 0), 120), dt(2026, 3, 1, 1, 0));
}

TEST_CASE("applyOffset - rollback crosses a year boundary") {
    checkEq(applyOffset(dt(2026, 1, 1, 2, 0), -300), dt(2025, 12, 31, 21, 0));
}

TEST_CASE("applyOffset - rollforward crosses a year boundary") {
    checkEq(applyOffset(dt(2026, 12, 31, 23, 0), 120), dt(2027, 1, 1, 1, 0));
}

TEST_CASE("applyOffset - the extremes of the band still land on a sane date") {
    checkEq(applyOffset(dt(2026, 8, 21, 0, 0), -720), dt(2026, 8, 20, 12, 0));
    checkEq(applyOffset(dt(2026, 8, 21, 23, 59), 840), dt(2026, 8, 22, 13, 59));
}

TEST_CASE("applyOffset - an out-of-band offset is ignored, not applied") {
    // A corrupt setting must never be able to move the calendar.
    checkEq(applyOffset(dt(2026, 8, 21, 14, 30), 1500), dt(2026, 8, 21, 14, 30));
    checkEq(applyOffset(dt(2026, 8, 21, 14, 30), -1500), dt(2026, 8, 21, 14, 30));
}

TEST_CASE("applyOffset - year is 4-digit throughout, no 2000s assumption") {
    checkEq(applyOffset(dt(1999, 12, 31, 23, 30), 60), dt(2000, 1, 1, 0, 30));
    checkEq(applyOffset(dt(2100, 3, 1, 0, 30), -60), dt(2100, 2, 28, 23, 30));
}

// ---------------------------------------------------------------------------
// minuteOfDay
// ---------------------------------------------------------------------------

TEST_CASE("minuteOfDay - endpoints and a midpoint") {
    CHECK(minuteOfDay(dt(2026, 8, 21, 0, 0))   == 0u);
    CHECK(minuteOfDay(dt(2026, 8, 21, 7, 0))   == 420u);
    CHECK(minuteOfDay(dt(2026, 8, 21, 19, 0))  == 1140u);
    CHECK(minuteOfDay(dt(2026, 8, 21, 23, 59)) == kMinutesPerDay - 1);
}

// ---------------------------------------------------------------------------
// isNight — the window is [nightStart, dayStart) modulo the day
// ---------------------------------------------------------------------------

TEST_CASE("isNight - the ordinary wrapped window, night 19:00 -> day 07:00") {
    const uint16_t day = 7 * 60, night = 19 * 60;
    CHECK_FALSE(isNight(7 * 60, day, night));       // 07:00 exactly = day
    CHECK_FALSE(isNight(7 * 60 + 1, day, night));
    CHECK_FALSE(isNight(12 * 60, day, night));
    CHECK_FALSE(isNight(18 * 60 + 59, day, night));
    CHECK(isNight(19 * 60, day, night));            // 19:00 exactly = night
    CHECK(isNight(23 * 60 + 59, day, night));
    CHECK(isNight(0, day, night));                  // midnight
    CHECK(isNight(6 * 60 + 59, day, night));
}

TEST_CASE("isNight - boundaries are half-open, so no minute is both") {
    const uint16_t day = 7 * 60, night = 19 * 60;
    for (uint16_t m = 0; m < kMinutesPerDay; m++) {
        const bool night_m = isNight(m, day, night);
        const bool expect = (m >= night) || (m < day);
        CHECK(night_m == expect);
    }
}

TEST_CASE("isNight - equal bounds disable the swap entirely") {
    for (uint16_t m = 0; m < kMinutesPerDay; m += 37) {
        CHECK_FALSE(isNight(m, 7 * 60, 7 * 60));
        CHECK_FALSE(isNight(m, 0, 0));
    }
}

TEST_CASE("isNight - an inverted (non-wrapping) window still works") {
    // Night 07:00 -> day 19:00: the window sits inside one calendar day.
    const uint16_t day = 19 * 60, night = 7 * 60;
    CHECK_FALSE(isNight(6 * 60, day, night));
    CHECK(isNight(7 * 60, day, night));
    CHECK(isNight(12 * 60, day, night));
    CHECK(isNight(18 * 60 + 59, day, night));
    CHECK_FALSE(isNight(19 * 60, day, night));
    CHECK_FALSE(isNight(23 * 60, day, night));
}

// ---------------------------------------------------------------------------
// End-to-end: the thing the LED gate actually asks
// ---------------------------------------------------------------------------

TEST_CASE("UTC 02:00 in UTC-5 is 21:00 local, which is night") {
    // The whole point of the feature. Without the offset this instant
    // reads as 02:00 and is night by luck; the case that breaks a naive
    // UTC gate is the one below.
    const DateTime lt = applyOffset(dt(2026, 8, 21, 2, 0), -300);
    CHECK(isNight(minuteOfDay(lt), 7 * 60, 19 * 60));
}

TEST_CASE("UTC 12:30 in UTC-5 is 07:30 local, which is DAY — a UTC gate gets this wrong") {
    const DateTime lt = applyOffset(dt(2026, 8, 21, 12, 30), -300);
    CHECK((int)lt.hour == 7);
    CHECK_FALSE(isNight(minuteOfDay(lt), 7 * 60, 19 * 60));
}

TEST_CASE("UTC 12:30 in UTC+9 is 21:30 local, which is night") {
    const DateTime lt = applyOffset(dt(2026, 8, 21, 12, 30), 540);
    CHECK(isNight(minuteOfDay(lt), 7 * 60, 19 * 60));
}
