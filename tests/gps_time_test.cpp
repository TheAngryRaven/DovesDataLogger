#include "doctest.h"
#include "gps_time.h"

#include <cstdint>
#include <cstring>
#include <string>

using namespace gps_time;

// ---------------------------------------------------------------------------
// timeOfDayMs
// ---------------------------------------------------------------------------

TEST_CASE("timeOfDayMs - midnight is 0") {
    CHECK(timeOfDayMs(0, 0, 0, 0) == 0u);
}

TEST_CASE("timeOfDayMs - end of day is one ms short of 24h") {
    // 23:59:59.999 = 86,399,999 ms = 24*3600000 - 1
    CHECK(timeOfDayMs(23, 59, 59, 999) == 86'399'999u);
}

TEST_CASE("timeOfDayMs - noon and components") {
    CHECK(timeOfDayMs(12, 0, 0, 0)  == 43'200'000u);  // 12*3600*1000
    CHECK(timeOfDayMs(0, 30, 0, 0)  == 1'800'000u);   // 30 min
    CHECK(timeOfDayMs(0, 0, 45, 0)  == 45'000u);
    CHECK(timeOfDayMs(0, 0, 0, 250) == 250u);
}

// ---------------------------------------------------------------------------
// isLeapYear — Gregorian rules
// ---------------------------------------------------------------------------

TEST_CASE("isLeapYear - divisible by 4 but not 100 -> leap") {
    CHECK(isLeapYear(2024));
    CHECK(isLeapYear(2020));
    CHECK(isLeapYear(1972));
    CHECK(isLeapYear(2004));
}

TEST_CASE("isLeapYear - divisible by 100 but not 400 -> not leap") {
    CHECK_FALSE(isLeapYear(1900));
    CHECK_FALSE(isLeapYear(2100));
    CHECK_FALSE(isLeapYear(2200));
}

TEST_CASE("isLeapYear - divisible by 400 -> leap") {
    CHECK(isLeapYear(2000));
    CHECK(isLeapYear(2400));
}

TEST_CASE("isLeapYear - not divisible by 4 -> not leap") {
    CHECK_FALSE(isLeapYear(2025));
    CHECK_FALSE(isLeapYear(2023));
    CHECK_FALSE(isLeapYear(1971));
}

// ---------------------------------------------------------------------------
// unixDaysSince1970
// ---------------------------------------------------------------------------

TEST_CASE("unixDaysSince1970 - epoch is day 0") {
    CHECK(unixDaysSince1970(1970, 1, 1) == 0u);
}

TEST_CASE("unixDaysSince1970 - second day of epoch is day 1") {
    CHECK(unixDaysSince1970(1970, 1, 2) == 1u);
}

TEST_CASE("unixDaysSince1970 - 1971-01-01 is day 365 (1970 not leap)") {
    CHECK(unixDaysSince1970(1971, 1, 1) == 365u);
}

TEST_CASE("unixDaysSince1970 - 1972-01-01 is day 730 (1971 not leap)") {
    CHECK(unixDaysSince1970(1972, 1, 1) == 730u);
}

TEST_CASE("unixDaysSince1970 - 1973-01-01 is day 1096 (1972 was leap)") {
    CHECK(unixDaysSince1970(1973, 1, 1) == 1096u);
}

TEST_CASE("unixDaysSince1970 - Y2K (2000-01-01)") {
    // 30 years * 365 + 7 leap days (1972,76,80,84,88,92,96) = 10957
    CHECK(unixDaysSince1970(2000, 1, 1) == 10957u);
}

TEST_CASE("unixDaysSince1970 - Feb 29 of leap year exists") {
    // 2024-03-01 should be 1 day past 2024-02-29
    const uint32_t feb29 = unixDaysSince1970(2024, 2, 29);
    const uint32_t mar01 = unixDaysSince1970(2024, 3, 1);
    CHECK(mar01 - feb29 == 1u);
}

TEST_CASE("unixDaysSince1970 - non-leap year February has 28 days") {
    // 2025-03-01 should be 28 days past 2025-02-01
    const uint32_t feb01 = unixDaysSince1970(2025, 2, 1);
    const uint32_t mar01 = unixDaysSince1970(2025, 3, 1);
    CHECK(mar01 - feb01 == 28u);
}

// ---------------------------------------------------------------------------
// unixTimestampSeconds — known reference values
// ---------------------------------------------------------------------------

TEST_CASE("unixTimestampSeconds - epoch is 0") {
    CHECK(unixTimestampSeconds(1970, 1, 1, 0, 0, 0) == 0ULL);
}

TEST_CASE("unixTimestampSeconds - Y2K (2000-01-01 00:00:00 UTC) = 946684800") {
    CHECK(unixTimestampSeconds(2000, 1, 1, 0, 0, 0) == 946684800ULL);
}

TEST_CASE("unixTimestampSeconds - 2024-01-15 14:30:00 UTC = 1705329000") {
    // Hand-derived: 19723 days from 1970-01-01 to 2024-01-01, +14 days = 19737;
    // *86400 = 1705276800; + 14h30m = 52200; total 1705329000.
    CHECK(unixTimestampSeconds(2024, 1, 15, 14, 30, 0) == 1705329000ULL);
}

TEST_CASE("unixTimestampSeconds - one second after epoch") {
    CHECK(unixTimestampSeconds(1970, 1, 1, 0, 0, 1) == 1ULL);
}

// ---------------------------------------------------------------------------
// unixTimestampMillis
// ---------------------------------------------------------------------------

TEST_CASE("unixTimestampMillis - epoch + 0ms = 0") {
    CHECK(unixTimestampMillis(1970, 1, 1, 0, 0, 0, 0) == 0ULL);
}

TEST_CASE("unixTimestampMillis - Y2K with milliseconds") {
    // 946684800 * 1000 + 123 = 946684800123
    CHECK(unixTimestampMillis(2000, 1, 1, 0, 0, 0, 123) == 946'684'800'123ULL);
}

TEST_CASE("unixTimestampMillis - real-world DOVEX example") {
    // Matches the example timestamp in the README (1741128001234).
    // Decodes to 2025-03-04 22:40:01.234 UTC.
    CHECK(unixTimestampMillis(2025, 3, 4, 22, 40, 1, 234) == 1'741'128'001'234ULL);
}

// ---------------------------------------------------------------------------
// u64ToDecimalString
// ---------------------------------------------------------------------------

TEST_CASE("u64ToDecimalString - zero prints '0'") {
    char buf[24] = {'X'};
    const size_t n = u64ToDecimalString(0, buf, sizeof(buf));
    CHECK(n == 1u);
    CHECK(std::string(buf) == "0");
}

TEST_CASE("u64ToDecimalString - single digits") {
    char buf[24];
    CHECK(u64ToDecimalString(7, buf, sizeof(buf)) == 1u);
    CHECK(std::string(buf) == "7");
}

TEST_CASE("u64ToDecimalString - multi-digit") {
    char buf[24];
    CHECK(u64ToDecimalString(12345, buf, sizeof(buf)) == 5u);
    CHECK(std::string(buf) == "12345");
}

TEST_CASE("u64ToDecimalString - 64-bit DOVEX-style timestamp") {
    char buf[24];
    const size_t n = u64ToDecimalString(1'741'128'001'234ULL, buf, sizeof(buf));
    CHECK(n == 13u);
    CHECK(std::string(buf) == "1741128001234");
}

TEST_CASE("u64ToDecimalString - UINT64_MAX (20 digits)") {
    char buf[24];
    const size_t n = u64ToDecimalString(UINT64_MAX, buf, sizeof(buf));
    CHECK(n == 20u);
    CHECK(std::string(buf) == "18446744073709551615");
}

TEST_CASE("u64ToDecimalString - buf too small writes empty string and returns 0") {
    char buf[3] = {'X', 'Y', 'Z'};
    // 12345 needs 6 bytes (5 digits + null); only 3 provided.
    const size_t n = u64ToDecimalString(12345, buf, sizeof(buf));
    CHECK(n == 0u);
    CHECK(buf[0] == '\0');
}

TEST_CASE("u64ToDecimalString - exact-fit buffer for single digit") {
    // val=0 special case needs at least 2 bytes ('0' + null).
    char buf[2];
    CHECK(u64ToDecimalString(0, buf, sizeof(buf)) == 1u);
    CHECK(std::string(buf) == "0");
}

TEST_CASE("u64ToDecimalString - zero-size buffer returns 0 without writing") {
    char sentinel = 'Z';
    CHECK(u64ToDecimalString(42, &sentinel, 0) == 0u);
    CHECK(sentinel == 'Z');  // untouched
}
