#include "doctest.h"
#include "gps_validation.h"

#include <cmath>
#include <limits>

using gps_validation::GpsSample;
using gps_validation::isSampleValid;
using gps_validation::isNumericString;

// A reasonable, real-world-ish baseline sample (Orlando Kart Center).
// Each test case clones this and tweaks one field.
static GpsSample goodSample() {
    return GpsSample{
        28.4127,    // lat
        -81.3797,   // lng
        100.0,      // alt (m)
        0.8,        // hdop
        45.0,       // speedMph
        12          // sats
    };
}

// ---------------------------------------------------------------------------
// isSampleValid — golden path
// ---------------------------------------------------------------------------

TEST_CASE("isSampleValid - real-world sample passes") {
    CHECK(isSampleValid(goodSample()));
}

// ---------------------------------------------------------------------------
// isSampleValid — sats / coordinate guards
// ---------------------------------------------------------------------------

TEST_CASE("isSampleValid - zero sats rejected") {
    auto s = goodSample();
    s.sats = 0;
    CHECK_FALSE(isSampleValid(s));
}

TEST_CASE("isSampleValid - negative sats rejected") {
    auto s = goodSample();
    s.sats = -1;
    CHECK_FALSE(isSampleValid(s));
}

TEST_CASE("isSampleValid - Null Island (0,0) rejected") {
    auto s = goodSample();
    s.lat = 0.0;
    s.lng = 0.0;
    CHECK_FALSE(isSampleValid(s));
}

TEST_CASE("isSampleValid - exact equator with non-zero lng is OK") {
    auto s = goodSample();
    s.lat = 0.0;
    s.lng = 1.0;  // off the coast of Africa, but not Null Island
    CHECK(isSampleValid(s));
}

// ---------------------------------------------------------------------------
// isSampleValid — range guards
// ---------------------------------------------------------------------------

TEST_CASE("isSampleValid - latitude bounds") {
    auto s = goodSample();
    s.lat = 90.0;   CHECK(isSampleValid(s));     // inclusive
    s.lat = -90.0;  CHECK(isSampleValid(s));     // inclusive
    s.lat = 90.001; CHECK_FALSE(isSampleValid(s));
    s.lat = -90.1;  CHECK_FALSE(isSampleValid(s));
}

TEST_CASE("isSampleValid - longitude bounds") {
    auto s = goodSample();
    s.lng = 180.0;   CHECK(isSampleValid(s));
    s.lng = -180.0;  CHECK(isSampleValid(s));
    s.lng = 180.001; CHECK_FALSE(isSampleValid(s));
    s.lng = -180.1;  CHECK_FALSE(isSampleValid(s));
}

TEST_CASE("isSampleValid - altitude bounds") {
    auto s = goodSample();
    s.alt = 50000.0;  CHECK(isSampleValid(s));
    s.alt = -1000.0;  CHECK(isSampleValid(s));
    s.alt = 50001.0;  CHECK_FALSE(isSampleValid(s));
    s.alt = -1001.0;  CHECK_FALSE(isSampleValid(s));
}

TEST_CASE("isSampleValid - hdop strictly positive") {
    auto s = goodSample();
    s.hdop = 0.0;    CHECK_FALSE(isSampleValid(s));  // exclusive lower bound
    s.hdop = -0.1;   CHECK_FALSE(isSampleValid(s));
    s.hdop = 0.001;  CHECK(isSampleValid(s));
}

TEST_CASE("isSampleValid - speed bounds") {
    auto s = goodSample();
    s.speedMph = 0.0;     CHECK(isSampleValid(s));
    s.speedMph = 500.0;   CHECK(isSampleValid(s));
    s.speedMph = -0.001;  CHECK_FALSE(isSampleValid(s));
    s.speedMph = 500.001; CHECK_FALSE(isSampleValid(s));
}

// ---------------------------------------------------------------------------
// isSampleValid — NaN / Inf guards
// ---------------------------------------------------------------------------

TEST_CASE("isSampleValid - NaN in any field is rejected") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto s = goodSample();

    auto check_field_nan = [](GpsSample s, double GpsSample::*field) {
        s.*field = std::numeric_limits<double>::quiet_NaN();
        return !isSampleValid(s);
    };
    CHECK(check_field_nan(s, &GpsSample::lat));
    CHECK(check_field_nan(s, &GpsSample::lng));
    CHECK(check_field_nan(s, &GpsSample::alt));
    CHECK(check_field_nan(s, &GpsSample::hdop));
    CHECK(check_field_nan(s, &GpsSample::speedMph));
    (void)nan;
}

TEST_CASE("isSampleValid - Inf in any field is rejected") {
    auto s = goodSample();
    auto check_field_inf = [](GpsSample s, double GpsSample::*field) {
        s.*field = std::numeric_limits<double>::infinity();
        return !isSampleValid(s);
    };
    CHECK(check_field_inf(s, &GpsSample::lat));
    CHECK(check_field_inf(s, &GpsSample::lng));
    CHECK(check_field_inf(s, &GpsSample::alt));
    CHECK(check_field_inf(s, &GpsSample::hdop));
    CHECK(check_field_inf(s, &GpsSample::speedMph));
}

// ---------------------------------------------------------------------------
// isNumericString
// ---------------------------------------------------------------------------

TEST_CASE("isNumericString - well-formed decimals pass") {
    CHECK(isNumericString("0", 20));
    CHECK(isNumericString("42", 20));
    CHECK(isNumericString("3.14", 20));
    CHECK(isNumericString("-12.345", 20));
    CHECK(isNumericString("28.41270817", 20));    // typical lat
    CHECK(isNumericString("-81.37973266", 20));   // typical lng
    CHECK(isNumericString("0.000", 20));          // accel idle
    CHECK(isNumericString("---", 20));            // allowed by the char set,
                                                  // even if semantically odd
}

TEST_CASE("isNumericString - rejects empty and null") {
    CHECK_FALSE(isNumericString(nullptr, 20));
    CHECK_FALSE(isNumericString("", 20));
}

TEST_CASE("isNumericString - rejects over-length input") {
    // 21-char string against max_len=20 should fail.
    const char twentyOne[] = "123456789012345678901";
    static_assert(sizeof(twentyOne) - 1 == 21, "length sanity");
    CHECK_FALSE(isNumericString(twentyOne, 20));
}

TEST_CASE("isNumericString - rejects scientific notation (dtostrf shouldn't produce it)") {
    CHECK_FALSE(isNumericString("1e10", 20));
    CHECK_FALSE(isNumericString("3.14E+2", 20));
}

TEST_CASE("isNumericString - rejects whitespace and letters") {
    CHECK_FALSE(isNumericString("12 34", 20));
    CHECK_FALSE(isNumericString("hello", 20));
    CHECK_FALSE(isNumericString("nan", 20));
    CHECK_FALSE(isNumericString("inf", 20));
    CHECK_FALSE(isNumericString(" 42", 20));      // leading space
    CHECK_FALSE(isNumericString("42\n", 20));     // trailing newline
}
