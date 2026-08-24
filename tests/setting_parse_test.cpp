#include "doctest.h"

#include "setting_parse.h"

#include <limits.h>

using setting_parse::parseIntSetting;

namespace {

// Helper: parse and return the value, or a sentinel when rejected.
int parsed(const char* s, int sentinel = -999999) {
  int v = 0;
  return parseIntSetting(s, &v) ? v : sentinel;
}

}  // namespace

TEST_CASE("parseIntSetting: accepts plain integers") {
  CHECK(parsed("0") == 0);
  CHECK(parsed("7") == 7);
  CHECK(parsed("64") == 64);
  CHECK(parsed("255") == 255);
  CHECK(parsed("15000") == 15000);
  CHECK(parsed("-360") == -360);
  CHECK(parsed("+330") == 330);
  CHECK(parsed("-0") == 0);
}

TEST_CASE("parseIntSetting: tolerates surrounding whitespace") {
  CHECK(parsed(" 64") == 64);
  CHECK(parsed("64 ") == 64);
  CHECK(parsed("\t 64 \r\n") == 64);
  CHECK(parsed(" -360 ") == -360);
}

TEST_CASE("parseIntSetting: rejects everything atoi() would silently zero") {
  // THE bug this unit exists for: each of these came back as 0 from
  // atoi(), and 0 is an in-range, destructive value for led_brightness.
  int v = 12345;
  CHECK(!parseIntSetting("", &v));
  CHECK(v == 12345);  // out is untouched on rejection
  CHECK(!parseIntSetting(" ", &v));
  CHECK(!parseIntSetting("abc", &v));
  CHECK(!parseIntSetting("null", &v));
  CHECK(!parseIntSetting("true", &v));
  CHECK(!parseIntSetting("-", &v));
  CHECK(!parseIntSetting("+", &v));
  CHECK(!parseIntSetting(nullptr, &v));
  CHECK(v == 12345);
}

TEST_CASE("parseIntSetting: rejects partial numbers rather than truncating") {
  int v = 0;
  CHECK(!parseIntSetting("12abc", &v));   // atoi -> 12
  CHECK(!parseIntSetting("1.5", &v));     // atoi -> 1
  CHECK(!parseIntSetting("0x10", &v));    // atoi -> 0
  CHECK(!parseIntSetting("64,", &v));     // atoi -> 64
  CHECK(!parseIntSetting("6 4", &v));     // atoi -> 6
  CHECK(!parseIntSetting("--5", &v));
}

TEST_CASE("parseIntSetting: rejects values that do not fit an int") {
  int v = 0;
  CHECK(!parseIntSetting("99999999999", &v));
  CHECK(!parseIntSetting("-99999999999", &v));
  CHECK(!parseIntSetting("340282366920938463463374607431768211456", &v));
  // The boundaries themselves still parse.
  CHECK(parsed("2147483647") == 2147483647);
  CHECK(parsed("-2147483648") == INT_MIN);
}

TEST_CASE("parseIntSetting: null out pointer is rejected, not dereferenced") {
  CHECK(!parseIntSetting("64", nullptr));
}

TEST_CASE("the caller contract: a rejected value keeps the compiled-in default") {
  // Mirrors BirdsEye.ino's read pattern for led_brightness, which is the
  // one where 0 is both in range and destructive (LEDs off, 5 V boost
  // rail never enabled). A blank or corrupt value must leave the default
  // standing rather than reading as "the user asked for 0".
  auto applyBrightness = [](const char* stored, unsigned char current) {
    int b = 0;
    if (parseIntSetting(stored, &b) && b >= 0 && b <= 255) {
      return (unsigned char)b;
    }
    return current;
  };

  CHECK(applyBrightness("", 64) == 64);        // was 0 -> LEDs dead
  CHECK(applyBrightness("garbage", 64) == 64); // was 0 -> LEDs dead
  CHECK(applyBrightness("128", 64) == 128);    // a real change still lands
  CHECK(applyBrightness("0", 64) == 0);        // a DELIBERATE 0 still works
  CHECK(applyBrightness("300", 64) == 64);     // out of range -> default
  CHECK(applyBrightness("-5", 64) == 64);
}
