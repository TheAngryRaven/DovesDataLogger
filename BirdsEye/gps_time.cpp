#include "gps_time.h"

namespace gps_time {

uint32_t timeOfDayMs(uint8_t hour, uint8_t minute, uint8_t sec, uint16_t ms) {
  return static_cast<uint32_t>(hour)   * 3600000UL
       + static_cast<uint32_t>(minute) * 60000UL
       + static_cast<uint32_t>(sec)    * 1000UL
       + static_cast<uint32_t>(ms);
}

bool isLeapYear(uint16_t year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

uint32_t unixDaysSince1970(uint16_t year, uint8_t month, uint8_t day) {
  static const uint8_t kDaysInMonth[] = {31, 28, 31, 30, 31, 30,
                                          31, 31, 30, 31, 30, 31};
  uint32_t days = 0;
  for (uint16_t y = 1970; y < year; y++) {
    days += isLeapYear(y) ? 366 : 365;
  }
  for (uint8_t m = 1; m < month; m++) {
    days += kDaysInMonth[m - 1];
    if (m == 2 && isLeapYear(year)) {
      days++;  // leap day
    }
  }
  days += day - 1;
  return days;
}

uint64_t unixTimestampSeconds(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t sec) {
  uint64_t t = static_cast<uint64_t>(unixDaysSince1970(year, month, day)) * 86400ULL;
  t += static_cast<uint64_t>(hour)   * 3600ULL;
  t += static_cast<uint64_t>(minute) * 60ULL;
  t += static_cast<uint64_t>(sec);
  return t;
}

uint64_t unixTimestampMillis(uint16_t year, uint8_t month, uint8_t day,
                             uint8_t hour, uint8_t minute, uint8_t sec,
                             uint16_t ms) {
  return unixTimestampSeconds(year, month, day, hour, minute, sec) * 1000ULL
       + static_cast<uint64_t>(ms);
}

size_t u64ToDecimalString(uint64_t val, char* buf, size_t buf_size) {
  if (buf_size == 0) return 0;

  if (val == 0) {
    if (buf_size < 2) { buf[0] = '\0'; return 0; }
    buf[0] = '0';
    buf[1] = '\0';
    return 1;
  }

  // UINT64_MAX = 18446744073709551615 = 20 digits, +1 for null = 21.
  char temp[24];
  size_t digits = 0;
  uint64_t t = val;
  while (t > 0 && digits < sizeof(temp)) {
    temp[digits++] = static_cast<char>('0' + (t % 10));
    t /= 10;
  }

  if (digits + 1 > buf_size) {
    buf[0] = '\0';
    return 0;
  }

  for (size_t j = 0; j < digits; j++) {
    buf[j] = temp[digits - 1 - j];
  }
  buf[digits] = '\0';
  return digits;
}

}  // namespace gps_time
