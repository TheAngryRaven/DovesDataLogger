#pragma once

///////////////////////////////////////////
// GPS TIME HELPERS
// Pure-math conversions extracted so they can be exercised by host
// unit tests. The public getGps*() functions in gps_functions.ino
// call into these after reading the global gpsData struct.
///////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

namespace gps_time {

// Milliseconds since the start of the current UTC day.
uint32_t timeOfDayMs(uint8_t hour, uint8_t minute, uint8_t sec, uint16_t ms);

// True if `year` is a Gregorian leap year. `year` is 4-digit (e.g. 2025).
bool isLeapYear(uint16_t year);

// Days from 1970-01-01 to (year, month, day). month is 1-12, day is 1-31.
// Undefined behavior if month or day are out of range.
uint32_t unixDaysSince1970(uint16_t year, uint8_t month, uint8_t day);

// Unix timestamp in seconds (UTC).
uint64_t unixTimestampSeconds(uint16_t year, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t sec);

// Unix timestamp in milliseconds.
uint64_t unixTimestampMillis(uint16_t year, uint8_t month, uint8_t day,
                             uint8_t hour, uint8_t minute, uint8_t sec,
                             uint16_t ms);

// Convert an unsigned 64-bit value to a decimal ASCII string.
// Writes at most 21 chars (20 digits for UINT64_MAX + null terminator).
// Returns the number of digits written, NOT counting the null terminator.
// Returns 0 and writes "" if buf_size is too small to hold the result.
// Caller should size buf to at least 21 bytes to be safe.
size_t u64ToDecimalString(uint64_t val, char* buf, size_t buf_size);

}  // namespace gps_time
