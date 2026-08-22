#pragma once

///////////////////////////////////////////
// LOCAL TIME — a fixed UTC offset, nothing more
//
// The GPS delivers UTC and that is what the device LOGS: DOVEX row
// timestamps are Unix epoch milliseconds (UTC by definition), the
// header datetime and every generated filename stay UTC, and the
// webapp does its own presentation-side conversion. NOTHING in this
// unit may be wired into the logging pipeline (plan 0010).
//
// What it exists for is the handful of decisions that must happen at a
// LOCAL wall-clock time — today exactly one: the NeoPixel day/night
// brightness swap, where "7am" has to mean the driver's 7am, not
// 7am in Greenwich.
//
// DELIBERATELY NO DST (plan 0010). A bare offset means the boundary
// walks an hour twice a year; for a "dim the strip after dark" gate
// that is noise, and the alternative (US/EU rule tables, or worse
// tzdata) buys precision nobody asked for. If a future feature needs
// true civil time this is where the rules go — the DateTime carries a
// 4-digit year precisely so that stays possible.
///////////////////////////////////////////

#include <stdint.h>

namespace local_time {

// UTC offsets in the real world run from -12:00 (Baker Island) to
// +14:00 (Kiribati). Minutes, not hours: India is +330, Newfoundland
// -210, Chatham Islands +765.
constexpr int16_t kOffsetMinLimit = 840;  // ±14 h

constexpr uint16_t kMinutesPerDay = 1440;

// A wall-clock instant with a FOUR-DIGIT year. The sketch's global
// gpsData.year is 2-digit for compatibility with the existing filename
// and header formats — callers add 2000 on the way in. Nothing here
// may assume the 2000s.
struct DateTime {
  uint16_t year;   // 4-digit, e.g. 2026
  uint8_t month;   // 1-12
  uint8_t day;     // 1-31
  uint8_t hour;    // 0-23
  uint8_t minute;  // 0-59
};

// True if `offsetMin` is inside ±kOffsetMinLimit. Settings clamp with
// this rather than an inline literal so the bound has one home.
bool isValidOffsetMinutes(int32_t offsetMin);

// Length of `month` (1-12) in `year`. Returns 0 for an out-of-range
// month so a garbled date can't index off the end of the table.
uint8_t daysInMonth(uint16_t year, uint8_t month);

// `utc` shifted by `offsetMin`, rolling the date correctly in both
// directions across month, year and leap-day boundaries. An offset
// outside the valid band is treated as 0 — a corrupt setting must not
// be able to move the calendar.
DateTime applyOffset(const DateTime& utc, int16_t offsetMin);

// Minutes elapsed since local midnight, 0-1439.
uint16_t minuteOfDay(const DateTime& dt);

// Is `minuteOfDay` inside the night window [nightStartMin, dayStartMin)?
//
// The window is defined modulo the day, so the ordinary case
// (night 19:00 -> day 07:00) wraps midnight and needs no special
// casing at the call site. Equal bounds mean the window is EMPTY —
// never night — which is how a user turns the whole day/night swap off
// without a separate enable flag.
bool isNight(uint16_t minuteOfDay, uint16_t dayStartMin,
             uint16_t nightStartMin);

}  // namespace local_time
