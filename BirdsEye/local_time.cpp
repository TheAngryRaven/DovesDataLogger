#include "local_time.h"

#include "gps_time.h"  // isLeapYear — one leap rule for the whole firmware

namespace local_time {

bool isValidOffsetMinutes(int32_t offsetMin) {
  return offsetMin >= -(int32_t)kOffsetMinLimit &&
         offsetMin <= (int32_t)kOffsetMinLimit;
}

uint8_t daysInMonth(uint16_t year, uint8_t month) {
  static const uint8_t kDays[] = {31, 28, 31, 30, 31, 30,
                                  31, 31, 30, 31, 30, 31};
  if (month < 1 || month > 12) return 0;
  if (month == 2 && gps_time::isLeapYear(year)) return 29;
  return kDays[month - 1];
}

uint16_t minuteOfDay(const DateTime& dt) {
  return (uint16_t)((uint16_t)dt.hour * 60 + dt.minute);
}

// Step the calendar one day back/forward, carrying month and year.
// Split out because applyOffset needs both directions and getting the
// month-length lookup right at a year boundary is the whole job.
static void stepDay(DateTime& dt, int8_t delta) {
  if (delta < 0) {
    if (dt.day > 1) {
      dt.day--;
      return;
    }
    if (dt.month > 1) {
      dt.month--;
    } else {
      dt.month = 12;
      dt.year--;
    }
    const uint8_t len = daysInMonth(dt.year, dt.month);
    dt.day = len ? len : 1;
    return;
  }
  const uint8_t len = daysInMonth(dt.year, dt.month);
  if (len && dt.day < len) {
    dt.day++;
    return;
  }
  dt.day = 1;
  if (dt.month < 12) {
    dt.month++;
  } else {
    dt.month = 1;
    dt.year++;
  }
}

DateTime applyOffset(const DateTime& utc, int16_t offsetMin) {
  DateTime out = utc;
  // A corrupt/out-of-band offset must not be able to walk the date.
  if (!isValidOffsetMinutes(offsetMin)) return out;

  // The band is ±840 min, strictly under a day, so this can only carry
  // by one — but the loop costs nothing and keeps the function honest
  // if the band is ever widened.
  int32_t mins = (int32_t)minuteOfDay(utc) + offsetMin;
  while (mins < 0) {
    mins += kMinutesPerDay;
    stepDay(out, -1);
  }
  while (mins >= (int32_t)kMinutesPerDay) {
    mins -= kMinutesPerDay;
    stepDay(out, +1);
  }

  out.hour = (uint8_t)(mins / 60);
  out.minute = (uint8_t)(mins % 60);
  return out;
}

bool isNight(uint16_t minuteOfDay, uint16_t dayStartMin,
             uint16_t nightStartMin) {
  if (dayStartMin == nightStartMin) return false;  // empty window = disabled
  if (nightStartMin < dayStartMin) {
    // Window sits inside one calendar day (an inverted setup, but valid).
    return minuteOfDay >= nightStartMin && minuteOfDay < dayStartMin;
  }
  // The ordinary case: the window wraps midnight.
  return minuteOfDay >= nightStartMin || minuteOfDay < dayStartMin;
}

}  // namespace local_time
