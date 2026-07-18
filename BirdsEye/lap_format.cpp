#include "lap_format.h"

#include <stdio.h>

namespace lap_format {

void formatLapTime(unsigned long ms, ZeroMinutesStyle style,
                   char* buf, size_t bufSize) {
  if (buf == nullptr || bufSize == 0) return;

  const unsigned long minutes = ms / 60000UL;
  const unsigned long seconds = (ms % 60000UL) / 1000UL;
  const unsigned long millis = ms % 1000UL;

  if (minutes > 0) {
    snprintf(buf, bufSize, "%lu:%02lu.%03lu", minutes, seconds, millis);
    return;
  }

  switch (style) {
    case kShow:
      snprintf(buf, bufSize, "0:%02lu.%03lu", seconds, millis);
      break;
    case kSpace:
      // Leading space stands in for the minutes slot; %2lu space-pads
      // seconds so the decimal point never moves on the big-font pages.
      snprintf(buf, bufSize, " %2lu.%03lu", seconds, millis);
      break;
    case kOmit:
    default:
      snprintf(buf, bufSize, "%lu.%03lu", seconds, millis);
      break;
  }
}

}  // namespace lap_format
