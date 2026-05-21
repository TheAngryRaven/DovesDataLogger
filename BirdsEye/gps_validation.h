#pragma once

///////////////////////////////////////////
// GPS VALIDATION
// Sanity checks applied to every PVT sample before it gets written
// to the DOVEX log. Extracted so the rule set lives in one place
// and is exercised by host unit tests.
///////////////////////////////////////////

#include <stddef.h>

namespace gps_validation {

// Container for the GPS fields we sanity-check before logging.
struct GpsSample {
  double lat;       // degrees, -90..+90
  double lng;       // degrees, -180..+180
  double alt;       // meters MSL
  double hdop;      // unitless
  double speedMph;  // miles per hour
  int    sats;      // satellite count
};

// Reject samples that are obviously corrupt. A sample is valid iff:
//   - sats > 0 (we have an actual fix)
//   - !(lat == 0 && lng == 0) (Null Island — signals 'no fix' too often)
//   - lat / lng / alt / hdop / speedMph are all finite (no NaN, no Inf)
//   - lat ∈ [-90, 90], lng ∈ [-180, 180]
//   - alt ∈ [-1000, 50000] m (covers Death Valley to LEO)
//   - hdop > 0
//   - speedMph ∈ [0, 500] (well above any vehicle)
bool isSampleValid(const GpsSample& s);

// Validate that a stringified numeric (lat/lng/speed/...) is a
// well-formed signed decimal: only digits, dot, leading minus.
// Returns false on null pointer, empty string, length > max_len, or
// any non-allowed character. Used to catch dtostrf() going haywire.
bool isNumericString(const char* s, size_t max_len);

}  // namespace gps_validation
