#include "gps_validation.h"

#include <math.h>
#include <string.h>

namespace gps_validation {

bool isSampleValid(const GpsSample& s) {
  // Fix presence.
  if (s.sats <= 0) return false;

  // Null Island — coordinates exactly (0,0) almost always mean
  // "no fix yet" rather than a boat off Ghana.
  if (s.lat == 0.0 && s.lng == 0.0) return false;

  // NaN guard. isnan/isinf are macros in <math.h>.
  if (isnan(s.lat) || isnan(s.lng) || isnan(s.alt) ||
      isnan(s.hdop) || isnan(s.speedMph)) {
    return false;
  }
  if (isinf(s.lat) || isinf(s.lng) || isinf(s.alt) ||
      isinf(s.hdop) || isinf(s.speedMph)) {
    return false;
  }

  // Range checks.
  if (s.lat < -90.0 || s.lat > 90.0) return false;
  if (s.lng < -180.0 || s.lng > 180.0) return false;
  if (s.alt < -1000.0 || s.alt > 50000.0) return false;
  if (s.hdop <= 0.0) return false;
  if (s.speedMph < 0.0 || s.speedMph > 500.0) return false;

  return true;
}

bool isNumericString(const char* s, size_t max_len) {
  if (s == nullptr) return false;
  const size_t len = strlen(s);
  if (len == 0 || len > max_len) return false;
  for (size_t i = 0; i < len; i++) {
    const char c = s[i];
    const bool ok = (c >= '0' && c <= '9') || c == '.' || c == '-';
    if (!ok) return false;
  }
  return true;
}

}  // namespace gps_validation
