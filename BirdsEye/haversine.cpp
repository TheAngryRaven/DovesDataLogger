#include "haversine.h"

#include <math.h>

// Local copy of DEG_TO_RAD so this TU is independent of Arduino headers
// (the firmware build picks up Arduino's own DEG_TO_RAD via replay.ino's
// existing include chain — defining our own here is harmless).
namespace {
constexpr double kDegToRad = 0.017453292519943295;  // M_PI / 180.0
constexpr double kEarthRadiusMiles = 3958.8;
}  // namespace

double haversineDistanceMiles(double lat1, double lng1,
                              double lat2, double lng2) {
  const double dLat = (lat2 - lat1) * kDegToRad;
  const double dLng = (lng2 - lng1) * kDegToRad;

  const double a = sin(dLat / 2) * sin(dLat / 2) +
                   cos(lat1 * kDegToRad) * cos(lat2 * kDegToRad) *
                   sin(dLng / 2) * sin(dLng / 2);

  const double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return kEarthRadiusMiles * c;
}
