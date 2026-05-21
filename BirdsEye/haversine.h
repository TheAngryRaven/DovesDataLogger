#pragma once

///////////////////////////////////////////
// HAVERSINE DISTANCE
// Pure math helper extracted so it can be exercised by host-side
// unit tests. Used by trackDetectionLoop() in BirdsEye.ino for
// proximity-based track matching.
///////////////////////////////////////////

/**
 * @brief Great-circle distance between two latitude/longitude
 *        points in miles, using the haversine formula on a
 *        spherical Earth model (R = 3958.8 mi).
 *
 * Inputs are in decimal degrees. Returns a non-negative distance.
 * Accurate to ~0.5% on Earth for typical short distances; that's
 * well within track-detection tolerances (we match within 5 miles).
 */
double haversineDistanceMiles(double lat1, double lng1,
                              double lat2, double lng2);
