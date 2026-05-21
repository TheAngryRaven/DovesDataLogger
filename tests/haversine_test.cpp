#include "doctest.h"
#include "haversine.h"

// ---------------------------------------------------------------------------
// Reference distances. All "expected" values are from independent online
// haversine calculators and a few cross-checked with great-circle tooling.
// We allow ~0.5% slop because haversine assumes a perfect sphere; the actual
// Earth is an oblate spheroid (WGS-84). Track-detection tolerance is ±5 mi
// so this is plenty accurate for the actual use case.
// ---------------------------------------------------------------------------

TEST_CASE("haversineDistanceMiles - identity (same point)") {
    CHECK(haversineDistanceMiles(40.7128, -74.0060, 40.7128, -74.0060)
          == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(haversineDistanceMiles(0.0, 0.0, 0.0, 0.0)
          == doctest::Approx(0.0).epsilon(1e-9));
    // Real-world track point (OKC start/finish): identity should be 0.
    CHECK(haversineDistanceMiles(28.4127081705638, -81.3797326641803,
                                 28.4127081705638, -81.3797326641803)
          == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("haversineDistanceMiles - symmetric") {
    // d(A,B) must equal d(B,A) for any A,B.
    const double d1 = haversineDistanceMiles(40.7128, -74.0060,
                                             34.0522, -118.2437);
    const double d2 = haversineDistanceMiles(34.0522, -118.2437,
                                             40.7128, -74.0060);
    CHECK(d1 == doctest::Approx(d2).epsilon(1e-9));
}

TEST_CASE("haversineDistanceMiles - NYC to LA (~2451 mi)") {
    // NYC (40.7128, -74.0060) -> LA (34.0522, -118.2437)
    // Reference: ~2451 mi great-circle distance.
    const double miles = haversineDistanceMiles(40.7128, -74.0060,
                                                34.0522, -118.2437);
    CHECK(miles == doctest::Approx(2451.0).epsilon(0.005));  // 0.5%
}

TEST_CASE("haversineDistanceMiles - NYC to Chicago (~711 mi)") {
    // NYC (40.7128, -74.0060) -> Chicago (41.8781, -87.6298)
    // Reference: ~711 mi great-circle distance (widely-cited value).
    const double miles = haversineDistanceMiles(40.7128, -74.0060,
                                                41.8781, -87.6298);
    CHECK(miles == doctest::Approx(711.0).epsilon(0.005));  // 0.5%
}

TEST_CASE("haversineDistanceMiles - one degree of equator (~69.09 mi)") {
    // One degree of longitude on the equator should be pi/180 * R
    // = 0.0174533 * 3958.8 = 69.0934 mi. This is a pure math check
    // — independent of any reference table.
    const double miles = haversineDistanceMiles(0.0, 0.0, 0.0, 1.0);
    CHECK(miles == doctest::Approx(69.0934).epsilon(1e-3));
}

TEST_CASE("haversineDistanceMiles - antipodal points (half circumference)") {
    // Two points exactly opposite on the globe should be ~half Earth's
    // circumference apart: pi * R = pi * 3958.8 ≈ 12437.6 mi.
    const double miles = haversineDistanceMiles(0.0, 0.0, 0.0, 180.0);
    CHECK(miles == doctest::Approx(12437.6).epsilon(1e-3));
}

TEST_CASE("haversineDistanceMiles - quarter-circle from equator to pole") {
    // Equator to North Pole = quarter of the circumference = pi/2 * R
    // ≈ 6218.8 mi.
    const double miles = haversineDistanceMiles(0.0, 0.0, 90.0, 0.0);
    CHECK(miles == doctest::Approx(6218.8).epsilon(1e-3));
}

TEST_CASE("haversineDistanceMiles - very short distance (track-scale)") {
    // Two points ~10 meters apart on an OKC sector line. 10 m ≈ 0.00621 mi.
    // start_a: 28.4127081705638, -81.3797326641803
    // start_b: 28.4127303867932, -81.3795704875378
    // Reference computed independently: ~0.0103 mi (16.5 m).
    const double miles = haversineDistanceMiles(
        28.4127081705638, -81.3797326641803,
        28.4127303867932, -81.3795704875378);
    CHECK(miles == doctest::Approx(0.0103).epsilon(0.05));  // 5% slop at this scale
    // Always non-negative.
    CHECK(miles > 0.0);
}

TEST_CASE("haversineDistanceMiles - never negative") {
    // Sweep a range of inputs; distance must always be >= 0.
    for (double lat = -85.0; lat <= 85.0; lat += 17.0) {
        for (double lng = -170.0; lng <= 170.0; lng += 47.0) {
            CHECK(haversineDistanceMiles(0.0, 0.0, lat, lng) >= 0.0);
        }
    }
}

TEST_CASE("haversineDistanceMiles - antimeridian wrap (handled correctly)") {
    // -179°E and +179°E are 2° apart, not 358°. sin² in the haversine
    // formula is symmetric, so sin²(358°/2) = sin²(179°) = sin²(1°)
    // collapses to the short-way answer naturally. ~138 mi at the equator.
    const double wrap = haversineDistanceMiles(0.0, -179.0, 0.0, 179.0);
    CHECK(wrap == doctest::Approx(2.0 * 69.0934).epsilon(1e-3));
}
