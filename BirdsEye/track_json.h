#pragma once

#include <stddef.h>
#include <stdint.h>

#include "course_creator.h"

///////////////////////////////////////////
// TRACK JSON WRITER (pure, host-tested)
//
// The firmware's first track-file WRITER — until the on-device course
// creator (plan 0002 §5) it only ever read them. Emits the same object
// format `parseTrackFile()` reads and the webapp round-trips:
//
//   {"longName":"N260803_1432","shortName":"08031432","type":"sprint",
//    "defaultCourse":"N260803_1432",
//    "courses":[{"name":"...","start_a_lat":...}]}
//
// Text is built by hand rather than through ArduinoJson because the
// coordinate formatting is the hard part either way: Arduino's snprintf
// has no working "%f" on this core (which is why the sketch reaches for
// dtostrf everywhere), and dtostrf does not exist on the host. So
// coordinates go through formatFixed() — integer math, identical on both
// targets, testable to the last digit. Same reasoning as gps_time's
// hand-rolled u64ToDecimalString.
//
// Every emitter returns the number of characters written (excluding the
// NUL) or -1 when the buffer is too small. A truncated track file is
// worse than no file, so callers must check.
//
// No Arduino headers — compiled into both the firmware and the host test
// harness (tests/track_json_test.cpp).
///////////////////////////////////////////

namespace track_json {

// Decimal places used for every coordinate. 1e-8 degrees is ~1.1 mm —
// far below GPS noise, and it matches the precision the DOVEX rows and
// the webapp's track files already carry.
constexpr uint8_t kCoordDecimals = 8;

// Longest coordinate this can emit: sign + 3 whole digits + '.' + 8 = 13,
// plus NUL.
constexpr size_t kCoordSize = 16;

/**
 * @brief Fixed-point decimal formatter, e.g. -97.12345678.
 *
 * Rounds half away from zero. Values are scaled through int64, which is
 * exact for any coordinate (180 * 1e8 is far inside the 2^53 range where
 * doubles represent integers exactly).
 *
 * @return chars written excluding the NUL, or -1 if it does not fit.
 */
int formatFixed(char* out, size_t outSize, double value, uint8_t decimals);

/**
 * @brief Emit one course object — the element of the "courses" array.
 *
 * Optional lines are emitted only when captured, matching how
 * parseTrackFile() probes for them with containsKey(). `dateCreated` is
 * written for SPRINT courses only: the webapp documents Course.dateCreated
 * as sprint-only and the device's own newest-course selection is the only
 * thing that reads it back.
 *
 * @return chars written excluding the NUL, or -1 if it does not fit.
 */
int formatCourse(char* out, size_t outSize,
                 const course_creator::State& course,
                 const char* courseName,
                 const char* dateCreated);

/**
 * @brief Emit a complete new track file holding exactly this one course.
 *
 * @return chars written excluding the NUL, or -1 if it does not fit.
 */
int formatTrackFile(char* out, size_t outSize,
                    const char* longName, const char* shortName,
                    const course_creator::State& course,
                    const char* courseName,
                    const char* dateCreated);

}  // namespace track_json
