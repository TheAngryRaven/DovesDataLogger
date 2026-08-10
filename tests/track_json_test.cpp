#include "doctest.h"

#include <string.h>

#include <string>

#include "course_creator.h"
#include "track_json.h"

using namespace track_json;
using course_creator::CourseKind;
using course_creator::LineId;

namespace {

// A course with the given lines already captured — bypasses the UI flow,
// which course_creator_test covers.
course_creator::State makeCourse(CourseKind kind) {
  course_creator::State s;
  s.kind = kind;
  return s;
}

void setLine(course_creator::State& s, LineId id,
             double aLat, double aLon, double bLat, double bLon) {
  course_creator::Line& l = s.lines[static_cast<uint8_t>(id)];
  l.aLat = aLat;
  l.aLon = aLon;
  l.bLat = bLat;
  l.bLon = bLon;
  l.hasA = true;
  l.hasB = true;
}

std::string course(const course_creator::State& s, const char* name, const char* date = "") {
  char buf[1024];
  const int n = formatCourse(buf, sizeof(buf), s, name, date);
  REQUIRE(n > 0);
  CHECK(strlen(buf) == static_cast<size_t>(n));
  return std::string(buf);
}

}  // namespace

// ─── formatFixed ────────────────────────────────────────────────────────────

TEST_CASE("formatFixed writes a plain fixed-point decimal") {
  char buf[kCoordSize];
  REQUIRE(formatFixed(buf, sizeof(buf), 28.41270817, 8) > 0);
  CHECK(std::string(buf) == "28.41270817");

  REQUIRE(formatFixed(buf, sizeof(buf), -97.12345678, 8) > 0);
  CHECK(std::string(buf) == "-97.12345678");
}

TEST_CASE("formatFixed zero-pads the fraction") {
  char buf[kCoordSize];
  // The leading zeros of the fraction are load-bearing: "35.00000001"
  // written as "35.1" is 11 km away.
  REQUIRE(formatFixed(buf, sizeof(buf), 35.00000001, 8) > 0);
  CHECK(std::string(buf) == "35.00000001");

  REQUIRE(formatFixed(buf, sizeof(buf), 35.0, 8) > 0);
  CHECK(std::string(buf) == "35.00000000");
}

TEST_CASE("formatFixed handles zero and small negatives") {
  char buf[kCoordSize];
  REQUIRE(formatFixed(buf, sizeof(buf), 0.0, 8) > 0);
  CHECK(std::string(buf) == "0.00000000");

  // A value that rounds to zero must not come out as "-0.00000000" — a
  // reader would take the sign at face value.
  REQUIRE(formatFixed(buf, sizeof(buf), -0.000000001, 8) > 0);
  CHECK(std::string(buf) == "0.00000000");

  REQUIRE(formatFixed(buf, sizeof(buf), -0.5, 8) > 0);
  CHECK(std::string(buf) == "-0.50000000");
}

TEST_CASE("formatFixed rounds half away from zero") {
  // 1.125 is exactly representable in binary, so this really is the
  // halfway case. Don't reach for something like 1.005 — that literal is
  // actually 1.00499999... and would be testing the double, not the
  // rounding.
  char buf[kCoordSize];
  REQUIRE(formatFixed(buf, sizeof(buf), 1.125, 2) > 0);
  CHECK(std::string(buf) == "1.13");
  REQUIRE(formatFixed(buf, sizeof(buf), -1.125, 2) > 0);
  CHECK(std::string(buf) == "-1.13");
}

TEST_CASE("formatFixed keeps full precision at the coordinate extremes") {
  char buf[kCoordSize];
  REQUIRE(formatFixed(buf, sizeof(buf), -179.99999999, 8) > 0);
  CHECK(std::string(buf) == "-179.99999999");
  REQUIRE(formatFixed(buf, sizeof(buf), 89.12345678, 8) > 0);
  CHECK(std::string(buf) == "89.12345678");
}

TEST_CASE("formatFixed reports a buffer that is too small") {
  char tiny[4];
  CHECK(formatFixed(tiny, sizeof(tiny), 28.41270817, 8) == -1);
  CHECK(formatFixed(nullptr, 32, 1.0, 8) == -1);
  CHECK(formatFixed(tiny, 0, 1.0, 8) == -1);
}

// ─── formatCourse ───────────────────────────────────────────────────────────

TEST_CASE("a minimal circuit course emits just its start/finish line") {
  course_creator::State s = makeCourse(CourseKind::kCircuit);
  setLine(s, LineId::kStart, 35.1, -97.1, 35.2, -97.2);

  CHECK(course(s, "N260803_1432") ==
        "{\"name\":\"N260803_1432\","
        "\"start_a_lat\":35.10000000,\"start_a_lng\":-97.10000000,"
        "\"start_b_lat\":35.20000000,\"start_b_lng\":-97.20000000}");
}

TEST_CASE("captured sectors are emitted, uncaptured ones are absent") {
  course_creator::State s = makeCourse(CourseKind::kCircuit);
  setLine(s, LineId::kStart, 1.0, 2.0, 3.0, 4.0);
  setLine(s, LineId::kSector2, 5.0, 6.0, 7.0, 8.0);
  setLine(s, LineId::kSector3, 9.0, 10.0, 11.0, 12.0);

  const std::string json = course(s, "C");
  CHECK(json.find("\"sector_2_a_lat\":5.00000000") != std::string::npos);
  CHECK(json.find("\"sector_3_b_lng\":12.00000000") != std::string::npos);

  // parseTrackFile() probes for these keys with containsKey(), so an
  // absent line and a line of zeroes mean very different things.
  course_creator::State bare = makeCourse(CourseKind::kCircuit);
  setLine(bare, LineId::kStart, 1.0, 2.0, 3.0, 4.0);
  CHECK(course(bare, "C").find("sector_2") == std::string::npos);
  CHECK(course(bare, "C").find("sector_3") == std::string::npos);
}

TEST_CASE("a half-captured line is not emitted") {
  course_creator::State s = makeCourse(CourseKind::kCircuit);
  setLine(s, LineId::kStart, 1.0, 2.0, 3.0, 4.0);
  s.lines[static_cast<uint8_t>(LineId::kSector2)].hasA = true;  // point B never taken

  CHECK(course(s, "C").find("sector_2") == std::string::npos);
}

TEST_CASE("a sprint course carries its finish line and date_created") {
  course_creator::State s = makeCourse(CourseKind::kSprint);
  setLine(s, LineId::kStart, 35.1, -97.1, 35.2, -97.2);
  setLine(s, LineId::kFinish, 36.1, -98.1, 36.2, -98.2);

  const std::string json = course(s, "N260803_1432", "2026-08-03T14:32");
  CHECK(json.find("\"finish_a_lat\":36.10000000") != std::string::npos);
  CHECK(json.find("\"finish_b_lng\":-98.20000000") != std::string::npos);
  CHECK(json.find("\"date_created\":\"2026-08-03T14:32\"") != std::string::npos);
}

TEST_CASE("a circuit course carries neither a finish line nor date_created") {
  course_creator::State s = makeCourse(CourseKind::kCircuit);
  setLine(s, LineId::kStart, 1.0, 2.0, 3.0, 4.0);
  // Populated but must not be written: finish is meaningless on a circuit,
  // and the webapp documents Course.dateCreated as sprint-only.
  setLine(s, LineId::kFinish, 9.0, 9.0, 9.0, 9.0);

  const std::string json = course(s, "C", "2026-08-03T14:32");
  CHECK(json.find("finish") == std::string::npos);
  CHECK(json.find("date_created") == std::string::npos);
}

TEST_CASE("an empty date_created is omitted rather than written blank") {
  course_creator::State s = makeCourse(CourseKind::kSprint);
  setLine(s, LineId::kStart, 1.0, 2.0, 3.0, 4.0);
  setLine(s, LineId::kFinish, 5.0, 6.0, 7.0, 8.0);
  CHECK(course(s, "S", "").find("date_created") == std::string::npos);
}

TEST_CASE("formatCourse reports a buffer that is too small") {
  course_creator::State s = makeCourse(CourseKind::kCircuit);
  setLine(s, LineId::kStart, 35.1, -97.1, 35.2, -97.2);

  char buf[40];
  // A truncated track file is worse than no file — the device would load
  // it and fail to parse, so the caller has to be told.
  CHECK(formatCourse(buf, sizeof(buf), s, "N260803_1432", "") == -1);
}

// ─── formatTrackFile ────────────────────────────────────────────────────────

TEST_CASE("a new circuit track file is a complete object with one course") {
  course_creator::State s = makeCourse(CourseKind::kCircuit);
  setLine(s, LineId::kStart, 35.1, -97.1, 35.2, -97.2);

  char buf[1024];
  const int n = formatTrackFile(buf, sizeof(buf), "N260803_1432", "08031432",
                                s, "N260803_1432", "");
  REQUIRE(n > 0);
  CHECK(std::string(buf) ==
        "{\"longName\":\"N260803_1432\",\"shortName\":\"08031432\","
        "\"defaultCourse\":\"N260803_1432\",\"courses\":["
        "{\"name\":\"N260803_1432\","
        "\"start_a_lat\":35.10000000,\"start_a_lng\":-97.10000000,"
        "\"start_b_lat\":35.20000000,\"start_b_lng\":-97.20000000}]}");
}

TEST_CASE("a new sprint track file declares its type") {
  course_creator::State s = makeCourse(CourseKind::kSprint);
  setLine(s, LineId::kStart, 1.0, 2.0, 3.0, 4.0);
  setLine(s, LineId::kFinish, 5.0, 6.0, 7.0, 8.0);

  char buf[1024];
  REQUIRE(formatTrackFile(buf, sizeof(buf), "N260803_1432", "08031432",
                          s, "N260803_1432", "2026-08-03T14:32") > 0);
  const std::string json(buf);
  // Redundant with the folder (which is authoritative), but it means a
  // file moved by hand still says what it is.
  CHECK(json.find("\"type\":\"sprint\"") != std::string::npos);
  CHECK(json.find("\"courses\":[{") != std::string::npos);
  CHECK(json.substr(json.size() - 2) == "]}");
}

TEST_CASE("formatTrackFile reports a buffer that is too small") {
  course_creator::State s = makeCourse(CourseKind::kCircuit);
  setLine(s, LineId::kStart, 35.1, -97.1, 35.2, -97.2);

  char buf[64];
  CHECK(formatTrackFile(buf, sizeof(buf), "N260803_1432", "08031432",
                        s, "N260803_1432", "") == -1);
}
