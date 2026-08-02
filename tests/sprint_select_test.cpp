#include "doctest.h"
#include "sprint_select.h"

using namespace sprint_select;

// ---------------------------------------------------------------------------
// compareDateCreated — lexicographic ISO-8601, empty sorts oldest
// ---------------------------------------------------------------------------

TEST_CASE("compareDateCreated - basic ordering") {
    CHECK(compareDateCreated("2026-08-01T09:00", "2026-08-02T09:00") < 0);
    CHECK(compareDateCreated("2026-08-02T09:00", "2026-08-01T09:00") > 0);
    CHECK(compareDateCreated("2026-08-02T09:00", "2026-08-02T09:00") == 0);
}

TEST_CASE("compareDateCreated - same-day morning vs afternoon relay") {
    // The reason date_created carries a time: courses get re-laid same-day.
    CHECK(compareDateCreated("2026-08-02T08:30", "2026-08-02T13:05") < 0);
}

TEST_CASE("compareDateCreated - year/month boundaries") {
    CHECK(compareDateCreated("2025-12-31T23:59", "2026-01-01T00:00") < 0);
    CHECK(compareDateCreated("2026-09-30T12:00", "2026-10-01T12:00") < 0);
}

TEST_CASE("compareDateCreated - empty and null sort oldest") {
    CHECK(compareDateCreated("", "2026-08-02T09:00") < 0);
    CHECK(compareDateCreated("2026-08-02T09:00", "") > 0);
    CHECK(compareDateCreated(nullptr, "2026-08-02T09:00") < 0);
    CHECK(compareDateCreated("", nullptr) == 0);
    CHECK(compareDateCreated(nullptr, nullptr) == 0);
}

TEST_CASE("compareDateCreated - date-only prefix still orders against timestamps") {
    // A webapp that only wrote a date still sorts correctly vs a full
    // timestamp on a different day; same-day it sorts before any timed
    // entry, which is acceptable (the timed one is the deliberate relay).
    CHECK(compareDateCreated("2026-08-01", "2026-08-02T00:00") < 0);
    CHECK(compareDateCreated("2026-08-02", "2026-08-02T08:00") < 0);
}

// ---------------------------------------------------------------------------
// newestCourseIndex — ties resolve to the LAST index
// ---------------------------------------------------------------------------

TEST_CASE("newestCourseIndex - picks the newest") {
    const char* dates[] = {"2026-07-12T09:00", "2026-08-02T08:30", "2026-07-26T10:00"};
    CHECK(newestCourseIndex(dates, 3) == 1);
}

TEST_CASE("newestCourseIndex - weekly venue accumulation, newest last") {
    const char* dates[] = {"2026-07-12T09:00", "2026-07-19T09:00", "2026-07-26T09:00", "2026-08-02T09:00"};
    CHECK(newestCourseIndex(dates, 4) == 3);
}

TEST_CASE("newestCourseIndex - all-empty legacy file picks the last course") {
    const char* dates[] = {"", "", ""};
    CHECK(newestCourseIndex(dates, 3) == 2);
}

TEST_CASE("newestCourseIndex - tie on equal dates picks the later entry") {
    const char* dates[] = {"2026-08-02T09:00", "2026-08-02T09:00"};
    CHECK(newestCourseIndex(dates, 2) == 1);
}

TEST_CASE("newestCourseIndex - dated beats undated regardless of position") {
    const char* dates[] = {"2026-08-02T09:00", "", ""};
    CHECK(newestCourseIndex(dates, 3) == 0);
}

TEST_CASE("newestCourseIndex - degenerate inputs") {
    const char* one[] = {"2026-08-02T09:00"};
    CHECK(newestCourseIndex(one, 1) == 0);
    CHECK(newestCourseIndex(one, 0) == -1);
    CHECK(newestCourseIndex(nullptr, 3) == -1);
}

// ---------------------------------------------------------------------------
// isSameDay
// ---------------------------------------------------------------------------

TEST_CASE("isSameDay - matches on the date prefix") {
    CHECK(isSameDay("2026-08-02T08:30", "2026-08-02"));
    CHECK(isSameDay("2026-08-02", "2026-08-02"));
    CHECK_FALSE(isSameDay("2026-08-01T23:59", "2026-08-02"));
}

TEST_CASE("isSameDay - malformed/short inputs are never today") {
    CHECK_FALSE(isSameDay("", "2026-08-02"));
    CHECK_FALSE(isSameDay(nullptr, "2026-08-02"));
    CHECK_FALSE(isSameDay("2026-08-02", nullptr));
    CHECK_FALSE(isSameDay("2026-08", "2026-08-02"));
}

// ---------------------------------------------------------------------------
// chooseKind — the mode tiebreak decision table (plan 0002 §7 Q1)
// ---------------------------------------------------------------------------

TEST_CASE("chooseKind - single kind in range wins regardless of preference") {
    CHECK(chooseKind(true, false, kPrefCircuit, false) == kCircuit);
    CHECK(chooseKind(true, false, kPrefSprint, false) == kCircuit);
    CHECK(chooseKind(false, true, kPrefCircuit, false) == kSprint);
    CHECK(chooseKind(false, true, kPrefSprint, true) == kSprint);
    // The event-day flag can't force sprint when no sprint track is near.
    CHECK(chooseKind(true, false, kPrefCircuit, true) == kCircuit);
}

TEST_CASE("chooseKind - both in range, circuit pref: sprint only on event day") {
    CHECK(chooseKind(true, true, kPrefCircuit, false) == kCircuit);
    CHECK(chooseKind(true, true, kPrefCircuit, true) == kSprint);
}

TEST_CASE("chooseKind - both in range, sprint pref always wins (fixed rally course)") {
    CHECK(chooseKind(true, true, kPrefSprint, false) == kSprint);
    CHECK(chooseKind(true, true, kPrefSprint, true) == kSprint);
}

TEST_CASE("chooseKind - neither in range defaults circuit (caller gates)") {
    CHECK(chooseKind(false, false, kPrefCircuit, false) == kCircuit);
    CHECK(chooseKind(false, false, kPrefSprint, false) == kCircuit);
}
