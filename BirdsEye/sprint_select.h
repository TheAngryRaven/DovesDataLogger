#pragma once

///////////////////////////////////////////
// SPRINT MODE SELECTION LOGIC (pure, host-tested)
//
// Sprint courses are dated, disposable layouts of a persistent venue
// (autocross re-lays cones every event), so course selection is
// date-based: load the newest course by its `date_created` field —
// a sortable ISO-8601 timestamp ("YYYY-MM-DDTHH:MM", any prefix
// accepted) stamped by whatever created the course (webapp or the
// future on-device creator). Lexicographic max == newest.
//
// Mode selection follows the detected track's folder (/TRACKS vs
// /TRACKS/SPRINT). When BOTH kinds are within detection range, the
// `race_mode` preference setting breaks the tie (plan 0002 §7 Q1):
//   - pref circuit (default): circuit wins — UNLESS the sprint track
//     has a course created *today* (it's an event day), then sprint.
//   - pref sprint: sprint always wins (fixed sprint courses, e.g. a
//     permanent rally layout, are never re-created per event).
// The preference never overrides what is actually detected — with only
// one kind in range, that kind is used regardless of the setting.
//
// No Arduino types — compiled into both the firmware and the host
// test harness (tests/sprint_select_test.cpp).
///////////////////////////////////////////

namespace sprint_select {

// Track kinds — values mirror TrackManifestEntry::kind.
enum Kind : unsigned char {
  kCircuit = 0,
  kSprint = 1,
};

// The race_mode preference setting ("circuit" default / "sprint").
enum Pref : unsigned char {
  kPrefCircuit = 0,
  kPrefSprint = 1,
};

/**
 * @brief Compare two date_created strings (sortable ISO-8601 prefixes).
 *
 * Plain lexicographic compare — valid because the format is fixed-width
 * most-significant-first. NULL or empty sorts oldest (a course with no
 * date must never beat a dated one).
 *
 * @return <0 if a older than b, 0 if equal, >0 if a newer than b.
 */
int compareDateCreated(const char* a, const char* b);

/**
 * @brief Index of the newest course by date_created.
 *
 * Ties (including the all-empty legacy case) resolve to the LAST
 * tied index — later entries in a track file were appended later, so
 * on missing/equal dates the most recently added course wins.
 *
 * @param dates Array of date_created strings (entries may be NULL/empty).
 * @param count Number of entries.
 * @return Index of the newest course, or -1 when count <= 0.
 */
int newestCourseIndex(const char* const* dates, int count);

/**
 * @brief True when a date_created string falls on the given day.
 *
 * Compares the "YYYY-MM-DD" prefix (10 chars). Either argument
 * NULL/shorter than a full date returns false.
 */
bool isSameDay(const char* dateCreated, const char* todayIsoDate);

/**
 * @brief The mode tiebreak: which track kind should the session use?
 *
 * @param circuitInRange A circuit track is within detection radius.
 * @param sprintInRange A sprint track is within detection radius.
 * @param pref The race_mode preference setting.
 * @param sprintCourseCreatedToday The in-range sprint track's newest
 *   course was created today (event-day heuristic; only consulted when
 *   both kinds are in range and pref is circuit).
 * @return The kind to use. With neither in range, returns kCircuit
 *   (callers gate on having a detection at all).
 */
Kind chooseKind(bool circuitInRange, bool sprintInRange, Pref pref,
                bool sprintCourseCreatedToday);

}  // namespace sprint_select
