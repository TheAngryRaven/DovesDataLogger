#include "doctest.h"

#include "../BirdsEye/course_prune.h"

#include <string.h>

using namespace course_prune;

namespace {

CourseSummary c(const char* name, const char* date = "") {
  CourseSummary s;
  s.name = name;
  s.dateCreated = date;
  return s;
}

// The names, in the order they'd be dropped.
void orderNames(const CourseSummary* courses, uint8_t n, const char** out) {
  uint8_t order[16];
  dropOrder(courses, n, order);
  for (uint8_t i = 0; i < n; i++) out[i] = courses[order[i]].name;
}

}  // namespace

TEST_CASE("isDeviceGeneratedName matches what the creator writes") {
  // course_creator::generatedName: 'N' + YYMMDD + '_' + HHMM.
  CHECK(isDeviceGeneratedName("N260803_1432"));
  CHECK(isDeviceGeneratedName("N000101_0000"));
  CHECK(isDeviceGeneratedName("N991231_2359"));
}

TEST_CASE("isDeviceGeneratedName rejects a name someone typed") {
  CHECK_FALSE(isDeviceGeneratedName("Sunset Park"));
  CHECK_FALSE(isDeviceGeneratedName("Course 1"));
  CHECK_FALSE(isDeviceGeneratedName(""));
  CHECK_FALSE(isDeviceGeneratedName(nullptr));
}

TEST_CASE("isDeviceGeneratedName is exact about the shape") {
  CHECK_FALSE(isDeviceGeneratedName("N260803_143"));    // too short
  CHECK_FALSE(isDeviceGeneratedName("N260803_14322"));  // too long
  CHECK_FALSE(isDeviceGeneratedName("X260803_1432"));   // wrong prefix
  CHECK_FALSE(isDeviceGeneratedName("N260803-1432"));   // wrong separator
  CHECK_FALSE(isDeviceGeneratedName("N26080A_1432"));   // non-digit
  CHECK_FALSE(isDeviceGeneratedName("N260803_14A2"));   // non-digit
}

// A typed name that merely STARTS like ours is not ours to assume is
// disposable — the length check is what keeps it safe.
TEST_CASE("isDeviceGeneratedName rejects a generated name someone extended") {
  CHECK_FALSE(isDeviceGeneratedName("N260803_1432 CW"));
}

TEST_CASE("dropOrder drops the oldest first") {
  const CourseSummary courses[] = {
      c("Mar", "2026-03-21T11:15"),
      c("Jan", "2026-01-04T09:00"),
      c("Aug", "2026-08-09T14:32"),
  };
  const char* got[3];
  orderNames(courses, 3, got);
  CHECK(strcmp(got[0], "Jan") == 0);
  CHECK(strcmp(got[1], "Mar") == 0);
  CHECK(strcmp(got[2], "Aug") == 0);
}

// Safety before age: a renamed course is provably in the app and rides cloud
// sync, so it goes first even when it is the NEWEST thing on the card.
TEST_CASE("dropOrder prefers a renamed course over a device-named one") {
  const CourseSummary courses[] = {
      c("N260101_0900", "2026-01-01T09:00"),  // oldest, but only on this card
      c("Sunset Park", "2026-08-09T14:32"),   // newest, but saved in the app
  };
  const char* got[2];
  orderNames(courses, 2, got);
  CHECK(strcmp(got[0], "Sunset Park") == 0);
  CHECK(strcmp(got[1], "N260101_0900") == 0);
}

TEST_CASE("dropOrder still sorts by age within each group") {
  const CourseSummary courses[] = {
      c("N260301_0900", "2026-03-01T09:00"),
      c("Named New", "2026-08-01T09:00"),
      c("N260101_0900", "2026-01-01T09:00"),
      c("Named Old", "2026-02-01T09:00"),
  };
  const char* got[4];
  orderNames(courses, 4, got);
  CHECK(strcmp(got[0], "Named Old") == 0);
  CHECK(strcmp(got[1], "Named New") == 0);
  CHECK(strcmp(got[2], "N260101_0900") == 0);
  CHECK(strcmp(got[3], "N260301_0900") == 0);
}

// A missing stamp predates the field. Calling it "newest" would protect it
// ahead of the run walked this morning.
TEST_CASE("dropOrder treats a missing stamp as the oldest") {
  const CourseSummary courses[] = {
      c("Stamped", "2026-08-09T14:32"),
      c("Bare", ""),
  };
  const char* got[2];
  orderNames(courses, 2, got);
  CHECK(strcmp(got[0], "Bare") == 0);
  CHECK(strcmp(got[1], "Stamped") == 0);
}

TEST_CASE("dropOrder handles a null stamp like an empty one") {
  CourseSummary courses[2];
  courses[0].name = "Stamped";
  courses[0].dateCreated = "2026-08-09T14:32";
  courses[1].name = "Null";
  courses[1].dateCreated = nullptr;
  const char* got[2];
  orderNames(courses, 2, got);
  CHECK(strcmp(got[0], "Null") == 0);
}

TEST_CASE("dropOrder breaks an exact tie by index, repeatably") {
  const CourseSummary courses[] = {
      c("First", "2026-08-09T14:32"),
      c("Second", "2026-08-09T14:32"),
  };
  uint8_t a[2];
  uint8_t b[2];
  dropOrder(courses, 2, a);
  dropOrder(courses, 2, b);
  CHECK(a[0] == 0);
  CHECK(a[1] == 1);
  CHECK(a[0] == b[0]);
  CHECK(a[1] == b[1]);
}

TEST_CASE("dropOrder returns every index exactly once") {
  const CourseSummary courses[] = {
      c("D", "2026-04-01T09:00"), c("A", "2026-01-01T09:00"),
      c("C", "2026-03-01T09:00"), c("B", "2026-02-01T09:00"),
      c("E", ""),
  };
  uint8_t order[5];
  dropOrder(courses, 5, order);
  bool seen[5] = {false, false, false, false, false};
  for (uint8_t i = 0; i < 5; i++) {
    REQUIRE(order[i] < 5);
    CHECK_FALSE(seen[order[i]]);
    seen[order[i]] = true;
  }
}

TEST_CASE("dropOrder survives degenerate inputs") {
  uint8_t order[2] = {9, 9};
  const CourseSummary one[] = {c("Only", "2026-01-01T09:00")};
  dropOrder(one, 1, order);
  CHECK(order[0] == 0);

  dropOrder(one, 0, order);  // nothing written, nothing crashed
  dropOrder(nullptr, 2, order);
  dropOrder(one, 1, nullptr);
}

// Dropping only renamed courses is not worth interrupting anyone for; dropping
// a run that exists nowhere else is.
TEST_CASE("needsConfirm is false while only renamed courses are dropped") {
  const CourseSummary courses[] = {
      c("Named Old", "2026-01-01T09:00"),
      c("Named New", "2026-02-01T09:00"),
      c("N260301_0900", "2026-03-01T09:00"),
  };
  uint8_t order[3];
  dropOrder(courses, 3, order);
  CHECK_FALSE(needsConfirm(courses, 3, order, 1));
  CHECK_FALSE(needsConfirm(courses, 3, order, 2));
}

TEST_CASE("needsConfirm turns true once a device-named course is reached") {
  const CourseSummary courses[] = {
      c("Named Old", "2026-01-01T09:00"),
      c("Named New", "2026-02-01T09:00"),
      c("N260301_0900", "2026-03-01T09:00"),
  };
  uint8_t order[3];
  dropOrder(courses, 3, order);
  CHECK(needsConfirm(courses, 3, order, 3));
}

TEST_CASE("needsConfirm is false when nothing is being dropped") {
  const CourseSummary courses[] = {c("N260301_0900", "2026-03-01T09:00")};
  uint8_t order[1];
  dropOrder(courses, 1, order);
  CHECK_FALSE(needsConfirm(courses, 1, order, 0));
}

TEST_CASE("needsConfirm clamps a drop count past the end") {
  const CourseSummary courses[] = {c("Named", "2026-01-01T09:00")};
  uint8_t order[1];
  dropOrder(courses, 1, order);
  CHECK_FALSE(needsConfirm(courses, 1, order, 200));
}

TEST_CASE("needsConfirm survives null inputs") {
  uint8_t order[1] = {0};
  const CourseSummary courses[] = {c("Named", "2026-01-01T09:00")};
  CHECK_FALSE(needsConfirm(nullptr, 1, order, 1));
  CHECK_FALSE(needsConfirm(courses, 1, nullptr, 1));
}
