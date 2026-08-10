#include "course_prune.h"

#include <string.h>

namespace course_prune {

namespace {

bool isDigits(const char* s, uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    if (s[i] < '0' || s[i] > '9') return false;
  }
  return true;
}

/** Plain string compare that treats null as `""`, so callers needn't guard. */
int cmp(const char* a, const char* b) {
  return strcmp(a == nullptr ? "" : a, b == nullptr ? "" : b);
}

}  // namespace

bool isDeviceGeneratedName(const char* name) {
  // N YYMMDD _ HHMM — exactly what course_creator::generatedName writes.
  // Length is checked exactly: "N260803_1432 CW" is a name someone typed that
  // happens to start the same way, and it is not ours to assume is disposable.
  if (name == nullptr) return false;
  if (strlen(name) != 12) return false;
  if (name[0] != 'N') return false;
  if (!isDigits(name + 1, 6)) return false;
  if (name[7] != '_') return false;
  return isDigits(name + 8, 4);
}

void dropOrder(const CourseSummary* courses, uint8_t count, uint8_t* outOrder) {
  if (courses == nullptr || outOrder == nullptr) return;
  for (uint8_t i = 0; i < count; i++) outOrder[i] = i;

  // Insertion sort: `count` is bounded by MAX_LAYOUTS (10), and a stable,
  // obvious sort beats a clever one at this size.
  for (uint8_t i = 1; i < count; i++) {
    const uint8_t key = outOrder[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0) {
      const uint8_t other = outOrder[j];

      // 1. Named before unnamed: a renamed course is provably saved in the
      //    app, a device-named one may exist nowhere but this card.
      const bool keyGen = isDeviceGeneratedName(courses[key].name);
      const bool otherGen = isDeviceGeneratedName(courses[other].name);
      bool keyFirst;
      if (keyGen != otherGen) {
        keyFirst = !keyGen;
      } else {
        // 2. Then oldest first. A missing stamp sorts oldest — it predates
        //    the field, and calling it newest would protect it ahead of the
        //    run walked this morning.
        const int byDate = cmp(courses[key].dateCreated, courses[other].dateCreated);
        // 3. Ties by index, so the order is total and repeatable.
        keyFirst = byDate < 0 || (byDate == 0 && key < other);
      }

      if (!keyFirst) break;
      outOrder[j + 1] = other;
      j--;
    }
    outOrder[j + 1] = key;
  }
}

bool needsConfirm(const CourseSummary* courses, uint8_t count,
                  const uint8_t* order, uint8_t dropCount) {
  if (courses == nullptr || order == nullptr) return false;
  if (dropCount > count) dropCount = count;
  for (uint8_t i = 0; i < dropCount; i++) {
    if (isDeviceGeneratedName(courses[order[i]].name)) return true;
  }
  return false;
}

}  // namespace course_prune
