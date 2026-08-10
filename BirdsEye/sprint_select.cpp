#include "sprint_select.h"

#include <string.h>

namespace sprint_select {

int compareDateCreated(const char* a, const char* b) {
  const bool aEmpty = (a == nullptr) || (a[0] == '\0');
  const bool bEmpty = (b == nullptr) || (b[0] == '\0');
  if (aEmpty && bEmpty) return 0;
  if (aEmpty) return -1;  // undated sorts oldest
  if (bEmpty) return 1;
  return strcmp(a, b);
}

int newestCourseIndex(const char* const* dates, int count) {
  if (count <= 0 || dates == nullptr) return -1;
  int best = 0;
  for (int i = 1; i < count; i++) {
    // >= : ties resolve to the LAST tied index (later file entries were
    // appended later — see header).
    if (compareDateCreated(dates[i], dates[best]) >= 0) {
      best = i;
    }
  }
  return best;
}

bool isSameDay(const char* dateCreated, const char* todayIsoDate) {
  if (dateCreated == nullptr || todayIsoDate == nullptr) return false;
  // Both must carry at least a full "YYYY-MM-DD".
  if (strlen(dateCreated) < 10 || strlen(todayIsoDate) < 10) return false;
  return strncmp(dateCreated, todayIsoDate, 10) == 0;
}

Kind chooseKind(bool circuitInRange, bool sprintInRange, Pref pref,
                bool sprintCourseCreatedToday) {
  if (sprintInRange && !circuitInRange) return kSprint;
  if (circuitInRange && !sprintInRange) return kCircuit;
  if (!circuitInRange && !sprintInRange) return kCircuit;  // caller gates

  // Both kinds in range — the preference setting breaks the tie.
  if (pref == kPrefSprint) {
    return kSprint;
  }
  // Circuit preference yields only on an event day: the nearby sprint
  // track has a course laid out today.
  return sprintCourseCreatedToday ? kSprint : kCircuit;
}

}  // namespace sprint_select
