#include "sd_access_policy.h"

namespace sd_access_policy {

bool canAcquire(int current, int requested) {
  if (current == requested) return true;  // idempotent re-acquire
  // Free, or held only by the preemptible (brief, leak-recoverable)
  // track-parse mode.
  return current == kNone || current == kTrackParse;
}

bool releaseClears(int current, int releasing) {
  return current == releasing && current != kNone;
}

}  // namespace sd_access_policy
