#include "sd_access_policy.h"

namespace sd_access_policy {

bool canAcquire(int current, int requested) {
  if (current == requested) return true;  // idempotent re-acquire
  // Track parse / settings reads are brief, same-task, and use their own
  // File objects — safe to nest inside a session-long logging hold.
  if (current == kLogging && requested == kTrackParse) return true;
  // Free, or held only by the preemptible (brief, leak-recoverable)
  // track-parse mode.
  return current == kNone || current == kTrackParse;
}

int ownerAfterAcquire(int current, int requested) {
  // A nested track parse is a guest of the logging hold: logging stays the
  // recorded owner so the guest's release (which releaseClears() ignores)
  // cannot free the card out from under the open log file.
  if (current == kLogging && requested == kTrackParse) return kLogging;
  return requested;
}

bool releaseClears(int current, int releasing) {
  return current == releasing && current != kNone;
}

}  // namespace sd_access_policy
