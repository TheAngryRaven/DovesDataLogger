#ifndef COURSE_PRUNE_H
#define COURSE_PRUNE_H

#include <stddef.h>
#include <stdint.h>

/**
 * @file course_prune.h
 * @brief Deciding which sprint courses to drop when a track file is full
 *        (plan 0005).
 *
 * A sprint venue re-lays its cones every event and the on-device creator mints
 * a dated course each time, so a track file grows without bound. The whole file
 * has to fit JSON_BUFFER_SIZE on the next boot — past that the read is cut
 * mid-JSON, the parse fails, buildTrackList() adds no manifest entry, and the
 * track stops being detected at the venue entirely.
 *
 * Before this, a full track meant the walked course was simply lost, on a
 * battery, in a field, at an event. Now the oldest courses can be dropped to
 * make room. This unit decides WHICH, and whether the user has to be asked
 * first; the SD side owns the measuring and the rewrite.
 *
 * Pure and Arduino-free so the ordering rule is host-tested.
 */
namespace course_prune {

/** One course on the card, as the pruner needs to see it. */
struct CourseSummary {
  /** Course name. Never null. */
  const char* name = "";
  /**
   * Sortable `YYYY-MM-DDTHH:MM` stamp, or `""` when the course has none.
   * Compared as a plain string — the same comparison the webapp makes.
   */
  const char* dateCreated = "";
};

/**
 * True when `name` is one the on-device creator generated: `N{YYMMDD}_{HHMM}`.
 *
 * This is the "has anyone kept a copy?" test, and it is the whole basis for
 * asking or not asking. The device has NO text entry, ever (plan 0002 §5), so
 * a course still carrying this shape has never been through the webapp — the
 * card may be the only place it exists. A course with any other name was
 * renamed in the app, which means the app has it and it rides cloud sync, so
 * dropping it from the card loses nothing.
 */
bool isDeviceGeneratedName(const char* name);

/**
 * The order courses should be dropped in, best candidate first.
 *
 * Fills `outOrder` with all `count` indices, so a caller can walk as far down
 * the list as it needs to free enough room. Two keys:
 *
 *  1. **Named before unnamed.** A renamed course is provably saved elsewhere;
 *     a device-named one may not be. Safety before age.
 *  2. **Oldest first**, by `dateCreated` ascending. A course with no stamp
 *     sorts oldest — it predates the field, and treating a missing value as
 *     newest would protect it ahead of the run walked this morning.
 *
 * Ties resolve to the lower index, so the order is total and the same inputs
 * always give the same answer.
 */
void dropOrder(const CourseSummary* courses, uint8_t count, uint8_t* outOrder);

/**
 * True when dropping the first `dropCount` of `dropOrder` would remove a course
 * the device itself named — i.e. one that may exist nowhere else.
 *
 * The caller uses this to decide between doing it silently and asking first.
 * Dropping only renamed courses is not a decision worth interrupting someone
 * for; dropping a run that exists only on this card is.
 */
bool needsConfirm(const CourseSummary* courses, uint8_t count,
                  const uint8_t* order, uint8_t dropCount);

}  // namespace course_prune

#endif  // COURSE_PRUNE_H
