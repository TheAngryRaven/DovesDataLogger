# Making room for one more run when the sprint track is full

> Status: **DONE.** Firmware half of a two-repo change; the webapp half is
> `DovesDataViewer/docs/plans/0017-device-course-curation.md`.

## Why this exists

A sprint venue re-lays its cones every event, so the on-device course creator
(plan 0002 §5) mints a dated course each weekend and they all pile into one
track file. That file has to fit `JSON_BUFFER_SIZE` on the next boot, and the
failure past it is total rather than partial: the read is cut mid-JSON,
`deserializeJson` fails, `buildTrackList()` adds no manifest entry, and **the
track stops being detected at the venue at all**.

PR #136 raised the budget 4 KB → 8 KB, which bought headroom — it explicitly did
not bound growth. This is the part that bounds it, from the device's end.

Before this, a full track meant the walked course was simply **lost**. The save
returned `SD_COURSE_WRITE_TOO_BIG`, the line menu said `track file full`, and
there was nothing the driver could do about it — on a battery, in a field, at an
event, with no laptop. That is the whole problem being solved.

## The rule: named first, then oldest

Two keys, in this order, and the order matters more than either key does.

**1. Named before unnamed.** The device has *no text entry, ever* (plan 0002
§5), so a course still called `N260803_1432` has never been through the webapp —
**this card may be the only place it exists**. A course with any other name was
renamed in the app, which means the app has it and it rides cloud sync, so
dropping it from the card loses nothing at all. Safety before age: a renamed
course goes first even when it is the newest thing on the file.

**2. Then oldest first**, by `date_created` ascending, compared as a plain
string — the same comparison `sprint_select` and the webapp make. A course with
no stamp sorts **oldest**: it predates the field, and treating a missing value as
newest would protect it ahead of the run walked this morning.

Ties break by index, so the order is total and the same file always gives the
same answer.

## Asking, or not

Whether the user is interrupted follows directly from key 1:

| Everything being dropped | Behaviour |
|---|---|
| renamed in the webapp | **silent** — it all exists elsewhere; there is nothing at stake |
| includes a device-named course | **confirm first** on `PAGE_COURSE_PRUNE` |

Interrupting someone to confirm the deletion of data that is provably backed up
trains them to dismiss the prompt, which is how the one that *does* matter gets
waved through.

**Circuit tracks are deliberately left as a dead end.** Their layouts are all
still driven — there is no "old" one — so there is nothing safe to drop and the
page just says the track is full.

## Where each piece lives

- **`course_prune.{h,cpp}`** — the ordering rule and `isDeviceGeneratedName`,
  Arduino-free and host-tested. The name matcher checks the length exactly:
  `N260803_1432 CW` is a name someone typed that happens to start the same way,
  and it is not ours to assume is disposable.
- **`sd_functions.ino`** — `sdPlanSprintPrune()` works out what it would take
  *without touching the card* (it parses into RAM and throws the copy away), and
  `sdSaveCreatedCourse(req, dropOldest)` commits. Both go through
  `openTrackForEdit()`, and the commit reuses the existing temp-file + rename
  swap — a power loss mid-write must not leave a truncated track file.
- **`BirdsEye.ino`** — `courseCreatorSave()` tries the plain save first and only
  reaches for pruning on `TOO_BIG`; `courseCreatorConfirmPrune()` handles the
  answer.

## Three things that would have been bugs

**Prune before adding, not after.** ArduinoJson's `overflowed()` flag is
**sticky** — once the document has overflowed, removing elements cannot clear
it. Grafting the course in and then shrinking back under the limit would have
looked right and never worked. So the space is made first, and `measureJson()`
(the exact serialized length) decides when enough has gone.

**Remove by name, not by index.** Every removal shifts the indices under it,
while the drop order was computed against the original positions. Removing
`order[0]` then `order[1]` would delete the wrong second course.

**Hold the generated names across the confirmation.** They carry the GPS clock
down to the minute. Re-deriving them after the user reads the prompt would
rename the course they just walked to whenever they happened to answer.

## Version bump

`FIRMWARE_VERSION` goes to **3.2.0**. It was `3.0.1` on this branch — *behind*
master's `3.1.0`, which is a problem the moment the webapp starts reading it:
the app decides a logger's track budget by comparing this against the last
release (8 KB at or above 3.2.0, 4 KB below), and `compareVersions` ignores
prerelease suffixes, so a beta build stamped `3.0.1-beta.<sha>` compares as
`3.0.1`. Every beta unit would have been handed the smaller budget it does not
have, and the app would have made users drop courses that fit perfectly well.
