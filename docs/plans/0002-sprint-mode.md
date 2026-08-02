# Sprint Mode (Autocross / Point-to-Point) — Concept & Cross-Repo Roadmap

> Status: **CONCEPT — design complete, implementation-ready.** All §7
> questions are decided; no implementation yet.
> Scope spans three repos; each phase lands on that repo's beta branch
> (DovesLapTimer `BETA` → DovesDataLogger `BETA` → DovesDataViewer, last).

## 1. Problem & Goal

Autocross (and hillclimb / sprint-style events) doesn't fit the current
timing model. A run is **point-to-point**: a start line and a *separate*
finish line, no laps. Drivers make multiple runs per session — cross the
finish, loop back around, wait (often minutes, engine on or off) at the
start line, run again. Sectors may exist, but the crossing tech is the
same as circuit — actually lighter: two lines to monitor plus optional
splits.

The circuit assumption is baked in at every layer today:

- **Timing library**: start line *is* the finish line; everything is lap
  accounting (`laps++`, `raceStarted`, best/last lap, direction detection).
- **Course detection**: `CourseDetector` identifies the course by driving a
  full lap back to your own dropped waypoint and matching odometer distance
  to `lengthFt`. In sprint you *never return to the start* mid-run —
  length-based detection is structurally impossible, not just untuned.
- **Session lifecycle**: `checkAutoIdle()` ends the session after 60 s
  below 2 mph (`BirdsEye.ino:1142-1183`). Staging at an autocross start
  line looks exactly like "done for the day" — the session would be killed
  between every run.

**Goal**: a `race_mode` device setting — `circuit` (default, current
behavior, untouched) vs `sprint` — toggled from the webapp, with sprint
tracks stored separately so everything existing stays backwards compatible.

## 2. Product Shape

| Aspect | Circuit (today, unchanged) | Sprint (new) |
|---|---|---|
| Mode select | default | **automatic — mode follows the detected track** (decided): if the nearest manifest match came from `/TRACKS/SPRINT/`, the device knocks into sprint mode. A `race_mode` **preference setting** (`circuit` default / `sprint`, webapp `SSET`) exists only as the **tiebreak when both kinds are in range** — see §7 Q1 for the decided rules (it also serves fixed sprint courses, e.g. a permanent rally layout, where the course-of-the-day heuristic never fires). |
| Track storage | `/TRACKS/*.json` | `/TRACKS/SPRINT/*.json` (new folder; circuit tracks stay put) |
| Track detection | haversine proximity vs manifest | same mechanism, scanning sprint manifest entries |
| Course detection | `CourseDetector` (length-based) → Lap Anything fallback | **none** — **date-based selection** (decided): load only the most-recently-created course by `date_created`. One course in RAM. |
| Course lines | S/F (+ S2+S3 pair) | **start + finish required; up to 2 optional sector lines** (a single split must be legal — see Phase 1) |
| Timing | lap timer (S/F crossing, laps) | run timer (start line → finish line, N runs/session) |
| Fallback | Lap Anything (`WaypointLapTimer`) | none meaningful — no matched sprint track ⇒ log-only session; the real answer is the on-device course creator (§5) |
| Session | one `.dovex`, laps line in header | one `.dovex` for the whole event, runs line in header |
| Race entry | auto (RPM > 500 or ≥ 10 mph) or manual | same triggers — driving to the queue should clear 5–10 mph, and karts have RPM — but **expect more manual race entry** at autocross; keep the manual path prominent and don't make sprint depend on auto triggers |

Future modes note: this two-value scheme (`CIRCUIT`/`SPRINT`) is expected to
grow — a **drag mode** is on the horizon and will be a bigger lift (staging,
reaction time, fixed distances). Everything mode-shaped (DOVEX marker, track
`type` field, folder scheme) should be an open enum, not a boolean.

### Course lifecycle at a sprint venue (user report #3) — date-based selection

The same entrant attends the same venue most weekends, **and the course is
different every event** (cones get re-laid). So sprint "courses" are
disposable, dated layouts of a persistent venue (track):

- New sprint-course JSON field: **`date_created`** — stamped automatically by
  whatever created the course (webapp or the on-device creator, §5); the user
  never edits it. **Decided: full sortable ISO-8601 timestamp
  (`YYYY-MM-DDTHH:MM`)** — autocross venues re-lay courses same-day
  (morning/afternoon configs) and a date-only value can't order those;
  lexicographic max = newest falls out for free. Device-generated file/course
  name suffixes use the same timestamp, killing same-day collisions too.
- Loading a sprint track loads **only the newest course** by `date_created`
  into memory — no CourseDetector, no default-course setting, one course, tiny
  RAM footprint. This may evolve (this user races weekly and will be a
  valuable data source), but v1 is deliberately simple.
- Old courses accumulate in the track file — the on-device JSON parse buffer
  is 4096 bytes, so a weekly venue overflows it within months. **Decided:
  webapp-side prune on sync** — when the app syncs a sprint track it pulls
  the full file (archiving all courses app-side), then rewrites the device
  copy keeping only the **most recent single day of courses**
  (sync → delete → push new file). v1 may ship *before* this lands, with a
  loud documented disclaimer that un-synced devices will eventually hit the
  course cap.

### Real-world cadence (user report, 2026-08)

An actual autocross entrant describes the day like this: runs happen in
**heats of ~4** — one run at a time, then a stop of only **30–45 seconds**
in the queue while others go, then the next run. After the 4th run the
whole run group rotates out ("then the cars go") for a long break —
tens of minutes — before another heat of 4. They leave the Insta360 X4
recording across an entire heat and end up with "a couple of 20 min
videos".

That cadence pins the session model (user has since **confirmed the
engine stays running between runs**, so the mapping below holds with no
caveats):

- **Session = heat.** One heat = one `.dovex` = one camera video. This is
  exactly what the camera-paired firmware already does today: the engine
  stays running through a heat, so the camera keeps recording; 30 s of
  engine-off at heat end auto-stops the recording *and* ends the log
  session (`cameraConsumeAutoStop()`), and the next heat's engine start is
  a fresh wake → new session. Sprint mode should preserve this mapping,
  not fight it — and **run boundaries must never touch the camera**.
- **Between-run stops are shorter than the 60 s auto-idle window**, so the
  idle killer is less catastrophic than first assumed — but queue position
  isn't guaranteed (grid delays, red flags), so the engine-aware idle rule
  below is still wanted as a safety margin rather than a rewrite.
- **Run counts are small** (4-ish per session), so run history/state is
  trivial memory-wise; per-run re-arm through the same start line ~every
  minute is the hot path to test.

Key research finding that makes this cheap: **the crossing math is already
line-agnostic.** `DovesLapTimer::_detectLineCrossing()`
(`DovesLapTimer.cpp:198-306` on the library's BETA branch) takes an
arbitrary (A, B) line + an external crossing flag and returns the
interpolated crossing — a separate finish line reuses it verbatim. There is
even a standing TODO at `DovesLapTimer.cpp:143` about making
`checkStartFinish()` portable for split timing. What's missing is
structural, not mathematical.

## 3. Phase 1 — DovesLapTimer (`BETA`): the timing engine

The engine lives in the library from day one (firmware BETA builds already
track the library's BETA branch in CI, so co-development is wired).

- **`SprintTimer` class**, sibling of `DovesLapTimer`, sharing the
  crossing-detection path. Refactor `_detectLineCrossing()` +
  `insideLineThreshold()` / `pointOnSideOfLine()` / the crossing ring
  buffer into a reusable core (the `DovesLapTimer.cpp:143` TODO), rather
  than copy-pasting.
- **Run accounting instead of lap accounting** (semantics decided):
  **both lines are monitored continuously** — same as the circuit timer
  monitors S/F + S2 + S3 every loop. Two states, purely line-driven:
  - `WAITING` → **start crossing** begins a run (interpolated time).
  - `RUNNING` → **finish crossing** completes the run;
    **start crossing cancels the in-progress run and starts a brand new
    one** (the botched-course case: driver messes up, drives back around,
    re-launches — sprint's equivalent of circuit's
    every-S/F-crossing-closes-and-opens-a-lap rule).
  - Finish crossings while `WAITING` are **ignored** (driving back past
    the finish on the return loop must not do anything).
  - **DNF is just normal operation**: an abandoned run never completes and
    records nothing; the driver eventually kills the engine and the
    normal session-end paths take over. No special DNF state.
  - Nuance: crossing detection is direction-agnostic, so a *backward*
    start crossing on the return loop opens a bogus run — which the
    restart rule then self-heals at the real launch (the forward crossing
    cancels it). A backward *finish* crossing while `RUNNING` is the only
    truly bogus completion; it requires re-entering the finish zone
    mid-run (rare on a one-way autocross course). Cheap hardening if it
    matters: gate the finish line on crossing sign
    (`pointOnSideOfLine()` already returns signed sides).

  Surface: run count, current run time, last run, best run (+ run
  number), run history. No `laps`, no `DirectionDetector`, no
  S/F-restarts-sector-1 entanglement.
- **Course model**: add `finish_a/b_lat/lng` to the sprint course config.
  v1 sectors (decided): **start + finish required, up to 2 optional split
  lines** — matching the S2/S3 shape the device UI and circuit model
  already speak — but **a single split must be legal**
  (`areSectorLinesConfigured()` requires *both* today; that all-or-nothing
  gate gets relaxed). The webapp's ordered `sectors[]` list means N splits
  can come later without a data-model change.
- **Course selection without detection**: a public `selectCourse(int)` on
  `CourseManager` (today `_activeCourseIndex` / `_detectionComplete` are
  private with no setter — `_activateLapAnything()` is the only way to
  short-circuit detection). Sprint mode always uses it — the firmware
  picks the newest course by `date_created` (§2) and selects it directly;
  it's independently useful for circuit too (adjacent to library issue
  #35, proximity-based detection).
- **Memory**: don't instantiate the 8-slot by-value course array for a
  mode that needs exactly one active course (~29 KB regardless of count
  today — library issue #22).
- **Known constraint to design around**: the crossing ring buffer is a
  single shared instance with only-one-line-crossing-at-a-time mutual
  exclusion (`loop()`, `DovesLapTimer.cpp:78-130`). Autocross paddocks
  often place start and finish within meters of each other — either give
  sprint lines independent buffers or document a placement constraint.
- **Tests**: follow the existing host-native pattern —
  `test_synthetic_track.cpp` builds a deterministic synthetic circuit; a
  sprint suite generates an open path (straight / L) with a start line
  near step 0 and finish near step N, asserting run times. Extend
  `replay_runner.h`'s `ReplayConfig` (today it only carries `sfA/sfB`)
  with finish-line fields; a real recorded autocross NMEA/DOVEX fixture is
  wanted eventually.
- Related library issues worth folding in while in there: #32
  (crossing-event callback — exactly the seam a run-complete edge wants),
  #30 (pit-lane special line pair — same "non-lap line" shape), #34
  (session-persistent state injection).

## 4. Phase 2 — DovesDataLogger (`BETA`): consume it

- **Mode-by-folder detection**: `trackDetectionLoop()` picks the nearest
  manifest entry as today; if that entry's `kind` says sprint, the session
  runs in sprint mode — build the sprint timer path instead of
  `CourseManager`+`CourseDetector`, and select the course by newest
  `date_created` (§2). When both kinds are in range, the `race_mode`
  **preference setting** breaks the tie (rules in §7 Q1): default row in
  `ensureDefaultSettings()` (`settings.ino:102-108`), global loaded at
  `BirdsEye.ino:765-795`, values `circuit` (default) / `sprint`.
- **RPM accuracy settings** (`spark_mode` + `cylinder_count`) are needed by
  this same user group (autocross karts, possibly a 2-cyl monster) but are
  independent of sprint mode — split out as **plan 0003**.
- **`/TRACKS/SPRINT/`**: the `/TRACKS` literal is duplicated across
  `trackFolder[8]` (`BirdsEye.ino:469`), `makeFullTrackPath()`
  (`sd_functions.ino:64`), `sdEnsureTracksFolder()`, the
  `buildTrackList()` walk, and four splices in `bluetooth.ino`
  (TLIST/TGET/TPUT/TDEL). Plan: add `kind` to `TrackManifestEntry`
  (`project.h:228`), factor `buildTrackList()`'s walk into a
  parameterized `scanTrackDir(folder, kind)`, make `makeFullTrackPath()`
  manifest-entry-aware, and provision the sprint folder wherever
  `sdEnsureTracksFolder()` runs (boot, upload, post-format).
- **Sprint track JSON**: same object format plus `"type": "sprint"` and
  `finish_a/b_*` fields (and the ordered sectors list), parsed with the
  established optional-`containsKey()` idiom in `parseTrackFile()`. The
  folder already discriminates kind; the `type` field is cheap redundancy
  and lets the webapp validate.
- **BLE track sync**: new folder-qualified opcodes (sprint variants of
  `TLIST`/`TGET:`/`TPUT:`/`TDEL:`) rather than path-carrying filenames —
  `filename_validator` deliberately rejects `/` and `..` to keep BLE
  clients jailed to the tracks folder, and it should stay strict.
- **Third timing backend**: the `activeTimer*()` helpers
  (`BirdsEye.ino:867-962`) are already a two-backend dispatch
  (DovesLapTimer / WaypointLapTimer); `SprintTimer` slots in as a third.
  Any new/changed helper must be mirrored in `sim/sim_prototypes.h` or the
  sim build breaks.
- **Session lifecycle** (shaped by the heat cadence in §2):
  - Session = heat. The camera-paired path already ends the session on
    30 s engine-off (`cameraConsumeAutoStop()`) — that *is* the heat
    boundary, keep it. Run boundaries never touch the camera and never
    end the session.
  - Sprint-aware `checkAutoIdle()` for the no-camera case: engine-aware
    idle (only start the idle clock when tach reads 0 *and* stationary)
    and re-arm the grace at every run completion (today the 3-min grace
    anchors once at session start and never re-arms). With 30–45 s
    between-run stops this is a safety margin for queue delays, not the
    main path. The camera-recording yield at `BirdsEye.ino:1158` is the
    precedent for "something else says we're still active".
  - Multiple runs live inside **one session / one `.dovex`** — a run
    boundary must never call `endRaceSession()` (it deletes
    `courseManager` and wipes best-run state).
  - Run completion is an explicit edge (event/callback), not the current
    `checkForNewLapData()` value-change dedupe (`BirdsEye.ino:389-406`) —
    two identical run times in a row would be silently dropped.
- **DOVEX** (decided): add a trailing `race_mode` column to header
  lines 1/2 — the format's established extension mechanism
  (`device_name` was added the same way, `dovex_header.h:21-23`).
  Values: **`CIRCUIT` / `SPRINT`** (parse case-insensitively; empty or
  absent = `CIRCUIT`, so every legacy log parses correctly). "Sprint" is
  the established motorsport term for one-at-a-time point-to-point timed
  runs and covers autocross / hillclimb / stage-style events alike. The
  column is deliberately just a **loading helper for the webapp** (pick
  run-derivation vs lap-derivation — a GPS trace alone doesn't reveal the
  mode) plus the replay page's Run-vs-Lap labels; nothing on-device
  depends on it. The lap times line doubles as the run times line —
  per-run sector/DNF detail, if ever wanted, would be a new line pair
  after line 4 (old parsers never read past it), not a change to this
  marker. Extend `tests/dovex_header_test.cpp`.
- **Display** (decided): **keep the existing "Lap" verbiage everywhere** —
  the AX drivers call them laps anyway, so no mode-conditional relabeling,
  no new page constants, and the fragile ordered page-constant blocks
  (`ENDURANCE_MODE` / `SENSOREGG` reshuffles) stay untouched. The only
  sprint-aware display change: while no run is active (`WAITING`), the
  **Current Lap and Pace pages show `*waiting*`** — every other page
  (tach, GPS debug, best lap, lap list, …) works normally, so the driver
  can watch RPM etc. between runs.

## 5. On-device course creator — the big ask

Sprint entrants can **walk the course** before the event (cones are laid out
fresh each time), so the device itself must be able to create a course —
standing at each cone and capturing GPS positions. Hard rule: **no text entry
on-device, ever.** All names are auto-generated and renameable later in the
webapp.

### UI flow (as specced)

New main-menu option **"Create Course"**:

1. **Track prompt** — run the normal proximity track search, then ask
   `Are you at {TRACK NAME}?` with options **Yes / New Track**. A new track
   is written as `NEWTRACK_{date}.json` (no typing; renamed in the app
   later).
2. **Type select** — choose **CIRCUIT** or **SPRINT** course.
3. **Line menu** — one row per line, labeled
   `{Name} {* if required} {DONE stamp when both points collected}`:
   - `Start/Finish *` (labeled just `Start *` in sprint)
   - `Sector 2`
   - `Sector 3`
   - `Finish *` (sprint only)
   - `Save` — writes the course as `NEWCOURSE_{date}`, back to main menu
   - `Cancel` — back to main menu, discard
4. **Per-line menu**:
   - `{Line label}` (reminder header)
   - `Point A * {DONE}`
   - `Point B * {DONE}`
   - `Save` / `Back`
5. **Point menu**: `{Line label : A/B}` → **Save current pos** / Back.

### Design notes / constraints found in research

- **GPS gating**: capture requires a fix, and the auto-generated names
  require GPS date/time — same `timeValid` gate as log-file creation. The
  point screen should show live `h_acc` and warn/refuse on poor accuracy.
- **Point capture averages, not snapshots** (decided): "Save current pos"
  runs a **3 s hold** — up to ~75 fixes at 25 Hz — averaged into the point,
  with a "hold still… n/N" progress and live `h_acc` (warn/refuse on poor
  accuracy). The user is standing at a cone anyway; this is free precision.
- **Name collisions** (resolved by §2's ISO decision): generated
  `NEWTRACK_`/`NEWCOURSE_` suffixes carry the same date+time stamp as
  `date_created`, so a second same-day creation can't collide. Note the
  track-browser display truncates to 13 chars (`MAX_LOCATION_LENGTH`), so
  on-screen disambiguation may still need the time portion favored over the
  literal `NEWTRACK_` prefix.
- **This is the firmware's first track-JSON writer**: today the device only
  reads track files (settings JSON read-modify-write is the closest
  precedent, `settings.ino`). Appending a course means parse-modify-write of
  a `/TRACKS/SPRINT/*.json` under the 4096-byte parse buffer — reinforces
  the pruning question (§7). Writes go through the existing SD arbitration.
- **Save semantics**: captured points live in RAM until line-menu `Save`;
  `Cancel`/power-loss discards. Fine for v1 — walking the course takes
  minutes, not hours.
- Circuit courses created on-device get S/F + optional S2/S3 — exactly the
  existing model, so the creator serves both modes from day one.

## 6. Phase 3 — DovesDataViewer (last)

The old BETA branch was merged (PR #373) and deleted — this phase starts by
cutting a fresh beta branch. Nothing in the app anticipates modes today.

- **Track/course editor**: a **circuit/sprint toggle on the track & course
  editor** (decided — the primary mode signal; the `race_mode` device
  setting is only the in-range tiebreak, §7 Q1). Sprint
  courses get a finish line + optional splits; `date_created` is stamped
  automatically on course creation and is **not user-editable**. The
  editor also needs rename support for device-generated
  `NEWTRACK_*`/`NEWCOURSE_*` names.
- **Sync-prune for sprint tracks** (decided, may land after v1 with a loud
  disclaimer): syncing a sprint track archives all courses app-side, then
  rewrites the device file keeping only the most recent single day of
  courses (sync → delete → push new file). See §2.
- **Settings control type**: the `race_mode` preference (§2) and plan
  0003's `spark_mode` both want a proper `boolean`/`enum` control in
  `deviceSettingsSchema.ts` + `DeviceSettingsTab.tsx` (today only
  string/number text inputs exist).
- **Track model**: `type: 'circuit' | 'sprint'` on `Course`/`Track`,
  flowing through `trackStorage.ts` ↔ `deviceTrackSync.ts` ↔
  `trackSubmission.ts` and a Supabase `courses` column. Relax
  `validateCourseSectors` (currently 0 sectors or exactly-3-majors) and
  add a finish-line handle to the track editor (`VisualEditor.tsx`,
  `LineId = 'sf' | number`). Note `coursesMatch` compares only S/F + two
  sector lines today — sprint fields must join the diff or sync status
  lies.
- **Sync**: implement the sprint-folder opcodes in `trackSync.ts` /
  `deviceTrackSync.ts` (all filenames are flat today, no directory
  concept). Update `docs/ble-protocol.md` §9.
- **Runs view**: the viewer re-derives laps from GPS
  (`lapCalculation.ts`) rather than trusting the header — sprint needs a
  run-derivation variant (start-crossing → finish-crossing per run) and a
  run-oriented `LapTable` view. The existing `isWaypointMode` branch is
  the precedent for "session without real laps" flowing through the UI.
- Per that repo's conventions: a numbered `docs/plans/` design doc and
  `docs/subsystems.md` updates.

## 7. Open Questions

1. ~~Mode selection?~~ **Decided: mode follows the detected track's
   folder; the `race_mode` setting survives as the both-kinds-in-range
   tiebreak.** Rules: with `race_mode=circuit` (default), prefer circuit —
   *unless* the nearby sprint track has a course created **today** (event
   day), in which case sprint wins. With `race_mode=sprint`, always prefer
   the sprint track and load its newest course regardless of date — this
   is the fixed-course case (e.g. a permanent rally layout that isn't
   re-created per event).
2. ~~Minimum sector support?~~ **Decided: start + finish required, up to 2
   optional splits, single split legal.**
3. ~~Course-history pruning?~~ **Decided: webapp prunes on sync, keeping
   the device's most recent single day of courses** (sync → delete → push
   new file). v1 may ship before it lands, with a loud disclaimer about
   the 4096-byte parse-buffer cap on un-synced devices.
4. ~~Run arming / DNF semantics?~~ **Decided.** Session arming is
   unchanged from circuit: RPM held for ~a second brings up all systems
   and recording, exactly like today. Run timing is purely line-driven
   with both lines always hot: start crossing begins a run (and cancels +
   restarts any run already in progress — the botched-course re-launch
   rule); finish crossing completes it; finish crossings with no active
   run are ignored. DNF needs no special record — the run just never
   completes, and the engine eventually dying ends the session through
   the normal paths. Full state machine in Phase 1 (§3).
5. ~~Between-run state?~~ **Decided: the device simply stays in normal
   race mode between runs.** All pages remain live (tach/RPM, GPS debug,
   best lap, lap list, …); the Current Lap and Pace pages show
   `*waiting*` while no run is active. Best run, run history, and the log
   file naturally survive the loop-back and queue wait since the session
   never ends between runs (Phase 2 lifecycle). Best run is per-session
   (one `.dovex` per heat) — matches existing `best_lap_ms` semantics.
6. ~~Runs-vs-laps wording?~~ **Decided: keep "laps" for now** — the AX
   drivers themselves call them laps. Applies on-device (display, replay
   page) and means the DOVEX `laps_ms` line needs no renaming; the
   `race_mode` header column alone tells the webapp how to interpret the
   data. Per-run sector detail, if ever wanted, is later.
