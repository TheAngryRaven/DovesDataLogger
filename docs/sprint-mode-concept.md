# Sprint Mode (Autocross / Point-to-Point) — Concept & Cross-Repo Roadmap

> Status: **CONCEPT** — research + direction only, no implementation yet.
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
| Mode select | default | `race_mode=sprint` setting, set via webapp `SSET`, applied on reboot (the existing settings contract — BLE disconnect auto-reboots) |
| Track storage | `/TRACKS/*.json` | `/TRACKS/SPRINT/*.json` (new folder; circuit tracks stay put) |
| Track detection | haversine proximity vs manifest | same mechanism, scanning sprint manifest entries |
| Course detection | `CourseDetector` (length-based) → Lap Anything fallback | **none** — single course, or webapp-selected `defaultCourse` when a track has several |
| Timing | lap timer (S/F crossing, laps) | run timer (start line → finish line, N runs/session) |
| Fallback | Lap Anything (`WaypointLapTimer`) | none meaningful — no matched sprint track ⇒ log-only session (Lap Anything is a lap timer; it produces nothing useful point-to-point) |
| Session | one `.dovex`, laps line in header | one `.dovex` for the whole event, runs line in header |

### Real-world cadence (user report, 2026-08)

An actual autocross entrant describes the day like this: runs happen in
**heats of ~4** — one run at a time, then a stop of only **30–45 seconds**
in the queue while others go, then the next run. After the 4th run the
whole run group rotates out ("then the cars go") for a long break —
tens of minutes — before another heat of 4. They leave the Insta360 X4
recording across an entire heat and end up with "a couple of 20 min
videos".

That cadence pins the session model:

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
- **Run accounting instead of lap accounting**: per-run state machine
  `ARMED → RUNNING → FINISHED`, re-arming when the driver returns to the
  start-line zone. Surface: run count, current run time, last run, best
  run (+ run number), run history. No `laps`, no `DirectionDetector`
  (direction is meaningless point-to-point), no S/F-restarts-sector-1
  entanglement.
- **Course model**: add `finish_a/b_lat/lng` to the sprint course config.
  Sectors become **N ordered split lines** between start and finish rather
  than the circuit's all-or-nothing S2+S3 pair
  (`areSectorLinesConfigured()` requires *both* today — a single
  mid-course split is currently impossible; the webapp's data model
  already carries an ordered `sectors[]` list, so the library is the
  bottleneck, not the data).
- **Course selection without detection**: a public `selectCourse(int)` on
  `CourseManager` (today `_activeCourseIndex` / `_detectionComplete` are
  private with no setter — `_activateLapAnything()` is the only way to
  short-circuit detection). Sprint mode always uses it (webapp-chosen
  `defaultCourse`); it's independently useful for circuit too (adjacent
  to library issue #35, proximity-based detection).
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

- **`race_mode` setting**: default row in `ensureDefaultSettings()`
  (`settings.ino:102-108`), loaded into a global in the settings block at
  `BirdsEye.ino:765-795`. `SSET` is already generic (no key allowlist), so
  the webapp can set it the moment the firmware reads it. Update the
  settings tables in README.md and CLAUDE.md.
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
- **DOVEX**: add a trailing `race_mode` column to header line 1 (the
  format has an explicit trailing-column back-compat story —
  `device_name` was added the same way, `dovex_header.h:21-23`); the lap
  times line doubles as the run times line. Old readers ignore the extra
  column; old files parse as circuit. Extend `tests/dovex_header_test.cpp`.
- **Display**: reuse existing page IDs with mode-conditional labels
  (Lap → Run, Lap History → Run List, Best Lap → Best Run). The ordered
  page-constant blocks with their `ENDURANCE_MODE` / `SENSOREGG`
  reshuffles are the most fragile part of the UI — avoid new page
  constants.

## 5. Phase 3 — DovesDataViewer (last)

The old BETA branch was merged (PR #373) and deleted — this phase starts by
cutting a fresh beta branch. Nothing in the app anticipates modes today.

- **Settings UI**: `race_mode` auto-appears via the generic key/value list,
  but a real toggle needs a new `boolean`/`enum` control type in
  `deviceSettingsSchema.ts` + `DeviceSettingsTab.tsx` (today only
  string/number text inputs exist). Update `docs/ble-protocol.md` §8.5.
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

## 6. Open Questions

1. **Is `race_mode` really a device mode?** Alternative: mode follows the
   detected track (which folder the nearest manifest entry came from), and
   the setting is only a tiebreak when a venue has both kinds nearby.
   Fewer webapp round-trips; slightly spookier behavior.
2. **Minimum sector support for v1** — none (start + finish only) ships
   faster; N ordered splits is where the library should end up.
3. **Run arming semantics** — auto-arm on entering the start-line zone vs
   a speed/launch threshold? What does a false start / DNF (never crosses
   finish, returns to start) record as?
4. **Between-run state** — best run, run history, and the log file must
   survive the loop-back and the ~30–45 s queue wait (they will, as long
   as sessions don't end between runs — see Phase 2 lifecycle).
   Cross-heat: does "best run" span the whole day or reset per heat/
   session? Per-session is what falls out naturally (one `.dovex` per
   heat); a day-spanning best would need header aggregation in the
   viewer, not firmware state.
5. **Replay page** — header `race_mode` column makes the replay results
   page label runs vs laps correctly; anything more (per-run sectors) is
   later.
