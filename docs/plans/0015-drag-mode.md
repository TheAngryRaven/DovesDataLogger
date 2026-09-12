# Drag Mode — Distance Runs Without a Track

> Status: **IMPLEMENTED**. A point-and-shoot acceleration-run mode: pick a
> distance from the menu, stage at a standstill, launch, and the device times
> the run to the target distance. No track file, no detection, no lines to
> walk — the whole point is that a drag strip (or any straight) needs zero
> setup.

## What it is

Main menu → **Drag** → pick a distance (1/8 Mile, 1000 ft, 1/4 Mile,
1/2 Mile, 1 Mile) → the session starts immediately. The device waits for a
standstill (staging), starts the clock **rollout-style** when the car has
moved 11.25 in from its staged position, and ends the run when cumulative
GPS distance reaches the target. Per run it reports **ET**, **trap speed**
(speed at the finish), and a **0–60 mph split**. After each run it re-arms
automatically — come back to a standstill and it stages again — so a whole
day of passes lands in one DOVEX session, saved per usual with
`race_mode=DRAG`.

## Structure: sprint mode is the template

Sprint mode (plan 0002) already proved the shape: a session type where
`courseManager` stays null, a single timer object IS the mode
(`dragTimer != nullptr`), runs duck-type as laps through the
`activeTimer*()` helpers, and run completion is captured on the run-count
edge in `checkForNewLapData()` (identical consecutive ETs are normal —
value-change dedupe would drop them). Drag follows it line for line:

- `startDragSession(distanceIdx)` stands up the timer, latches
  `trackDetected = true` (the same latch sprint uses — it keeps
  `trackDetectionLoop()` from standing up a CourseManager when the strip
  happens to be near a saved track), then routes through
  `startRaceSession(RACE_ENTRY_MANUAL)` like every other session start.
- `createLapAnythingCourseManager()` gains a drag guard beside the sprint
  one, `endRaceSession()` tears the timer down beside the sprint teardown.
- The GPS feed is a third branch in `gps_functions.ino` next to
  courseManager/sprintTimer — the three are mutually exclusive by
  construction.

The run logic itself is a new **pure unit, `drag_timer.{h,cpp}`** —
Arduino-free, host-tested, in the sim build. The sketch keeps only the glue.

## The state machine (drag_timer::DragTimer)

Three phases; every transition edges on a GPS fix. The glue gates on
`gpsData.fix` (like the sprint feed), so a dropout reaches the unit purely
as a timestamp gap.

- **ARMED** (initial, post-run, post-abort): speed ≤ 1 mph held 1 s →
  STAGED. Standstill fixes feed the anchor mean the whole time.
- **STAGED**: a fix gap ≥ 2 s **re-stages** instead of evaluating the
  launch condition — a launch that happened inside a GPS dropout would
  otherwise interpolate its ET start back to a parked-car fix from
  before the gap, inflating the ET by up to the gap length (a car still
  parked simply stages again one second later). The anchor is a running
  mean of standstill fixes and it
  **keeps re-latching** while the car is stopped. This is the load-bearing
  detail: GPS drift over a multi-minute staging-lane wait would otherwise
  walk the fix past the 11.25 in rollout radius and fake a launch. With a
  re-latching anchor the only false-launch source left is real motion
  (queue creep), and that self-cancels through the abort rule without
  recording anything. Launch = displacement from anchor ≥ rollout **AND**
  speed ≥ 2 mph (jitter or dead-slow creep never launches); the ET start is
  linearly interpolated on the displacement curve between the two straddling
  fixes, and the run's distance is seeded with the overshoot past rollout.
- **LAUNCHED**: per fix, add the chord distance (drag runs are straight —
  chord ≈ path at 25 Hz). A duplicate timestamp is dropped whole; a
  backwards one resyncs (see below).
  Gap ≥ 2 s → run abandoned (distance across the gap is untrustworthy at
  speed). 0–60: first fix pair with `v_prev < 60 ≤ v`, interpolated on
  speed; 0 if never reached (short cars on the 1/8). Finish: cumulative
  distance crosses the target → ET and trap both interpolated between the
  straddling fixes; run recorded (count, last/best ET, best-run trap and
  0–60 snapshot); back to ARMED. There is no FINISHED phase — "re-arm"
  IS "wait for standstill", which is ARMED. Two aborts, both silent:
  ≤ 2 mph held 3 s before the target (this also self-cancels queue-creep
  phantom launches), and the **prove-out gate** — a launch that fails to
  reach 15 mph within 5 s of the ET start is not a pass, it is a wave-off
  driven to the pits at 4 mph, which never holds the sub-2 mph standstill
  the first abort needs and would otherwise record 660 ft of pit road as
  a ~90 s "run".

A **backwards time step** (receiver clock correction) aborts whatever is
in flight and resyncs the stream — the first-cut guard just dropped the
fix without updating its reference, so one backwards step rejected every
later fix and wedged the timer for the rest of the session.

Tunables (all `constexpr` in `drag_timer.h`): rollout 0.9375 ft (11.25 in,
the drag-strip standard), staged ≤ 1 mph held 1000 ms, launch ≥ 2 mph,
prove-out ≥ 15 mph within 5000 ms, abort ≤ 2 mph held 3000 ms, fix-gap
abort 2000 ms (mid-run AND on the staged launch edge), split target
60 mph.

### Units at the boundary

The unit speaks **mph** (glue converts once: `gpsData.speed × 1.15078`,
knots→mph, the same constant the logging row uses), **feet** (targets are
exact integers: 660 / 1000 / 1320 / 2640 / 5280; `distanceFeet()`
delegates to the existing `haversine` unit × 5280, so track detection and
drag distance can never disagree on the Earth), and **Unix epoch
milliseconds** from `getGpsUnixTimestampMillis()` — NOT
`getGpsTimeInMilliseconds()`, which is time-of-day and wraps to zero at
UTC midnight: prime evening test-and-tune hours across the US, and the
wrap would look like a backwards step every night. No `millis()` inside
the unit, sprint-timer precedent.

## Accuracy — what's honest to claim

- GPS h_acc (~0.5–1.5 m CEP) is *larger* than the 0.29 m rollout, so the
  absolute rollout position is noisy. What makes the ET credible anyway is
  that the start is **interpolated on the displacement curve between two
  25 Hz fixes during a hard launch** (a car covers the rollout in
  ~150–250 ms), so the timing error is bounded by fix-to-fix noise plus the
  anchor mean's residual (~±1–2 ft ≈ ±10–30 ms), not by absolute accuracy.
- Cumulative 25 Hz haversine over 660–5280 ft carries sub-0.5 % distance
  error; the anchor is the dominant term.
- **Trap speed is the interpolated instantaneous speed at the target
  distance**, not a strip-style 66 ft trap-zone average. Deliberate
  simplification — GPS speed at 25 Hz is already smooth, and the rows carry
  everything needed to compute a zone average offline if anyone cares.

## DOVEX — no format change

Header: `race_mode = DRAG` (fits the 8-byte field; `dovex_header` passes
any string through and legacy readers ignore trailing columns — the same
backwards-compat story as SPRINT), course name = `DRAG 1/4 MILE` etc.
(single source: `drag_timer::dovexName()`), short name = `DRAG`. The laps
line is the run ETs via the ordinary `lapHistory` path.

**Trap and 0–60 are deliberately NOT in the header.** The 4-line header
layout is frozen for backwards compatibility, and the 25 Hz rows carry mph
speed, so any viewer can derive both exactly (and better — e.g. a real
trap-zone average). Follow-up for the webapp: a `DRAG` case in the
race_mode loader (the laps line is a runs line, same as SPRINT); an old
viewer degrades gracefully to "unknown mode with a runs line".

## Session end (auto-idle)

Drag is a manual-entry session, so the manual rules apply (5 min < 5 mph),
with the usual promotion to tach rules if the tach proves itself. Two
grace re-arms keep a staging queue from idling the session out on tach-less
cars: each **completed run** resets the grace (sprint precedent), and so
does each **ARMED→STAGED edge** — queue creep re-stages every couple of
minutes, so an active queue never idles out, while a genuinely parked car
still ends ~8 min after its last movement. The engine-aware sprint reset in
`checkAutoIdle()` was widened to cover drag too.

## UI

- Main menu grows a **Drag** row (index 1); the menu was already a
  scrolling 3-row window so a 6th item costs nothing.
- `PAGE_DRAG_DISTANCE` (−17): distance picker, five rows from
  `drag_timer::label()` + Back (every menu carries a Back row), rendered
  with the same scrolling-window pattern as the main menu (6 rows can't fit
  statically at size 2).
- **No new rotation pages.** The existing pages branch on
  `dragModeIsActive()` the way they already branch for sprint's
  `*waiting*`: the Current Lap page shows the live ET while running, the
  last ET + `trap / 0-60` subtext after a run, and `*staged*` /
  `*waiting*` between runs; the Best Lap page adds a best-run
  `trap / 0-60` subtext; the Pace page shows the live 0–60 status during a
  run instead of a meaningless +0.00 pace. The LED pace pip is suppressed
  between runs exactly like sprint.
- No settings persistence for the distance choice — the picker is two
  presses, and a stale remembered distance is worse than none.

## What was deliberately NOT built

- No staging-tree / reaction-time simulation — the device has no christmas
  tree; ET starts at rollout like the strip's clocks.
- No trap-zone speed averaging (see Accuracy).
- No per-distance best history across sessions — the DOVEX files are the
  record; the webapp is the place to compare days.
