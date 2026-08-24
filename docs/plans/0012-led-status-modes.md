# 0012 — Assignable status LEDs, speed bar, lap/sector verdicts, two-stage purple

Phase 2 of the NeoPixel subsystem. Plan 0006 ended with an explicit
promise — *"settings-driven assignment: strip mode selection,
per-status-LED action source/threshold/color… The POD tables above are
the interface that phase fills in."* This fills it in, plus the four
usability gaps a season of driving turned up.

## The five problems

1. **Both status LEDs were hardcoded** — left the rev flasher, right the
   SensorEgg Temp1 tri-state. The right one was a shipped bug: on a
   build without `BIRDSEYE_ENABLE_SENSOREGG` the accessor is permanently
   NaN, so `evalStatus` took its `invalidColor` branch and the pixel sat
   **solid blue for every second of every race on every stock logger**.
   Plan 0007 recorded it as "known, accepted until phase-2
   assignability".
2. **The bar was dead on a car with no tachometer.** With RPM pinned at
   0 the scale renders nine dark pixels from lights-out until the first
   lap completes and pace takes over.
3. **`rev_limit` was misnamed.** It is the SHIFT/warning point;
   `overrev_limit` (plan 0007) is the actual limit. Anyone opening
   `/SETTINGS.json` learned the wrong thing.
4. **Purple only ever fired on a sector**, never on a session-best lap —
   the bigger achievement had no signal at all.
5. **Egg gating leaked.** Three things were not behind the flag: the
   status LED above, the `temp1_alert_c` setting, and the DOVEX
   `Temp1`/`Junction1`/`Temp2` columns.

## Status-LED modes — the new `led_status` pure unit

Two settings, `led_status_left` and `led_status_right`, each naming one
of eight modes. `led_status::evalMode()` is a pure function of an
`Inputs` snapshot the glue builds once per frame and evaluates twice, so
the two pixels can never disagree about the same instant.

| Mode | Token | Rendering |
|---|---|---|
| Off | `off` | dark |
| Target RPM | `rpm` | red flash at/above `target_rpm`, clears below 97 % — byte-for-byte the pre-0012 left LED |
| Target speed | `speed` | red flash at/above `target_speed_mph`, clears 2 mph below; **off** with no GPS fix |
| GPS | `gps` | steady: red no sats · yellow sats-no-fix · blue fix-no-time-lock · green locked |
| Camera | `camera` | off unpaired · yellow session running with the camera not up · **flashing** blue linked but not ce82-subscribed · steady blue linked + subscribed · red recording |
| Last lap | `lap` | steady, held: off no comparison · green faster than the previous lap · red slower · purple session best |
| Last sector | `sector` | same, for the most recently closed sector vs the same sector last lap |
| EGT | `egt` | Temp1 tri-state — red flash hot, off good, solid blue no probe signal. **SensorEgg builds only** |

Three design choices worth keeping:

- **Threshold modes delegate.** `rpm`/`speed`/`egt` build a
  `led_modes::StatusAction` on the stack and hand it to the existing
  `led_modes::evalStatus()`. Hysteresis, latch-release-on-invalid and
  flash phase stay in one place with the regression tests they already
  have — including the inverted-hysteresis strobe guard. `led_status`
  owns the mode table; `led_modes` keeps owning threshold evaluation.
  `flashOn()` was extracted from `evalStatus` so the camera mode's flash
  and the threshold modes' flash are one definition, not two.
- **`eggSupported` is an `Inputs` field, not an `#if` inside the unit.**
  The host test binary compiles with the flag off, so a compile-time
  gate would make the beta behaviour untestable. It also keeps the
  *setting* round-tripping: `egt` still parses, still reports back over
  `SGET`/`SLIST`, and only the rendering is gated. There is a test that
  drives `kEgt` with `eggSupported = false` and a hot **valid** reading
  and requires darkness — proving the gate is independent of the NaN
  path that caused the original bug.
- **`parseMode` is strict** (`bool parseMode(const char*, Mode*)`),
  deliberately unlike `tach_filter::modeFromSetting()`'s safe fallback.
  A typo must not silently pick a different mode or dark an LED: false
  means "keep the compiled-in default", the same contract
  `setting_parse::parseIntSetting()` has.

### They stay lit on the menu — but only two of them

`gps` and `camera` answer "am I ready to drive", which is a question you
ask in the paddock, not on track. `led_status::activeOutsideRace()` names
exactly those two; every other mode, and the whole 9-px bar, stays
race-only as before. A pixel that is not being rendered has its latch
**released**, not merely darkened — leaving one set would let a threshold
that tripped in the last seconds of a session flash the instant the next
one starts.

## Speed bar for tach-less sessions

The strip ladder's final `else` splits on `raceEntryCause ==
RACE_ENTRY_TACH`:

```
engine stopped (proven tach session, 0 rpm) -> bar off
no fix + timeValid                          -> green search pip
pace valid                                  -> pace pip
tach session                                -> RPM scale   (0..target_rpm)
else                                        -> SPEED scale (0..target_speed_mph)
```

`RACE_ENTRY_TACH` is the right gate and needs no new state:
`idle_policy::tachProven()` already promotes MANUAL and SPEED sessions to
TACH the moment the engine clears 500 rpm, so "not TACH" means precisely
"this session has never seen an engine". It is the same predicate
`raceEngineStopped()` keys on, so a speed-scale session can never blank
its own bar. The rejected alternative — a live `tachLastReported > 0`
test — would flip the bar between two scales at every stall, idle dip and
pit stop.

Accepted consequence: a tach kart entered via the menu shows the speed
bar until the engine first crosses 500 rpm, a second or two of pit-out.
It self-heals and never reverts.

**`kSpeedRedFrac = 1.0f` — the speed bar has no red band.** The RPM bar's
red half means "approaching the limiter, back off". There is no
equivalent hazard in approaching your target speed, and painting the same
nine pixels red for the thing you were aiming at inverts their meaning on
identical hardware. `renderScale`'s `redFrom` lands at `kStripCount`,
which no lit index reaches, so the bar is green all the way up. The "you
got there" signal is a full bar plus the `speed` status mode.

## Lap/sector verdicts — one state machine, not two

The `lap` and `sector` modes need the previous lap's per-sector times and
the previous lap time. `sector_purple` already owned the close-edge state
machine that survives the library updating bests **at the start/finish
crossing** — snapshot at sector OPEN, derive S3 as `lastLapTime - s1 -
s2`. Duplicating that was the real hazard, so it was extended instead:
`update()` now returns an `Event` (which sector or lap closed, its
verdict, and the two purple flags) and `State` gained
`lastSectorTime[3]`, `prevLapTime` and `bestLapAtOpen` — the same
open-time snapshot trick, one level up, because the library folds a
finished lap into `getBestLapTime()` at the crossing too.

**The verdict is against the LAST RECORDED time, not the best.** Against
the best you only ever get purple or red, which says nothing about
whether you are improving. So `kBetter`/`kWorse` compare against the
previous same-sector / previous lap, and `kBest` overrides when the
session best also fell. `kNone` (nothing to compare against yet) renders
dark rather than guessing — so lap 1 is unlit.

Two smaller rules: equal times read as `kWorse` (the unit's existing
strict-improvement rule, extended), and a sector that never closed —
a dropout across the line leaving a zero current-lap time — emits **no
event at all** and leaves the recorded time alone, so one glitch does not
dark the indicator for a lap.

**Sectors are now optional.** Lap tracking runs whenever the race has
started, including a Lap Anything session with no sector lines; before
this the whole monitor reset every frame on that input. Only the sector
half is gated on `sectorsConfigured`.

The file keeps the name `sector_purple` — purple is now one of its
outputs rather than its whole job, and renaming it would have churned two
CMake lists, a workflow, a test filename and three includes for zero
behaviour change, burying the diff. A rename is a clean standalone
follow-up; the header says so.

## Two-stage purple

A session-best **sector** keeps the shipped 1600 ms animation. A
session-best **lap** gets `renderPurpleLap()`: 2600 ms with two
centre-out wave passes — the same length as the boot flourish, already
proven not to overstay. Both call one static renderer parameterised by
duration/wave/passes, so a visual tweak lands in one place and
`renderPurple()`'s golden frames still pin the sector animation byte for
byte (there is a test asserting the two produce identical output through
the first wave).

A lap purple **outranks** a sector one: they land on the same frame
whenever the last sector of a purple lap is also purple, and re-arming
the shorter animation there would cut the bigger moment short.
`neopixelNotifyPurpleLap()` takes over an in-flight sector animation;
`neopixelNotifyPurpleSector()` declines to stomp a lap one.

## `rev_limit` → `target_rpm`

The key is renamed, with a migration at the top of
`ensureDefaultSettings()` — before the defaults table walk, because that
table writes the *default* for a missing key and would otherwise stamp
15000 over a user's tuned 7550 on the very next boot:

```
if target_rpm absent and rev_limit present:
    if setSetting("target_rpm", <rev_limit's value>) succeeded:
        removeSetting("rev_limit")
```

The old key is dropped only once the new one is confirmed written, so a
failed or refused write leaves the device exactly as it was rather than
silently resetting its shift point. `settings.ino` gained
`removeSetting()` for this — deliberately without `setSetting()`'s
corrupt-file heal, since dropping a key is never urgent enough to justify
quarantining someone's settings behind their back.

## Egg gating

| Item | Before | After |
|---|---|---|
| `sensoregg.ino`, accessors, Temp1/Temp2 pages, page constants, camera-test `egg:` line | gated | unchanged |
| Right status LED hardwired to Temp1 | **ungated** — solid blue all race on stock builds | one selectable mode; `eggSupported == false` renders it dark |
| `temp1_alert_c` default + boot read | **ungated** | gated |
| DOVEX `Temp1`/`Junction1`/`Temp2` columns + CSV header | **ungated** (`nan` on every stock row) | gated |

**The log format now forks by channel**, and that is a deliberate reversal
of the rule the DOVEX columns were originally written under ("a log from
a SensorEgg build and one from a stock build have identical shape").
Stock logs are 13 data columns, SensorEgg logs 16. Three dead `nan`
columns on every row of every stock log paid for nothing, and readers key
the data section off its own CSV header line where all three are
optional — the companion app's `doveParser.ts` builds a name→index map
and does not read those columns at all today, so both shapes parse with
no client change.

`int settingTemp1AlertC` stays defined unconditionally (only the default
and the read are gated) so the LED glue needs no `#if` at its use site.

## Settings

| Key | Default | Clamp | Notes |
|---|---|---|---|
| `target_rpm` | `15000` | 1000–20000 | replaces `rev_limit`, migrated |
| `target_speed_mph` | `60` | 5–250 | mph on device; the app converts for display. Floor is 5, not 0, because `renderScale`'s span guard blanks the bar on a zero ceiling — indistinguishable from dead hardware |
| `led_status_left` | `rpm` | strict `parseMode` | preserves the old left LED |
| `led_status_right` | `egt` (SensorEgg) / `lap` (stock) | strict `parseMode` | preserves the old right LED on beta; gives stock builds something useful instead of a blue pixel |
| `temp1_alert_c` | `650` | 50–1200 | now SensorEgg-only |

**Byte budget.** The read cap is 1023 bytes and exceeding it is a silent,
self-inflicting boot loop (see `settings.h`). Measured, not estimated:
the default file goes from 543 bytes to **570 on a stock build** (23
keys) and **592 on a SensorEgg one** (24) — 599 / 621 with the longest
value every key will accept, and ~612 for the single boot on a SensorEgg
device where both `target_rpm` and `rev_limit` exist. Gating
`temp1_alert_c` and dropping `rev_limit` paid for two of the three new
keys. That leaves roughly sixteen average keys of headroom; each costs
`len(key) + len(value) + 6`.

## CI

`.github/workflows/clang-tidy.yml` lists every pure unit by hand —
`led_status.cpp` is added there, or it would be silently unanalysed.

`compile-sketch.yml` gained a `compile-stock-arm` job, gated on the same
BETA condition as the flag-on build. The main job picks ONE flag set per
PR, so a BETA-targeted PR compiled only with the flags on — and this plan
adds several `#else` arms (the DOVEX row and header shapes, the settings
defaults) that would otherwise reach master having never been through a
compiler. One board is enough; the arms differ only in feature flags.

## Testing

`tests/led_status_test.cpp` is new (mode-name round-trip and strict-parse
rejection, every ladder level, the camera flash-vs-steady distinction,
threshold hysteresis, the egg gate, verdict colours, latch lifetime).
`sector_purple_test.cpp`'s eight existing cases are preserved through a
`purpleOf()` shim and joined by the lap/sector verdict cases, including
the mirror of the S3 anti-race case one level up: fold the new best into
`bestLapTime` *before* the edge is seen and require the purple lap to
still fire.
