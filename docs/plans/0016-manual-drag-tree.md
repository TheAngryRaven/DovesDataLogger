# Manual Drag Mode — the Christmas Tree

> Status: **IMPLEMENTED**. Plan 0015's drag mode becomes the **Automatic**
> flavor of a two-mode feature; this plan adds **Manual**: a drag-strip
> christmas tree run off the 9-px LED strip and mirrored on the OLED,
> with red-light fouls, a failed-to-launch timeout, a reaction-time stat,
> and press-any-button re-arm between runs. Automatic behavior is
> untouched (its only delta is one extra menu hop through the new mode
> page).

## What it is

Main menu → **Drag** → distance → **Auto / Manual**. Manual runs the
strip like a staging tree:

```
(press any button to arm)
----w----   staged: white pip once the car has been stopped a moment
---ywy---   3        \
--yywyy--   2         }  500 ms cadence (sportsman tree)
-yyywyyy-   1        /
gyyywyyyg   GO (screen flashes GO; physics launch gate opens)
```

The screen is **pinned** to staging info for the whole manual session —
`STOP TO STAGE`, the big 3/2/1 countdown, flashing `GO`, the live ET
during the pass, and a results screen (ET, trap, 0-60, **RT**) — until
the driver presses a button to re-arm. Movement during the yellows is a
**RED LIGHT** foul; sitting still for 5 s after green is **FAILED TO
LAUNCH**; both flash the strip red, say so on screen, and wait for a
button. Holding Select ~2 s in any non-running state ends the session.

## Structure: one state machine, two outputs

`drag_tree.{h,cpp}` (pure, host-tested) is the single sequencer driving
BOTH the LED strip and the OLED — `renderStrip()` for the pixels,
`countdownDigit()`/`flashPhase()` for the screen — so the two can never
disagree. Flash phases derive from timestamps via `led_modes::flashOn`
(never a per-render toggle: the OLED refreshes at 3 Hz and would drift
from the 30 Hz strip). The flash half-period is 500 ms, deliberately
slower than the LED alert modes' 100–250 ms, because a half-period near
the display's ~333 ms render period aliases into irregular flicker.

The tree **never duplicates physics**. It observes `drag_timer`:
staging = `staged()` (the existing ≤1 mph/1 s standstill logic), the run
= `runActive()`/`runs()`, and the movement that fouls is the same
threshold that would launch the physics (`drag_timer::kLaunchMinMph`).
The sketch keeps only `dragStagingLoop()` — an Inputs/step/Effects pump
in the `gpsStatusPageLoop()` slot (after `readButtons()`, before
`displayLoop()`, with `resetButtons()` on consumed presses — the
gps_status_page pattern exactly).

## The launch gate

`DragTimer::setLaunchEnabled(bool)` (default **true**, so automatic mode
is byte-identical): while disabled, a rollout-at-speed in STAGED
`resetToArmed()`s instead of launching — a pre-green move is the tree's
foul, never a run. The glue re-asserts the gate every loop iteration:
open exactly while the tree shows green. `GPS_LOOP()` (the physics feed)
runs before `dragStagingLoop()` in the frame, so a foul move reaches the
physics while the gate is still closed, and the green-edge enable
applies to the next fixes ~4 ms later — noise against a ~200 ms human
reaction.

## Reaction time

RT = interpolated rollout crossing − green light, both in **Unix epoch
ms**: the green edge is stamped with `getGpsUnixTimestampMillis()` and
the crossing comes from the new `DragTimer::runStartEpochMs()` (the
same interpolated `runStartMs_` the ET uses), so RT inherits the ET's
sub-fix precision and the two clocks can never mix bases. Display-only:
shown on the results screen, **not** in the DOVEX header (same rationale
as trap/0-60 — the format is frozen and the rows carry everything).

## States and transitions (drag_tree)

kAwaitArm → (button) → kWaitStop → (physics staged) → kPreStage →
(held `kPreStageHoldMs` = 2000 ms — deliberately ON TOP of the physics'
1 s promotion, the requirement's "stopped a couple seconds") → kYellow1
→ kYellow2 → kYellow3 (500 ms each) → kGreen → (runActive rises) →
kRunning → (runs incremented) → kResults → (button) → kWaitStop.

Exits from the happy path:
- Yellows + movement ≥ 2 mph → **kRedLight**.
- Yellows + unstaged/fix lost *without* movement → kWaitStop, silent — a
  GPS hiccup is not a foul.
- kGreen + still for `kFailedLaunchMs` (5 s) → **kFailedLaunch**.
- kGreen + moving but the physics never launched for `kGreenNoRunMs`
  (3 s; degenerate anchor-loss case) → **kAborted** — never a hang.
- kRunning + `runActive` falls without a run recorded (the physics'
  silent aborts: mid-run standstill, fix gap, prove-out, time step) →
  **kAborted** — surfaced, because a pinned screen showing a frozen ET
  with no explanation reads as a crash.
- Any non-running stage: Select held `kExitHoldMs` (2 s) → end session.
  Requires one observed release first (the press that entered the
  session can't be the exit), and a held side button disarms it so the
  Select+side reboot combo stays reachable (sd_format_page precedent).
  The hold deliberately survives the results→re-arm press: press Select
  on the results screen and keep holding, and it still exits 2 s later.

## Display pin

`PAGE_DRAG_STAGING` (−19) is pinned exactly like the GPS-lock hold:
direct `currentPage =` assignment at the top of `displayLoop()` (applied
AFTER the lock hold so this pin wins — the staging page has its own
WAITING FOR GPS line), `buttonsDisabled = true`, and a no-op branch in
the button chain. The pin's only gate is `dragManualMode`, cleared in
`endRaceSession()` — the Select-hold exit, auto-idle, and shutdown all
funnel through it, so every ender releases the pin by construction (the
2026-07-19 wedge-precedent check).

## LEDs

The tree renders **strip-only** as a new arm of the strip-selection
cascade in `NEOPIXEL_LOOP()`, after the fix/timeValid gate (no fix →
the familiar search pip wins; the physics can't stage anyway) and above
pace/RPM/speed. The status LEDs stay live, and boot/overrev/purple keep
their priority above the whole branch. During the run (`kRunning`) the
arm goes inactive and the normal RPM/speed scale takes over;
`paceValid` gains `!dragManualActive()` so manual runs 2+ get the scale
instead of a meaningless centered 0.0-pace pip (constant false for every
other mode — auto unchanged).

## Menu

Distance picker → `PAGE_DRAG_MODE` (−18): Auto / Manual / Back, a
static 3-row reversed menu titled with the picked distance
(`dragPendingDistanceIdx` carries it between the pages). Automatic lands
on GPS_SPEED exactly as before.

## What was deliberately NOT built

- No pro tree / adjustable cadence, no random stage delay.
- RT not logged (display-only; derive better offline from rows if ever
  needed).
- No jump-start detection during PRE-STAGE (movement there just
  re-stages silently — the foul window is the yellows, like a real tree
  between pre-stage and green).
