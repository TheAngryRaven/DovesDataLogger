# 0011 — Main-loop CPU profiling (beta channel)

**Status:** implemented (beta channel only)
**Flag:** `BIRDSEYE_ENABLE_PROFILING` (`project.h`, default 0; `beta.yml`
and `compile-sketch.yml` pass `=1` on the BETA branch)
**Touches:** `loop_profile.{h,cpp}` (new pure unit), `profiling.{h,ino}`
(new glue module), `BirdsEye.ino` (loop instrumentation, page constant),
`display_pages.{h,ino}` (LOOP PROFILE page), `display_ui.ino` (routing),
`neopixel.ino` (yields the boost EN pin), `beta.yml`,
`compile-sketch.yml`, `clang-tidy.yml`, `tests/`

## Why

The commercial board has to be specified, and two open questions gate it:

1. **nRF52840 or nRF5340?** The 5340 is the reliability upgrade for BLE
   (dedicated network core, so the radio stack stops sharing a CPU with
   the superloop), and it is more silicon than the product may need.
2. **Arduino core or Nordic SDK?** Leaving the Arduino core is a large,
   one-way piece of work. It is worth doing only if the core is where a
   meaningful fraction of the time goes.

Both are answerable with measurement rather than argument, and neither
is answerable today: the firmware carries no timing instrumentation
beyond a `HAS_DEBUG` "SLOW LOOP" print with millisecond resolution — too
coarse to see a subsystem that costs 400 µs, and compiled out of every
shipped build.

So: measure a real iteration, on the real hardware, under a real 25 Hz
GPS + logging + LED + BLE workload, and see where it goes.

## What "CPU usage" means here

> **Superseded, and the correction is the interesting part.** This
> section originally argued that a duty cycle was meaningless because
> `loop()` runs back to back and therefore the CPU is busy 100% of the
> time by construction. That was an *assumption*, it was baked into the
> measurement (see *Two clocks* below), and it made the first hardware
> reading wrong. The profiler now measures how much wall time is
> actually spent executing `loop()` and reports the balance as `SLP`.
> Everything below about the shape of an iteration still holds; it is
> just no longer the only thing worth reporting.

The number that carries information is the **shape of an iteration**:

- how long one takes (mean **and** worst case — an SD garbage-collection
  stall of 100 ms–2 s is invisible in a mean),
- how that time divides between subsystems,
- how much of it no subsystem accounts for.

That is what decides both questions above. If the loop is 4 ms and
`readButtons()`'s multi-sample debounce is 1.5 ms of pure blocking
`delayMicroseconds`, the answer is "fix the firmware", not "buy a bigger
chip". If it is spread evenly across genuine work, the chip is the
conversation.

## Two instruments

**The pin.** Pin 30 is driven HIGH for the span being profiled and LOW
outside it. A scope or logic analyser reads the loop period off the
rising edges and the span's cost off the high time, with no software in
the measurement path — which is exactly what makes it worth having
alongside the software numbers: it is the thing that says the software
is telling the truth. `PROFILING_PIN_SECTION` picks the span (the whole
loop body by default; `-DPROFILING_PIN_SECTION=PROF_SEC_GPS` and friends
re-point it).

**The rollup.** Every section is also timed in software and rolled up
once a second onto the LOOP PROFILE race page — first page of the race
rotation on a profiling build, three Lefts from the speed page the
session lands on. That is the instrument you can read while driving, and
the one that survives not having a scope at the track.

## The pin costs the 5 V rail, and that is the whole trade

Pin 30 is the NeoPixel boost converter's EN line. It cannot be both.

The profiler takes it, so a profiling build never drives EN — not at
setup, not at sleep, not on the charging-loop resume. The regulator sits
at its hardware default (EN pulled up = rail on), which is why this
works at all: the rail does not *need* firmware control, it only needs
it to be switchable, and switching it is a *use* requirement, not a
*testing* one.

Two consequences, both accepted:

- **The rail stays up through System OFF.** GPIO levels are retained
  there and the driven LOW was the only thing holding it down (the same
  retention behind the "blue conn LED stays on after sleep" report). A
  profiling unit left asleep on a battery with a strip wired to it goes
  flat. Bench builds only.
- **If the EN jumper is still physically connected on the rig, the
  toggling chops the rail at loop rate.** Pull it, or tie EN high,
  before profiling. The LEDs are not what is being measured.

Prod is untouched: `BIRDSEYE_ENABLE_PROFILING` defaults to 0, the
section brackets are macros that expand to the bare call, and pin 30
goes on being EN exactly as it always has.

## Timebase: DWT, with a fallback that announces itself

Most sections are well under a microsecond, so `micros()` alone would
quantise half of them to zero. The profiler uses the Cortex-M4's DWT
cycle counter — 64 ticks per microsecond at 64 MHz — and falls back to
`micros()` when it will not run (a debug probe can own TRCENA, and
CYCCNTENA is architecturally optional).

"We set the bit" is not evidence, so `profEnableDwt()` reads the counter
across a short spin and requires it to have moved. When the fallback is
live the page prefixes its first row with `*`, because at 1 µs
resolution the small numbers are noise and a reader has to know that.

The pure unit therefore accumulates **ticks**, not microseconds, and is
handed `ticksPerUs` only at rollup — ratios come out of raw ticks and
lose nothing.

## Design notes worth keeping

- **Two clocks, and mixing them was the first real bug.** DWT counts CPU
  *cycles*: it stops dead when the core halts (WFE/WFI in the FreeRTOS
  idle task, `sd_app_evt_wait`). It is the right instrument for timing
  code that is definitely executing and the wrong one for deciding how
  much time has passed. Closing the rollup window on it meant the "one
  second" window was one second of CPU-awake time, so the loop rate came
  out multiplied by the sleep factor and every share was a fraction of
  awake time wearing a wall-time label. Windows now close on `millis()`
  and all shares are of wall time; durations are still ticks. The
  `micros()` fallback never had the bug — it is a real clock.
- **Saturating accumulators.** A uint32 of DWT ticks is only ~67 s and a
  rollup can be arbitrarily late if the loop stalls. A saturated window
  reads as pegged, which is true; a wrapped one reads as near-idle,
  which is a lie.
- **`OTH` is reported, not hidden.** Loop time that no section bracketed
  gets its own slot. It is the honesty check on the instrumentation: if
  it is large, time is going somewhere `loop()` is not bracketing.
- **A scope guard, not a `PROFILING_LOOP()` call.** Both parked branches
  (`bleActive`, `usbMscActive`) return early and `enterShutdown()` never
  returns at all. A profiler that goes quiet exactly when the firmware
  parks would be measuring the wrong thing, so the whole-iteration
  timing and the rollup live in `~ProfLoopScope()`.
- **The overhead is inside the numbers — and at the real loop rate it is
  order 10%, not the 0.1% first claimed.** A bracket is two counter reads
  plus a saturating add: ~30 cycles, ~0.5 µs, so ~6–8 µs per iteration
  across all 13. That was written off against the ~4 ms iteration this
  project's docs assumed; the first hardware run measured a mean
  iteration under 100 µs. A bracket still lands in the section it
  brackets rather than in `OTH`, because for a subsystem's *share* that
  is the honest accounting — but sections under ~1% sit at their own
  bracket's noise floor, and the un-instrumented loop is faster than the
  rate shown.
- **The pin edges are `#if`'d, not branched.** With the default
  whole-loop setting the runtime comparison is provably false for every
  section, and a dead branch in the two hottest functions in the
  firmware is precisely the cost a profiler must not add.
- **Profiling implies `debug_pages=show`.** The race rotation is a
  contiguous range and the profile page sits below `GPS_DEBUG`, so
  lowering the start to reach it pulls the two diagnostic pages in too.
  On a bench build that is what you want anyway.

## What to do with the numbers

Read a full session — idle in the pits, out on track, during an SD
flush, with the camera connected and streaming its 10 Hz GPS overlay.
Then:

- **One subsystem dominates** → fix or offload that, and neither the
  5340 nor the SDK port is justified yet.
- **`BLE`/`CAM` cost is spiky and correlates with radio activity** →
  that is the 5340's actual argument: a separate network core takes the
  stack off this CPU entirely.
- **Time is spread evenly across genuine work and `loopMaxUs` is
  brushing the 40 ms 25 Hz PVT budget** → the chip is the conversation.
- **`OTH` is large** → the instrumentation is incomplete; bracket more
  before drawing any conclusion.

## First hardware run (bench, 2026-08-24)

Not a track session — treat these as a bench baseline, not the answer:

```
999Hz av0.0 mx42          <- 999 and 0.0 are display clamp artifacts, since fixed
GPS 20.6   TCH  3.0
ACC  0.7   BLE  0.8
EGG  2.3   TRK  1.3
LAP  2.0   IDL  4.6
CAM  7.9   LED  3.1
BTN  8.8   PGE  1.1
DSP 14.9   OTH 17.6
```

No `*` and no `!`, so the DWT timebase and the pin were both live. What
it says, and what it changes:

- **The rate is not trustworthy as printed, and chasing why found a
  bug.** It hit the 999 display clamp and the mean read 0.0 — both
  fields had been sized from the stale ~250 Hz figure. But the deeper
  problem was the window: it was closed on the DWT cycle counter, which
  stops when the core sleeps, so the reported rate is the true rate
  multiplied by however much of the second the CPU was awake, and every
  share is a fraction of awake time rather than of wall time. Whether
  the loop really is that fast, or the CPU sleeps most of the second,
  the fixed build now answers directly: `SLP` reads ~0 in the first
  case and large in the second.
- **`mx` 42 ms against a 1600 ms GPS-ring ceiling and a 4000 ms
  watchdog.** Worst-case latency is nowhere near anything that matters.
- **Nothing dominates.** The largest section is `GPS` at 20.6%, and the
  sections sum to ~88.7%. Shares are ratios of the same (mis-sized)
  denominator, so their RELATIVE sizes survive the bug even though their
  absolute values do not — the ranking below is still worth reading.
- **The shape is polling overhead, not work.** `BTN` at 8.8% with nobody
  touching a button is six `digitalRead()` calls per iteration at over a
  kilohertz; `DSP` at 14.9% is mostly a `displayLoop()` that early-returns
  on a 3 Hz gate; `OTH` at 17.6% is the unbracketed glue's `millis()`
  calls. Subsystems that need 3–50 Hz are being polled ~1000× faster.

**Read on the board question, held loosely until a re-run:** `mx` is
nowhere near any budget and no section dominates, so nothing here argues
for an nRF5340 on throughput grounds. If the 5340 is bought it should be
for BLE reliability — which this page cannot measure directly (see the
caveat below).

## Still outstanding

- **A re-run on the fixed build.** The absolute rate and every absolute
  share from the run above are suspect until the window is wall-clocked.
  The first thing to read on the new build is `SLP`: near zero means the
  loop genuinely is that fast and the old numbers were only clamped;
  large means the CPU sleeps and the old numbers were inflated by
  exactly that factor.
- **A session under real load.** The run above did have a GPS lock and
  did trigger camera recording, so it was not idle — but a full track
  session with sustained logging is still the number that decides
  things.
- **The BLE question.** Neither instrument separates SoftDevice /
  Bluefruit task time from the section it preempted — FreeRTOS preemption
  smears it across whatever was running. Watching how section *variance*
  moves when a phone connects or the camera links is the available proxy;
  a direct measurement would need the ISRs instrumented too.
- The nRF5340 comparison needs the same instrument on that target. The
  pure unit is board-portable by construction (no Arduino headers, no
  platform `#ifdef`s) — same rule as `camera_fsm`, for the same reason.
