# RPM Accuracy — Spark Type & Cylinder Count Settings

> Status: **SHIPPED.** Split out of plan 0002 (sprint mode); independent of
> it. Prompted by the same autocross-kart user group: unknown engine, "kart
> kart or some wacky 2cyl monster", so the logger can no longer assume one
> ignition pulse per revolution.
>
> Settings key is **`cylinder_count`** as drafted. The webapp's `enum` control
> landed alongside it (DovesDataViewer), so `spark_mode` is a real dropdown
> rather than the free-text stopgap this plan allowed for.

## Goal / problem

The tachometer counts **ignition pulses** from the inductive pickup and
currently treats one pulse as one revolution. That's only true for a 1-cyl
2-stroke or a 1-cyl wasted-spark 4-stroke — the common kart cases, which is
why it's been fine so far. Other engines skew RPM by a fixed factor:

| Engine | Pulses per revolution |
|---|---|
| 2-stroke, per cylinder | 1 × cylinders |
| 4-stroke wasted spark, per cylinder | 1 × cylinders |
| 4-stroke single-fire (no wasted spark), per cylinder | 0.5 × cylinders |

The logger should reflect **true RPM** in both the display and the DOVEX
`rpm` column, based on two new device settings.

## New settings (2)

| Key | Default | Meaning |
|---|---|---|
| `spark_mode` | `wasted` | `wasted` = one spark per rev (2T, or 4T wasted spark); `single` = one spark per two revs (4T single-fire) |
| `cylinder_count` | `1` | Cylinders visible to the pickup |

`pulses_per_rev = cylinder_count × (spark_mode == wasted ? 1.0 : 0.5)`
`true RPM = pulse RPM ÷ pulses_per_rev`

**Defaults exactly reproduce today's behavior** (divisor 1.0), so existing
devices are unaffected by `ensureDefaultSettings()` auto-populating the keys.

## Approach

- Settings rows in `ensureDefaultSettings()` (`settings.ino:102-108`),
  loaded into globals in the settings block (`BirdsEye.ino:765-795`),
  applied on boot like every other setting (webapp `SSET` + auto-reboot).
- **Single source of truth (decided)**: true RPM is computed **once** — at
  the period→RPM conversion in `TACH_LOOP()`, *before* the Kalman filter —
  and published as the one canonical value (`tachLastReported`). **No
  consumer ever re-applies or re-derives the correction**: display (3 Hz),
  DOVEX rows (25 Hz), camera FSM inputs, and the auto-race check all read
  the same already-corrected number. Any future consumer must read it too,
  never the raw pulse rate. While implementing, audit that nothing else
  computes RPM from pulse periods independently.
- **Kalman note**: the correction must sit *before* the filter because the
  filter's tuning is in true-RPM units — Q = 800 RPM² models kart engine
  inertia and R_BASE = 2500 RPM² models measurement noise
  (`tach_filter.h`). Feeding pulse-RPM and dividing afterward would make a
  2-cyl engine's process/measurement noise effectively half-scale, i.e.
  differently-filtered behavior per engine type. Correct-then-filter keeps
  one tuning valid for everyone.
- The divisor math itself belongs in the host-tested `tach_filter` pure unit
  (or a tiny sibling) with tests per the repo convention.

## Ripple effects (mostly good)

- **RPM thresholds become engine-independent**: auto-race entry (>500),
  camera wake/record/stop (500 / 1500 / 300) currently assume 1 pulse/rev;
  on a 2-cyl they'd fire at half the true RPM. With correction, thresholds
  mean what they say on every engine.
- **Pulse-rate ceiling** (resolved): the min gap now derives from
  `pulses_per_rev` — 3 ms ÷ ppr — in `tach_filter::minPulseGapUs()`.
  **The floor is 750 µs, not the 1.5 ms sketched here**: 1.5 ms still put a
  3-cylinder at ~13 300 true RPM, below the old ceiling. 750 µs holds the
  full ~20 000 through four cylinders; past that the floor binds (10 000 at
  eight), which is far clear of anything this logger targets.
  ISR headroom was never the constraint — the body is <1 µs, so the floor's
  worst case (~1300 int/s) is negligible. Ringing was, and the margin holds
  from both ends: the tach input is RC-filtered (~100 µs) and the documented
  pickup circuits emit pulses **milliseconds** wide — `TACHOMETER/README.md`
  records circuit 1's 5 ms pulse as itself the ~9800 RPM limit on that
  hardware, i.e. the pulse width, not the debounce, is what binds there.
- **Pickup placement nuance (document for users)**: `cylinder_count` is
  "cylinders the pickup *sees*". A pickup clamped around ONE plug wire of a
  2-cyl sees one cylinder → leave `cylinder_count = 1`. Only a pickup on a
  shared coil/all-cylinder harness sees them all.

## Touch points

`settings.ino` (defaults), `BirdsEye.ino` (globals + load), `tachometer.ino`
+ `tach_filter.{h,cpp}` + `tests/tach_filter_test.cpp` (divisor + tests),
README.md + CLAUDE.md settings tables. Webapp: keys auto-appear via the
generic settings list; a proper `enum` control type in
`deviceSettingsSchema.ts` is wanted for `spark_mode` (shared need with
nothing else right now — plain string works until then).

## Status

**Shipped.** No dependency on plan 0002. Landed before any camera/auto-race
tuning for multi-cylinder engines, as intended — those thresholds now mean
what they say on every engine.

### What actually landed

- `tach_filter::revsPerPulse()` / `minPulseGapUs()` — the geometry, in the
  host-tested pure unit, sharing one private `pulsesPerRev()` so the RPM
  scale and the debounce can never disagree.
- `tachRevsPerPulse` / `tachMinPulseGapUs` became boot-set globals; the
  correction point in `TACH_LOOP()` was already there and already ahead of
  the Kalman filter, so no restructuring was needed.
- Settings default in `ensureDefaultSettings()`; loaded in `setup()`.
  Anything other than an explicit `"single"` degrades to `wasted`, so a
  blank or future value reads as today rather than doubling every RPM.
- The audit the plan asked for came back clean: nothing else derives RPM
  from pulse periods. The only other `60e6` in the tree is the simulator's
  pulse *generator*, which is the inverse and matches the default.

### Verified

Unit tests cover the geometry, the clamps (a corrupt `cylinder_count` can
never divide by zero) and the ceiling property. End-to-end in the simulator:
6000 pulses/min reports 6000 RPM at the defaults and 3000 with
`revsPerPulse` forced to a twin — so the wiring is proven, not just the
math. Golden fixtures and the lap oracle are unchanged, which is the
evidence that existing devices are unaffected.
