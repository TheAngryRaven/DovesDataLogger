# RPM Accuracy — Spark Type & Cylinder Count Settings

> Status: **CONCEPT** — split out of plan 0002 (sprint mode); independent of
> it. Prompted by the same autocross-kart user group: unknown engine, "kart
> kart or some wacky 2cyl monster", so the logger can no longer assume one
> ignition pulse per revolution.

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
- Apply the divisor **once, at the period→RPM conversion in `TACH_LOOP()`**,
  *before* the Kalman filter — the filter's Q (800 RPM², tuned for kart
  inertia) and R are in true-RPM units, so feeding it corrected measurements
  keeps the tuning meaningful. Every consumer (display 3 Hz, DOVEX rows
  25 Hz, camera FSM, auto-race check) reads `tachLastReported`, so one
  conversion point corrects them all.
- The divisor math itself belongs in the host-tested `tach_filter` pure unit
  (or a tiny sibling) with tests per the repo convention.

## Ripple effects (mostly good)

- **RPM thresholds become engine-independent**: auto-race entry (>500),
  camera wake/record/stop (500 / 1500 / 300) currently assume 1 pulse/rev;
  on a 2-cyl they'd fire at half the true RPM. With correction, thresholds
  mean what they say on every engine.
- **Pulse-rate ceiling**: the 3 ms minimum pulse gap caps at ~20 000
  pulses/min. At divisor 2 (2-cyl every-rev) that's only **~10 000 true
  RPM** — a screaming 2-cyl 2T could exceed it. The min-gap likely needs to
  derive from `pulses_per_rev` (e.g. 3 ms ÷ ppr, floor ~1.5 ms) — verify
  ISR headroom before lowering.
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

Concept only. No dependency on plan 0002 — can ship before or after sprint
mode. Should ship *before* any camera/auto-race tuning work for multi-cyl
engines.
