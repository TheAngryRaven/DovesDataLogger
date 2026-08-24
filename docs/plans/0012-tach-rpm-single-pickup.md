# 0012 — Tach RPM from a single pickup: cylinder count is not a divider

> Status: implemented.
> Supersedes the RPM geometry of
> [0003](0003-rpm-spark-cylinder-settings.md). `spark_mode` survives
> unchanged; `cylinder_count` survives as a setting but leaves the math.

## The bug

Plan 0003 made RPM "true RPM" with:

```
pulses_per_rev = cylinder_count × (spark_mode == wasted ? 1.0 : 0.5)
```

That formula is only correct if the pickup sees **every** cylinder — a
clamp on a shared coil or king lead. This device has **one sense wire and
one clamp**, and it goes around **one spark plug wire**. On that hardware
the formula divides RPM by the cylinder count.

The case that surfaced it: a **V8 on a traditional magneto**, which is
electrically compatible with the pickup. Configured honestly — 8
cylinders, single fire — the device read **an eighth** of the real crank
speed:

| | pulses/min at 3000 RPM | plan 0003 reads | correct |
|---|---|---|---|
| V8, single fire, one plug wire | 1500 | 375 | 3000 |
| Twin, wasted spark, one plug wire | 3000 | 1500 | 3000 |

Plan 0003 documented its way around this ("`cylinder_count` is cylinders
the pickup *sees*; a clamp on one plug wire of a twin sees ONE, so leave
it at 1"), which is the actual defect: a settings field named **Cylinders**
that must be set to something other than the engine's cylinder count is a
trap, not a configuration. Every user who reads it literally gets wrong
RPM, and the docs' answer is that they held it wrong.

## The rule

The count is **dumb** — it is the engine's cylinder count, and it does not
enter the RPM math:

```
revs_per_pulse = (spark_mode == wasted ? 1.0 : 2.0)
```

One clamped wire carries one cylinder's ignition. A 2-stroke or
wasted-spark plug fires every crank revolution; a 4-stroke single-fire
(distributor/magneto) plug fires every other one. Cylinder count changes
neither. 500 pulses/min is 500 RPM on a single and on a twin alike.

The **debounce** loses the term for the same reason — one wire never
delivers pulses faster than the cylinder it is wrapped around fires — so
it returns to a flat 3 ms (6 ms single-fire), and the ~20 000 RPM ceiling
is now the same in both spark modes instead of falling with each
configured cylinder.

## What cylinder_count is for now

It stays, as the engine descriptor it was always read as, and it drives
the **warning**: above one cylinder, crank speed is *inferred* from one
cylinder's firing rate. Between firings the RPM is an assumption, and a
cylinder that stops firing reads as an engine that stopped.

That is not a defect to fix — it is how every clamp-on inductive tach
works, and it is known and accepted for this class of pickup. It is
stated in the settings UI (DovesDataViewer), in the README settings
table, and on the boot debug line, so nobody has to discover it from a
trace.

`clampCylinderCount()` and `rpmIsInferred()` are the only consumers.
Nothing in the RPM path branches on the count, which is enforced by
`revsPerPulse()`'s signature no longer accepting one.

## Compatibility

- **Single-cylinder devices (the default, and the fleet):** byte-identical.
  `revsPerPulse(wasted) == revsPerPulse(1, wasted)` for every value the
  old function could return at one cylinder.
- **Anyone who set `cylinder_count` > 1:** their RPM changes — it stops
  being divided. That is the fix, and it is a **breaking behaviour
  change** for those devices' logs.
- **`SETTINGS.json`:** unchanged. Same keys, same defaults, so no
  migration, no file-size movement, and no re-provisioning.
