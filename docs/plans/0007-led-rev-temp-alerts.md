# 0007 — LED rev/overrev + temp alerts, GPS-search pip, engine-stopped gate

First track-shakedown follow-up to the NeoPixel subsystem (plan 0006). Four
changes, all driven by real use on a kart with a 7600 RPM hard limiter.

## Two rev thresholds, two meanings

- **`rev_limit`** (existing setting, semantics sharpened): the WARNING
  limit — where the driver wants to know the engine is at the ceiling
  (7550 on the reference engine, just under its 7600 limiter — a carb
  tuning indicator). Drives the left status LED's red flasher and the
  RPM-scale ceiling, exactly as before. Also now drives the OLED tach
  page's `*OVER REV*` header, which had been hardcoded at a meaningless
  `> 9999`.
- **`overrev_limit`** (new, default `0` = disabled): the PROBLEM limit —
  if the engine ever spins here (8500 reference), something is
  mechanically wrong. While latched, the **whole 11-px chain flashes
  red**, overriding everything except the boot animation (an engine
  problem outranks the purple celebration). The latch releases below
  `rev_limit × 0.97` — the same clear fraction the warning flasher uses,
  so a brief spike leaves an unambiguous flash rather than a flicker.
  Implemented with the existing `StatusAction`/`evalStatus` machinery
  (its `StatusState.active` latch drives a whole-frame fill in the
  glue) — no new evaluation code, and phase 2 can expose it like any
  other action.

## Temp alert on the right status LED — tri-state

New setting **`temp1_alert_c`** (default `650` °C ≈ 1200 °F, the value
that was previously a constexpr). The right LED becomes:

- **flashing red** at/above the threshold (was orange; red per driver
  preference), clearing 20 °C below it (the existing 650/630 hysteresis
  generalized);
- **off** when the reading is good and below the limit;
- **solid blue** when there is *no probe signal* — Temp1 is NaN/stale
  (egg dropout, hung egg, no egg). `StatusAction` grew an
  `invalidColor` field for this; the rev flasher sets it to off, so
  nothing else changes. A logger with no egg paired shows blue all
  race — accepted for now; phase 2's assignability makes it
  configurable.

## GPS-search pip

Race mode with no GPS lock used to show the RPM scale — nothing told
the driver why laps weren't timing. Now the strip shows a single green
pixel bouncing end-to-end (triangle wave, 1.6 s round trip,
`led_modes::renderSearchPip` — a pure function of elapsed time, host
tested) until `gpsData.fix && gpsData.timeValid` — the same condition
that allows log-file creation and releases the WAITING-GPS-LOCK page
pin. Then RPM scale, then pace once the first lap is in (unchanged).

## Engine-stopped gate

If a session's engine has proven itself (`raceEntryCause ==
RACE_ENTRY_TACH` — which manual/speed sessions promote to at >500 RPM,
so no-tach devices are untouched) and the tach reads 0, the session is
in a stall/DNF posture, not a lap:

- the 9-px bar goes dark (a pace pip counting away next to a dead
  engine reads as a glitch);
- the **status LEDs stay live** — the temp alert while a hot engine
  cools is exactly when it matters;
- the OLED pace page shows **STOPPED** instead of a still-counting pace
  number (and suppresses the notably-faster flash animation).

A restart clears it automatically — the tach reports nonzero within one
loop of the first pulse burst.

Shared condition: `raceEngineStopped()` in the sketch, used by both the
LED glue and the pace page.

## Strip composition after this plan

```
boot animation
 > overrev whole-chain red flash          (race only, latched)
  > purple sector celebration
   > parked / not racing / brightness 0 -> off
    > race strip:
        engine stopped        -> bar off (status LEDs live)
        no fix+timeValid      -> green search pip bouncing
        pace valid            -> pace pip
        else                  -> RPM scale
      status px0: rev-limit flasher (red, >= rev_limit, clear 97%)
      status px10: temp tri-state (blue = no signal / off / red flash)
```

## Settings summary

| Key | Default | Notes |
|---|---|---|
| `rev_limit` | `15000` | unchanged; now also the tach-page OVER REV header threshold |
| `overrev_limit` | `0` | 0 = disabled; clamp 1000–20000; whole-chain red flash |
| `temp1_alert_c` | `650` | °C; clamp 50–1200; clear = alert − 20 °C |
