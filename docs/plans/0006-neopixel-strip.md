# 0006 — NeoPixel strip subsystem (2 status LEDs + 9-px strip)

## The hardware

An 11-pixel WS2812 (NeoPixel) strip was added to the device, powered by an
Adafruit 5 V boost converter with an ENABLE pin. The two nRF52840 NFC pads
are converted to plain GPIO to drive it:

| Pin | P-number | Role |
|---|---|---|
| Arduino 30 (`PIN_NFC1`) | P0.09 | Boost converter EN — HIGH = 5 V rail on, LOW = off |
| Arduino 31 (`PIN_NFC2`) | P0.10 | NeoPixel data (GRB, 800 kHz) |

Both pads are mapped in the Seeed XIAO variant (`g_ADigitalPinMap[30] = 9`,
`[31] = 10`), so `pinMode`/`digitalWrite`/Adafruit_NeoPixel work on them
directly once NFC protection is off. The assignment between the two pads is
arbitrary — `neopixel.h` holds both as single-point `#define`s so they can be
swapped to match the actual wiring.

Pixel layout: **pixel 0 and pixel 10 are status indicators**; **pixels 1–9
are the strip**, with pixel 5 the centerline.

## NFC → GPIO: runtime UICR write, not a core build flag

The nRF52840 ships with the NFC pads dedicated to NFCT; converting them to
GPIO means programming `UICR->NFCPINS` bit 0 to 0, which latches only at
reset. **This is one-way in practice** (undoing it needs a full chip erase =
bootloader reflash) — accepted for this project; NFC is never used.

Two ways to do it, and we deliberately chose the second:

1. `-DCONFIG_NFCT_PINS_AS_GPIOS`: consumed by the *core's*
   `system_nrf52840.c` — a `.c` file, which the repo's established
   `compiler.cpp.extra_flags` mechanism does not reach. It would need a
   second `compiler.c.extra_flags` build property in every workflow AND
   local IDE configuration nobody will remember.
2. **Runtime write in `NEOPIXEL_SETUP()`** (what we do): if
   `NRF_UICR->NFCPINS` bit 0 is still set, unlock NVMC, write
   `0xFFFFFFFE` (a 1→0-only word write — no erase needed), relock, and
   `NVIC_SystemReset()`. One reset per device lifetime; every later boot
   skips the branch entirely. Works identically for CI, IDE, and OTA
   builds.

Constraints that dictate the placement of `NEOPIXEL_SETUP()` in `setup()`:

- **Before the SoftDevice comes up.** Direct NVMC access is illegal once the
  SoftDevice is enabled; on the beta channel `SENSOREGG_SETUP()` calls
  `bleCoreEnsureInit()` at boot, so the neopixel setup must run earlier.
  It sits right after the settings block, before `CAMERA_SETUP()`.
- **Before `wdtSetup()`** (armed last in `setup()`), so the one-time reset
  can't race the watchdog.
- After `SETTINGS_SETUP()`, so `led_brightness` is readable (brightness 0 =
  LEDs disabled entirely — the boost is never even enabled).

The reset lands in `captureBootWakeCause()` as a plain soft reset — a normal
boot, no special handling.

**The flag-off build must never touch UICR or drive pins 30/31.** The entire
module body is inside `#if BIRDSEYE_ENABLE_NEOPIXEL`; the `#else` stubs do
nothing (sensoregg.ino pattern).

## The global brightness cap — the core requirement

One rule: **once set, no LED may ever exceed the cap.** Enforced in one
place — `led_frame::applyCap()`, called exactly once per frame in the push
path. Every mode and animation authors colors in full 0–255 and never
worries about brightness; the cap is a post-condition tested by sweeping
caps × saturated frames in the host tests. We deliberately do NOT use
`Adafruit_NeoPixel::setBrightness()` (it rewrites the pixel buffer lossily
and would spread the invariant across call sites).

Default `led_brightness` = 64 (~25 %). 11 px full-white at 255 is ~660 mA
on the 5 V rail; at 64 it's ~165 mA — inside the boost's budget and sane on
battery, still clearly visible in daylight peripheral vision.

## Architecture

Pure logic (host-tested, Arduino-free) does everything except talk to pins:

| Unit | Owns |
|---|---|
| `led_frame.{h,cpp}` | Pixel layout constants, `Rgb`/`Frame` PODs, integer color scaling, **`applyCap()`** |
| `led_modes.{h,cpp}` | Pace pip math, generic `ScaleSpec` fill (RPM now, temps later), the `StatusAction` table + `evalStatus()` hysteresis/flash |
| `led_animations.{h,cpp}` | Boot + purple-sector animations as pure functions of `(tMs, seed)` |
| `sector_purple.{h,cpp}` | Session-best ("purple") sector detection, poll-and-diff |

The glue (`neopixel.{h,ino}`) snapshots inputs each frame, composes by
priority, applies the cap, and pushes to the strip:

```
NEOPIXEL_LOOP()  (30 Hz self-throttle)
 ├─ inputs: tachLastReported, activeTimerPaceDifference() + validity,
 │          sensoreggEgtC() (isNanF-guarded), raceActive, parked flags
 ├─ sector_purple::update() → nonzero: restart purple animation
 ├─ compose (first match wins):
 │    1. boot animation running
 │    2. purple animation running
 │    3. parked (BLE/USB) ‖ !raceActive ‖ brightness==0 → all off
 │    4. strip: pace valid ? pace pip : RPM scale
 │       status px0/px10: evalStatus(action table)
 ├─ led_frame::applyCap(frame, settingLedBrightness)
 └─ strip.setPixelColor()×11, strip.show()
```

### Strip mode policy

Off outside a race session — the strip is a driving aid, menu glow burns
battery. In race: **RPM scale** until pace is valid, then the **pace pip**.
Pace validity mirrors the OLED pace page:
`activeTimerRaceStarted() && activeTimerLaps() >= 1` and not
sprint-between-runs. Sprint between runs falls back to the RPM scale.

### Pace pip

`activeTimerPaceDifference()` is **ms per meter**, positive = slower than
best. Full deflection at ±1.0 ms/m (`kPaceFullScaleMsPerM`) — on a ~1.2 km
kart lap that's ~1.2 s/lap, a pin-the-needle delta; 0.25 ms/m per pixel
step. Deadband ±0.125 (half a step) shows the dim-white center pixel =
"on pace". Slower = pip LEFT of center in red, faster = RIGHT in green.

### Scale mode (RPM, later temps)

Generic `ScaleSpec {min, max, redFrac, lowColor, highColor}` — fill from
the left, lit pixels at/past `redFrac` of the fill render the high color.
RPM uses `{0, rev_limit, 0.5, green, red}` — red past halfway, per spec.
The same renderer will serve EGT/temp scales when phase 2 makes the mode
selectable.

### Status LED actions — the phase-2 assignability hook

Each status LED is driven by a `StatusAction` POD:
`{source, threshold, clearBelow, color, flashHalfPeriodMs}` evaluated
against per-frame values with a hysteresis latch. Hardcoded defaults now:

- **px 0**: rev-limit flasher — red, fires ≥ `rev_limit`, clears < 97 %,
  100 ms half-period.
- **px 10**: EGT flasher — orange, fires ≥ 650 °C (`kEgtAlertC`), clears
  < 630 °C, 250 ms half-period. NaN/stale EGT (isNanF!) → LED off AND
  latch released — never latch stale data (house rule).

Phase 2 ("user controls these settings") parses settings into the same
POD table — the evaluation code doesn't change, only where the table comes
from. `Source` is an enum (`kNone/kRpm/kEgtC`) that grows entries as new
sources appear.

### Animations — deterministic by construction

No `rand()`, no `millis()` inside the units: a frame is a pure function of
`(tMs since start, seed)`. Sparkle positions come from an xorshift-style
hash of `(seed, timeSlot, pixel)`, so host tests golden-lock exact frames.
The glue supplies `millis()` deltas and a `micros()`-derived seed (house
entropy rule — never `analogRead`).

- **Boot** (2600 ms): hue comet circling all 11 px as a ring (2
  revolutions with trailing fade, 0–1500 ms), global fade-out
  (1500–2200 ms), white sparkle glints overlapping (900–2600 ms).
  Non-blocking — plays over the GPS status page.
- **Purple sector** (1600 ms): purple wave expanding from the center px
  outward across strip + status (0–400 ms), solid purple with sparkles
  (400–1200 ms), fade out (1200–1600 ms). Overrides strip AND status;
  normal mode resumes automatically; a new purple mid-animation restarts
  it.

### Purple detection — and the sector-3 / lap-line race

The library (DovesLapTimer / SprintTimer) exposes `getCurrentSector()`,
`getCurrentLapSector{1,2,3}Time()`, `getBestSector{1,2,3}Time()` — but
`updateBestSectors()` runs **at the start/finish crossing**, not at each
sector line. So at the exact moment sector 3 closes, the library may have
already folded this lap into the "best" values and rolled the current-lap
times to the new lap. `sector_purple` defeats this by never comparing
against live state at a close edge:

- **Best times are snapshotted when each sector OPENS** (`bestAtOpen[3]`).
- S1/S2 close on `getCurrentSector()` transitions (1→2, 2→3): compare the
  just-closed `getCurrentLapSectorNTime()` against `bestAtOpen[n]`, then
  snapshot the next sector's best.
- **S3 closes on the lap edge** (`laps` increment), and its time is
  *derived*: `s3 = lastLapTime − s1 − s2` (guarded on both being present
  and the difference positive) — both inputs immune to the library's
  lap-line side effects. Compare vs `bestAtOpen[2]`, re-arm for the new
  lap.
- Purple fires only when the beaten best was **nonzero** — no purple
  anywhere on lap 1 (flashing every first-lap sector is noise).
- `sectorsConfigured == false` (Lap Anything / WaypointLapTimer has no
  sectors) or race not started → full reset, never fires.

Three new sprint-first wrappers join the `activeTimer*()` family:
`activeTimerCurrentSector()`, `activeTimerLapSectorTime(n)`,
`activeTimerBestSectorTime(n)` (WLT/null → 0).

## Sleep / wake

GPIO state is **retained in System OFF** (the "blue LED stays on after
sleep" field report). `NEOPIXEL_SLEEP()` — slotted in `enterShutdown()`
with the IMU rail-off, before the charging branch so the strip is dark
while charging too — therefore unconditionally:

1. blanks the strip and `show()`s while 5 V is still up,
2. drives the data pin LOW (never leave data high into an unpowered strip
   — it back-powers the pixels through the data diode),
3. drives boost EN LOW — the 5 V rail is truly off, retained through
   System OFF.

`NEOPIXEL_WAKE()` (charging-loop soft resume, beside `SENSOREGG_WAKE()`):
EN HIGH, ~5 ms boost settle, blank frame; composition resumes next loop.

## Parked loop branches

The BLE-transfer and USB-MSC branches early-return before the main frame.
`NEOPIXEL_LOOP()` is called inside both so composition sees "parked" and
blanks the strip — a frozen mid-pattern strip would look crashed.

## Settings

| Key | Default | Meaning |
|---|---|---|
| `led_brightness` | `"64"` | Global cap 0–255; 0 = LEDs disabled (boost never enabled) |
| `rev_limit` | `"15000"` | True RPM for the scale ceiling + rev flasher; clamp 1000–20000 |

Both rows added unconditionally to `ensureDefaultSettings()` (uniform
SETTINGS.json across channels — the SensorEgg DOVEX-column precedent).
Phase 2 adds mode/action assignment keys.

## Feature flag / channels

`BIRDSEYE_ENABLE_NEOPIXEL` (`project.h`, `#ifndef` default 0, tested with
`#if`): **on** in beta (`beta.yml` + `compile-sketch.yml` for
BETA-targeted PRs), **off** in master/release. The `Adafruit NeoPixel`
library is installed in all three workflows either way (harmless when
unused; keeps the lists uniform).

## Radio / timing safety

Adafruit_NeoPixel's nRF52 backend claims a *free* PWM instance
(`ENABLE == 0`) per `show()`, drives it via EasyDMA and busy-waits SEQEND
with **interrupts on** (~0.4 ms for 11 px ≈ 1 % CPU at 30 Hz). This sketch
uses no `tone()`/`analogWrite()`/HwPWM, so PWM0–2 are always free; TIMER3's
GPS drain and the tach ISR are unaffected. Two things future maintainers
must not "fix":

- If ever all PWMs are occupied, the library falls back to a
  cycle-counted bit-bang **with interrupts disabled** for ~0.4 ms. Still
  inside the 256 B serial ring's ~44 ms slack, but don't create that path.
- `show()` mallocs/frees a ~560 B PWM pattern buffer every call. Same-size
  alloc/free is fragmentation-benign on this allocator — it is not a leak.

## Sim

`neopixel.ino` is excluded from the sim TU (BLE-module precedent) and its
surface no-op'd in `module_stubs.cpp`. The four pure units join
`SIM_CORE_SOURCES`. The three sector wrappers get sim_prototypes entries.

## Hardware-only verification (open items)

- One-time UICR write + single self-reset on first flag-on boot.
- Boost EN polarity (assumed HIGH = on, Adafruit precedent) + settle time.
- GRB color order and strip orientation (`kStripReversed` if mounted
  data-end-right).
- Real current draw at cap 64; sleep current with EN low.
- 3.3 V data into 5 V pixels — user bench-proven on another nRF board.
- Cold power-on EN float: before `setup()` the EN pin floats; if the boost
  board pulls EN up, the strip powers briefly with idle data — harmless
  (pixels stay dark), noted for completeness.

## Phase 2 (deliberately out of scope here)

Settings-driven assignment: strip mode selection, per-status-LED action
source/threshold/color, temp scales via `ScaleSpec`, C/F preference,
brightness UI. The POD tables above are the interface that phase fills in.
