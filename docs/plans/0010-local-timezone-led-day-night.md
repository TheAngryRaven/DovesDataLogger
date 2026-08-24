# Local Timezone (fixed offset) + LED Day/Night Brightness

> Status: **IMPLEMENTED**. Scope was deliberately cut twice during design —
> once to drop DST, once to keep local time out of the logging pipeline
> entirely. Both cuts are load-bearing; read *Non-goals* before widening this.

## The ask

> "I want to toggle the lights between daylight and night time, but sunrise/
> sunset isn't warranted — 7am needs to be local 7am."

The NeoPixel strip (plan 0006) runs at one brightness cap around the clock.
At a night event that cap is either blinding or, if turned down for the dark,
useless in daylight. The fix is a second cap and a clock to switch on.

## Why the device had no clock to switch on

The GPS delivers UTC and nothing in the firmware had ever needed anything
else. `gpsData.hour` is raw UTC out of the PVT callback
(`gps_functions.ino`), and every consumer — filenames, the DOVEX header
`datetime`, generated course names — wants UTC anyway. So there was no
timezone concept to fix; there was one to add.

The failure mode without it is not subtle. A US Central driver at 07:30
local is at 12:30 UTC: a naive UTC gate calls that the middle of the night
and dims the strip in broad daylight. Symmetrically, a 21:00 local session
is 02:00 UTC *the next day*, so the gate gets the right answer for the wrong
reason and the date is off by one.

## What was built

A pure unit, `local_time.{h,cpp}`, and a single consumer.

**`local_time`** — a fixed signed offset in minutes, applied to a 4-digit-year
`DateTime`, with correct date rollover in both directions across month, year
and leap-day boundaries. Plus `isNight()`, which tests membership in the
window `[nightStart, dayStart)` *modulo the day*, so the ordinary wrapped case
(19:00 → 07:00) needs no special casing at the call site. Host-tested
(`tests/local_time_test.cpp`), Arduino-free, builds into the sim.

Three details worth keeping:

- **Minutes, not hours.** India is +5:30, Newfoundland −3:30, the Chatham
  Islands +12:45. An hours-only offset is wrong for real places.
- **Four-digit year.** `GpsData.year` is 2-digit for compatibility with the
  existing filename and header format strings; the unit takes 4 digits and
  callers add 2000. The leap rule (`gps_time::isLeapYear`, reused rather than
  re-derived) genuinely needs the century, and a future feature that wants
  real civil time will need it too.
- **An out-of-band offset is ignored, not clamped.** A corrupt setting must
  not be able to walk the calendar; it falls back to 0, which is exactly the
  pre-0010 behaviour.

**The consumer** is `npxEffectiveBrightness()` in `neopixel.ino`, feeding
`npxPushFrame()`. That function was already the single global-brightness
choke point (`led_frame::applyCap`, plan 0006), so the whole feature is a
change of argument at one call site — the invariant that no channel ever
exceeds the cap is untouched.

## Non-goals — the two cuts

**No DST, on purpose.** A fixed offset means the boundary walks an hour twice
a year. For a "stop blinding the driver after dark" gate that is beneath the
feature's resolution. The alternatives were US/EU rule tables (~40 lines and
a permanent correctness liability as legislatures keep changing the dates) or
tzdata (~100 KB and needs shipping updates to a sealed device). Neither buys
anything a driver would notice. `local_time` is where the rules would go if
that ever changes.

**Local time never touches saved data.** DOVEX row timestamps are Unix epoch
milliseconds — UTC by definition — and the header `datetime`, the log
filenames and the generated course names all stay UTC. This is not laziness:
a log is routinely *viewed* somewhere other than where it was recorded, so
the conversion belongs on the presentation side, where the viewer's own
preference is known. The webapp already does most of this (it does not yet
offer local-timezone picking, which is where that work continues).

The practical consequence: nothing in the logging pipeline may call into
`local_time`. The header comment says so.

## Traps

1. **`led_brightness == 0` means "cut the 5 V rail", and only that setting
   does.** `NEOPIXEL_SETUP()`/`NEOPIXEL_WAKE()` hold the boost EN pin low when
   it is 0. A *night* brightness of 0 must therefore blank the frame and leave
   the rail up — power-cycling a boost converter at 19:00 mid-session is not a
   brightness change. `npxEffectiveBrightness()` keeps the two meanings apart.
2. **No clock means day, not night.** `gpsData.timeValid` requires the module's
   `fullyResolved`, which is up to ~12.5 minutes from a cold start. Rendering
   at night brightness in that window would mean a strip that comes up dark
   and reads as broken hardware. It renders at day brightness until the lock
   lands, then swaps.
3. **Equal hours disable the swap.** `isNight()` returns false for an empty
   window, so `led_day_start_hour == led_night_start_hour` means one cap
   around the clock — no separate enable flag to keep in sync.

## The settings file was one key away from bricking

Adding four keys turned up a latent cliff. `settings.ino` had a 512-byte
file buffer and a `StaticJsonDocument<512>`, and **every read path caps at
`sizeof(settingsFileBuffer) - 1`** — 511 bytes. The existing 18-key file
already serialized to **436 bytes**. The four keys here put it at **543**.

Past the cap the failure is silent and self-inflicting:

1. `settingsFile.read()` truncates at 511 bytes, mid-token.
2. `deserializeJson` returns `IncompleteInput`, so *every* `getSetting()`
   fails — not just the new keys.
3. `SETTINGS_SETUP()` reads that as corruption and quarantines the file to
   `SETTINGS.json.bad`, regenerating fresh defaults (new random BLE name,
   new PIN, new device name — the user's pairing is gone).
4. `ensureDefaultSettings()` then adds the missing keys back, one
   read-modify-write each, until the file crosses 511 again.
5. Loop, every boot, forever.

The document was a second, independent wall: a `<512>` doc returns
`NoMemory` at 22 string pairs (measured against the pinned ArduinoJson
6.21.5, not estimated).

Fixed by raising both to **1024** (keep them equal — the invariant is that
the buffer can always hold what the document serializes) and, because the
next contributor will not have measured, adding a guard in
`setSettingInner()` that refuses a write when the document `overflowed()`
or `measureJson()` exceeds the buffer. That converts a silent permanent
brick into one loudly refused write with the old file left intact.

Worth stating plainly: **adding a settings key is not free.** Check the
serialized size.

## Settings

| Key | Default | Notes |
|---|---|---|
| `utc_offset_min` | `0` | Minutes east of UTC; clamp ±840. US Central standard = `-360` |
| `led_brightness_night` | `16` | Night cap 0-255; 0 blanks the strip, rail stays up |
| `led_day_start_hour` | `7` | Local hour the day cap takes over |
| `led_night_start_hour` | `19` | Local hour the night cap takes over |

Written on every channel (the SensorEgg DOVEX-column precedent: uniform
`SETTINGS.json` shape), read only by a `BIRDSEYE_ENABLE_NEOPIXEL` build.
Settable over BLE via the existing `SSET` — no protocol work, and no
on-device entry UI, consistent with the no-text-entry rule.

## Follow-ups, not done here

- The webapp learning to pick a viewing timezone for logs.
- Local time on the OLED, if a page ever wants to show a clock.
- A day/night *look* (different colors/modes), rather than just a different
  cap. The cap was chosen for v1 precisely because it is one argument.
