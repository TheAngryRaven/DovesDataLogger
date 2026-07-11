# Architecture

This document explains how BirdsEye is put together and the reasoning
behind the parts that aren't obvious from reading the code. For the
exhaustive file-by-file map and the full constant tables, see
[`CLAUDE.md`](CLAUDE.md); this doc is the narrative version for humans.

## Overview

BirdsEye is a GPS lap timer and data logger built on the **Seeed XIAO
nRF52840 Sense** — an ARM Cortex-M4F with 256 KB RAM, 1 MB flash, BLE 5.0,
and an onboard LSM6DS3 IMU. It runs the Adafruit-based Bluefruit nRF52
core (not mbed), which means a SoftDevice handles the BLE stack and a UF2
bootloader handles flashing.

The device drives at 25 Hz off the GPS, logs to an SD card, shows live
timing on a 128×64 OLED, reads RPM from an inductive tachometer, and
serves logged files over BLE.

## Source layout

```
BirdsEye/
  BirdsEye.ino          entry point: globals, setup(), loop(), state machine
  <subsystem>.ino + .h  one module per subsystem (see below)
  <unit>.cpp + .h       pure, Arduino-free logic (host-testable)
tests/                  doctest + CMake harness for the pure units
.github/workflows/      CI: compile, lint, unit tests, clang-tidy, release
```

Arduino concatenates the `.ino` files into one translation unit, so
cross-module globals declared in `BirdsEye.ino` are visible everywhere.
Each module's `.h` documents its public surface; the `.ino` includes its
own header first so declaration/definition drift is caught at compile
time.

The **pure units** (`haversine`, `gps_time`, `gps_validation`,
`dovex_header`, `filename_validator`) deliberately avoid Arduino headers.
The *same* `.cpp` is compiled into both the firmware (Arduino picks up
`.cpp` files in the sketch folder) and the host test binary (CMake). There
is no copy-paste — the tests exercise the exact code that ships.

## Main loop

`loop()` runs at roughly 250 Hz and is a flat dispatch with a few
early-return short-circuits:

```
loop()
 ├─ wdtPet()                     feed the 4 s hardware watchdog
 ├─ if sleeping  -> handle wake triggers / charge screen / WFE; return
 ├─ if BLE active -> BLUETOOTH_LOOP(); minimal UI; return
 ├─ GPS_LOOP()                   drain buffer, fire PVT callback, feed timer, log
 ├─ TACH_LOOP()                  drain pulse ring buffer, Kalman-filter RPM
 ├─ ACCEL_LOOP()                 read g-force (rate-limited to 50 Hz)
 ├─ BLUETOOTH_LOOP()             service deferred BLE work
 ├─ trackDetectionLoop()         haversine match -> create CourseManager
 ├─ checkForNewLapData()         append completed laps to history
 ├─ checkAutoIdle()              60 s < 2 mph -> end session
 ├─ autoRaceModeCheck()          RPM/speed on menu -> enter race
 ├─ CAMERA_LOOP()                step the Insta360 auto-record FSM
 ├─ button hold combos           sleep / reboot
 ├─ readButtons() / displayLoop() / resetButtons()
```

Each subsystem exposes `*_SETUP()` (called once from `setup()`) and
`*_LOOP()` (called each iteration). ISRs stay trivially short and hand off
to the matching `*_LOOP()`.

## Subsystems

- **GPS** (`gps_functions`) — SparkFun u-blox GNSS v3, UBX-PVT binary at
  25 Hz. A registered callback fills a cached `gpsData` struct. Validated
  rows stream to the DOVEX log.
- **Tachometer** (`tachometer`) — falling-edge ISR timestamps pulses into a
  ring buffer; the loop computes mean inter-pulse period and runs it
  through a 1-D Kalman filter.
- **Accelerometer** (`accelerometer`) — onboard LSM6DS3, ±16 g, raw
  g-force. Degrades gracefully if absent (non-Sense board).
- **SD + tracks** (`sd_functions`) — SdFat (FAT16/32), track JSON parsing
  (two formats, auto-detected), and an in-RAM track manifest for proximity
  detection.
- **Display/UI** (`display_ui`, `display_pages`) — OLED driver abstraction,
  multi-sample debounced buttons, page routing.
- **Bluetooth** (`bluetooth`) — BLE service for file transfer, settings,
  and track sync, plus buttonless Secure DFU (`BLEDfu`) for OTA firmware
  updates and a Device Information Service (`BLEDis`) that reports
  `FIRMWARE_VERSION` for the update check.
- **Camera** (`camera_ble` + the `camera_fsm` / `insta360_protocol` pure
  units) — hands-free Insta360 X4 auto-record: the device emulates the
  Insta360 GPS Remote as a pure BLE peripheral, wakes the paired camera on
  engine start, and starts/stops/powers it off automatically via ce82
  remote-button notifications.
- **Replay** (`replay`) — instant DOVEX header replay.
- **Settings** (`settings`) — JSON key/value store on the SD card.
- **CourseManager** (external library) — owns course detection, sector
  timing, and the Lap Anything fallback once a track is matched.

## Design decisions worth knowing

These are the parts that look over-engineered until you've watched them
fail the simple way.

### GPS serial ring buffer (TIMER3 ISR)
At 25 Hz PVT the GPS emits ~2.5 KB/s. The hardware UART FIFO is tiny, and
an SD-card garbage-collection pause can block the main loop for
100 ms – 2 s — long enough to overflow the FIFO and lose fixes. A TIMER3
ISR drains Serial1 into a 4 KB RAM ring buffer every 10 ms, independent of
the main loop, and the GPS library reads from that buffer. This is why
**TIMER3 is reserved** project-wide.

### SD access arbitration
The BLE callbacks run in a **separate FreeRTOS task** from `loop()`, and
SdFat is not thread-safe. Every SD user (logging, replay, BLE transfer,
track parsing) must take a single mutex via `acquireSDAccess(mode)` /
`releaseSDAccess(mode)`. Two layers make this sound:

1. **Atomic transitions.** `acquireSDAccess()` evaluates the grant rules
   and commits the new owner inside a FreeRTOS critical section
   (`taskENTER_CRITICAL`, BASEPRI-masked so the SoftDevice's radio
   interrupts are untouched) — a plain check-then-set on the shared flag
   would be a TOCTOU between the two tasks. The grant/deny decision table
   itself (same-mode re-acquire is idempotent; the brief `TRACK_PARSE`
   mode is preemptible as leak recovery) is the host-tested
   `sd_access_policy` pure unit.
2. **Single-task SdFat.** *Every* BLE command that touches the card —
   `LIST`/`GET`/`DELETE`, `TLIST`/`TGET`/`TPUT`/`TDEL`, settings, firmware
   OTA — is parsed in the callback (filenames validated there, RAM only)
   and executed by `BLUETOOTH_LOOP()` on the main loop. Nothing calls
   SdFat from the Bluefruit callback task, so the filesystem only ever
   has one task in it; directory listings hold the lock for the whole
   walk, and `DELETE` refuses while a transfer is streaming.

### DOVEX crash safety
A `.dovex` file reserves the first 1 KB for session metadata (driver,
course, lap times) but the device **writes that header last**, when the
session ends cleanly. On creation the region is pre-filled with newline
padding and the GPS rows stream in after byte 1024. If the device loses
power mid-session the header is blank but every logged GPS row is still
intact and recoverable. Header layout/parsing lives in the tested
`dovex_header` unit.

### EMI hardening
The device lives next to an ignition system. Defenses: multi-sample
button reads with a refire lockout, a Kalman-filtered tach that absorbs
ISR jitter, a reduced 2 MHz SD SPI clock, an I2C bus-recovery routine that
bit-bangs the display bus free if it hangs, and a 4 s hardware watchdog as
the last resort.

### GPS sleep/wake recovery
The SAM-M10Q keeps its config in volatile RAM backed by V_BCKP. If that
rail sags (cranking brownout, loose connector) the module reverts to
9600 baud NMEA. `GPS_WAKE()` re-applies config and arms a 5 s PVT
watchdog; if no fix data arrives, `GPS_BAUD_RECOVERY()` renegotiates the
baud rate and reconfigures.

### OTA firmware updates
The board has no internet radio — only BLE — so it cannot pull a release
itself. OTA is a two-hop flow: the companion (DovesDataViewer, over Web
Bluetooth) downloads the firmware `.zip` from a GitHub release, writes the
buttonless-DFU command to reboot the board into the Adafruit/Nordic Secure
DFU bootloader, then force-feeds the image over GATT. The bootloader is a
passive receiver: it validates the package's signed init packet (device
type, SoftDevice requirement, CRC) before writing, so a corrupt or wrong
image is rejected rather than bricking the board. The companion picks
*which* image (version, Sense vs non-Sense) by reading the installed
`FIRMWARE_VERSION` and DIS model (`BirdsEye-sense` / `BirdsEye-nonsense`)
over the Device Information Service and comparing against a `manifest.json`
the release workflow publishes to the `gh-pages` branch (GitHub Pages
serves it with permissive CORS, so the browser can fetch both the manifest
and the `.zip` — raw release-asset URLs can't be relied on for that). Sense
and non-Sense are the same MCU + SoftDevice, so a mismatched image still
boots — it just skips IMU init.

### Insta360 camera auto-record
An Insta360 X4 has no wired trigger, but it *does* trust its own BLE
accessory: the "GPS Remote". So the device **is** the remote — a pure BLE
**peripheral**. It hosts the remote's GATT (service `0xCE80`: `ce81`
write camera→us, `ce82` notify us→camera, `ce83` read), advertises the
remote's manufacturer-data payload (carrying the camera's serial) to wake
a powered-off camera, and the camera connects back to *us* as central and
subscribes to `ce82`. **All** control is a `ce82` button notification,
byte-for-byte the physical remote's frames: recording toggles with the
shutter button, and power-off streams the remote's 3-second power-hold.
We never act as central — no scanning, no connecting to the camera's own
`0xBE80` service. (That central path was an earlier design; it let us
send explicit start/stop-video and a 1 Hz GPS-overlay frame, but power-off
only exists as a remote `ce82` hold, and one BLE link can hold only one
role — you cannot be central to the camera and have the camera be central
to you at once. The role conflict is what made power-off impossible, so
the device commits fully to the remote role. The in-camera GPS overlay
went with it: its true remote→camera transport is unidentified. GPS still
logs to SD exactly as before.)

The single peripheral slot is shared with the file-transfer service via
an explicit `bleOwner` (NONE/TRANSFER/CAMERA); the owner model keeps a
camera link from ever triggering the transfer path's
auto-reboot-on-disconnect, and opening the transfer page force-releases
the camera first. The link is Just-Works bonded (the genuine remote link
is encrypted), and BLE comes up lazily on the first camera action, so an
unpaired device pays nothing.

The entire lifecycle — wake on engine start, wait for the camera to
connect and subscribe, record on GPS lock (or a 30 s timeout), stop, cool
down, power off — is a **pure FSM** (`camera_fsm`): it consumes a
telemetry snapshot each loop tick and returns at most one action for the
glue to execute. Because the shutter is a *toggle*, the FSM tracks a
`recordingActive` belief so a mid-session BLE drop and reconnect never
blind-toggles the wrong way. All the temporal behavior (debounce,
retries, timeouts) lives inside it, so every path is host-tested with a
fake clock rather than discovered at the track; it is also the
board-portable core intended to move unchanged to the nRF54 ("Falcon")
target. The stop condition is deliberately **AND, not OR**: recording
ends only after 60 s of stationary *and* engine-off, because either
signal alone lies — a kart idles on the grid at 0 mph, and a coasting
stall has speed but no RPM. Both together mean the session is really over
(a manual session end stops the camera immediately).

Two guarantees bound the feature's blast radius. First, the FSM is a
*read-only* consumer of telemetry — logs are byte-identical whether a
camera is paired or not, and camera mode never parks the main loop the
way BLE transfer and USB MSC do. Second, the protocol bytes live in the
host-tested `insta360_protocol` unit with golden-byte tests. The wake
advert + scan response are **X4-confirmed** — captured from a genuine
remote and replayed to wake a sleeping X4; the ce82 button frames are
captured from a live remote too. The camera's own `ce81` state pushes and
the eventual GPS transport remain to be sniffed.

## Data formats

- **`.dovex`** — 1 KB reserved header (metadata + lap times) then streaming
  CSV GPS rows after byte 1024. Default and only logging format.
- **Track JSON** (`/TRACKS/*.json`) — new object format with `courses[]`
  and `lengthFt`, or an older bare-array format (parsed, but falls back to
  Lap Anything since it has no length to rank courses by).
- **Settings JSON** (`/SETTINGS.json`) — string key/value store.

See [`CLAUDE.md`](CLAUDE.md) and the README for field-level detail.

## Testing & CI

The pure units are unit-tested with doctest on a host toolchain; the
firmware itself is compile-checked for the XIAO in CI, linted with
arduino-lint, statically analyzed with clang-tidy, and gated on flash
size. Releases are built and published (`.hex` + `.uf2`) by the `release`
workflow on a version tag. Hardware behavior still needs manual
verification on a real device — CI proves it builds and that the logic
units are correct, not that a lap was timed right on track.
