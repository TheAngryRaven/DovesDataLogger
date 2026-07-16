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
 ├─ button hold combos           shutdown / reboot; menu idle -> shutdown
 ├─ readButtons() / gpsStatusPageLoop() / displayLoop() / resetButtons()
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
  detection. Built for a soldered-in module: a missing `/TRACKS` folder is
  created automatically, and a card that responds without a mountable FAT
  volume boots into a hold-to-confirm on-device format page
  (`sd_format_page` pure unit) rather than a dead-end fault screen.
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

### Shutdown is System OFF, wake is a reboot
There is no power switch (deliberately — the next hardware revision drops
it), so "off" is nRF52 **System OFF** at ~µA with GPIO SENSE armed on the
tach pin and the three buttons, plus VBUS. Waking is a full chip reset:
`setup()` runs fresh, and the very first thing it does is read (then
clear) the sticky `RESETREAS` + GPIO `LATCH` registers to decode *why* it
booted (the host-tested `wake_cause` unit). An engine-start (tach) wake
routes the GPS status page's exit straight into race mode with logging —
the old software sleep loop's RPM wake, rebuilt on hardware. GPREGRET is
never touched; register 0 belongs to the OTA/bootloader handoff. The one
soft exception is charging: the fast-charge (HICHG) pin is software-held,
so while VBUS is present the device parks in a live charging loop instead
of System OFF, wakes fully on any button, and powers off when unplugged.

### GPS boot recovery
The SAM-M10Q keeps its config in volatile RAM (backed by V_BCKP), and
with shutdown being a real power-down the module can be in **any** state
at boot: software backup mode holding a 57600 config (the normal wake),
already configured and running (an MCU-only reset), or factory 9600 NMEA
(true cold power / brownout). Boot therefore sends the u-blox backup-wake
byte first (harmless if awake), probes 57600 before 9600 so warm boots
connect near-instantly, and pays a cold-boot delay only when nothing
answers. A `begin()` ping proves the module answers — not that data
flows — so boot also arms a 5 s PVT-arrival watchdog; if no fix data
arrives, `GPS_BAUD_RECOVERY()` renegotiates the baud rate and
reconfigures. A GPS that never appears is re-probed a bounded number of
times from the status page and surfaced as "CHECK WIRING".

### GPS status boot page
Every boot lands on a MyChron-style satellite status page: sat counts,
HDOP, lock state, and per-satellite CNO signal bars (UBX-NAV-SAT). The
GPS runs a 5 Hz status config while the page is up and switches to the
25 Hz PVT-only race config on exit. The page holds until a stable lock
(fix + fully-resolved time held 3 s) then auto-advances; any button skips
it immediately; a tach-wake boot or a running engine turns the exit into
race-mode entry. Its hold/auto-close/destination logic is the host-tested
`gps_status_page` unit, and the bar selection/geometry is `sat_bars`.

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
rides that same remote channel: a capture of the genuine link showed the
remote streams GPS on `ce82` at 10 Hz as a non-standard NMEA-RMC frame,
which the firmware now emits continuously while connected — doubling as
the remote's liveness heartbeat. GPS still logs to SD independently.)

The single peripheral slot is shared with the file-transfer service via
an explicit `bleOwner` (NONE/TRANSFER/CAMERA); the owner model keeps a
camera link from ever triggering the transfer path's
auto-reboot-on-disconnect, and opening the transfer page force-releases
the camera first. The link is Just-Works bonded (the genuine remote link
is encrypted), and BLE comes up lazily on the first camera action, so an
unpaired device pays nothing.

The lifecycle is a **pure FSM** (`camera_fsm`), deliberately RPM-driven and
simple: wake on engine start, connect+subscribe, record once RPM has held a
few seconds, stop after the engine has been off a while, then WATCH. It
consumes a telemetry snapshot each loop tick and returns at most one action
for the glue to execute. **Recording starts** from the WATCHING hub once RPM
has held above the ON threshold for ~5 s — there is **no GPS-lock gate**
(GPS still streams to the camera continuously). Because the shutter is a
*toggle*, a wrong record belief flips the camera the wrong way — so the FSM
does not merely believe: it **confirms** record state from the camera's own
`0x10` status frame (a live `.HH:MM:SS` timer while recording; the `0x02`
word is unreliable) and reconciles `recordingActive` against it. On a
mid-session BLE drop it preserves the belief and on reconnect adopts the
camera's real state instead of blind-toggling; if the camera reports idle
while we think we're recording, it re-asserts the shutter once. **Recording
stops** after ~30 s of engine-off (RPM only — a stationary but running grid
idle keeps recording), returning to WATCHING. WATCHING keeps the camera on
and connected so a brief on-track stall recovers straight back into
recording when RPM returns; the camera powers off **only** when the device
shuts down (there is no post-record cooldown/power-off timeout). All temporal
behavior lives inside the FSM, host-tested with a fake clock, and it is the
board-portable core intended to move unchanged to the nRF54 ("Falcon")
target.

One deliberate coupling bounds the feature: the camera's 30 s-engine-off
auto-stop also **ends the race log session** (the glue latches it and the
main sketch calls `endRaceSession()` + returns to the menu), and while the
camera is recording `checkAutoIdle()` yields so the speed-based log idle
can't cut the log out from under it. So with a camera paired+recording the
log ends on engine-off, not on 60 s-stationary; without a camera, logging is
unchanged (a tach-less setup reads RPM≈0, which is why the RPM-idle can't be
made universal). The protocol bytes live in the host-tested
`insta360_protocol` unit with golden-byte tests: the wake advert + scan
response, the ce82 button frames, the ce82 GPS/RMC frame, and the `0x10`
record-timer parse are all captured from a genuine remote (the wake advert
was even replayed to wake a sleeping X4).

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
