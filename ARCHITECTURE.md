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
`releaseSDAccess(mode)`. BLE commands that need the card defer their work
into `BLUETOOTH_LOOP()` (main-loop context) instead of touching SdFat from
the callback.

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
`FIRMWARE_VERSION` over the Device Information Service and comparing it
against the latest release. Sense and non-Sense are the same MCU +
SoftDevice, so a mismatched image still boots — it just skips IMU init.

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
