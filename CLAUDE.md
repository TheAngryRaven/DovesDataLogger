# BirdsEye - Project Guide

> **MAINTAINERS: Keep this file updated when adding/removing files, changing pin
> assignments, modifying subsystem interfaces, or altering the build configuration.
> This file is loaded into Claude's context window on every session and must
> accurately reflect the current state of the project.**

## Maintaining the Quality Bar

This project went through a deliberate hardening pass (tests, CI, static
analysis, security, release pipeline, docs). **Keep it there.** When making
any change — whether you're Claude or a human contributor — hold the line:

- **Add tests when possible.** New pure logic (math, parsing, validation,
  formatting, anything Arduino-free) belongs in a `BirdsEye/*.{h,cpp}` unit
  with a matching `tests/<unit>_test.cpp`. If you're touching existing logic
  that *could* be a pure unit but isn't yet, prefer extracting it so it can
  be tested rather than leaving it tangled in an `.ino`. Don't add untested
  pure logic when a test is feasible.
- **Keep the CHANGELOG updated.** Any user-visible change gets an entry under
  `[Unreleased]` in `CHANGELOG.md` (Added / Changed / Removed / Fixed /
  Security). Flag breaking changes explicitly — they drive the next version
  number per the semver policy in that file.
- **Keep CI green and meaningful.** The checks (compile-sketch + flash-size
  gate, arduino-lint, unit-tests, clang-tidy, coverage) must pass. Fix the
  root cause rather than loosening a check; if a clang-tidy finding is a
  genuine false positive, suppress that one line with `// NOLINT(check)` and
  a reason, never by disabling the check globally. The coverage floor
  (`COVERAGE_MIN` in `coverage.yml`) is intentionally low — raise it as
  coverage grows; don't lower it to pass.
- **Keep the docs in sync.** Update this file's File Map and the relevant
  subsystem section, plus `ARCHITECTURE.md`, when you add/remove a module or
  change a subsystem interface. Stale docs are worse than none.
- **Hold the conventions.** No Arduino `String` in hot paths, all SD access
  through the mutex, never `analogRead()`, `TIMER3` reserved, ISRs trivially
  short. See *Development Conventions* at the bottom for the full list.
- **One concern per PR.** Keep refactors, behavior changes, and new tests in
  separate PRs so each is reviewable and revertable on its own.

The goal: every change should leave the codebase at least as professional as
it found it. If a shortcut would lower the bar, flag it instead of taking it.

## What Is BirdsEye?

A high-precision GPS lap timer and data logger for motorsports / track days.
Built on the **Seeed XIAO nRF52840 Sense** (ARM Cortex-M4, 256 KB RAM, BLE 5.0, onboard LSM6DS3 IMU).

Core capabilities:
- 25 Hz GPS lap timing with sector support (DovesLapTimer library)
- **"Just Drive" auto-detection** via CourseManager: automatic track
  proximity matching, course detection, and Lap Anything fallback
- RPM monitoring via inductive tachometer pickup
- Accelerometer logging (g-force X/Y/Z) via onboard LSM6DS3 IMU
- DOVEX data logging with reserved 1 KB header (crash-safe GPS data)
- 8+ display pages on a 128x64 OLED (3 Hz refresh)
- Bluetooth LE file download to companion apps / HackTheTrack.net
- On-device session replay: instant DOVEX header replay

---

## File Map

All sketch sources live in `BirdsEye/` so the folder name matches the
`.ino` filename Arduino IDE expects. Each module has both a `.ino`
(implementation) and a `.h` (public interface, documentation).

### Sketch Sources (`BirdsEye/`)

| File | Purpose |
|---|---|
| `BirdsEye.ino` | Entry point: globals, `setup()`, `loop()`, state machine, course/timer helpers |
| `project.h` | Shared types (`ButtonState`, `TrackLayout`, `TrackManifestEntry`, `TrackMetadata`), debug macros, `MAX_*` constants |
| `display_config.h` | Display driver abstraction (SH110X vs SSD1306 toggle) |
| `gps_config.h` | GPS configuration constants (baud rate, nav rate, serial port) |
| `images.h` | PROGMEM bitmap data (splash screen, animations) |
| `accelerometer.{h,ino}` | LSM6DS3 IMU init and g-force reads (onboard XIAO Sense) |
| `bluetooth.{h,ino}` | BLE service (file listing, transfer, settings, track sync), auto-reboot on disconnect |
| `firmware_ota.{h,ino}` | SD-staged firmware OTA: `FW*` BLE protocol, SD staging, CRC verify, self-flash apply (see subsystem 11) |
| `display_pages.{h,ino}` | All page rendering functions (`displayPage_*()`) |
| `display_ui.{h,ino}` | Display init, button reading (multi-sample debounce), menu navigation, I2C bus recovery |
| `gps_functions.{h,ino}` | GPS init (SparkFun UBX PVT), time conversion, DOVEX logging pipeline, TIMER3 serial buffer ISR, V_BCKP recovery |
| `replay.{h,ino}` | Instant DOVEX header replay |
| `sd_functions.{h,ino}` | SD init, track list/JSON parsing (dual format), track manifest, SD access arbitration |
| `settings.{h,ino}` | Persistent JSON settings on SD (`/SETTINGS.json`), `getSetting()`/`setSetting()` |
| `tachometer.{h,ino}` | Falling-edge ISR on D0, Kalman-filtered RPM calculation |
| `diagram.json` | Wokwi simulator wiring |
| `libraries.txt` | Wokwi simulator library list |

### Pure-Logic Units (`BirdsEye/*.{h,cpp}`)

Arduino-free `.cpp` files — compiled into the firmware AND into the
host test harness (`tests/`). No Arduino headers, so they build on a
desktop toolchain. This is where logic worth unit-testing lives.

| File | Purpose |
|---|---|
| `haversine.{h,cpp}` | Great-circle distance in miles (track proximity) |
| `gps_time.{h,cpp}` | Leap-year/Unix-epoch math, `u64ToDecimalString` |
| `gps_validation.{h,cpp}` | PVT sample sanity gate + dtostrf-output check |
| `dovex_header.{h,cpp}` | DOVEX 1 KB header `format()` / `parse()` |
| `filename_validator.{h,cpp}` | FAT-safe / traversal-proof check for BLE filenames |
| `crc32.{h,cpp}` | CRC-32/IEEE-802.3 (zlib) incremental + hex; pins firmware-OTA CRC to the web client |

### Non-Source

| Path | Contents |
|---|---|
| `.github/workflows/` | CI: compile-sketch (+ flash-size gate), arduino-lint, unit-tests, clang-tidy, coverage, release (dual-board build + GitHub Release + prod OTA manifest to `gh-pages`), beta (dual-board build on `BETA`-branch push → latest-only `beta/` OTA channel on `gh-pages`, no Release) |
| `tests/` | Host doctest harness (CMake) for the pure-logic units |
| `CHANGELOG.md` | Keep-a-Changelog history; release workflow ties to version tags |
| `ARCHITECTURE.md` | Human-facing architecture narrative (subsystems, design decisions) |
| `CONTRIBUTING.md` | Build/test/PR workflow and code conventions |
| `SECURITY.md` | Private vulnerability reporting + known posture |
| `.github/ISSUE_TEMPLATE/` | Bug report + feature request templates |
| `.github/PULL_REQUEST_TEMPLATE.md` | PR checklist |
| `SDCARD/TRACKS/` | Example track JSON files |
| `CASE/` | 3D-printable enclosure STLs |
| `TACHOMETER/` | Tachometer circuit documentation |
| `README.md` | User-facing project documentation |
| `LICENSE` | GPL v3 |

---

## Hardware & Pin Map

| Pin | Function | Detail |
|---|---|---|
| Serial1 RX/TX | GPS UART | u-blox SAM-M10Q, 57600 baud |
| I2C SDA/SCL | OLED display | 128x64, address 0x3C, 400 kHz |
| I2C SDA/SCL | LSM6DS3 IMU | Onboard accelerometer/gyro (Sense variant), address 0x6A |
| SPI MOSI/SCK/MISO | SD card | 2 MHz SPI clock (EMI hardened), CS grounded on PCB |
| D1 | Button 1 (Left) | INPUT_PULLUP, RC filter recommended |
| D2 | Button 2 (Select) | INPUT_PULLUP, RC filter recommended |
| D3 | Button 3 (Right) | INPUT_PULLUP, RC filter recommended |
| D0 | Tachometer input | INPUT_PULLUP, falling-edge ISR |
| PIN_VBAT / VBAT_ENABLE | Battery ADC | 1510/510 ohm divider, 3.6 V ref |

---

## Subsystem Architecture

### Main Loop Flow (`loop()`)

```
loop()  ~250 Hz
 ├─ GPS_LOOP()              checkUblox, feed CourseManager, log DOVEX
 ├─ TACH_LOOP()             re-enable ISR after debounce, apply EMA filter
 ├─ ACCEL_LOOP()            read LSM6DS3 accelerometer X/Y/Z (g-force)
 ├─ BLUETOOTH_LOOP()        stream file chunks if transfer active
 ├─ trackDetectionLoop()    haversine scan → create CourseManager on match
 ├─ checkForNewLapData()    reads from active timer (CourseManager or lapTimer)
 ├─ checkAutoIdle()         60s at <2mph → end session, write DOVEX header
 ├─ calculateGPSFrameRate() 1-second PVT counter
 ├─ readButtons()           multi-sample debounce + edge detection
 ├─ displayLoop()           pages read from active timer helpers
 ├─ autoRaceModeCheck()     RPM>500 or speed>=10 → enter race from menu
 └─ resetButtons()          clear pressed flags
```

### 1. GPS & Lap Timing (`gps_functions.ino`, `gps_config.h`)

- Uses SparkFun u-blox GNSS v3 library with UBX binary protocol.
- `myGNSS` (SFE_UBLOX_GNSS_SERIAL) is stack-allocated in `BirdsEye.ino`.
- `GPS_SETUP()` configures module via VALSET API: 57600 baud, 25 Hz nav,
  GPS-only constellation, automotive dynamic model, PVT callback registered.
- **GPS serial buffer**: A 4 KB RAM ring buffer (`gpsRxBuf`) sits between
  Serial1 and the SparkFun library. A TIMER3 ISR drains Serial1 into this
  buffer every 10 ms, independent of the main loop. This prevents GPS data
  loss during SD card write stalls (GC pauses can block 100 ms–2 s).
  The SparkFun library reads from the buffer via `GpsBufferedStream` (a
  `Stream` wrapper). During `GPS_SETUP()` (before timer starts), reads pass
  through to Serial1 directly. Timer stopped during sleep, restarted on wake.
- `GPS_LOOP()` calls `checkUblox()` + `checkCallbacks()`. The registered
  `onPVTReceived()` callback fires with the full `UBX_NAV_PVT_data_t` struct,
  populates `gpsData`, and sets `gpsDataFresh` flag for downstream processing.
- PVT data is cached in `gpsData` struct (GpsData) for access by display
  pages and other subsystems.
- Feeds lat/lng/alt/speed into `CourseManager.loop()` which handles
  course detection, Lap Anything fallback, and sector timing internally.
- Logs validated data rows to SD as DOVEX: reserved 1 KB header,
  CSV data after byte 1024 (9-check validation pipeline).
- **Log file creation requires a valid time lock**: `onPVTReceived()` sets
  `gpsData.timeValid` only when the module asserts `validDate + validTime +
  fullyResolved` (and folds `gnssFixOK` into `gpsData.fix`). The log file is
  not created from the module's placeholder date — this prevents garbage-named
  files (e.g. `20210307_0000.dovex`) that collided every boot and corrupted on
  reboot. Until the lock arrives, `updateGpsLockHold()` pins the user to the
  tachometer page (engine running) and logging waits; a failed open is retried
  at 1 Hz and a write failure stops logging — **none fault out of race mode**.
- Time helpers: `getGpsTimeInMilliseconds()`, `getGpsUnixTimestampMillis()`.
- 64-bit timestamps are manually converted to strings (Arduino lacks `%llu`).
- **Sleep wake hardening**: `GPS_WAKE()` clears stale `gpsDataFresh`/`gpsData.fix`,
  re-applies VALSET config via `GPS_RECONFIGURE()`, and arms a 5 s PVT watchdog.
  If no PVT arrives, `GPS_BAUD_RECOVERY()` re-negotiates baud (9600→57600) and
  reconfigures. The SAM-M10Q has no flash; all config is volatile RAM only.

### 2. Tachometer (`tachometer.ino`)

- ISR `TACH_COUNT_PULSE()` fires on falling edge of D0.
- 3 ms minimum pulse gap (supports up to ~20 000 RPM).
- **Ring buffer architecture**: ISR timestamps every valid pulse into a
  16-entry ring buffer (`tachRingBuf`). `TACH_LOOP()` drains the buffer
  each main-loop iteration, computes mean inter-pulse period from ALL
  accumulated pulses, and feeds the result through a 1D Kalman filter.
- **Kalman filter** replaces the old median-of-3 + EMA. Two floats of
  state (RPM estimate `kalmanX` + uncertainty `kalmanP`). Process noise
  Q = 800 (tuned for kart engine inertia). Measurement noise R scales
  inversely with pulse count (more pulses = more confident).
- Time-based debounce only (3 ms). Old volatile flag gate removed — ISR
  body is trivially fast (<1 µs) and cannot cause interrupt storms.
- `tachLastReported` updates every main-loop call (~250 Hz). Consumers
  (display at 3 Hz, logging at 25 Hz) rate-limit themselves.
- 500 ms timeout sets RPM to 0 (engine stopped), resets Kalman state.

### 3. Accelerometer (`accelerometer.ino`)

- Onboard LSM6DS3 6-axis IMU on XIAO nRF52840 Sense (I2C address 0x6A).
- Shares I2C bus with OLED display (0x3C) — different addresses, no conflict.
- `ACCEL_SETUP()` initializes IMU; sets `accelAvailable` flag. Graceful
  degradation if IMU not present (non-Sense board).
- `ACCEL_LOOP()` reads `readFloatAccelX/Y/Z()` into global floats every
  main loop iteration (~250 Hz). Values in g-force (1g = 9.81 m/s²).
- No filtering — raw g-force is the standard unit for motorsports data.

### 4. SD Card & Logging (`sd_functions.ino`)

- SdFat library, FAT16/32, 1 MHz SPI (reduced from default for EMI hardening).
- Track files live under `/TRACKS/*.json` (ArduinoJson 6 parsing).
- **Dual JSON format**: `parseTrackFile()` auto-detects root type:
  - **Object** (HackTheTrack format): `longName`, `shortName`,
    `defaultCourse`, `courses[]` with `lengthFt`.
  - **Array** (older format, still accepted): bare array of course
    objects, metadata blank, `lengthFt = 0`. CourseDetector can't
    rank by distance without `lengthFt`, so these tracks fall back
    to Lap Anything mode.
- **Track manifest**: `buildTrackList()` also builds an in-RAM
  `trackManifest[]` (up to 200 entries) with first lat/lon per track
  for haversine proximity matching. ~10 KB RAM.
- **SD access arbitration** prevents concurrent access:
  - `acquireSDAccess(mode)` / `releaseSDAccess(mode)`
  - Modes: `SD_ACCESS_NONE` (0), `LOGGING` (1), `REPLAY` (2),
    `BLE_TRANSFER` (3), `TRACK_PARSE` (4).
- Data flushes every 10 seconds during logging.

### 5. Display & UI (`display_ui.ino`, `display_pages.ino`, `display_config.h`)

- Driver selected at compile time (`USE_1306_DISPLAY` define).
- Button debounce: 3 samples at 500 us intervals, 200 ms refire lockout.
- Pages are integer constants; key pages:
  - Boot/menu: `PAGE_BOOT` (999), `PAGE_MAIN_MENU` (-1).
  - Racing: `GPS_STATS` (4) through `LOGGING_STOP` (12).
  - Replay: `PAGE_REPLAY_FILE_SELECT` (-3), `PAGE_REPLAY_RESULTS` (-8),
    `PAGE_REPLAY_EXIT` (-9).
  - BLE: `PAGE_BLUETOOTH` (-2).
  - Errors: `PAGE_INTERNAL_WARNING` (100), `PAGE_INTERNAL_FAULT` (105).

### 6. Bluetooth (`bluetooth.ino`)

- BLE service UUID `0x1820`.
- Characteristics: file list (0x2A3D), file request (0x2A3E),
  file data (0x2A3F), file status (0x2A40).
- **OTA + version services** (set up in `BLE_SETUP()`):
  - `BLEDfu bledfu` — buttonless Nordic Secure DFU. A companion
    (DovesDataViewer over Web Bluetooth) writes the "enter bootloader"
    command; the board reboots into the bootloader's Secure DFU mode and
    receives a new firmware image over the air — no reset double-tap. The
    bootloader validates the signed/CRC'd DFU `.zip` before writing, so a
    bad/mismatched image is rejected rather than bricking the device. The
    board has no internet radio (BLE only): the companion downloads the
    GitHub release `.zip` and force-feeds it — the bootloader never
    "chooses" a file.
  - `BLEDis bledis` — Device Information Service (0x180A). Publishes
    `FIRMWARE_VERSION` (from `project.h`) via the Firmware Revision
    characteristic (0x2A26) so the companion can compare against the latest
    GitHub release and decide whether to offer an update. The Model string
    is `"BirdsEye-" FIRMWARE_VARIANT` (`BirdsEye-sense` / `BirdsEye-nonsense`)
    — equal to the release asset prefix, so the companion maps model →
    download directly. `FIRMWARE_VARIANT` is set by the per-FQBN build flag
    `-DBIRDSEYE_BOARD_SENSE` / `-DBIRDSEYE_BOARD_NONSENSE` (defaults to
    `sense`).
- MTU negotiation (requests 247, default 23).
- File listing does not require exclusive SD access; transfer does.
- **Filename validation**: every BLE command carrying a filename
  (`GET:`, `DELETE:`, `TGET:`, `TPUT:`, `TDEL:`) runs the name through
  `filename_validator::isValidFilename()` BEFORE any `SD.open()` /
  `SD.remove()` / `"/TRACKS/%s"` path build. Rejects path traversal
  (`..`, leading `.`), separators (`/`, `\`), and FAT-unsafe bytes.
  `GET`/`DELETE` reject with `ERROR` / `NOT_FOUND`; track commands
  reject with `TERR:BAD_NAME`.
- **Settings commands** (via `fileRequestChar` / `fileStatusChar`):
  - `SLIST` → `SVAL:key=value` per entry, then `SEND`
  - `SGET:key` → `SVAL:key=value` or `SERR:NOT_FOUND`
  - `SSET:key=value` → `SOK:key` or `SERR:reason`
  - `SBUSY` returned if a command is already pending.
  - Uses deferred execution: BLE callback copies command into buffer,
    `BLUETOOTH_LOOP()` processes it in main loop for thread-safe SD access.
- **Track management commands** (via `fileRequestChar` / `fileStatusChar`):
  - `TLIST` → `TFILE:name.json` per file, then `TEND`
  - `TGET:name.json` → reuses existing file transfer (`SIZE:N` → data chunks → `DONE`)
  - `TPUT:name.json` → `TREADY` → app sends data chunks → `TDONE` → `TOK`
  - `TDEL:name.json` → `TOK` or `TERR:NO_FILE`
  - Upload uses a 4096-byte static RAM buffer; `TERR:TOO_LARGE` if exceeded.
  - Error responses: `TERR:SD_BUSY`, `TERR:BUSY`, `TERR:WRITE_FAIL`, `TERR:NO_FILE`, `TERR:BAD_NAME`.
  - Upload/delete state machines: BLE callback sets flags, `BLUETOOTH_LOOP()`
    calls `processTrackUpload()` / `processTrackDelete()` for thread-safe SD
    access. Both call `buildTrackList()` after success.
- **Firmware OTA commands** (`FW*`, handled by `firmware_ota.ino` — see
  subsystem 11): `FWBEGIN`/`FWPUT`/`FWDONE`/`FWAPPLY`/`FWABORT`. The BLE
  callback dispatches them via `fwIsCommand()`/`fwHandleCommand()` and routes
  raw image chunks to `fwReceiveChunk()` while `fwReceiving()`. The request
  characteristic max length was raised from 64 to **244** so ~240-byte image
  chunks fit. `BLUETOOTH_LOOP()` calls `FW_OTA_LOOP()` each iteration.
- **Auto-reboot on BLE disconnect**: `bleDisconnectCallback()` flags a
  deferred teardown that `BLUETOOTH_LOOP()` runs on the main loop —
  `NVIC_SystemReset()` after a 100 ms delay so new settings take effect
  without a manual power cycle, plus `fwReset()` to abort any in-flight OTA
  and free the staging file + SD access. **Exception — OTA apply**: if an
  apply has been requested (`fwApplyRequested()`), the teardown skips *both*
  the abort and the reboot. After `FWAPPLY` the web app disconnects on purpose
  to let the device self-flash; rebooting here would discard the staged image
  and boot the old firmware, so the apply is left to `FW_OTA_LOOP()` (called
  later in the same `BLUETOOTH_LOOP()`), which owns its own reset.

### 7. Replay (`replay.ino`)

- Instant DOVEX header-only replay. `parseDovexHeader()` reads the
  metadata line and the lap-times line from the first 1 KB of the
  file, populates `dovexReplay*` globals + `lapHistory[]`, and the
  results page renders directly from those — no file streaming, no
  re-running the lap timer. Only `.dovex` files shown in the browser.
- `haversineDistanceMiles()` lives here too; the track-detection loop
  in `BirdsEye.ino` uses it for proximity matching.

### 8. Settings (`settings.ino`)

- Persistent JSON key-value store at `/SETTINGS.json` on SD card.
- `SETTINGS_SETUP()` called once from `setup()` after SD init; creates
  default file on first boot (random BLE name + PIN).
- **Auto-populate**: `ensureDefaultSettings()` checks for missing keys on
  boot and adds them with defaults. Existing values are never overwritten.
- `getSetting(key, buf, bufSize)` reads a value into a caller-provided
  buffer. Returns `true` if found, `false` on any failure (buf set empty).
  Always reads fresh from disk (no cache).
- `setSetting(key, value)` does read-modify-write to update a single key.
- Uses `SD_ACCESS_TRACK_PARSE` mode for brief SD access.
- Separate `StaticJsonDocument<512>` — does not share the track parser's
  4096-byte buffer.
- Total RAM cost: ~1 KB (512-byte file buffer + 512-byte JSON document).

### 9. CourseManager Integration

- **CourseManager** (`courseManager` global pointer): created when a track
  is detected via haversine proximity match, or with `courseCount=0` for
  immediate Lap Anything activation.
- **Track detection flow** (`trackDetectionLoop()`):
  1. Valid GPS time lock acquired → DOVEX log file created (see GPS section).
  2. Every GPS fix, scans `trackManifest[]` via haversine.
  3. Closest match within 5 miles → parse full JSON, build `TrackConfig`.
  4. Create `CourseManager` with settings-configurable thresholds.
  5. CourseManager handles course detection + Lap Anything fallback.
  6. No tracks / no match → `CourseManager(courseCount=0)` → Lap Anything.
- **Active timer abstraction**: helper functions (`activeTimerLaps()`,
  `activeTimerBestLapTime()`, etc.) provide a unified interface for display
  pages. They check CourseManager's active timer (DovesLapTimer or
  WaypointLapTimer) and return appropriate values.
- **Auto-race** (`autoRaceModeCheck()`): from main menu, if RPM > 500 or
  speed >= 10 mph, jumps directly to race mode.
- **Auto-idle** (`checkAutoIdle()`): if speed < 2 mph for 60 seconds
  continuously, writes DOVEX header, closes file, cleans up CourseManager,
  and returns to main menu.

### 10. Sleep Mode

- **Entry**: long-press left+right (5 s) on main menu, 5-min menu idle,
  or USB connected on main menu.
- **`enterSleepMode()`**: ends active race session, stops BLE, display off
  (I2C `DISPLAYOFF`), GPS backup mode (`powerOff(0)`), IMU power off,
  GPS serial timer stopped.
- **Wake triggers** (checked every loop in sleep):
  - **RPM wake**: tach ISR fires → `exitSleepMode(true)` → straight into
    race mode with logging enabled, Lap Anything CourseManager created.
  - **Button wake**: any button → `exitSleepMode(false)` → main menu.
- **`exitSleepMode()`**: re-enables IMU, GPS wake, display on, GPS serial
  timer restarted. RPM wake skips menu and goes directly to race mode.
- **GPS wake hardening** (`GPS_WAKE()`): clears stale `gpsDataFresh` and
  `gpsData.fix`, wakes module (0xFF + 100 ms), re-applies full VALSET
  config via `GPS_RECONFIGURE()` (idempotent), starts PVT watchdog.
  The SAM-M10Q has no flash — all config is volatile RAM. If V_BCKP drops
  during backup (cranking brownout, loose connector), module reverts to
  9600 baud NMEA. The watchdog in `GPS_LOOP()` detects missing PVT after
  5 s and triggers `GPS_BAUD_RECOVERY()` which re-negotiates baud rate
  (tries 57600, falls back to 9600 → switch → reconfigure).
- **Charging mode**: USB detected during sleep → show battery screen for
  10 s, then display off. Button re-shows for another 10 s. GPS periodic
  checks and WFE skipped while charging.
- **Periodic GPS fix**: `SLEEP_GPS_WAKE_INTERVAL` (24 h) — rarely fires
  in practice. Wakes GPS briefly, checks fix, re-sleeps.
- CPU idle via `sd_app_evt_wait()` (SoftDevice-safe WFE).

### 11. Firmware OTA (`firmware_ota.ino`, `crc32.{h,cpp}`)

- **Why self-flash**: Chrome's Web Bluetooth blocklist bans the Nordic
  *legacy* DFU service `BLEDfu` exposes, and the sealed units have no
  button/SWD pins to install a web-allowed Secure-DFU bootloader. So the app
  updates itself: the web app streams the image to SD over the existing
  `0x1820` service, the firmware CRC-verifies it, stages it to a free flash
  region, and a RAM flasher swaps it into the app region and resets. The
  bootloader is **not** changed for the field flow.
- **Wire protocol** (text on `0x2A3E` in / `0x2A40` out; image bytes are raw
  binary writes to `0x2A3E`):
  - `FWBEGIN:<size>,<crc32hex>,<variant>` → `FWCRC:<crc32hex>` (echo handshake
    to verify the control channel before any upload). `<variant>` (`sense` /
    `nonsense`) is the target board variant the web app derives from the
    device's DIS Model Number; the firmware compares it (case-insensitive) to
    `FIRMWARE_VARIANT` and replies `FWERR:VARIANT` here — the single variant
    gate, before any upload.
  - `FWPUT:<size>` → `FWREADY`, then raw ≤240-byte chunks streamed to SD
    (`/fw/pending.bin`), then `FWDONE` → `FWOK:<crc>` (CRC of the stored
    file) or `FWERR:CRC|SIZE|WRITE`.
  - `FWAPPLY` → `FWSTAGE:<pct>` (0–100, repeatable) → `FWAPPLIED` then reset,
    or `FWERR:<reason>`. `FWABORT` cancels at any point.
  - Error tokens: `CRC`, `SIZE`, `WRITE`, `BATTERY`, `VARIANT`, `STATE`,
    `FLASH`.
- **CRC**: CRC-32/IEEE-802.3 (zlib), reflected poly `0xEDB88320`, init/xor
  `0xFFFFFFFF`, lowercase 8-char hex, compared case-insensitively. Shared
  with the web client via the host-tested `crc32` pure unit. Sanity vector
  `crc32("123456789") == 0xcbf43926`.
- **Threading**: like track upload, the BLE callback only parses commands and
  copies chunk bytes into a RAM double-buffer; `FW_OTA_LOOP()` (main loop)
  does all SD writes, CRC verify, and the apply sequence. SD held via
  `SD_ACCESS_BLE_TRANSFER` for the receive.
- **Apply** (`fwDoApply()`): guards first — refuse below `FW_MIN_APPLY_VOLTAGE`
  (3.6 V, uses cached `lastBatteryVoltage`) → `FWERR:BATTERY`. (Variant is
  validated earlier, at `FWBEGIN`; no image-byte scan here. The image still
  embeds `kFwImageDescriptor` for forensics.) Then `fwStageToFlash()` copies
  SD → upper flash
  (`FW_STAGE_BASE`, via the `flash_nrf5x` HAL) and **re-verifies the CRC in
  flash before the app region is ever erased** (`FWERR:FLASH` on mismatch).
  Only then: emit `FWAPPLIED`, arm the GPREGRET recovery flag
  (`FW_GPREGRET_OTA_DFU`), disable the SoftDevice, and call the RAM-resident
  `fwRamFlasher()` to erase the app region, copy the staged image down, and
  reset.
- **Recovery net**: an interrupted swap leaves an invalid app, so the
  bootloader comes up in BLE DFU and the unit is re-flashable over the air via
  the nRF Connect mobile app — no pins. **The apply path needs the Phase 0
  hardware spikes signed off before field release** — see
  `docs/firmware-ota-phase0.md`.
- **Fleet migration**: the first firmware carrying `FW*` is pushed to sealed
  units once via nRF Connect (native app, buttonless trigger works on the
  existing single-bank bootloader); all later updates go through the web app.

---

## Data Formats

### DOVEX Log (`.dovex` files) — New UI default

```
datetime,driver_name,course_name,short_name,best_lap_ms,optimal_lap_ms
lap1_ms,lap2_ms,lap3_ms,...
\n padding to byte 1024
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm,accel_x,accel_y,accel_z
1710512400123,12,0.8,35.12345678,-97.12345678,65.32,234.56,...
```

- **Reserved header** (bytes 0–1023): Line 1 = session metadata, Line 2 =
  all lap times (comma-separated ms values), padded with `\n` to 1024 bytes.
- **GPS data** (byte 1024+): CSV column header then streaming GPS rows.
- **Crash safety**: file created with pre-filled newlines to 1024 bytes
  before any data. Header written on session end. If header is empty
  (crash), GPS data after 1024 is still valid.
- **Filename**: `20YYMMDD_HHMM.dovex`
- 1 KB handles ~100 laps (8 chars per lap time). Extremely unlikely to exceed.

### Track JSON (`/TRACKS/*.json`)

**New format** (HackTheTrack / web simulator):
```json
{
  "longName": "Orlando Kart Center",
  "shortName": "OKC",
  "defaultCourse": "Normal",
  "courses": [
    {
      "name": "Normal",
      "lengthFt": 3383,
      "start_a_lat": 28.4127081705638,
      ...
    }
  ]
}
```

**Older format** (bare array, still parsed):
```json
[
  {
    "name": "Full Course",
    "start_a_lat": 28.41270817,
    ...
  }
]
```

Auto-detected by JSON root type (object vs array). The older bare-array
form sets `lengthFt = 0` for all courses, which means CourseDetector
cannot rank by distance — CourseManager falls back to Lap Anything
immediately.

Stored in `trackLayouts[MAX_LAYOUTS]` (max 10 per track).

### Settings JSON (`/SETTINGS.json`)

```json
{
  "bluetooth_name": "DovesDataLogger-042",
  "bluetooth_pin": "7391",
  "driver_name": "Driver",
  "lap_detection_distance": "7",
  "waypoint_detection_distance": "30",
  "waypoint_speed": "30"
}
```

| Key | Type | Default | Purpose |
|-----|------|---------|---------|
| `bluetooth_name` | string | Random | BLE device name |
| `bluetooth_pin` | string | Random 4-digit | BLE pairing PIN |
| `driver_name` | string | `"Driver"` | Logged in DOVEX header |
| `lap_detection_distance` | int | `7` | DovesLapTimer crossing threshold (meters) |
| `waypoint_detection_distance` | int | `30` | WaypointLapTimer proximity zone (meters) |
| `waypoint_speed` | int | `30` | Speed threshold (mph) for waypoint/detection |

- Created automatically on first boot with random BLE values.
- Missing keys auto-populated on boot via `ensureDefaultSettings()`.
- Editable on a computer or via BLE `SSET` command — changes take effect
  on next reboot (BLE disconnect triggers auto-reboot).
- Read on-demand via `getSetting()`, written via `setSetting()`.

---

## Key Constants

| Constant | Value | Location |
|---|---|---|
| GPS baud | 57 600 | `gps_config.h` |
| GPS nav rate | 25 Hz | `gps_config.h` |
| Crossing threshold | 7.0 m | `BirdsEye.ino` |
| Max laps/session | 1 000 | `BirdsEye.ino` |
| Max locations | 200 | `project.h` |
| Max layouts/track | 10 | `project.h` |
| Max replay files | 20 | `replay.ino` |
| DOVEX header size | 1 024 bytes | `project.h` |
| Auto-idle timeout | 60 s at <2 mph | `BirdsEye.ino` |
| Track detect radius | 5 miles | `BirdsEye.ino` |
| Tach min pulse gap | 3 ms | `BirdsEye.ino` |
| Tach ring buffer | 16 entries | `BirdsEye.ino` |
| Tach Kalman Q | 800 RPM² | `tachometer.ino` |
| Tach Kalman R_BASE | 2500 RPM² | `tachometer.ino` |
| Tach stop timeout | 500 ms | `BirdsEye.ino` |
| Display refresh | 3 Hz | `display_ui.ino` |
| Button debounce | 200 ms | `display_ui.ino` |
| SD SPI clock | 2 MHz | `BirdsEye.ino` |
| Battery check interval | 5 s | `BirdsEye.ino` |
| BLE default MTU | 23 | `bluetooth.ino` |
| JSON buffer | 4096 (1024 on Wokwi) | `sd_functions.ino` |
| Settings JSON buffer | 512 | `settings.ino` |
| Settings file path | `/SETTINGS.json` | `settings.ino` |
| Track upload buffer | 4096 | `bluetooth.ino` |
| GPS serial buffer | 4096 | `gps_functions.ino` |
| GPS serial timer | TIMER3, 10 ms | `gps_functions.ino` |
| OTA staging path | `/fw/pending.bin` | `firmware_ota.ino` |
| OTA receive buffer | 2 × 4096 (double-buffer) | `firmware_ota.ino` |
| OTA app base | `0x27000` | `firmware_ota.ino` |
| OTA staging flash base | `0xA4000` | `firmware_ota.ino` |
| OTA max image size | 320 KB | `firmware_ota.ino` |
| OTA min apply voltage | 3.6 V | `firmware_ota.ino` |

---

## Required Libraries

| Library | Purpose |
|---|---|
| Adafruit GFX | Graphics primitives |
| Adafruit SSD1306 | SSD1306 OLED driver |
| Adafruit SH110X | SH110X OLED driver |
| SparkFun u-blox GNSS v3 | UBX binary PVT GPS interface |
| ArduinoJson 6.x | Track file JSON parsing |
| SdFat | SD card (FAT16/32) |
| DovesLapTimer | Lap/sector timing (external: TheAngryRaven/DovesLapTimer) |
| Seeed Arduino LSM6DS3 | Onboard IMU accelerometer/gyro (Sense variant, ±16g) |
| Bluefruit nRF52 | BLE (built into board package) |

---

## EMI Mitigation

This device operates in ignition-noise environments. Three layers of defense:

1. **Hardware**: RC low-pass filters on buttons (10 K + 100 nF) and tach
   (1 K + 100 nF + optional TVS diode).
2. **ISR design**: Volatile flag gating (never `noInterrupts()` in ISR);
   3 ms minimum pulse gap in tachometer.
3. **Software**: Multi-sample button reads (3x at 500 us), 200 ms refire
   lockout, Kalman-filtered RPM (absorbs ISR jitter), 2 MHz SPI clock for
   SD stability.
4. **GPS serial buffer**: TIMER3 ISR drains Serial1 into a 4 KB RAM ring
   buffer every 10 ms, preventing GPS data loss during SD card GC pauses
   that can block writes for 100 ms–2 s.

---

## Build Notes

- **Board**: Seeed XIAO nRF52840 Sense (Arduino IDE). The firmware also
  builds and runs on the plain (non-Sense) Seeed XIAO nRF52840 — same MCU,
  BLE, bootloader and pin map; it just lacks the onboard LSM6DS3 IMU, so
  accelerometer logging degrades gracefully (`accelAvailable = false`). CI
  (`compile-sketch`) and the `release` workflow build a matrix of both
  variants (FQBNs `xiaonRF52840Sense` and `xiaonRF52840`), publishing
  per-board `BirdsEye-sense.*` / `BirdsEye-nonsense.*` assets. The `.zip`
  in each is the Secure DFU package used for OTA. Each build passes
  `-DBIRDSEYE_BOARD_SENSE` / `-DBIRDSEYE_BOARD_NONSENSE` (via
  `compiler.cpp.extra_flags`) so the image self-reports its variant over
  BLE; a plain IDE build with no flag defaults to `sense`.
- **Firmware version** is a single `#define FIRMWARE_VERSION` in `project.h`.
  Keep it in sync with the release git tag (`v2.0.0` -> `"2.0.0"`); it is
  reported over BLE (DIS) for the OTA update check. `FIRMWARE_VARIANT`
  (also in `project.h`) feeds the DIS model string. The version literal can be
  overridden at build time with `-DFIRMWARE_VERSION_OVERRIDE=<token>` (a bare
  token; `project.h` stringizes it) — the `beta` workflow uses this to stamp
  nightly builds as `<base>-beta.<gitsha>`. Normal builds leave it undefined.
- The sketch lives in `BirdsEye/` so the folder name matches the
  `.ino` file — required by Arduino IDE / arduino-cli.
- `project.h` is included before other `.ino` modules so Arduino's
  auto-prototype generator sees custom types first.
- PROGMEM is used for bitmap images to save RAM.
- Avoid Arduino `String` in hot paths (heap fragmentation risk on 256 KB).
- SD chip-select is hardwired to GND; pass `-1` to SdFat.
- `#define WOKWI` enables simulator-specific tweaks (smaller JSON buffer).
- `#define ENDURANCE_MODE` hides the tachometer page and reshuffles
  page numbers — for endurance racing where RPM isn't relevant.
- **TIMER3 is reserved** for the GPS serial buffer ISR. Use TIMER4 if another
  hardware timer is needed. TIMER0 is reserved by SoftDevice; TIMER1/2 may
  be used by PWM/tone.
- **CRITICAL: NEVER use `analogRead()` on any GPIO pin.** On the nRF52840,
  `analogRead()` permanently disables the digital input buffer on the target
  pin for the remainder of the session. Every analog-capable pin on the XIAO
  is also a critical digital function: A0=tach ISR, A1-A3=buttons, A4=SDA,
  A5=SCL. Use `micros()` or the hardware RNG for entropy instead.

---

## Development Conventions

- `.ino` files act as modules; Arduino IDE concatenates them alphabetically
  after the main sketch file.
- Each module has a matching `.h` declaring its public surface. The `.ino`
  includes its own header as the first include so any drift between
  declaration and definition is caught at compile time.
- Each subsystem exposes `*_SETUP()` and `*_LOOP()` entry points called
  from `BirdsEye.ino`.
- SD access must go through `acquireSDAccess()` / `releaseSDAccess()`.
- GPS data validation (9 checks) must pass before any CSV row is written.
- Display pages are rendered by `displayPage_*()` functions routed via
  `currentPage` in `displayLoop()`.
- Cross-module globals (e.g. `dovexReplay*`, `trackManifest[]`, `courseManager`)
  are declared and defined in `BirdsEye.ino`. Module headers may `extern`-declare
  them where the module's own API touches that state.
- Library includes that define return types used in auto-prototyped functions
  (`DovesLapTimer.h`, `CourseManager.h`, `SparkFun_u-blox_GNSS_v3.h`) must be
  in the top include block of `BirdsEye.ino` (before Arduino generates
  prototypes).
