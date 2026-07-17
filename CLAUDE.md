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
- **Once CI is green on a PR, STOP.** Report the green status once and end.
  Do NOT schedule recurring re-checks, polling wake-ups, or "babysit"
  timers on a passing PR — they burn the owner's session usage confirming
  nothing changed. Watch a PR only when explicitly asked, and even then a
  green CI run ends the loop.

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
- **Insta360 X4 camera auto-record**: emulates the Insta360 GPS Remote as a
  pure BLE peripheral — wakes the camera on engine start, records via a ce82
  shutter toggle, stops and powers off automatically (see subsystem 13)

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
| `bluetooth.{h,ino}` | BLE service (file listing, transfer, settings, track sync), auto-reboot on disconnect; shared peripheral BLE core init (+ Just-Works bonding) + `bleOwner` radio-ownership routing |
| `camera_ble.{h,ino}` | Insta360 X4 auto-record BLE glue: peripheral remote GATT (0xCE80), all control via ce82 button notifies, executes `camera_fsm` actions, deferred callback→loop pattern (see subsystem 13) |
| `firmware_ota.{h,ino}` | SD-staged firmware OTA: `FW*` BLE protocol, SD staging, CRC verify, self-flash apply (see subsystem 11) |
| `display_pages.{h,ino}` | All page rendering functions (`displayPage_*()`) |
| `display_ui.{h,ino}` | Display init, button reading (multi-sample debounce), menu navigation, I2C bus recovery |
| `gps_functions.{h,ino}` | GPS init (SparkFun UBX PVT), time conversion, DOVEX logging pipeline, TIMER3 serial buffer ISR, V_BCKP recovery |
| `replay.{h,ino}` | Instant DOVEX header replay |
| `sd_functions.{h,ino}` | SD init, track list/JSON parsing (dual format), track manifest, SD access arbitration |
| `settings.{h,ino}` | Persistent JSON settings on SD (`/SETTINGS.json`), `getSetting()`/`setSetting()` |
| `tachometer.{h,ino}` | Falling-edge ISR on D0, Kalman-filtered RPM calculation |
| `usb_msc.{h,ino}` | USB Mass Storage (TinyUSB MSC): SD card as a drag-and-drop drive (see subsystem 12) |

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
| `sd_access_policy.{h,cpp}` | SD access arbitration decision table (mode values + grant/deny rules) |
| `lap_format.{h,cpp}` | ms → `M:SS.mmm` lap-time rendering (three zero-minutes styles), used by all display pages |
| `tach_filter.{h,cpp}` | Tachometer 1-D Kalman filter (predict/update math + Q/R tuning constants) |
| `camera_fsm.{h,cpp}` | Insta360 auto-record lifecycle FSM (8 states, all debounce/retry/timeout timing + tunables); board-portable core shared with the nRF54 "Falcon" target |
| `insta360_protocol.{h,cpp}` | Insta360 X4 BLE frame builders/parsers (wake advert, remote scan response, ce82 buttons, ce82 GPS/RMC frame, ce81 serial parsing, ce81 `0x10` record-timer state parse) with golden-byte tests |
| `wake_cause.{h,cpp}` | Boot wake-cause decode: RESETREAS + GPIO LATCH register snapshots → tach / button / USB / watchdog / soft-reset / cold boot (System OFF shutdown, subsystem 10) |
| `gps_status_page.{h,cpp}` | GPS status boot page state machine: hold, 3 s auto-close after fix+timeValid, button skip, exit destination (menu vs race), idle → shutdown |
| `sd_format_page.{h,cpp}` | SD format-confirm boot page state machine: Select held 3 s continuously → format (release restarts the full window; other buttons never confirm), 5 min idle → shutdown |
| `sat_bars.{h,cpp}` | Status-page satellite signal bars: NAV-SAT CNO selection (used-in-nav first, strongest first) + bar x/w/h layout math for the 128×~30 px bottom half |

### Simulator (`BirdsEye/sim/`)

Host build of the REAL firmware TU under the `SIM` flag (browser/WASM
target in a later phase; native + CI today). The `.ino` sources compile
unmodified — all sim behavior lives in `BirdsEye/sim/` or behind `SIM`.
See `ARCHITECTURE.md` → *Simulator* and the phased plan in the simulator
handoff spec.

| Path | Purpose |
|---|---|
| `sim_main.cpp` | Single TU replicating Arduino's .ino concatenation (bluetooth/camera_ble/usb_msc/firmware_ota deliberately absent) + host glue (`sim_init`/`sim_step_millis`/buttons/state peeks) |
| `sim_prototypes.h` | Hand-written stand-in for Arduino's auto-generated prototypes |
| `virtual_clock.{h,cpp}` | Host-advanced virtual time; `delay()` consumes it; no wall clock (determinism) |
| `arduino_shim/` | Arduino core + nRF52 registers/SoftDevice/FreeRTOS surface, Wire/SPI, LSM6DS3 (settable), Bluefruit types |
| `busio_shim/` | The display stack's entire hardware boundary: `Adafruit_I2CDevice` whose `begin()` is true and whose writes discard — everything above it (GFX/GrayOLED/SH110X) is the REAL pinned library, so the framebuffer is pixel-perfect |
| `sdfat_shim/` | In-memory VFS implementing the exact SdFat subset the firmware calls; preloads `assets/` (fixed SETTINGS.json + OKC track) via cmake-embedded byte arrays |
| `stubs/` | No-op surfaces of the excluded modules + SparkFun GNSS driver (real header, stubbed methods — PVT is injected directly into `onPVTReceived()`) |
| `frame_hash.{h,cpp}` | FNV-1a 32 over the 1024-byte framebuffer (golden fixtures, viewer dirty-check, future HIL tap) |
| `png_dump.{h,cpp}` | Dependency-free PNG writer (stored-deflate + repo crc32) for eyeballing frames |
| `native_main.cpp` | Phase-1 driver: boot → skip GPS status page → 60 s soak, state prints |
| `golden_main.cpp` | Phase-2 driver: scripted real-menu walk capturing 8 golden page hashes (`golden/golden_hashes.txt`; regenerate with `--print`, eyeball with `--dump`) |
| `oracle_main.cpp` | Phase-3 driver: lap-timing oracle. Default = synthetic constant-speed OKC circle (period exact by construction) through the whole real pipeline (boot page → race entry → proximity detect → CourseDetector "Normal" → laps ±40 ms); `--dovex <file>` replays a hardware log against its own header laps |
| `CMakeLists.txt` | Native build; FetchContent pins: DovesLapTimer `BETA` (matches CI channel), SparkFun GNSS v3.1.9 (header-only use), ArduinoJson v6.21.5, ArxTypeTraits v0.3.2, Adafruit GFX 1.12.6 + SH110X 2.1.14 (real display stack) |

### Non-Source

| Path | Contents |
|---|---|
| `.github/workflows/` | CI: compile-sketch (+ flash-size gate), arduino-lint, unit-tests, clang-tidy, coverage, sim-build (native sim TU + 60 s boot soak + determinism check), release (dual-board build + GitHub Release + prod OTA manifest to `gh-pages`), beta (dual-board build on `BETA`-branch push → latest-only `beta/` OTA channel on `gh-pages`, no Release). DovesLapTimer ref per channel: `BETA` builds track the library's `BETA` branch, master/release pin `v4.1.0` |
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
 ├─ checkAutoIdle()         60s at <2mph → end session (yields while camera recording)
 ├─ updateGpsLockHold()     pin user to tach page until GPS time lock
 ├─ CAMERA_LOOP()           step Insta360 auto-record FSM (GPS/tach fresh)
 ├─ cameraConsumeAutoStop() camera 30s-engine-off stop → endRaceSession + menu
 ├─ calculateGPSFrameRate() 1-second PVT counter
 ├─ readButtons()           multi-sample debounce + edge detection
 ├─ gpsStatusPageLoop()     boot GPS status page: GPS re-detect + hold/auto-close/exit
 ├─ sdFormatPageLoop()      boot SD format page: hold-Select confirm → format + reboot
 ├─ displayLoop()           pages read from active timer helpers
 ├─ autoRaceModeCheck()     RPM>500 or speed>=10 → enter race from menu
 └─ resetButtons()          clear pressed flags
```

### 1. GPS & Lap Timing (`gps_functions.ino`, `gps_config.h`)

- Uses SparkFun u-blox GNSS v3 library with UBX binary protocol.
- `myGNSS` (SFE_UBLOX_GNSS_SERIAL) is stack-allocated in `BirdsEye.ino`.
- **Boot probe ladder** (`GPS_SETUP()` → `gpsSetupProbe()`): the SAM-M10Q
  has no flash and every boot is a cold start (sleep = System OFF), so the
  module can be in backup mode, configured-57600, or factory-9600. Setup
  sends the `0xFF` backup-wake byte FIRST (harmless if awake), probes 57600
  (warm case ≈ instant), falls back to 9600 + `setSerialRate`, and only
  pays a 1.5 s cold-boot delay + one retry when nothing answers. Config is
  applied via the VALSET API (GPS-only constellation, automotive dynamic
  model) at the current rate target, PVT (+ NAV-SAT when wanted) callbacks
  registered, and the **5 s PVT-arrival watchdog is armed at boot too** —
  a module that answers the ping but streams nothing gets
  `GPS_BAUD_RECOVERY()`. A GPS missing entirely is re-probed by
  `GPS_STATUS_RETRY_LOOP()` (3×, 10 s apart) while the status page shows
  `NOT DETECTED` / `CHECK WIRING`.
- **Two rate modes** (`gpsEnterStatusMode()` / `gpsEnterRaceMode()`): boot
  starts in status mode — `GPS_NAV_RATE_STATUS_HZ` (5 Hz) with UBX-NAV-SAT
  at ~1 Hz feeding per-satellite CNO into `gpsSatCnos[]` for the status
  page's bars. Leaving the page switches to race mode: `GPS_NAV_RATE_HZ`
  (25 Hz) PVT-only. `GPS_RECONFIGURE()` and every wake/recovery path
  re-assert the *current* targets (`gpsNavRateTarget` / `gpsNavSatWanted`)
  — never hardcode a rate.
- **GPS serial buffer**: A 4 KB RAM ring buffer (`gpsRxBuf`) sits between
  Serial1 and the SparkFun library. A TIMER3 ISR drains Serial1 into this
  buffer every 10 ms, independent of the main loop. This prevents GPS data
  loss during SD card write stalls (GC pauses can block 100 ms–2 s).
  The SparkFun library reads from the buffer via `GpsBufferedStream` (a
  `Stream` wrapper). During `GPS_SETUP()` (before timer starts), reads pass
  through to Serial1 directly. Timer stopped on shutdown/charging entry,
  restarted on the charging-loop resume (`GPS_WAKE()`).
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
- **Wake hardening**: `GPS_WAKE()` (charging-loop resume) clears stale
  `gpsDataFresh`/`gpsData.fix`, re-applies VALSET config via
  `GPS_RECONFIGURE()`, and arms the same 5 s PVT watchdog as boot.
  If no PVT arrives, `GPS_BAUD_RECOVERY()` re-negotiates baud (9600→57600) and
  reconfigures. The SAM-M10Q has no flash; all config is volatile RAM only.

### 2. Tachometer (`tachometer.ino`)

- ISR `TACH_COUNT_PULSE()` fires on falling edge of D0.
- 3 ms minimum pulse gap (supports up to ~20 000 RPM).
- **Ring buffer architecture**: ISR timestamps every valid pulse into a
  16-entry ring buffer (`tachRingBuf`). The ISR checks full before
  publishing (SPSC, one slot sacrificed) and drops + sets
  `tachRingOverflow` instead of lapping the consumer during SD GC stalls;
  `TACH_LOOP()` then discards the one period spanning the gap. `TACH_LOOP()` drains the buffer
  each main-loop iteration, computes mean inter-pulse period from ALL
  accumulated pulses, and feeds the result through a 1D Kalman filter.
- **Kalman filter** replaces the old median-of-3 + EMA. Two floats of
  state (estimate + uncertainty in `tach_filter::Kalman`); the
  predict/update math and tuning constants live in the host-tested
  `tach_filter` pure unit. Process noise
  Q = 800 (tuned for kart engine inertia). Measurement noise R scales
  inversely with pulse count (more pulses = more confident).
- Time-based debounce only (3 ms). Old volatile flag gate removed — ISR
  body is trivially fast (<1 µs) and cannot cause interrupt storms.
- `tachLastReported` updates every main-loop call (~250 Hz). Consumers
  (display at 3 Hz, logging at 25 Hz) rate-limit themselves.
- 500 ms timeout sets RPM to 0 (engine stopped), resets Kalman state.
- The engine-start wake from shutdown is NOT this module's job: System OFF
  wakes on the tach pin's GPIO SENSE and the boot decodes the LATCH bit via
  `wake_cause` (the old `tachHavePeriod` latch + `TACH_SLEEP()` are gone).

### 3. Accelerometer (`accelerometer.ino`)

- Onboard LSM6DS3 6-axis IMU on XIAO nRF52840 Sense (I2C address 0x6A).
- Shares I2C bus with OLED display (0x3C) — different addresses, no conflict.
- `ACCEL_SETUP()` initializes IMU; sets `accelAvailable` flag. Graceful
  degradation if IMU not present (non-Sense board).
- `ACCEL_LOOP()` reads `readFloatAccelX/Y/Z()` into global floats every
  main loop iteration (~250 Hz). Values in g-force (1g = 9.81 m/s²).
- No filtering — raw g-force is the standard unit for motorsports data.

### 4. SD Card & Logging (`sd_functions.ino`)

- SdFat library, FAT16/32, 2 MHz SPI (reduced from default for EMI hardening).
  Raised to 8 MHz (`SD_SPI_SPEED_FAST`) for the duration of a BLE or USB
  transfer via `sdSetTransferSpeed(true)` and reverted afterward — transfers
  happen parked (motor off), so the EMI rationale doesn't apply. Re-`SD.begin()`
  is the runtime clock switch; it falls back to 2 MHz if the fast re-init fails.
- Track files live under `/TRACKS/*.json` (ArduinoJson 6 parsing).
- **Blank-card self-provision**: `buildTrackList()` creates `/TRACKS`
  when missing (SdFat's `open()` never creates parent dirs), and
  `processTrackUpload()` mkdirs it again before every upload — so a
  factory-blank soldered-in module can sync tracks over BLE on first boot.
- **On-device format** (`sdPerformFormat()` + the host-tested
  `sd_format_page` unit): when `SD_SETUP()` finds the card answers at the
  SPI level (`SD.cardBegin` + `sectorCount()`) but `volumeBegin()` fails
  — the volume re-check matters: a transient card-level failure with a
  healthy FAT must remount, not be offered an erase — boot lands on
  `PAGE_SD_FORMAT` (buttons live, unlike FAULT). Hold Select ALONE 3 s
  to format FAT16/32 via SdFat's `SD.format()` (zero new RAM; WDT fed
  through the formatter's Print callbacks; 8 MHz clock only when the
  engine isn't turning — EMI corrupts writes silently — else 2 MHz),
  then `/TRACKS` is provisioned (`sdEnsureTracksFolder()`) and the
  device reboots clean. The confirm can never fire from the wake press
  (Select must be seen released once) nor beat the Select+side reboot
  combo (a held side button disarms it). A paired camera is stopped and
  powered off (`CAMERA_SLEEP()`) before the format blocks the loop, since
  the ending hard reset runs no shutdown teardown. Format failure returns
  to the confirm page (marked `FAILED - retry`; fresh full hold to retry —
  keeps the idle-timeout battery protection the FAULT dead-end lacks);
  a dead/absent card never offers the format. 5 min idle → shutdown
  (deferred while the engine runs), and a charging-loop resume with the
  card still unformatted returns to the format page, not the menu.
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
    `BLE_TRANSFER` (3), `TRACK_PARSE` (4), `USB_MSC` (5), `FORMAT` (6) —
    values and grant/deny rules live in the host-tested `sd_access_policy`
    pure unit.
    `USB_MSC` is a normal exclusive holder (held for the whole USB
    mass-storage session; see subsystem 12).
  - Transitions are **atomic**: the check-then-set runs inside a FreeRTOS
    critical section (`taskENTER_CRITICAL`, BASEPRI-masked so SoftDevice
    radio interrupts are unaffected) because the Bluefruit callback task
    and the main loop share the owner flag.
  - Belt-and-suspenders only: all SD-touching BLE work is deferred to the
    main loop (see subsystem 6), so SdFat itself is single-task.
- Data flushes every 10 seconds during logging.

### 5. Display & UI (`display_ui.ino`, `display_pages.ino`, `display_config.h`)

- Driver selected at compile time (`USE_1306_DISPLAY` define).
- Button debounce: 3 samples at 500 us intervals, 200 ms refire lockout.
- All lap times render via the host-tested `lap_format::formatLapTime()`
  (ms → `M:SS.mmm`, always 3-digit ms; zero-minutes styles: `kOmit` for
  replay results, `kShow` for the lap list, `kSpace` column-stable for the
  big-font live pages). Never hand-roll the `60000`/`%1000` math inline.
- Pages are integer constants; key pages:
  - Boot/menu: `PAGE_BOOT` (999), `PAGE_GPS_STATUS` (900, satellite status
    page every boot lands on — driven by `gpsStatusPageLoop()`, buttons
    deliberately no-op'd in `displayLoop()`), `PAGE_MAIN_MENU` (-1).
  - Racing: `GPS_STATS` (4) through `LOGGING_STOP` (12).
  - Replay: `PAGE_REPLAY_FILE_SELECT` (-3), `PAGE_REPLAY_RESULTS` (-8),
    `PAGE_REPLAY_EXIT` (-9).
  - Transfer: `PAGE_TRANSFER_MENU` (-4) Bluetooth/USB submenu,
    `PAGE_USB_STORAGE` (-5) USB drive active.
  - BLE: `PAGE_BLUETOOTH` (-2).
  - Camera: `PAGE_PAIR_CAMERA` (-6) pairing / paired-status management,
    `PAGE_CAMERA_SERIAL_ENTRY` (-7) manual 6-char serial entry fallback,
    `PAGE_CAMERA_TEST` (-10) bench test menu (paired-only manual controls).
  - Errors: `PAGE_INTERNAL_WARNING` (100), `PAGE_INTERNAL_FAULT` (105),
    `PAGE_SD_FORMAT` (106, card responds but FAT won't mount — driven by
    `sdFormatPageLoop()`, buttons live unlike FAULT).

### 6. Bluetooth (`bluetooth.ino`)

- **Shared BLE core, used by transfer and the camera** (see subsystem 13):
  the one-time `bleCoreEnsureInit()` runs `Bluefruit.begin(1, 0)` — one
  peripheral connection, no central (both the transfer service and the
  camera remote are peripheral roles) — configures Just-Works bonding, and
  registers *every* GATT service (DFU, DIS, file service, camera remote via
  `cameraBleRegisterServices()`) before any advertising starts.
  `BLE_SETUP()` / `BLE_STOP()` are now just the transfer-mode owner
  transitions on top of that core.
- **Radio ownership (`BleOwner`)**: the single advert set + peripheral
  connection slot are shared between the transfer service and the camera
  remote. `bleOwner` (`NONE` / `TRANSFER` / `CAMERA`, enum in `project.h`,
  variable in `BirdsEye.ino`) records the current owner; the shared
  connect/disconnect callbacks route on it, so a camera link can never
  trigger the transfer auto-reboot and file commands are ignored unless
  the transfer service owns the radio. `bleActive` / `bleConnected` keep
  their transfer-only meanings — camera mode never sets them. Owner
  transitions happen only on the main loop, never in a Bluefruit callback.
  - **Reboot gate (`bleTransferEngaged`)**: the radio has one BD_ADDR, so a
    bonded camera can connect to the *transfer* advert and be routed as "the
    phone". The auto-reboot-on-disconnect is therefore gated on the peer
    having actually written the file/settings/OTA service — a camera that
    only vets our GATT and drops never reboots the logger out of a transfer
    session (and held no SD, so there's nothing to tear down).
  - **Advert teardown**: `BLE_STOP()` disarms `restartOnDisconnect(false)`
    *before* its async disconnect, so Bluefruit's core handler can't restart
    a stale ownerless transfer advert after the stop (which would let a phone
    reconnect into a mute session and occupy the slot camera auto-record
    needs).
- BLE service UUID `0x1820`.
- Characteristics: file list (0x2A3D), file request (0x2A3E),
  file data (0x2A3F), file status (0x2A40).
- **OTA + version services** (registered in `bleCoreEnsureInit()`):
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
- **No SdFat in the callback task — ever.** Every SD-touching command
  (`LIST`, `GET:`, `DELETE:`, `TLIST`, `TGET:` via the deferred
  `fileCmdBuffer`; settings, `TPUT:`/`TDEL:`, and `FW*` via their own
  deferred state) is only parsed/validated in the BLE callback and is
  executed by `BLUETOOTH_LOOP()` on the main loop. Listings hold the SD
  lock for the whole directory walk; `DELETE` takes the lock and refuses
  (`BUSY`) while a transfer is streaming. One file command may be queued
  at a time — a second gets the protocol's busy reply (`BUSY` /
  `TERR:BUSY`).
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
  default file on first boot (random BLE name + PIN + racing-word device name).
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
  2. Scans `trackManifest[]` via haversine, throttled to 1 Hz (the scan
     is O(N) software-double math; `gpsData.fix` stays true between PVT
     updates, so an unthrottled scan ran every ~250 Hz loop iteration).
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

### 10. Shutdown (System OFF)

"Sleep" is a full power-down: nRF52 **System OFF** (~µA), designed so the
hardware needs no power switch. Wake = chip reset = fresh `setup()`.

- **Entry** (`enterShutdown()`): long-press left+right (5 s) on main menu,
  5-min menu idle, the GPS status page's idle timeout, the SD format
  page's idle timeout (deferred while the engine runs, so a tach-wake
  with a bad card doesn't power-cycle all session), or USB present on
  the main menu after 60 s of button inactivity (`USB_MENU_CHARGE_IDLE_MS`
  — not immediate, so a charging-loop button wake doesn't bounce and the
  device stays usable for replay/transfer while plugged in).
- **Teardown order** (wdtPet-bracketed — `CAMERA_SLEEP()`'s 3 s ce82
  power-off hold is the longest step under the armed ~4 s WDT): end race
  session → `CAMERA_SLEEP()` → `BLE_STOP()` if active → `DISPLAY_SLEEP()`
  → `GPS_SLEEP()` (u-blox software backup, µA, config retained while
  powered; TIMER3 stopped) → IMU power rail off.
- **System OFF entry** (`shutdownSystemOff()`, no return): wait for the
  entry combo's buttons to release (a held button = SENSE satisfied =
  instant wake-reset), configure `nrf_gpio_cfg_sense_input(pull-up,
  SENSE-LOW)` on the tach pin + all 3 buttons (P-numbers via
  `g_ADigitalPinMap`, never hardcoded), clear the GPIO LATCH registers
  (a set latch = pending DETECT = instant re-wake), clear pending FPU
  exceptions, then `sd_power_system_off()` when the SoftDevice is enabled
  (BLE is lazy — check `sd_softdevice_is_enabled()`) else raw
  `NRF_POWER->SYSTEMOFF`. **GPREGRET is untouched** — register 0 belongs
  to the OTA/bootloader handoff (subsystem 11). The WDT halts in System
  OFF (all clocks stop); `wdtSetup()` re-arms on the fresh boot.
- **Wake sources**: tach pulse (D0 falling, engine start), any button,
  or VBUS (USB plug-in, always armed on nRF52840).
- **Wake-cause decode** (`captureBootWakeCause()`, FIRST thing in
  `setup()`): reads then clears `RESETREAS` + `NRF_P0/P1->LATCH` (sticky,
  cumulative) and decodes via the host-tested `wake_cause` unit. A tach
  wake makes the GPS status page exit into race mode with logging; a USB
  wake skips the status page straight into the charging loop.
- **Charging loop — the one soft-sleep survivor** (`runChargingShutdownLoop()`):
  System OFF is never entered while VBUS is present, because the HICHG
  fast-charge pin (`PIN_CHARGING_CURRENT`) is software-held. After the
  same full teardown, the loop shows the charging screen for 10 s then
  turns the display off; **any button is a full wake to the main menu**
  (`softResumeFromCharging()`: IMU re-init, race-mode GPS targets +
  `GPS_WAKE()`, display on); unplugging drops to System OFF. CPU idles
  via `shutdownIdleWait()` — `sd_app_evt_wait()` only when the SoftDevice
  is actually enabled, `__WFE()` otherwise.

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

### 12. USB Mass Storage (`usb_msc.ino`)

- **Why**: a wired, app-free way to move files. The SD is FAT16/32, so a
  host PC can mount it as a drive and drag-and-drop track JSON / DOVEX logs.
  Complements (does not replace) the BLE transfer service.
- **Stack**: TinyUSB `Adafruit_USBD_MSC` (bundled in the Seeed/Adafruit
  nRF52 core; the core's default USB stack is TinyUSB). Three block
  callbacks wrap SdFat's block device: `msc_read_cb` →
  `SD.card()->readSectors()`, `msc_write_cb` → `writeSectors()`,
  `msc_flush_cb` → `syncDevice()` + `SD.cacheClear()`. These run on the
  USBD task, not the main loop.
- **UI flow**: main-menu **Transfer** → `PAGE_TRANSFER_MENU` (Bluetooth /
  USB). **Bluetooth** keeps the existing `BLE_SETUP()` + `PAGE_BLUETOOTH`
  path untouched. **USB** → `PAGE_USB_STORAGE` + `USB_MSC_ENABLE()`.
- **Opt-in enumeration**: `USB_MSC_SETUP()` (called from `setup()` after a
  successful `SD_SETUP()`) only registers the callbacks — no drive is
  presented at boot, so charging/plug-in behaves as before.
  `USB_MSC_ENABLE()` first requires **VBUS present** (`isUsbConnected()`) —
  without a cable there is nothing to mount and the parked loop would read
  absent VBUS as a cable-pull and instantly reset, so it bails before taking
  the lock or enumerating (the menu shows "Plug in USB cable first"). It then
  acquires `SD_ACCESS_USB_MSC`, sets the capacity from
  `sectorCount()`, marks the unit ready, `begin()`s the interface (bailing
  out — restoring the clock and releasing the lock — if `begin()` fails),
  and forces a `TinyUSBDevice.detach()` / 50 ms / `attach()` re-enumeration
  so the host mounts the drive. If the SD mutex is busy it bails to a
  warning page and changes nothing.
- **Loop parking**: while `usbMscActive`, `loop()` takes an early-return
  branch (mirroring the `bleActive` branch) that skips all GPS/tach/lap/SD
  processing — the host PC owns the FAT, so the firmware must not touch it
  concurrently. The branch services only the Exit button and the status
  page, and watches VBUS: if the cable is unplugged it calls
  `USB_MSC_DISABLE()` so the SD lock and fast SPI clock can't leak past the
  session.
- **Exit = reboot**: `USB_MSC_DISABLE()` drops media-ready, **quiesces** (waits
  up to 1 s for the write block callback to go quiet — `setUnitReady(false)`
  only blocks *new* SCSI commands, so an in-flight `WRITE10` keeps calling
  `msc_write_cb` on the USBD task, and resetting mid-write would truncate a
  file / corrupt the FAT), `syncDevice()`s
  the card, then calls `NVIC_SystemReset()` (mirrors the BLE auto-reboot on
  disconnect). The reboot drops the MSC interface and remounts a clean
  filesystem, so host edits are picked up without any SdFat cache-coherency
  dance. Triggered by the on-device Exit button or a cable unplug.
- **Mutex**: the whole session holds `SD_ACCESS_USB_MSC`, so logging,
  replay, and BLE transfer are locked out (and vice-versa) — though loop
  parking, not the mutex, is the primary guarantee the firmware stays off
  the card. The transfer/USB pages are not the main menu, so the
  USB-on-main-menu charge-mode entry never fires while transferring.
- **No pure unit test**: the block-callback glue is TinyUSB/Arduino-bound
  (hardware), so there is no host-testable logic here — only the
  `sd_access_policy` mode addition is unit-tested.

### 13. Camera Auto-Record (`camera_ble.ino`, `camera_fsm.{h,cpp}`, `insta360_protocol.{h,cpp}`)

- **What**: hands-free Insta360 X4 control. The device *is* the Insta360
  "GPS Remote": it emulates the physical remote so a paired X4 is woken
  when the engine starts, records the session, and powers itself off
  afterward — the driver never touches the camera.
- **Single BLE role: peripheral remote** on the one SoftDevice
  (`Bluefruit.begin(1, 0)`). We host the remote's GATT — service `0xCE80`
  (ce81 WRITE camera→us carrying its serial + status frames, ce82 NOTIFY
  us→camera button frames, ce83 READ static) — advertise the wake/identity
  payload, and **the camera connects to us as central** and subscribes to
  ce82. **All control is a ce82 button notification**, byte-for-byte the
  physical remote's frames: recording toggles via the shutter button, and
  power-off streams the 3-second power-hold. We never act as central: no
  scanning, no `be80` client, no `be81` writes. (The old central role held
  a `be80` link to the camera for start/stop-video — removed. It made
  power-off impossible: power-off only exists as a remote `ce82` hold,
  which cannot coexist with being the camera's `be80` client.)
- **GPS overlay** (`cameraServiceGpsStream()`): a Wireshark capture of the
  genuine remote↔camera link showed the remote streams GPS on **`ce82` at
  10 Hz** as a non-standard NMEA-RMC frame (`FC EF FE 83 00 <len>
  ,26.7,\x07,$GNRMC,...` — signed longitude with a constant `E`, an extra
  `V` field; built + golden-tested in `insta360_protocol::buildGpsRmcFrame`).
  The firmware streams it continuously while the camera is connected +
  subscribed — this doubles as the remote's **liveness heartbeat** (never
  go silent or the camera drops us; status `V` with last-known coords when
  there's no fix), paused only during a power-off hold. GPS still logs to
  SD independently.
- **Lifecycle FSM** (`camera_fsm` pure unit, host-tested): the race-mode
  lifecycle is deliberately **RPM-driven and simple**. 7 states — UNPAIRED /
  IDLE / WAKING / AWAIT_READY / RECORDING / **WATCHING** / PAIRING (the old
  COOLDOWN/POWERING_OFF tail is gone — power-off is now sleep-only).
  RPM > 500 held 2 s enters WAKING, which broadcasts the 31-byte CONNECTABLE
  wake advert — the sniffed GPS-Action-Remote manufacturer payload (serial at
  mfg[14..19], per the primary `pchwalek/insta360_ble_esp32` source) in the
  primary PDU with the "Insta360 GPS Remote" name in the scan response, both
  set as raw bytes so the stack can't reshape them (retry ×3). The woken
  camera connects back and the FSM moves to AWAIT_READY (wait for the ce82
  subscription; bounded re-wake if it never subscribes) → **WATCHING**.
  (Wake only reaches an ARMED camera: on the X4, Bluetooth Wakeup is armed
  when **QuickCapture is OFF** — an armed camera keeps its radio scanning even
  fully powered off. Every advert goes through `bleAdvFinalizePadded()` — see
  bluetooth.ino — to defeat the Bluefruit 0.21.0 frozen-packet-length core
  bug.) **Recording starts** from WATCHING once RPM has held above ON for
  `kRecordStartDelayMs` (5 s) — **no GPS-lock gate** (GPS still streams the
  whole time) — by sending one shutter-toggle ce82 frame. The shutter is a
  stateful TOGGLE, so the FSM never blind-fires it: it **confirms** record
  state from the camera's own `0x10` ce81 display-string frame (a live
  `.HH:MM:SS` timer while recording — the `0x02` status word is not reliable;
  `insta360_protocol::parseRecordingState`) and reconciles `recordingActive`
  against that observation (`RecordObs` Input). On reconnect it adopts the
  camera's real state instead of toggling; if the camera reports idle while we
  believe we're recording it re-asserts the shutter once; and the belief is
  preserved on any path where the camera is unreachable, so a dropped link
  can't invert on reconnect. **Recording stops** after `kStopRecordDelayMs`
  (30 s) of engine-off (RPM < 300) — **RPM only, no speed**, so a
  stationary-but-running grid idle keeps recording — sending one shutter
  toggle and returning to WATCHING. The 30 s-engine-off auto-stop also **ends
  the race log session** (see the read-only note below). A manual session end
  (`CAMERA_NOTIFY_SESSION_END()` from the logging-stop confirm) stops the
  camera immediately, also to WATCHING. **WATCHING** keeps the camera ON and
  connected: if RPM returns it re-records (stall recovery), and it powers the
  camera off **only when the device shuts down** (`CAMERA_SLEEP()` streams the
  ce82 power-hold synchronously) — there is no post-record cooldown/power-off
  timeout. All timing lives in the FSM so the temporal behavior is
  host-testable; every tunable is a single-point `constexpr` in
  `camera_fsm.h`. The unit is the board-portable core shared with the nRF54
  ("Falcon") target — nothing in it may `#ifdef` on the platform.
- **Telemetry consumer, with one deliberate write-back**: the FSM consumes an
  `Inputs` snapshot (RPM, link state, observed record state, one-shot events)
  built fresh each `CAMERA_LOOP()` and returns at most one `Action`. Camera
  mode never parks the main loop (unlike `bleActive` / `usbMscActive`). The
  ONE exception to the old read-only guarantee: when the camera auto-stops
  (30 s engine-off), the glue latches `cameraConsumeAutoStop()` and the main
  sketch calls `endRaceSession()` + returns to the menu — so with a camera
  paired+recording the log ends on 30 s-no-RPM instead of the speed-based
  auto-idle (which `checkAutoIdle()` suppresses while `cameraActivelyRecording()`).
  Without a camera, logging is unchanged.
- **Pairing / bonding**: entering pairing from `PAGE_PAIR_CAMERA`
  advertises connectably as "Insta360 GPS Remote"; after connecting, the
  camera writes its 6-char ASCII serial to ce81, which is captured and
  persisted in the `camera_serial` setting (empty = unpaired). The manual
  6-char entry page (`PAGE_CAMERA_SERIAL_ENTRY`) is the fallback. Pairing
  times out after 2 min. The genuine remote link is encrypted + bonded, so
  we support Just-Works (NoInputNoOutput) pairing as peripheral — the
  camera may withhold its ce82 subscription until the link is encrypted.
- **Coexistence** (`bleOwner`, see subsystem 6): the camera shares the
  single advert set + peripheral slot with the transfer service. Opening
  the Bluetooth transfer page calls `CAMERA_FORCE_RELEASE()` before
  `BLE_SETUP()` — best-effort stop recording, drop the camera link, stop
  camera-owned advertising, force the FSM to IDLE, release the radio.
  Shutdown entry runs `CAMERA_SLEEP()` (same, plus a power-off that is
  **streamed synchronously** before the disconnect — the chip powers off
  right after, so the non-blocking ce82 hold would otherwise never transmit
  a frame and the camera would run all night). BLE
  comes up **lazily** on the first camera action (first advertising
  `Action`), so unpaired users pay zero RAM/power cost.
- **Threading**: same deferred pattern as `firmware_ota` — Bluefruit
  callbacks (connect/disconnect/ce81 writes/ce82 CCCD writes) only copy
  into RAM and set volatile flags; `CAMERA_LOOP()` on the main loop
  consumes them, steps the FSM, and does all real work (including the one
  `setSetting()` that persists a captured serial and streaming the
  power-off hold). The ce82 CCCD callback latches a cached `ce82NotifyOn`
  flag so the hot paths (per-loop Inputs snapshot + 10 Hz GPS tick) never
  SVC into the SoftDevice for the subscription state; `CAMERA_LOOP()`
  re-reads `notifyEnabled()` only at a low rate while linked-but-not-yet-known-
  subscribed, to catch a bonded peer's silent sys-attr CCCD restore.
- **X4-VERIFY posture**: all frame bytes live in the host-tested
  `insta360_protocol` pure unit with golden-byte tests. The **wake
  advert + scan response are X4-CONFIRMED ground truth** — captured from
  a genuine GPS Remote with nRF Connect (2026-07-10 bench session) and
  the replayed packet woke the sleeping X4; flags are `0x05`, and the
  paired camera-mode advert presents this same remote-identity packet.
  Remaining `// X4-VERIFY(sniff)`: the ce82 button frames (proven capture
  bytes; the record/power-off *effect* still to be confirmed end-to-end on
  an X4) and the ce81/ce83 parsers.
- **Bench test menu** (`PAGE_CAMERA_TEST`): the paired Camera page has a
  **Test** entry opening a manual-control menu (Wake / Record / Power Off /
  Back) plus live remote (**R**) link status (`R:UP+` when the camera is
  connected AND subscribed to ce82, `R:UP` when connected but ignoring our
  buttons), advert (**Adv:**) status, a **G:** GPS-feed indicator (`SYNC`
  with a fix / `V` voided-but-streaming / `--` not streaming) and a
  **rec:yes/no** state driven by the camera's own `0x10` record timer
  (`rec:--` when there's no fresh observation — no link, or the camera hasn't
  pushed a `0x10` yet — so a missing signal isn't misread as "not recording")
  — so both the GPS link and that a Record press actually started the camera
  can be verified without staging RPM/GPS to drive the FSM. **Wake** presents the
  wake / remote-identity advert so a standby camera wakes and an on camera
  connects back to us; **Record** sends the ce82 shutter toggle; **Power
  Off** streams the ce82 hold — both need the camera connected + subscribed.
  `cameraTestEnterMode()` forces the FSM to IDLE and sets `cameraTestActive`,
  which makes `CAMERA_LOOP()` suppress the FSM step (so auto-record can't
  fight the manual actions) while the Bluefruit callbacks keep the link
  serviced; `cameraTestExitMode()` stops any recording (tracked in a
  bench-local belief so the camera is **guaranteed** left stopped on exit)
  and tears the session down. The `cameraTest*()` action helpers reuse the exact
  `cameraExecuteAction()` code paths the FSM would run — no new FSM states,
  so the board-portable pure unit is untouched.
- **X4 field notes** (from live bench testing, informing the above): the
  camera connects to our ce80 remote (R link) only *after* it has been
  paired from the camera's own **Settings → Bluetooth remote** menu —
  capturing the serial (subsystem UI pairing) is not sufficient. The
  wake-burst advert only wakes an *armed* camera (X4: QuickCapture OFF =
  "Bluetooth Wakeup Enabled"; armed cameras keep the radio scanning even
  powered off, un-armed ones are BLE-dead). There is **no** `be80`
  power-off command in any known reference implementation — power-off is
  the `ce82` 3-second-hold button frame over the R link, so both Record and
  Power Off depend on the R link being up and subscribed.

---

## Data Formats

### DOVEX Log (`.dovex` files) — New UI default

```
datetime,driver_name,course_name,short_name,best_lap_ms,optimal_lap_ms,device_name
lap1_ms,lap2_ms,lap3_ms,...
\n padding to byte 1024
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm,accel_x,accel_y,accel_z
1710512400123,12,0.8,35.12345678,-97.12345678,65.32,234.56,...
```

- **Reserved header** (bytes 0–1023): Line 1 = session metadata, Line 2 =
  all lap times (comma-separated ms values), padded with `\n` to 1024 bytes.
- **`device_name`** is the trailing metadata column (after `optimal_lap_ms`).
  Appending it keeps old logs readable (parsed as empty) and lets older
  readers ignore the extra column — backwards compatible by design.
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
  "camera_serial": "",
  "device_name": "ApexTurbo",
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
| `camera_serial` | string | `""` (empty = unpaired) | Paired Insta360 X4's 6-char serial (auto-captured on pairing, or entered manually) |
| `device_name` | string | Random racing words | Identifies the logging device (DOVEX header) |
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
| GPS nav rate (race) | 25 Hz | `gps_config.h` |
| GPS nav rate (boot/status page) | 5 Hz + NAV-SAT ~1 Hz | `gps_config.h` |
| Status page auto-close | 3 s after fix+timeValid | `gps_status_page.h` |
| Status page idle shutdown | 5 min (no lock, no engine) | `gps_status_page.h` |
| SD format confirm hold | 3 s continuous Select | `sd_format_page.h` |
| SD format page idle shutdown | 5 min | `sd_format_page.h` |
| GPS boot re-detect | 3 tries, 10 s apart | `gps_functions.ino` |
| Menu idle shutdown | 5 min (`SLEEP_IDLE_TIMEOUT_MS`) | `project.h` |
| USB-on-menu charge idle | 60 s (`USB_MENU_CHARGE_IDLE_MS`) | `project.h` |
| Charging screen timeout | 10 s (`CHARGE_DISPLAY_TIMEOUT_MS`) | `project.h` |
| Sat bars display cap / CNO ceiling | 16 bars / 50 dB-Hz | `sat_bars.h` |
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
| Tach Kalman Q | 800 RPM² | `tach_filter.h` |
| Tach Kalman R_BASE | 2500 RPM² | `tach_filter.h` |
| Track manifest scan throttle | 1 Hz | `BirdsEye.ino` |
| Tach stop timeout | 500 ms | `BirdsEye.ino` |
| Display refresh | 3 Hz | `display_ui.ino` |
| Button debounce | 200 ms | `display_ui.ino` |
| SD SPI clock (normal) | 2 MHz | `BirdsEye.ino` |
| SD SPI clock (transfer) | 8 MHz (`SD_SPI_SPEED_FAST`) | `BirdsEye.ino` |
| Battery check interval | 5 s | `BirdsEye.ino` |
| BLE default MTU | 23 | `bluetooth.ino` |
| JSON buffer | 4096 (SIM builds too) | `sd_functions.ino` |
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
| Camera record-start delay | 5 s RPM held above ON (no GPS gate) | `camera_fsm.h` |
| Camera stop-record delay | 30 s engine-off (RPM only) → also ends log session | `camera_fsm.h` |
| Camera power-off | shutdown only (no post-record cooldown/timeout) | `camera_ble.ino` (`CAMERA_SLEEP`) |
| Camera RPM on/off thresholds | 500 / 300 (2 s on-debounce) | `camera_fsm.h` |
| Camera wake attempt window | 20 s ×3 (beacon) | `camera_fsm.h` |
| Camera connect / subscribe timeouts | 20 s connect / 10 s ce82 subscribe, 3 retries each | `camera_fsm.h` |
| Camera record-confirm re-assert | 2.5 s camera-idle before re-shutter | `camera_fsm.h` |
| Camera record-obs freshness | 3 s (stale 0x10 → kUnknown) | `camera_ble.ino` |
| Camera pairing timeout | 120 s | `camera_fsm.h` |

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
| DovesLapTimer | Lap/sector timing (external: TheAngryRaven/DovesLapTimer). CI refs: `BETA`-targeted builds track the library's `BETA` branch; master/release builds pin `v4.1.0` (bump deliberately) |
| Seeed Arduino LSM6DS3 | Onboard IMU accelerometer/gyro (Sense variant, ±16g) |
| Bluefruit nRF52 | BLE (built into board package) |
| Adafruit TinyUSB | USB Mass Storage (`Adafruit_USBD_MSC`); built into board package |

---

## EMI Mitigation

This device operates in ignition-noise environments. Three layers of defense:

1. **Hardware**: RC low-pass filters on buttons (10 K + 100 nF) and tach
   (1 K + 100 nF + optional TVS diode).
2. **ISR design**: Volatile flag gating (never `noInterrupts()` in ISR);
   3 ms minimum pulse gap in tachometer.
3. **Software**: Multi-sample button reads (3x at 500 us), 200 ms refire
   lockout, Kalman-filtered RPM (absorbs ISR jitter), 2 MHz SPI clock for
   SD stability (raised to 8 MHz only during parked BLE/USB transfers, where
   the motor is off and ignition EMI is absent — see subsystem 4).
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
- `#define SIM` enables simulator-specific tweaks (no WDT, fixed battery
  voltage, placeholder GPS setup, sim button pins). Never defined in CI
  firmware builds — it's the compile flag for the browser/WASM simulator
  build (sources under `BirdsEye/sim/`; replaces the old Wokwi target).
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
