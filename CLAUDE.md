# BirdsEye - Project Guide

> **MAINTAINERS: Keep this file updated when adding/removing files, changing pin
> assignments, modifying subsystem interfaces, or altering the build configuration.
> This file is loaded into Claude's context window on every session and must
> accurately reflect the current state of the project.**

## What Is BirdsEye?

A high-precision GPS lap timer and data logger for motorsports / track days.
Built on the **Seeed XIAO nRF52840 Sense** (ARM Cortex-M4, 256 KB RAM, BLE 5.0, onboard LSM6DS3 IMU).

Core capabilities:
- 25 Hz GPS lap timing with sector support (DovesLapTimer library)
- **"Just Drive" auto-detection** via CourseManager (`ENABLE_NEW_UI`): automatic
  track proximity matching, course detection, and Lap Anything fallback
- RPM monitoring via inductive tachometer pickup
- Accelerometer logging (g-force X/Y/Z) via onboard LSM6DS3 IMU
- DOVEX data logging with reserved 4 KB header (crash-safe GPS data)
- Legacy CSV (`.dove`) logging still supported via `use_legacy_csv` setting
- 8+ display pages on a 128x64 OLED (3 Hz refresh)
- Bluetooth LE file download to companion apps / HackTheTrack.net
- On-device session replay: instant DOVEX header replay (new) or streamed
  DOVE/NMEA replay (legacy)

---

## File Map

### Source Files (Arduino sketch modules)

| File | Purpose |
|---|---|
| `BirdsEye.ino` | Entry point: globals, `setup()`, `loop()`, state machine, `ENABLE_NEW_UI` helpers |
| `project.h` | Shared types (`ButtonState`, `TrackLayout`, `ReplaySample`, `TrackManifestEntry`, `TrackMetadata`), debug macros, `MAX_*` constants |
| `display_config.h` | Display driver abstraction (SH110X vs SSD1306 toggle) |
| `display_ui.ino` | Display init, button reading (multi-sample debounce), menu navigation |
| `display_pages.ino` | All page rendering functions (`displayPage_*()`) |
| `gps_config.h` | GPS configuration constants (baud rate, nav rate, serial port) |
| `gps_functions.ino` | GPS init (SparkFun UBX PVT), time conversion, DOVEX/CSV logging pipeline |
| `sd_functions.ino` | SD init, track list/JSON parsing (dual format), track manifest, SD access arbitration |
| `accelerometer.ino` | LSM6DS3 IMU init and g-force reads (onboard XIAO Sense) |
| `tachometer.ino` | Falling-edge ISR on D0, EMA-filtered RPM calculation |
| `bluetooth.ino` | BLE service (file listing, transfer, status characteristics), auto-reboot on disconnect |
| `replay.ino` | Session replay: DOVEX header replay (new UI), DOVE/NMEA streamed replay (legacy) |
| `settings.ino` | Persistent JSON settings on SD (`/SETTINGS.json`), `getSetting()`/`setSetting()` |
| `images.h` | PROGMEM bitmap data (splash screen, animations) |

### Non-Source

| Path | Contents |
|---|---|
| `SDCARD/TRACKS/` | Example track JSON files |
| `CASE/` | 3D-printable enclosure STLs |
| `TACHOMETER/` | Tachometer circuit documentation |
| `libraries.txt` | Wokwi simulator library list |
| `README.md` | User-facing project documentation |

---

## Hardware & Pin Map

| Pin | Function | Detail |
|---|---|---|
| Serial1 RX/TX | GPS UART | u-blox SAM-M10Q, 115200 baud |
| I2C SDA/SCL | OLED display | 128x64, address 0x3C |
| I2C SDA/SCL | LSM6DS3 IMU | Onboard accelerometer/gyro (Sense variant), address 0x6A |
| SPI MOSI/SCK/MISO | SD card | 4 MHz SPI clock, CS grounded on PCB |
| D1 | Button 1 (Left) | INPUT_PULLUP, RC filter recommended |
| D2 | Button 2 (Select) | INPUT_PULLUP, RC filter recommended |
| D3 | Button 3 (Right) | INPUT_PULLUP, RC filter recommended |
| D0 | Tachometer input | INPUT_PULLUP, falling-edge ISR |
| PIN_VBAT / VBAT_ENABLE | Battery ADC | 1510/510 ohm divider, 3.6 V ref |

---

## Subsystem Architecture

### Main Loop Flow (`loop()`)

**Legacy mode** (without `ENABLE_NEW_UI`):
```
loop()  ~250 Hz
 ├─ GPS_LOOP()              checkUblox, checkCallbacks, feed DovesLapTimer, log CSV
 ├─ TACH_LOOP()             re-enable ISR after debounce, apply EMA filter
 ├─ ACCEL_LOOP()            read LSM6DS3 accelerometer X/Y/Z (g-force)
 ├─ BLUETOOTH_LOOP()        stream file chunks if transfer active
 ├─ checkForNewLapData()    append completed laps to lapHistory[]
 ├─ calculateGPSFrameRate() 1-second PVT counter
 ├─ readButtons()           multi-sample debounce + edge detection
 ├─ displayLoop()           render current page at 3 Hz
 └─ resetButtons()          clear pressed flags
```

**New UI mode** (with `ENABLE_NEW_UI`):
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
- `GPS_SETUP()` configures module via VALSET API: 115200 baud, 25 Hz nav,
  GPS-only constellation, automotive dynamic model, PVT callback registered.
- `GPS_LOOP()` calls `checkUblox()` + `checkCallbacks()`. The registered
  `onPVTReceived()` callback fires with the full `UBX_NAV_PVT_data_t` struct,
  populates `gpsData`, and sets `gpsDataFresh` flag for downstream processing.
- PVT data is cached in `gpsData` struct (GpsData) for access by display
  pages and other subsystems.
- **Legacy mode**: Feeds lat/lng/alt/speed into `DovesLapTimer.loop()` for
  line-crossing and sector detection.
- **New UI mode**: Feeds into `CourseManager.loop()` which handles course
  detection, Lap Anything fallback, and sector timing internally.
- Logs validated data rows to SD (9-check validation pipeline).
  - **DOVEX** (new UI default): reserved 4 KB header + CSV data after byte 4096.
  - **Legacy CSV** (`.dove`): flat CSV with track/layout/direction in filename.
- Time helpers: `getGpsTimeInMilliseconds()`, `getGpsUnixTimestampMillis()`.
- 64-bit timestamps are manually converted to strings (Arduino lacks `%llu`).

### 2. Tachometer (`tachometer.ino`)

- ISR `TACH_COUNT_PULSE()` fires on falling edge of D0.
- 3 ms minimum pulse gap (supports up to ~20 000 RPM).
- Uses volatile flag gating instead of `noInterrupts()` to avoid deadlock.
- `TACH_LOOP()` applies EMA filter (alpha 0.20) and updates at 3 Hz.
- 500 ms timeout sets RPM to 0 (engine stopped).

### 3. Accelerometer (`accelerometer.ino`)

- Onboard LSM6DS3 6-axis IMU on XIAO nRF52840 Sense (I2C address 0x6A).
- Shares I2C bus with OLED display (0x3C) — different addresses, no conflict.
- `ACCEL_SETUP()` initializes IMU; sets `accelAvailable` flag. Graceful
  degradation if IMU not present (non-Sense board).
- `ACCEL_LOOP()` reads `readFloatAccelX/Y/Z()` into global floats every
  main loop iteration (~250 Hz). Values in g-force (1g = 9.81 m/s²).
- No filtering — raw g-force is the standard unit for motorsports data.

### 4. SD Card & Logging (`sd_functions.ino`)

- SdFat library, FAT16/32, 4 MHz SPI.
- Track files live under `/TRACKS/*.json` (ArduinoJson 6 parsing).
- **Dual JSON format**: `parseTrackFile()` auto-detects root type:
  - **Object** (new HackTheTrack format): `longName`, `shortName`,
    `defaultCourse`, `courses[]` with `lengthFt`.
  - **Array** (legacy): bare array of course objects, metadata derived from
    filename, `lengthFt = 0`.
- **Track manifest** (`ENABLE_NEW_UI`): `buildTrackList()` also builds an
  in-RAM `trackManifest[]` (up to 1000 entries) with first lat/lon per track
  for haversine proximity matching. ~48 KB RAM.
- **SD access arbitration** prevents concurrent access:
  - `acquireSDAccess(mode)` / `releaseSDAccess(mode)`
  - Modes: `SD_ACCESS_NONE` (0), `LOGGING` (1), `REPLAY` (2),
    `BLE_TRANSFER` (3), `TRACK_PARSE` (4).
- Data flushes every 10 seconds during logging.

### 5. Display & UI (`display_ui.ino`, `display_pages.ino`, `display_config.h`)

- Driver selected at compile time (`USE_1306_DISPLAY` define).
- Button debounce: 3 samples at 500 us intervals, 200 ms refire lockout.
- Pages are integer constants; key pages:
  - Boot/menu: `PAGE_BOOT` (999), `PAGE_MAIN_MENU` (-1),
    `PAGE_SELECT_LOCATION` (0), `PAGE_SELECT_TRACK` (1),
    `PAGE_SELECT_DIRECTION` (2).
  - Racing: `GPS_STATS` (4) through `LOGGING_STOP` (12).
  - Replay: `PAGE_REPLAY_FILE_SELECT` (-3), `PAGE_REPLAY_DETECTING` (-4),
    `PAGE_REPLAY_RESULTS` (-8).
  - BLE: `PAGE_BLUETOOTH` (-2).
  - Errors: `PAGE_INTERNAL_WARNING` (100), `PAGE_INTERNAL_FAULT` (105).

### 6. Bluetooth (`bluetooth.ino`)

- BLE service UUID `0x1820`.
- Characteristics: file list (0x2A3D), file request (0x2A3E),
  file data (0x2A3F), file status (0x2A40).
- MTU negotiation (requests 247, default 23).
- File listing does not require exclusive SD access; transfer does.
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
  - Upload uses a 2048-byte static RAM buffer; `TERR:TOO_LARGE` if exceeded.
  - Error responses: `TERR:SD_BUSY`, `TERR:BUSY`, `TERR:WRITE_FAIL`, `TERR:NO_FILE`.
  - Upload state machine: BLE callback accumulates data + sets flags,
    `BLUETOOTH_LOOP()` sends `TREADY` and calls `processTrackUpload()`
    for thread-safe SD writes. Calls `buildTrackList()` after successful upload.
- **Auto-reboot on BLE disconnect**: `bleDisconnectCallback()` calls
  `NVIC_SystemReset()` after 100 ms delay, ensuring new settings take
  effect without manual power cycle.

### 7. Replay (`replay.ino`)

- **DOVEX replay** (`ENABLE_NEW_UI`): instant header-only replay.
  `parseDovexHeader()` reads line 1 (metadata) and line 2 (lap times CSV)
  from the first 4 KB. Populates `dovexReplay*` globals and `lapHistory[]`.
  No file streaming needed — jumps straight to results page.
  Only `.dovex` files shown in file browser.
- **Legacy replay** (without `ENABLE_NEW_UI`): reads `.dove`/`.nmea` files.
  Auto-detects track via haversine distance (< 20 miles threshold).
  Streams file line-by-line (128-byte buffer) through DovesLapTimer.
  Shows max speed, max RPM, lap times, and optimal lap.

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
  2048-byte buffer.
- Total RAM cost: ~1 KB (512-byte file buffer + 512-byte JSON document).

### 9. CourseManager Integration (`ENABLE_NEW_UI`)

- **Compile-time flag**: `#define ENABLE_NEW_UI` in `BirdsEye.ino`. When
  not defined, the entire legacy flow (manual track/course/direction
  selection) is preserved unchanged.
- **CourseManager** (`courseManager` global pointer): created when a track
  is detected via haversine proximity match, or with `courseCount=0` for
  immediate Lap Anything activation.
- **Track detection flow** (`trackDetectionLoop()`):
  1. GPS fix acquired → DOVEX logging starts immediately.
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

---

## Data Formats

### DOVEX Log (`.dovex` files) — New UI default

```
datetime,driver_name,course_name,short_name,best_lap_ms,optimal_lap_ms
lap1_ms,lap2_ms,lap3_ms,...
\n padding to byte 4096
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm,accel_x,accel_y,accel_z
1710512400123,12,0.8,35.12345678,-97.12345678,65.32,234.56,...
```

- **Reserved header** (bytes 0–4095): Line 1 = session metadata, Line 2 =
  all lap times (comma-separated ms values), padded with `\n` to 4096 bytes.
- **GPS data** (byte 4096+): CSV column header then streaming GPS rows.
- **Crash safety**: file created with `seekSet(4096)` before any data.
  Header written on session end. If header is empty (crash), GPS data
  after 4096 is still valid.
- **Filename**: `20YYMMDD_HHMM.dovex`
- 4 KB handles ~450 laps (8 chars per lap time). Extremely unlikely to exceed.

### CSV Log Row (`.dove` files) — Legacy format

```
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm,accel_x,accel_y,accel_z
```

- `timestamp`: Unix epoch milliseconds (uint64, written as string).
- `lat`/`lng`: 8 decimal places (~1.1 mm precision).
- `speed_mph`, `altitude_m`: 2 decimal places.
- `heading_deg`: heading of motion in degrees (0–360), 2 decimals. From UBX `headMot`.
- `h_acc_m`: horizontal accuracy estimate in meters, 2 decimals. From UBX `hAcc`.
- `accel_x`/`accel_y`/`accel_z`: accelerometer g-force, 3 decimal places. From LSM6DS3.
  Logs `0.000` if IMU not available (non-Sense board).
- **Filename**: `{location}_{track}_{dir}_20{yy}_{mmdd}_{hhmm}.dove`
- Selected when `use_legacy_csv` setting is `true`.

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

**Legacy format** (bare array, still supported):
```json
[
  {
    "name": "Full Course",
    "start_a_lat": 28.41270817,
    ...
  }
]
```

Auto-detected by JSON root type (object vs array). Legacy format sets
`lengthFt = 0` for all courses, which means CourseDetector cannot rank
by distance — CourseManager falls back to Lap Anything immediately.

Stored in `trackLayouts[MAX_LAYOUTS]` (max 10 per track).

### Settings JSON (`/SETTINGS.json`)

```json
{
  "bluetooth_name": "DovesDataLogger-042",
  "bluetooth_pin": "7391",
  "driver_name": "Driver",
  "lap_detection_distance": "7",
  "waypoint_detection_distance": "30",
  "use_legacy_csv": "false",
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
| `use_legacy_csv` | bool | `false` | When true, save `.dove` instead of `.dovex` |
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
| GPS baud | 115 200 | `gps_config.h` |
| GPS nav rate | 25 Hz | `gps_config.h` |
| Crossing threshold | 7.0 m | `BirdsEye.ino` |
| Max laps/session | 1 000 | `BirdsEye.ino` |
| Max locations | 1 000 | `project.h` |
| Max layouts/track | 10 | `project.h` |
| Max replay files | 20 | `replay.ino` |
| DOVEX header size | 4 096 bytes | `gps_functions.ino` |
| Auto-idle timeout | 60 s at <2 mph | `BirdsEye.ino` |
| Track detect radius | 5 miles | `BirdsEye.ino` |
| Tach min pulse gap | 3 ms | `tachometer.ino` |
| Tach EMA alpha | 0.20 | `tachometer.ino` |
| Tach stop timeout | 500 ms | `tachometer.ino` |
| Display refresh | 3 Hz | `display_ui.ino` |
| Button debounce | 200 ms | `display_ui.ino` |
| SD SPI clock | 4 MHz | `sd_functions.ino` |
| Battery check interval | 5 s | `BirdsEye.ino` |
| BLE default MTU | 23 | `bluetooth.ino` |
| JSON buffer | 2048 (1024 on Wokwi) | `sd_functions.ino` |
| Settings JSON buffer | 512 | `settings.ino` |
| Settings file path | `/SETTINGS.json` | `settings.ino` |
| Track upload buffer | 2048 | `bluetooth.ino` |

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
   lockout, EMA filtering on RPM, 4 MHz SPI clock for SD stability.

---

## Build Notes

- **Board**: Seeed XIAO nRF52840 Sense (Arduino or PlatformIO).
- `project.h` must be included before other `.ino` modules so Arduino's
  auto-prototype generator sees custom types first.
- PROGMEM is used for bitmap images to save RAM.
- Avoid Arduino `String` in hot paths (heap fragmentation risk on 256 KB).
- SD chip-select is hardwired to GND; pass `-1` to SdFat.
- `#define WOKWI` enables simulator-specific tweaks (smaller JSON buffer).
- `#define ENABLE_NEW_UI` activates CourseManager integration, auto-detection,
  DOVEX logging, auto-race/auto-idle, and instant DOVEX replay. When not
  defined, the entire legacy flow is preserved unchanged.
- **CRITICAL: NEVER use `analogRead()` on any GPIO pin.** On the nRF52840,
  `analogRead()` permanently disables the digital input buffer on the target
  pin for the remainder of the session. Every analog-capable pin on the XIAO
  is also a critical digital function: A0=tach ISR, A1-A3=buttons, A4=SDA,
  A5=SCL. Use `micros()` or the hardware RNG for entropy instead.

---

## Development Conventions

- `.ino` files act as modules; Arduino IDE concatenates them alphabetically
  after the main sketch file.
- Each subsystem exposes `*_SETUP()` and `*_LOOP()` entry points called
  from `BirdsEye.ino`.
- SD access must go through `acquireSDAccess()` / `releaseSDAccess()`.
- GPS data validation (9 checks) must pass before any CSV row is written.
- Display pages are rendered by `displayPage_*()` functions routed via
  `currentPage` in `displayLoop()`.
- All new UI code is gated behind `#ifdef ENABLE_NEW_UI` / `#else` / `#endif`.
  Globals that must be visible across `.ino` files (e.g. `dovexReplay*`,
  `trackManifest[]`) are declared in `BirdsEye.ino` (first in concatenation
  order) inside the `ENABLE_NEW_UI` guard.
- Library includes that define return types used in auto-prototyped functions
  (`DovesLapTimer.h`, `CourseManager.h`) must be in the top include block
  of `BirdsEye.ino` (before Arduino generates prototypes).
