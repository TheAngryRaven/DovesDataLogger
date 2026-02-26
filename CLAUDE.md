# BirdsEye - Project Guide

> **MAINTAINERS: Keep this file updated when adding/removing files, changing pin
> assignments, modifying subsystem interfaces, or altering the build configuration.
> This file is loaded into Claude's context window on every session and must
> accurately reflect the current state of the project.**

## What Is BirdsEye?

A high-precision GPS lap timer and data logger for motorsports / track days.
Built on the **Seeed XIAO nRF52840** (ARM Cortex-M4, 256 KB RAM, BLE 5.0).

Core capabilities:
- 25 Hz GPS lap timing with sector support (DovesLapTimer library)
- RPM monitoring via inductive tachometer pickup
- CSV data logging to SD card at full GPS rate
- 8+ display pages on a 128x64 OLED (3 Hz refresh)
- Bluetooth LE file download to companion apps / HackTheTrack.net
- On-device session replay with automatic track detection

---

## File Map

### Source Files (Arduino sketch modules)

| File | Purpose |
|---|---|
| `BirdsEye.ino` | Entry point: globals, `setup()`, `loop()`, state machine |
| `project.h` | Shared types (`ButtonState`, `TrackLayout`, `ReplaySample`), debug macros |
| `display_config.h` | Display driver abstraction (SH110X vs SSD1306 toggle) |
| `display_ui.ino` | Display init, button reading (multi-sample debounce), menu navigation |
| `display_pages.ino` | All page rendering functions (`displayPage_*()`) |
| `gps_config.h` | GPS configuration constants (baud rate, nav rate, serial port) |
| `gps_functions.ino` | GPS init (SparkFun UBX PVT), time conversion, SD logging pipeline |
| `sd_functions.ino` | SD init, track list/JSON parsing, SD access arbitration |
| `tachometer.ino` | Falling-edge ISR on D0, EMA-filtered RPM calculation |
| `bluetooth.ino` | BLE service (file listing, transfer, status characteristics) |
| `replay.ino` | Session replay: file parsing (DOVE/NMEA), track auto-detection |
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
| SPI MOSI/SCK/MISO | SD card | 4 MHz SPI clock, CS grounded on PCB |
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
 ├─ GPS_LOOP()          checkUblox, checkCallbacks, feed DovesLapTimer, log CSV
 ├─ TACH_LOOP()         re-enable ISR after debounce, apply EMA filter
 ├─ BLUETOOTH_LOOP()    stream file chunks if transfer active
 ├─ checkForNewLapData()  append completed laps to lapHistory[]
 ├─ calculateGPSFrameRate()  1-second PVT counter
 ├─ readButtons()       multi-sample debounce + edge detection
 ├─ displayLoop()       render current page at 3 Hz
 └─ resetButtons()      clear pressed flags
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
- Feeds lat/lng/alt/speed into `DovesLapTimer.loop()` for line-crossing
  and sector detection.
- Logs validated CSV rows to SD (9-check validation pipeline).
- Time helpers: `getGpsTimeInMilliseconds()`, `getGpsUnixTimestampMillis()`.
- 64-bit timestamps are manually converted to strings (Arduino lacks `%llu`).

### 2. Tachometer (`tachometer.ino`)

- ISR `TACH_COUNT_PULSE()` fires on falling edge of D0.
- 3 ms minimum pulse gap (supports up to ~20 000 RPM).
- Uses volatile flag gating instead of `noInterrupts()` to avoid deadlock.
- `TACH_LOOP()` applies EMA filter (alpha 0.20) and updates at 3 Hz.
- 500 ms timeout sets RPM to 0 (engine stopped).

### 3. SD Card & Logging (`sd_functions.ino`)

- SdFat library, FAT16/32, 4 MHz SPI.
- Track files live under `/TRACKS/*.json` (ArduinoJson 6 parsing).
- **SD access arbitration** prevents concurrent access:
  - `acquireSDAccess(mode)` / `releaseSDAccess(mode)`
  - Modes: `SD_ACCESS_NONE` (0), `LOGGING` (1), `REPLAY` (2),
    `BLE_TRANSFER` (3), `TRACK_PARSE` (4).
- Data flushes every 10 seconds during logging.

### 4. Display & UI (`display_ui.ino`, `display_pages.ino`, `display_config.h`)

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

### 5. Bluetooth (`bluetooth.ino`)

- BLE service UUID `0x1820`.
- Characteristics: file list (0x2A3D), file request (0x2A3E),
  file data (0x2A3F), file status (0x2A40).
- MTU negotiation (requests 247, default 23).
- File listing does not require exclusive SD access; transfer does.

### 6. Replay (`replay.ino`)

- Reads `.dove` (CSV) and `.nmea` files from SD root.
- Auto-detects track via haversine distance (< 20 miles threshold).
- Streams file line-by-line (128-byte buffer) through DovesLapTimer.
- Shows max speed, max RPM, lap times, and optimal lap.

### 7. Settings (`settings.ino`)

- Persistent JSON key-value store at `/SETTINGS.json` on SD card.
- `SETTINGS_SETUP()` called once from `setup()` after SD init; creates
  default file on first boot (random BLE name + PIN).
- `getSetting(key, buf, bufSize)` reads a value into a caller-provided
  buffer. Returns `true` if found, `false` on any failure (buf set empty).
  Always reads fresh from disk (no cache).
- `setSetting(key, value)` does read-modify-write to update a single key.
- Uses `SD_ACCESS_TRACK_PARSE` mode for brief SD access.
- Separate `StaticJsonDocument<512>` — does not share the track parser's
  2048-byte buffer.
- Total RAM cost: ~1 KB (512-byte file buffer + 512-byte JSON document).

---

## Data Formats

### CSV Log Row (`.dove` files)

```
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm
```

- `timestamp`: Unix epoch milliseconds (uint64, written as string).
- `lat`/`lng`: 8 decimal places (~1.1 mm precision).
- `speed_mph`, `altitude_m`: 2 decimal places.
- `heading_deg`: heading of motion in degrees (0–360), 2 decimals. From UBX `headMot`.
- `h_acc_m`: horizontal accuracy estimate in meters, 2 decimals. From UBX `hAcc`.

### Track JSON (`/TRACKS/*.json`)

Array of layout objects, each with a start/finish line (two GPS points)
and optional sector lines. Stored in `trackLayouts[MAX_LAYOUTS]` (max 10).

### Settings JSON (`/SETTINGS.json`)

```json
{
  "bluetooth_name": "DovesDataLogger-042",
  "bluetooth_pin": "7391"
}
```

- Created automatically on first boot with random values.
- Editable on a computer — changes take effect on next reboot.
- Read on-demand via `getSetting()`, written via `setSetting()`.

---

## Key Constants

| Constant | Value | Location |
|---|---|---|
| GPS baud | 115 200 | `gps_config.h` |
| GPS nav rate | 25 Hz | `gps_config.h` |
| Crossing threshold | 7.0 m | `BirdsEye.ino` |
| Max laps/session | 1 000 | `BirdsEye.ino` |
| Max locations | 100 | `BirdsEye.ino` |
| Max layouts/track | 10 | `BirdsEye.ino` |
| Max replay files | 20 | `replay.ino` |
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

- **Board**: Seeed XIAO nRF52840 (Arduino or PlatformIO).
- `project.h` must be included before other `.ino` modules so Arduino's
  auto-prototype generator sees custom types first.
- PROGMEM is used for bitmap images to save RAM.
- Avoid Arduino `String` in hot paths (heap fragmentation risk on 256 KB).
- SD chip-select is hardwired to GND; pass `-1` to SdFat.
- `#define WOKWI` enables simulator-specific tweaks (smaller JSON buffer).

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
