# BirdsEye - GPS Lap Timer & Data Logger

A high-precision GPS-based lap timer and data logger designed for motorsports and track day enthusiasts. Features 25Hz logging, sector timing, RPM monitoring via tachometer, and multiple customizable display pages.

<p align="center">
  <img src="realworld_1.jpg" />
</p>

## Features

### Core Functionality
- **25Hz GPS Logging** - High-frequency data capture straight to SD card
- **Accelerometer** - On-board 6-axis IMU when using Seeed XIAO nRF52840 Sense, +/-16g
- **RPM Monitoring** - Tachometer input with noise filtering for ignition systems
- **"Just Drive" Mode** - Automatic track detection, course detection, and lap timing — no manual selection needed (`ENABLE_NEW_UI`)
- **Track & Layout Selection** - Multiple tracks and configurations loaded from SD card (legacy mode)
- **Sector Timing** - Optional 2 and 3-sector support for detailed performance analysis
- **Lap Anything** - Automatic waypoint-based lap timing when no track files match or no sectors configured
- **Lap Timing** - Current lap, best lap, last lap, and optimal lap calculation
- **Pace Comparison** - Real-time pace difference vs. best lap
- **Lap History** - Session-based lap history (up to 1000 laps)
- **Speed Display** - Large, easy-to-read speed display
- **Sleep Mode** - Low-power sleep with display/GPS/IMU off (~2-4 mA), instant wake on button press or engine start
- **DOVEX Format** - Crash-safe logging with reserved header for instant replay
- **Simple CSV Format** - Legacy `.dove` data files with millisecond-precision timestamps
- **Review Data** - Instant replay of DOVEX session headers (new UI), or streamed replay of legacy logs (slower)

#### WebApp Features (no login)
- **Bluetooth Downloads** - Can now download files directly to [HackTheTrack.net](http://HackTheTrack.net)
- **Configure settings** - none of us want to fill in text with three buttons
- **Track Sync** - Update on-device track library via the webapp

#### To-Do
- **Update GPS Tray** - Matek now wants $50 per module, when they are $15 from digikey, doesn't require any supporting circuitry
- **External Sensors** - Add thermocouple sensor / m8 circle connector
- **Tachometer** - **I** have a working tachometer... it's not easy to replicate, I need to fix that
- **Pin Lock** - require pin to pull logs from device

### Display Pages
- GPS Statistics (battery, satellites, HDOP, logging status)
- Speed (with current lap number)
- Tachometer (RPM with max RPM tracking)
- Current Lap Time
- Pace Off Best Lap (visual indicators when beating best)
- Best Lap Time
- Optimal Lap (combined best sectors with lap numbers)
- Lap History (paginated list)
- GPS Debug (distance to line, crossing status, etc.)

### Track Configuration
- Loads track layouts from JSON files on SD card
- Support for forward/reverse direction selection
- Optional sector timing lines (2 and 3 sectors)
- Easy to add new tracks - just add a JSON file

## Hardware Requirements

### Core Components
- **MCU**: Seeed XIAO nRF52840 Sense (64MHz ARM Cortex-M4 with FPU + BLE 5.0 + 6-axis IMU)
  - ~120 mA active draw with 2.45" screen, ~2-4 mA sleep
  - Built-in battery charging circuit (BQ25101)
- **GPS**: u-blox SAM-M10Q (Matek SAM-M10Q recommended, found from most RC hobby shops)
  - Configured for 25Hz UBX binary (PVT) update rate
  - GPS-only constellation for maximum nav rate
  - **TODO** Update GPS tray to use bare $15 GPS module, battery backup is optional
- **Display**: 2.45" 128x64 OLED (SH110X or SSD1306 compatible)
  - I2C interface (address: 0x3C)
  - Software sleep/wake via I2C commands (~10uA when off)
- **SD Card**: Standard SD card module
  - FAT16/FAT32 formatted
  - 1 MHz SPI for EMI resistance
- **Battery**: 1500mAh 103050 LiPo
  - ~12.5 hours active, ~15-30 days in sleep mode
- **Buttons**: 3x momentary pushbuttons for navigation
  - Left, Select/Enter, Right
  - RC low-pass filters recommended (10K + 100nF) for EMI rejection

### Optional Components
- **Tachometer Circuit**: Inductive pickup for RPM sensing
  - Input on pin D0
  - Noise filtering for ignition systems
  - *Circuit diagram: `tachometer-circuit.jpg` (README coming soon)*
  - diagram is missing required optocoupler and isolated dc-dc power supply

### Power Usage
- Active draw: ~120 mA with 2.45" OLED display
- Active battery life: ~12.5 hours continuous on 1500 mAh
- Sleep draw: ~2-4 mA (display, GPS, IMU off)
- Sleep battery life: ~15-30 days on 1500 mAh
- Battery percentage and voltage displayed on stats page and charging screen

## Track Loading System

### How It Works

The system loads track configurations from JSON files stored on the SD card. Each track file can contain multiple layouts (e.g., "Full Course", "Short Course", "Chicane Bypass").

### SD Card Structure

```
SDCARD/
└── TRACKS/
    ├── OKC.json
    ├── BMP.json
    ├── PIQUET.json
    └── [your-track].json
```

### Track JSON Format

Two formats are supported. The device auto-detects which format is used.

**New format** (recommended, can use [HackTheTrack.net](http://HackTheTrack.net) to update tracks on the device):

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
      "start_a_lng": -81.3797326641803,
      "start_b_lat": 28.4127303867932,
      "start_b_lng": -81.3795704875378,
      "sector_2_a_lat": 28.4119049886871,
      "sector_2_a_lng": -81.3790708193926,
      "sector_2_b_lat": 28.4118316342961,
      "sector_2_b_lng": -81.3791856652217,
      "sector_3_a_lat": 28.4115010664104,
      "sector_3_a_lng": -81.3799856475317,
      "sector_3_b_lat": 28.4115084390461,
      "sector_3_b_lng": -81.3798064021136
    }
  ]
}
```

**Legacy format** (still supported):

```json
[
  {
    "name": "Full Course",
    "start_a_lat": 28.41270817,
    "start_a_lng": -81.37973266,
    "start_b_lat": 28.41273039,
    "start_b_lng": -81.37957049,
    "sector_2_a_lat": 28.41190499,
    "sector_2_a_lng": -81.37907082,
    "sector_2_b_lat": 28.41183163,
    "sector_2_b_lng": -81.37918567
  }
]
```

**New format fields:**
- `longName` / `shortName`: track display names
- `defaultCourse`: which course to prefer (used by CourseDetector)
- `courses[].lengthFt`: track length in feet (enables automatic course detection ranking)

**Coordinate Requirements:**
- **Start/Finish Line**: `start_a_lat`, `start_a_lng`, `start_b_lat`, `start_b_lng` (required)
- **Sector 2 Line**: `sector_2_a_lat`, `sector_2_a_lng`, `sector_2_b_lat`, `sector_2_b_lng` (optional)
- **Sector 3 Line**: `sector_3_a_lat`, `sector_3_a_lng`, `sector_3_b_lat`, `sector_3_b_lng` (optional)

Each line is defined by two GPS coordinate points (A and B). The system detects crossing when you pass through the line segment between these points.

**Getting Coordinates:**
Use [Google Maps](https://maps.google.com) or your preferred mapping tool:
1. Right-click on a point → Copy coordinates
2. Paste into JSON (format: latitude, longitude)
3. Precision: 8 decimal places recommended for racing accuracy

### Adding a New Track

1. Create a new `.json` file in `SDCARD/TRACKS/` directory
2. Use the format shown above
3. Filename should be short (max 8 characters for FAT16 compatibility)
4. Example: `LAGUNA.json`, `COTA.json`, `BRANDS.json`

The track will automatically appear in the track selection menu on next boot.

## Device Settings

Settings are stored in `/SETTINGS.json` on the SD card. The file is created automatically on first boot with random default values. Missing keys are auto-populated on boot when firmware is updated.

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

| Setting | Description | Default |
|---|---|---|
| `bluetooth_name` | BLE device name visible during pairing | Random (e.g. `DovesDataLogger-042`) |
| `bluetooth_pin` | PIN displayed on device for webapp pairing | Random 4-digit |
| `driver_name` | Driver name logged in DOVEX session header | `Driver` |
| `lap_detection_distance` | Crossing detection threshold in meters | `7` |
| `waypoint_detection_distance` | Waypoint proximity zone in meters (Lap Anything) | `30` |
| `use_legacy_csv` | Use legacy `.dove` CSV format instead of DOVEX | `false` |
| `waypoint_speed` | Minimum speed in mph to activate lap timing | `30` |

## Data Formats

### DOVEX Format (default)

DOVEX files (`.dovex`) use a reserved 8 KB header for session metadata, with GPS data streaming after byte 8192. This enables crash-safe logging and instant on-device replay.

**Structure:**
```
Bytes 0-8191:    Session header (written when session ends)
  Line 1:        datetime,driver,course,short_name,best_lap_ms,optimal_ms   (column labels)
  Line 2:        2025-03-11 14:30:00,Driver,Normal,OKC,62345,61890          (session metadata)
  Line 3:        laps_ms                                                     (column label)
  Line 4:        65432,63210,62345,64567,...                                 (all lap times in ms)
  Remaining:     \n padding to byte 8192

Bytes 8192+:     GPS data
  Header row:    timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm,accel_x,accel_y,accel_z
  Data rows:     1741128001234,12,0.8,28.41270817,-81.37973266,87.32,125.45,182.34,1.25,8450,0.123,-0.945,0.032
```

The 8 KB header can store ~1000 lap times. If the device loses power mid-session, the header will be empty but all GPS data after byte 8192 is still valid and recoverable.

**File naming:** `20YYMMDD_HHMM.dovex` (e.g. `20240115_1430.dovex`)

### Legacy CSV Format

When `use_legacy_csv` is set to `true`, data is logged in flat CSV format (`.dove` files).

```csv
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm,accel_x,accel_y,accel_z
1741128001234,12,0.8,28.41270817,-81.37973266,87.32,125.45,182.34,1.25,8450,0.123,-0.945,0.032
```

**File naming:** `[TRACK]_[LAYOUT]_[DIRECTION]_[YYYY]_[MMDD]_[HHMM].dove` (e.g. `OKC_Normal_fwd_2024_0115_1430.dove`)

### Column Reference

- **timestamp**: Unix timestamp in milliseconds (since Jan 1, 1970)
- **sats**: Number of GPS satellites
- **hdop**: Horizontal Dilution of Precision (GPS accuracy indicator)
- **lat/lng**: GPS coordinates (8 decimal places)
- **speed_mph**: Speed in miles per hour (2 decimal places)
- **altitude_m**: Altitude in meters (2 decimal places)
- **heading_deg**: Heading of motion in degrees (0-360, 2 decimal places, from UBX headMot)
- **h_acc_m**: Horizontal accuracy estimate in meters (2 decimal places, from UBX hAcc)
- **rpm**: Engine RPM from tachometer input
- **accel_x/y/z**: Accelerometer g-force from onboard LSM6DS3 IMU (3 decimal places, logs `0.000` if IMU not available)

## File Structure

```
BirdsEye/
├── BirdsEye.ino              # Entry point: globals, setup(), loop(), state machine
├── project.h                 # Shared types, debug macros, constants
├── display_config.h          # Display driver abstraction (SH110X/SSD1306)
├── gps_config.h              # GPS configuration constants (baud, nav rate)
├── images.h                  # PROGMEM bitmap data (splash, animations)
│
├── display_ui.ino            # Display init, button handling, menu navigation
├── display_pages.ino         # All page rendering functions (displayPage_*())
├── gps_functions.ino         # GPS init, PVT callback, time conversion, logging
├── sd_functions.ino          # SD init, track JSON parsing, track manifest
├── accelerometer.ino         # LSM6DS3 IMU init and g-force reads
├── tachometer.ino            # Falling-edge ISR, EMA-filtered RPM
├── bluetooth.ino             # BLE service (file transfer, settings, track sync)
├── replay.ino                # Session replay (DOVEX header / legacy streaming)
├── settings.ino              # Persistent JSON settings (/SETTINGS.json)
│
├── CASE/                     # 3D printable enclosure files
│   └── *.STL
│
├── SDCARD/                   # Example SD card file structure
│   └── TRACKS/
│       ├── OKC.json
│       ├── BMP.json
│       └── ...
│
├── TACHOMETER/               # Tachometer circuit documentation
└── README.md                 # This file
```

## Required Libraries

| Library | Purpose |
|---|---|
| Adafruit GFX | Graphics primitives |
| Adafruit SSD1306 | SSD1306 OLED driver (if using SSD1306) |
| Adafruit SH110X | SH110X OLED driver (if using SH1106) |
| SparkFun u-blox GNSS v3 | UBX binary PVT GPS interface |
| ArduinoJson 6.x | Track file and settings JSON parsing |
| SdFat | SD card (FAT16/FAT32) |
| [DovesLapTimer](https://github.com/TheAngryRaven/DovesLapTimer) | Lap/sector timing |
| [CourseManager](https://github.com/TheAngryRaven/DovesLapTimer) | Auto course detection + Lap Anything (part of DovesLapTimer) |
| Seeed Arduino LSM6DS3 | Onboard IMU accelerometer/gyro (Sense variant) |
| Bluefruit nRF52 | BLE (built into Seeed nRF52 board package) |

**Board package:** Use "Seeed nRF52 Boards" (non-mbed). The mbed variant uses ArduinoBLE instead of Bluefruit and is incompatible.

## Setup & Usage

### Initial Setup

1. **Format SD Card**
   - Format as FAT32 (or FAT16 for maximum compatibility)
   - Create folder structure: `TRACKS/` in root

2. **Add Track Files**
   - Copy or create `.json` track files in `SDCARD/TRACKS/`
   - See "Track Loading System" section above

3. **Flash Firmware**
   - Install "Seeed nRF52 Boards" board package (non-mbed)
   - Install required libraries (see table above)
   - Select board: "Seeed XIAO nRF52840 Sense"
   - Compile and upload `BirdsEye.ino`

4. **Hardware Assembly**
   - *3D printed case assembly instructions coming soon*
   - *Tachometer circuit README coming soon*
   - STL files available in `CASE/` directory
   - Note: Requires some glue, measuring, and jeweler's screws

### Usage

**Button Controls:**
- **Left/Right Buttons**: Navigate between pages
- **Middle Button**: Select/Enter (in menus), Quick-jump to Pace/Best Lap (while racing)
- **Hold Left + Right (5s)**: Enter sleep mode (from main menu)
- **Hold Select + Side (5s)**: Reboot device (from any page)

**Startup Sequence (New UI / "Just Drive" mode):**
1. Power on → Boot screen → Main menu
2. Start driving — device auto-enters race mode at 10+ mph or 500+ RPM
3. GPS fix acquired → DOVEX logging starts immediately
4. Track auto-detected via GPS proximity → course detection begins
5. Lap timing activates automatically (sector timing if configured, "Lap Anything" otherwise)

**Startup Sequence (Legacy mode):**
1. Power on → Boot screen
2. Select Track location
3. Select Track layout
4. Select Direction (Forward/Reverse)
5. Logging begins automatically when GPS fix is acquired

**Ending Session:**
- **Auto-idle** (new UI): stops automatically after 60 seconds below 2 mph
- **Manual**: navigate to "END RACE" page (only accessible when speed < 2 mph), confirm to stop
- Returns to main menu

**Sleep Mode** (new UI):
- Activates via 5-second left+right hold on main menu, or automatically after 5 minutes idle on main menu
- Turns off display, GPS (backup mode), and IMU — draws ~2-4 mA vs ~120 mA active
- Wakes instantly on any button press or tachometer pulse (engine start)
- GPS retains ephemeris in backup RAM for fast warm-start on wake (seconds, not minutes)
- Daily GPS wake keeps ephemeris fresh (~2 min every 24 hours)
- Shows charging screen with battery percentage when USB is plugged in during sleep
- Estimated sleep runtime: ~15-30 days on 1500 mAh battery

## Technical Details

### GPS Configuration
- **Protocol**: UBX binary (PVT messages via SparkFun callback)
- **Update Rate**: 25Hz (40ms between fixes)
- **Mode**: Automotive dynamic model
- **Constellations**: GPS-only (max nav rate, no multi-constellation overhead)
- **Baud Rate**: 57600 (auto-configured from 9600 default on first boot)

### SD Card Logging
- **Write Frequency**: Every PVT update (~25Hz)
- **Flush Interval**: Every 10 seconds (prevents data loss)
- **SPI Speed**: 1 MHz (reduced for EMI resistance)
- **Access Arbitration**: Mutex prevents concurrent access from logging, replay, BLE, and track parsing

### Tachometer Input
- **Input Pin**: D0
- **Detection**: Falling-edge interrupt
- **Filtering**: 3ms minimum pulse gap (supports up to ~20,000 RPM)
- **Dead Time**: Volatile flag gating prevents interrupt storms from noisy ignition pickups
- **Update Rate**: 3Hz with EMA filter (alpha 0.20)
- **Timeout**: 500ms with no pulse = engine stopped (RPM 0)
- **Configuration**: 1 pulse per revolution (adjust `tachRevsPerPulse` for multi-cylinder)

### Display Update Rate
- **Refresh**: 3Hz (reduces power consumption)
- **Exception**: Instant refresh on button press

### Battery Monitoring
- **Update Interval**: 5 seconds
- **Voltage Divider**: 1510Ω / 510Ω ratio
- **Display**: Percentage (3.3V = 0%, 4.2V = 100%) with voltage readout

## Adding Custom Data Pages

The page system is straightforward to extend. Each page needs:

1. **Page Constant** in `BirdsEye.ino` (in the running page section):
   ```cpp
   const int MY_NEW_PAGE = 13;
   ```

2. **Display Function** in `display_pages.ino`:
   ```cpp
   void displayPage_my_new_page() {
     resetDisplay();
     display.println(F("My Custom Page"));
     // Your display code here
     safeDisplayUpdate();
   }
   ```

3. **Page Routing** in `displayLoop()` in `display_ui.ino`:
   ```cpp
   else if (currentPage == MY_NEW_PAGE) {
     displayPage_my_new_page();
   }
   ```

4. **Update Page Range** in `BirdsEye.ino`:
   ```cpp
   int runningPageEnd = MY_NEW_PAGE; // Update to new last page
   ```

Look at existing pages like `displayPage_gps_speed()` or `displayPage_tachometer()` in `display_pages.ino` as examples.

## Related Projects

- **Data Viewer**: [DovesDataViewer](https://github.com/TheAngryRaven/DovesDataViewer)
  - Web-based viewer for logged data
  - Preview at [HackTheTrack.net](http://HackTheTrack.net)

- **Core GPS Timing Library**: [DovesLapTimer](https://github.com/TheAngryRaven/DovesLapTimer)
  - Line-crossing detection
  - Sector timing
  - Optimal lap calculation

## Future Enhancements

- Pin lock - require PIN to pull logs from device
- Additional sensor inputs (exhaust temp, water temp)
- WiFi automatic data transfer
- Real-time telemetry to pit crew

## Notes

- Crossing detection threshold: configurable via `lap_detection_distance` setting (default 7m)
- Maximum track locations: 1000
- Maximum layouts per track: 10
- Maximum lap history: 1000 laps per session
- GPS coordinates stored with 8 decimal places (~1.1mm precision)
- `ENABLE_NEW_UI` compile flag controls new vs legacy behavior (defined in `BirdsEye.ino`)
- BLE disconnect triggers automatic device reboot (ensures settings changes take effect)

## License

See LICENSE file for details.

## Support

For issues, questions, or contributions, please visit:
- GitHub Issues: [https://github.com/TheAngryRaven/DovesDataLogger/issues](https://github.com/TheAngryRaven/DovesDataLogger/issues)


