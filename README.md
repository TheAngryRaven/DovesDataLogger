# BirdsEye - GPS Lap Timer & Data Logger

A high-precision GPS-based lap timer and data logger designed for motorsports and track day enthusiasts. Features 25Hz logging, sector timing, RPM monitoring via tachometer, and multiple customizable display pages.

<p align="center">
  <img src="sample.gif" />
</p>

## Features

### Core Functionality
- **25Hz GPS Logging** - High-frequency data capture straight to SD card
- **Track & Layout Selection** - Multiple tracks and configurations loaded from SD card
- **Sector Timing** - Optional 2 and 3-sector support for detailed performance analysis
- **Lap Timing** - Current lap, best lap, last lap, and optimal lap calculation
- **Pace Comparison** - Real-time pace difference vs. best lap
- **Lap History** - Session-based lap history (up to 1000 laps)
- **RPM Monitoring** - Tachometer input with noise filtering for ignition systems
- **Speed Display** - Large, easy-to-read speed display
- **Simple CSV Format** - Easy-to-parse data files with millisecond-precision timestamps
#### Experimental Features
- **Bluetooth Downloads** - Can now download files directly to [HackTheTrack.net](http://HackTheTrack.net)
  - (fast and heckin cool)
- **Review Data** - Can now "replay" old logs on-device to see laptimes/optimals/etc (slow)
  - (kind of slow but it works)

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
- **MCU**: Seeed XIAO nRF52840 (64MHz ARM Cortex-M4 with FPU + Bluetooth)
  - Low power consumption: ~120mAh with 2.45" screen
  - Built-in battery charging circuit
  - *Note: May switch to Seeed XIAO ESP32 (same form factor) for WiFi data transfer at cost of higher power usage*
- **GPS**: u-blox SAM-M10Q (Matek SAM-M10Q-CAN recommended)
  - Configured for 25Hz update rate
  - GPS-only mode for maximum performance
- **Display**: 2.45" 128x64 OLED LCD (SH110X or SSD1306 compatible)
  - I2C interface (address: 0x3C)
- **SD Card**: Standard SD card module
  - FAT16/FAT32 formatted
  - 4MHz SPI for EMI resistance
- **Battery**: 1500mAh 103050 LiPo
  - ~13 hours runtime on full charge
- **Buttons**: 3x momentary pushbuttons for navigation
  - Up/Left, Select/Enter, Down/Right

### Optional Components
- **Tachometer Circuit**: Inductive pickup for RPM sensing
  - Input on pin D0
  - Noise filtering for ignition systems
  - *Circuit diagram: `tachometer-circuit.jpg` (README coming soon)*

### Power Usage
- Current draw: ~120mAh with 2.45" OLED display
- Battery life: ~13 hours continuous operation
- Voltage monitoring with visual battery indicator

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

Each track file contains an array of layouts. Each layout defines a start/finish line and optional sector lines using GPS coordinates:

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
    "sector_2_b_lng": -81.37918567,

    "sector_3_a_lat": 28.41150107,
    "sector_3_a_lng": -81.37998565,
    "sector_3_b_lat": 28.41150844,
    "sector_3_b_lng": -81.37980640
  },
  {
    "name": "Short Course",
    "start_a_lat": 28.41199350,
    "start_a_lng": -81.37995885,
    "start_b_lat": 28.41199385,
    "start_b_lng": -81.37986484
  }
]
```

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

## CSV Data Format

Data is logged in simple CSV format with the following columns:

```
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm
```

**Example:**
```csv
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm
1704724801234,12,0.8,28.41270817,-81.37973266,87.32,125.45,182.34,1.25,8450
1704724801274,12,0.8,28.41270821,-81.37973270,87.45,125.46,182.50,1.22,8475
```

- **timestamp**: Unix timestamp in milliseconds (since Jan 1, 1970)
- **sats**: Number of GPS satellites
- **hdop**: Horizontal Dilution of Precision (GPS accuracy indicator)
- **lat/lng**: GPS coordinates (8 decimal places)
- **speed_mph**: Speed in miles per hour (2 decimal places)
- **altitude_m**: Altitude in meters (2 decimal places)
- **heading_deg**: Heading of motion in degrees (0-360, 2 decimal places, from UBX headMot)
- **h_acc_m**: Horizontal accuracy estimate in meters (2 decimal places, from UBX hAcc)
- **rpm**: Engine RPM from tachometer input

**Log File Naming:**
`[TRACK]_[LAYOUT]_[DIRECTION]_[YYYY]_[MMDD]_[HHMM].dove`

Example: `OKC_Normal_fwd_2024_0115_1430.dove`

## File Structure

```
DovesDataLogger/
├── BirdsEye.ino              # Main Arduino sketch
├── display_config.h          # Display configuration (SH110X/SSD1306)
├── gps_config.h              # u-blox GPS UBX configuration commands
├── images.h                  # Bitmap images for display
├── libraries.txt             # Required Arduino libraries
├── diagram.json              # Wokwi simulator configuration
│
├── CASE/                     # 3D printable enclosure files
│   ├── *.STL                 # STL files for 3D printing
│   └── README.md             # Assembly instructions (coming soon)
│
├── SDCARD/                   # SD card file structure
│   └── TRACKS/               # Track configuration files
│       ├── OKC.json
│       ├── BMP.json
│       └── ...
│
├── tachometer-circuit.jpg    # Tachometer circuit schematic
└── README.md                 # This file
```

### Key Files

**BirdsEye.ino** (2020 lines)
- Main application logic
- GPS parsing and configuration
- Lap timing integration with [DovesLapTimer library](https://github.com/TheAngryRaven/DovesLapTimer)
- SD card logging with EMI-resistant retry logic
- Tachometer input with noise filtering
- Multi-page display system
- Button input handling
- Battery voltage monitoring

**display_config.h**
- Abstraction layer for SH110X and SSD1306 displays
- Screen dimensions and I2C configuration
- Easy to swap display types

**gps_config.h**
- u-blox UBX binary configuration commands
- NMEA sentence filtering
- Update rate configuration (5/10/18/20/25 Hz)
- Constellation selection (GPS-only for best performance)

**images.h**
- Bitmap graphics for splash screen and crossing animations

## Required Arduino Libraries

Install these libraries via Arduino Library Manager:

- **Adafruit GFX Library** - Graphics primitives
- **Adafruit SSD1306** - SSD1306 OLED driver (if using SSD1306)
- **Adafruit SH110X** - SH110X OLED driver (if using SH1106)
- **Adafruit GPS Library** - GPS parsing
- **ArduinoJson** - JSON parsing for track files
- **SdFat** - SD card library (use v1.x for FAT16 support)
- **DovesLapTimer** (v0.2.1+) - Core GPS timing library
  - [https://github.com/TheAngryRaven/DovesLapTimer](https://github.com/TheAngryRaven/DovesLapTimer)

## Setup & Usage

### Initial Setup

1. **Format SD Card**
   - Format as FAT32 (or FAT16 for maximum compatibility)
   - Create folder structure: `TRACKS/` in root

2. **Add Track Files**
   - Copy or create `.json` track files in `SDCARD/TRACKS/`
   - See "Track Loading System" section above

3. **Flash Firmware**
   - Install required libraries
   - Select board: "Seeed XIAO nRF52840"
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

**Startup Sequence:**
1. Power on → Boot screen
2. Select Track location
3. Select Track layout
4. Select Direction (Forward/Reverse)
5. Logging begins automatically when GPS fix is acquired

**Ending Session:**
- Navigate to "END RACE" page (only accessible when speed < 2 mph)
- Confirm to stop logging and close file
- Returns to track selection

## Technical Details

### GPS Configuration
- **Update Rate**: 25Hz (40ms between fixes)
- **Mode**: Automotive navigation mode
- **Constellations**: GPS-only (reduces processing time)
- **NMEA Sentences**: GGA only (reduces serial bandwidth)
- **Baud Rate**: 115200

### SD Card Logging
- **Write Frequency**: Every GGA packet (~25Hz)
- **Flush Interval**: Every 2 seconds (prevents data loss)
- **SPI Speed**: 4MHz (reduced from 25MHz for EMI resistance)
- **Retry Logic**: 3 attempts on initialization failures

### Tachometer Input
- **Input Pin**: D0
- **Detection**: Rising edge interrupt
- **Filtering**: 2ms minimum pulse gap (noise rejection)
- **Dead Time**: Prevents interrupt storms from noisy ignition pickups
- **Update Rate**: 3Hz filtered display
- **Configuration**: 1 pulse per revolution (adjust `tachRevsPerPulse` for multi-cylinder)

### Display Update Rate
- **Refresh**: 3Hz (reduces power consumption)
- **Exception**: Instant refresh on button press

### Battery Monitoring
- **Update Interval**: 5 seconds
- **Voltage Divider**: 1510Ω / 510Ω ratio
- **Display**: 5-segment battery indicator

## Adding Custom Data Pages

The page system is straightforward to extend. Each page needs:

1. **Page Constant** (around line 650):
   ```cpp
   const int MY_NEW_PAGE = 13;
   ```

2. **Display Function**:
   ```cpp
   void displayPage_my_new_page() {
     resetDisplay();
     display.println(F("My Custom Page"));
     // Your display code here
     display.display();
   }
   ```

3. **Page Routing** in `displayLoop()` (around line 1650):
   ```cpp
   else if (currentPage == MY_NEW_PAGE) {
     displayPage_my_new_page();
   }
   ```

4. **Update Page Range** (line 697):
   ```cpp
   int runningPageEnd = MY_NEW_PAGE; // Update to new last page
   ```

Look at existing pages like `displayPage_gps_speed()` (line 992) or `displayPage_tachometer()` (line 1196) as examples.

## Related Projects

- **Data Viewer**: [DovesDataViewer](https://github.com/TheAngryRaven/DovesDataViewer)
  - Web-based viewer for logged data
  - Preview at [HackTheTrack.net](http://HackTheTrack.net)

- **Core GPS Timing Library**: [DovesLapTimer](https://github.com/TheAngryRaven/DovesLapTimer)
  - Line-crossing detection
  - Sector timing
  - Optimal lap calculation

## Future Enhancements

- WiFi data transfer (if switching to ESP32 variant)
- Additional sensor inputs (exhaust temp, water temp)
- Bluetooth data streaming
- Real-time telemetry to pit crew

## Notes

- Crossing detection threshold: 7.0 meters (configurable in code)
- Maximum track locations: 100
- Maximum layouts per track: 10
- Maximum lap history: 1000 laps per session
- GPS coordinates stored with 8 decimal places (~1.1mm precision)

## License

See LICENSE file for details.

## Support

For issues, questions, or contributions, please visit:
- GitHub Issues: [https://github.com/TheAngryRaven/DovesDataLogger/issues](https://github.com/TheAngryRaven/DovesDataLogger/issues)


