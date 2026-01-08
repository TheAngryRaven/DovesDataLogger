# BirdsEye Data Logger

A high-performance GPS-based lap timer and data logger designed for motorsports and racing applications. Features 25Hz GPS logging, sector timing, real-time pace analysis, and comprehensive telemetry recording to SD card in a simple CSV format.

Originally developed as a personal project, now open-sourced for the racing community.

## Features

### Lap Timing
- **High-frequency GPS logging** - 25Hz data capture rate for precise position tracking
- **Multi-layout track support** - Load multiple track configurations from SD card
- **Sector timing** - Up to 3 sectors per lap with independent timing
- **Optimal lap calculation** - Combines your best sector times to show theoretical best lap
- **Best lap tracking** - Automatic detection and recording of session best
- **Pace comparison** - Real-time delta to your best lap
- **Lap history** - Review all lap times from current session
- **Bidirectional support** - Run tracks in forward or reverse direction

### Data Logging
- **Simple CSV format** - Easy to parse and analyze with any tool
- **Millisecond-precision timestamps** - Unix timestamps with full millisecond accuracy
- **Comprehensive telemetry** - GPS position (8 decimal precision), speed, altitude, HDOP, satellite count
- **Engine RPM logging** - Tachometer input for engine speed monitoring
- **Automatic file naming** - Files named with track, layout, direction, and timestamp
- **Continuous logging** - All data logged to SD card at 25Hz GPS update rate
- **Error handling** - Graceful handling of SD card failures with user notification

### Display & Interface
- **Multiple data pages** - Speed, lap time, pace, best lap, optimal lap, tachometer, GPS stats, lap history
- **Visual crossing indicator** - Animated display when crossing start/finish or sector lines
- **Battery monitoring** - Visual battery level indicator
- **3-button navigation** - Simple up/down/select interface
- **OLED display** - 128x64 monochrome display with low power consumption
- **Real-time feedback** - Instant lap time updates and pace calculations

### Power Efficiency
- **Low power design** - ~120mAh average consumption
- **Long runtime** - 13+ hours on 1500mAh battery
- **NRF52840 MCU** - 64MHz ARM Cortex-M4 with FPU and Bluetooth capability
- **Efficient display updates** - 3Hz display refresh rate to conserve power

## Hardware Requirements

### Core Components
- **MCU**: Seeed XIAO nRF52840 (64MHz ARM Cortex-M4, FPU, Bluetooth)
  - *Alternative*: Seeed XIAO ESP32C3 (same form factor, adds WiFi for data transfer, higher power consumption)
- **GPS Module**: u-blox SAM-M10Q (recommended: Matek M10Q-5883 drone GPS)
  - Configured for 25Hz update rate, 115200 baud, GPS-only mode
- **Display**: 128x64 OLED (2.42" - 2.45" diagonal)
  - Supported: SH1106G (recommended) or SSD1306
  - I2C address: 0x3C
- **Storage**: Micro SD card module (any inexpensive SPI module)
- **Battery**: 1500mAh LiPo (103050 format recommended)
- **Buttons**: 3x momentary push buttons (navigation interface)
- **Tachometer Circuit**: Optional - for engine RPM input (circuit diagram included, readme coming soon)

### Pin Configuration (Seeed XIAO)
- **GPS**: Serial1 (hardware UART)
- **Display**: I2C (Wire)
- **SD Card**: SPI (CS pin grounded for permanent selection)
- **Tachometer**: D0 (interrupt-driven pulse counting)
- **Buttons**: D1 (left/up), D2 (select), D3 (right/down)
- **Battery Monitor**: A0 (voltage divider)

### 3D Printed Case
- STL files included in `/CASE` directory
- Custom-fit enclosure for all components
- **Assembly note**: Detailed assembly instructions coming soon
- Some glue and jeweler's screws required
- Recommend test-fitting components and measuring before final assembly
- Take your time - it's a tight but functional fit

## Software Requirements

### Arduino Libraries
The following libraries are required (install via Arduino Library Manager):

```
Adafruit GFX Library
Adafruit SH110X (or Adafruit SSD1306 if using that display)
Adafruit GPS Library
ArduinoJson
SdFat
DovesLapTimer (v0.2.1+)
```

### Arduino Board Support
Install Seeed nRF52 Boards support package in Arduino IDE:
- Board Manager URL: `https://files.seeedstudio.com/arduino/package_seeeduino_boards_index.json`
- Select: **Seeed nRF52 Boards** → **Seeed XIAO nRF52840**

## Track Configuration

### JSON Track Files

Track layouts are defined in JSON files stored on the SD card in the `/TRACKS` directory. Each file represents one location and can contain multiple layout configurations.

#### File Location
```
/TRACKS/
  ├── TRACKNAME1.json
  ├── TRACKNAME2.json
  └── TRACKNAME3.json
```

#### JSON Format

Each track file contains an array of layout objects. At minimum, each layout needs:
- `name` - Display name for this layout
- `start_a_lat`, `start_a_lng` - First point of start/finish line
- `start_b_lat`, `start_b_lng` - Second point of start/finish line

Optionally, add sector timing lines:
- `sector_2_a_lat`, `sector_2_a_lng`, `sector_2_b_lat`, `sector_2_b_lng` - Sector 2 timing line
- `sector_3_a_lat`, `sector_3_a_lng`, `sector_3_b_lat`, `sector_3_b_lng` - Sector 3 timing line

**Note**: Sector 1 is always the section from start/finish to the sector 2 line. The final sector always ends at the start/finish line.

#### Example Track File

**`/TRACKS/MYTRACK.json`**
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

### Creating Track Files

1. **Determine your start/finish line coordinates** using GPS or mapping tools (Google Maps, etc.)
2. **Define two points** that form a line across the track
3. **(Optional) Define sector timing lines** the same way
4. **Save as JSON** in the `/TRACKS` folder on your SD card
5. **Power on the device** - your new track will appear in the track selection menu

**Tips**:
- GPS coordinates should have 8 decimal places for racing precision
- The line is defined by two points - the device detects when you cross the line between them
- Test your coordinates on a map to ensure they cross the track perpendicular to direction of travel
- You can have as many layouts as memory allows (defined by `MAX_LAYOUTS` in code)

## Data Format

### CSV Log Files

Data is logged to `.dove` files (CSV format) on the SD card. Files are automatically named:
```
TRACKNAME_LAYOUTNAME_DIRECTION_YYYY_MMDD_HHMM.dove
```

Example: `OKC_Normal_fwd_2024_1215_1430.dove`

### CSV Structure

**Header Row:**
```csv
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,rpm,exhaust_temp_c,water_temp_c
```

**Data Columns:**
- `timestamp` - Unix timestamp in milliseconds (e.g., 1704300123456)
- `sats` - Number of GPS satellites in use
- `hdop` - Horizontal dilution of precision (lower is better, <2.0 is excellent)
- `lat` - Latitude in decimal degrees (8 decimal precision)
- `lng` - Longitude in decimal degrees (8 decimal precision)
- `speed_mph` - Speed in miles per hour (2 decimal precision)
- `altitude_m` - Altitude in meters (2 decimal precision)
- `rpm` - Engine RPM (requires tachometer circuit)
- `exhaust_temp_c` - Reserved for future sensor (currently 0)
- `water_temp_c` - Reserved for future sensor (currently 0)

**Note**: Temperature columns are placeholders for future expansion. Additional sensor inputs can be easily added to the CSV format.

### Sample Data
```csv
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,rpm,exhaust_temp_c,water_temp_c
1704300123456,12,0.8,28.41270817,-81.37973266,45.32,125.50,5200,0,0
1704300123496,12,0.8,28.41270915,-81.37973145,45.67,125.48,5250,0,0
1704300123536,12,0.8,28.41271018,-81.37973021,46.01,125.46,5300,0,0
```

### Data Analysis

CSV files can be analyzed with:
- **DovesDataViewer** - Web-based analysis tool (see Related Projects below)
- Spreadsheet software (Excel, Google Sheets, LibreOffice)
- Python/R for custom analysis
- Any GPS track analysis software that supports CSV import

## Display Pages

The device includes multiple display pages accessible via the three-button interface:

### Setup Pages
1. **Track Selection** - Choose your track location
2. **Layout Selection** - Choose track layout (if multiple exist)
3. **Direction Selection** - Forward or Reverse

### Data Pages
1. **GPS Stats** - Battery level, satellite count, GPS frequency, HDOP, SD card status
2. **GPS Debug** - Distance to line, lap count, odometer (development/troubleshooting)
3. **Speed** - Large speed display with current lap number
4. **Tachometer** - Large RPM display with max RPM (requires tachometer circuit)
5. **Lap Time** - Current lap time with milliseconds
6. **Lap Pace** - Delta time to best lap (flashing display when ahead of pace)
7. **Best Lap** - Best lap time and lap number for current session
8. **Optimal Lap** - Theoretical best lap from best sector times
9. **Lap History** - Scrollable list of all lap times from session
10. **End Race** - Stop logging and return to track selection

### Navigation
- **Left Button** - Previous page
- **Middle Button** - Context action (select menu item, page-specific function)
- **Right Button** - Next page

## Extending the System

### Adding New Display Pages

The display system is designed to be easily extensible. To add a new page:

1. **Define page constant** (around line 635):
```cpp
const int MY_NEW_PAGE = 13;
```

2. **Create display function**:
```cpp
void displayPage_my_new_page() {
  resetDisplay();

  display.println(F("My Page Title"));
  display.setTextSize(2);
  display.print(F("Data: "));
  display.println(myDataVariable);

  display.display();
}
```

3. **Add to display loop** (around line 1609):
```cpp
} else if (currentPage == MY_NEW_PAGE) {
  displayPage_my_new_page();
}
```

4. **Update page range** if needed (line 682):
```cpp
int runningPageEnd = MY_NEW_PAGE; // if this is the last page
```

The display system automatically handles page navigation. Study existing pages like `displayPage_gps_speed()` (line 977) or `displayPage_tachometer()` (line 1181) as templates.

### Adding New Sensors

To log additional sensor data:

1. Read your sensor in the `loop()` function
2. Modify the CSV header string (line 1907)
3. Update the `snprintf` call (line 1849) to include your data
4. Increase `csvLine` buffer size if needed (line 1832)

The system's 25Hz GPS rate leaves plenty of MCU time for additional sensor reads between GPS updates.

## File Structure

### Core Files
- **`BirdsEye.ino`** - Main application code
  - Setup and main loop
  - GPS integration and lap timing logic
  - Display management and UI navigation
  - SD card logging system
  - Tachometer input handling
  - All display page implementations

### Configuration Files
- **`gps_config.h`** - GPS module configuration
  - u-blox UBX protocol commands
  - Baud rate settings (115200)
  - Navigation rate configurations (5Hz to 25Hz)
  - Satellite system selection (GPS-only mode)
  - Power mode settings

- **`display_config.h`** - Display configuration
  - I2C address (0x3C)
  - Screen dimensions (128x64)
  - Support for SH1106G or SSD1306 displays
  - Display object initialization

- **`images.h`** - Display graphics
  - Bitmap images for splash screens
  - Crossing animation frames
  - Logo and branding graphics

### Data Files
- **`SDCARD/TRACKS/*.json`** - Track definition files
  - JSON format track layouts
  - GPS coordinates for start/finish lines
  - Optional sector timing lines
  - Multiple layouts per file supported

### Hardware Files
- **`CASE/*.STL`** - 3D printed enclosure
  - Main housing
  - Battery compartment with lid
  - CPU/GPS compartment with lid
  - SD card holder with lid
  - Switch housing and cover
  - Baseplate and front/back panels

- **`tachometer-circuit.jpg`** - Tachometer input circuit diagram
  - Interface circuit for ignition pulse sensing
  - Detailed readme coming soon

### Documentation
- **`libraries.txt`** - Required Arduino libraries list
- **`diagram.json`** - Wiring diagram (board.io format)
- **`LICENSE`** - Project license
- **`README.md`** - This file

## Power Consumption

**Typical Operation:**
- Average: ~120mAh
- Runtime: 13+ hours on 1500mAh battery
- Display updates: 3Hz (power optimized)
- GPS: 25Hz continuous
- SD writes: Flushed every 2 seconds

**Hardware Variants:**
- **nRF52840** (current) - Best for battery life, includes Bluetooth for future features
- **ESP32C3** (compatible) - Same XIAO form factor, adds WiFi for wireless data transfer, ~30% higher power consumption

The nRF52840 was specifically chosen for its excellent power efficiency and floating-point performance, critical for the GPS math operations.

## Known Limitations

- Maximum 100 track locations and 10 layouts per location (configurable in code)
- Lap history limited to 1000 laps per session (configurable in code)
- JSON buffer size of 2048 bytes limits track file complexity
- Display refresh rate limited to 3Hz to conserve power
- No GPS time zone adjustment (timestamps in UTC)
- Requires clear sky view for GPS fix

## Future Enhancements

Potential additions being considered:
- Bluetooth data transfer (hardware capable, software not yet implemented)
- WiFi data sync with ESP32 variant
- Additional sensor inputs (temperatures, accelerometer)
- G-force display page
- Live telemetry streaming
- Session comparison tools
- Predictive lap time calculations

## Related Projects

- **DovesDataViewer** - Web-based data analysis and visualization tool
  - GitHub: [TheAngryRaven/DovesDataViewer](https://github.com/TheAngryRaven/DovesDataViewer)
  - Live Demo: [HackTheTrack.net](https://hackthetrack.net)

- **DovesLapTimer** - Core GPS timing library used by this project
  - GitHub: [TheAngryRaven/DovesLapTimer](https://github.com/TheAngryRaven/DovesLapTimer)
  - Handles line crossing detection, sector timing, and lap calculations

## Contributing

This project was originally developed for personal use but is now open source. Contributions, bug reports, and feature suggestions are welcome!

Please note:
- Code comments and structure may be informal in places
- Some features were developed iteratively and could benefit from refactoring
- The project prioritizes functionality and reliability over code elegance

## License

See LICENSE file for details.

## Support

For issues, questions, or suggestions:
- Open an issue on GitHub
- Check existing issues for solutions
- Review the code - it's extensively commented

---

**Note**: This is an active project that evolved from private development. Documentation and refinements are ongoing. The system is fully functional and race-tested, but some assembly instructions and circuit details are still being documented.

*Built for racers, by a racer. Fast data, low power, simple format.*
