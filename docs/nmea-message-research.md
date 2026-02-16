# NMEA Message Research for DovesDataLogger

## Current GPS Configuration (SAM-M10Q)

| Setting | Value |
|---------|-------|
| Baud Rate | 115200 (NMEA-only) |
| Update Rate | 25Hz |
| Constellations | GPS-only (M10) |
| Navigation Mode | Automobile |
| Power Mode | Full Power |
| NMEA Version | 2.3 |
| Enabled Messages | GGA only |
| Disabled Messages | RMC, VTG, GSA, GSV, GLL |

## NMEA Message Types -- Full Reference

### GGA -- Global Positioning System Fix Data (ENABLED @ 25Hz)

The primary fix sentence. Contains position, altitude, and quality indicators.

**Fields:**
| # | Field | Description |
|---|-------|-------------|
| 1 | UTC Time | hhmmss.ss |
| 2-3 | Latitude + N/S | ddmm.mmmmm |
| 4-5 | Longitude + E/W | dddmm.mmmmm |
| 6 | Fix Quality | 0=Invalid, 1=GPS, 2=DGPS, 4=RTK, 5=Float RTK |
| 7 | Satellites Used | Count of sats in solution |
| 8 | HDOP | Horizontal Dilution of Precision |
| 9-10 | Altitude + Units | MSL altitude in meters |
| 11-12 | Geoid Separation + Units | WGS-84 vs MSL difference |
| 13 | DGPS Age | Seconds since last DGPS update |
| 14 | DGPS Station ID | Reference station ID |

**Typical size:** ~75 bytes
**Unique data:** Altitude, geoid separation, fix quality (0-8 scale), satellite count, HDOP

---

### RMC -- Recommended Minimum Navigation Information (DISABLED)

The single most information-dense NMEA sentence. Combines position, velocity, and time in one message.

**Fields:**
| # | Field | Description |
|---|-------|-------------|
| 1 | UTC Time | hhmmss.ss |
| 2 | Status | A=Active/Valid, V=Void |
| 3-4 | Latitude + N/S | ddmm.mmmmm |
| 5-6 | Longitude + E/W | dddmm.mmmmm |
| 7 | Speed Over Ground | Knots |
| 8 | Course Over Ground | Degrees true |
| 9 | Date | ddmmyy |
| 10-11 | Magnetic Variation + Dir | Degrees E/W |
| 12 | Mode Indicator | A=Autonomous, D=Differential, E=Estimated, N=Invalid |

**Typical size:** ~72 bytes
**Unique data:** Speed over ground, course/heading, date, magnetic variation

**IMPORTANT:** The Adafruit GPS library parses `gps->speed` and `gps->angle` from RMC sentences. With RMC disabled, these fields have no source data.

---

### VTG -- Course Over Ground and Ground Speed (DISABLED)

Velocity information in multiple formats.

**Fields:**
| # | Field | Description |
|---|-------|-------------|
| 1-2 | Course True + T | Track angle, true north |
| 3-4 | Course Magnetic + M | Track angle, magnetic north |
| 5-6 | Speed Knots + N | Speed in knots |
| 7-8 | Speed km/h + K | Speed in km/h |
| 9 | Mode | A=Autonomous, D=Differential, E=Estimated |

**Typical size:** ~42 bytes
**Unique data:** Pre-converted km/h speed (convenience only -- all data derivable from RMC)
**Verdict:** Redundant when RMC is enabled.

---

### GSA -- DOP and Active Satellites (DISABLED)

Reports which satellites are used in the fix and provides all three DOP values.

**Fields:**
| # | Field | Description |
|---|-------|-------------|
| 1 | Selection Mode | A=Auto, M=Manual |
| 2 | Fix Type | 1=No fix, 2=2D, 3=3D |
| 3-14 | Satellite PRNs | Up to 12 satellite IDs used in solution |
| 15 | PDOP | Position (3D) Dilution of Precision |
| 16 | HDOP | Horizontal Dilution of Precision |
| 17 | VDOP | Vertical Dilution of Precision |

**Typical size:** ~65 bytes (one sentence per constellation when multi-GNSS is active)
**Unique data:** PDOP, VDOP, 2D/3D fix type, specific satellite PRNs used
**Note:** With GPS-only mode, only 1 GSA sentence per epoch.

---

### GSV -- Satellites in View (DISABLED)

Detailed per-satellite diagnostics including signal strength and sky position.

**Fields (per satellite, 4 per sentence):**
| # | Field | Description |
|---|-------|-------------|
| 1 | Total Messages | Number of GSV sentences in group |
| 2 | Message Number | Sequence number |
| 3 | Satellites in View | Total visible count |
| 4-7 | Sat PRN, Elevation, Azimuth, SNR | Per-satellite data (repeated 4x) |

**Typical size:** ~70 bytes per sentence, 2-4 sentences per constellation
**Unique data:** Per-satellite signal strength (SNR/C/N0), satellite elevation/azimuth
**Verdict:** Massive bandwidth cost. Diagnostic use only. Should remain disabled for racing.

---

### GLL -- Geographic Position (DISABLED)

Compact position-only sentence.

**Fields:**
| # | Field | Description |
|---|-------|-------------|
| 1-2 | Latitude + N/S | ddmm.mmmmm |
| 3-4 | Longitude + E/W | dddmm.mmmmm |
| 5 | UTC Time | hhmmss.ss |
| 6 | Status | A=Valid, V=Void |
| 7 | Mode | A/D/E/N |

**Typical size:** ~50 bytes
**Unique data:** None -- 100% redundant with GGA and RMC.
**Verdict:** Should always remain disabled.

---

### ZDA -- Time and Date (NOT CONFIGURED)

Precise time and date with timezone.

**Fields:**
| # | Field | Description |
|---|-------|-------------|
| 1 | UTC Time | hhmmss.ss |
| 2 | Day | 01-31 |
| 3 | Month | 01-12 |
| 4 | Year | 4-digit year (yyyy) |
| 5-6 | TZ Hours + Minutes | Local timezone offset |

**Typical size:** ~38 bytes
**Unique data:** 4-digit year, timezone offset
**Verdict:** Unnecessary when RMC provides date. RMC only has 2-digit year but that's fine for logging.

---

## Bandwidth Analysis (115200 baud = 11,520 bytes/sec max)

| Configuration | Bytes/sec | % Bandwidth | Status |
|---|---|---|---|
| GGA only @ 25Hz (current) | 1,875 | 16% | Very safe |
| **GGA + RMC @ 25Hz** | **3,675** | **32%** | **Recommended** |
| GGA + RMC + VTG @ 25Hz | 4,725 | 41% | Safe but VTG redundant |
| GGA + RMC @ 25Hz + GSA @ 1Hz | 3,740 | 32% | Recommended+ |
| All messages @ 25Hz (GPS-only) | ~7,000 | 61% | Wasteful |
| All messages @ 25Hz (multi-GNSS) | 34,000+ | 300%+ | Impossible |

## Key Findings

### 1. RMC Should Be Enabled
The Adafruit GPS library parses `gps->speed` and `gps->angle` (course) from RMC sentences. With RMC disabled at the module level, the library never receives speed/course data. This means:
- `gps->speed` logged in CSV may be stale/zero
- No heading data available for cornering analysis
- The `gps->day`/`gps->month`/`gps->year` date fields also come from RMC

### 2. GGA + RMC Is the Ideal Pair for Racing
- GGA: position, altitude, HDOP, sat count, fix quality
- RMC: speed, heading, date/time
- Combined they cover all essential racing telemetry at only 32% bandwidth

### 3. GSA at 1Hz Is a Nice Diagnostic Addition
- Provides PDOP and VDOP for post-session quality analysis
- Costs only ~65 extra bytes/sec at 1Hz
- Helps diagnose sessions with poor vertical accuracy

### 4. GSV, GLL, VTG Should Remain Disabled
- GSV: massive bandwidth cost, only useful for signal diagnostics
- GLL: 100% redundant
- VTG: fully redundant when RMC is enabled

## Data Field Cross-Reference

| Data | GGA | RMC | VTG | GSA | GSV | GLL | ZDA |
|------|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| Position (lat/lng) | X | X | | | | X | |
| UTC Time | X | X | | | | X | X |
| Altitude (MSL) | **X** | | | | | | |
| Speed Over Ground | | **X** | **X** | | | | |
| Course/Heading | | **X** | **X** | | | | |
| Date | | **X** | | | | | **X** |
| HDOP | X | | | X | | | |
| PDOP | | | | **X** | | | |
| VDOP | | | | **X** | | | |
| Fix Quality (detailed) | **X** | | | | | | |
| Satellite Count (used) | **X** | | | | | | |
| 2D/3D Fix Type | | | | **X** | | | |
| Magnetic Variation | | **X** | X | | | | |
| Per-Sat Signal Strength | | | | | **X** | | |
| Geoid Separation | **X** | | | | | | |

## u-blox M10 Max Nav Rates by Constellation

| Configuration | Max Rate |
|---|---|
| GPS only | 25 Hz |
| GPS + 1 other (e.g., Galileo) | 20 Hz |
| GPS + 2 others | 16 Hz |
| GPS + 3 others (all) | 10 Hz |
