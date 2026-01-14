# SD Card Test Branch

## ⚠️ IMPORTANT: THIS IS A TEST BRANCH ⚠️

This branch is specifically designed for SD card stress testing and does NOT contain normal GPS functionality.

## Purpose

This branch is used to test SD card write performance, error handling, and stability under continuous high-frequency logging conditions. It simulates the data logging behavior of the main system but removes GPS dependencies to allow for extended testing without GPS lock.

## Key Differences from Main Branch

### Removed Features
- ❌ **GPS Hardware**: All GPS libraries and hardware initialization removed
- ❌ **GPS Lock Wait**: System starts logging immediately, no waiting for GPS fix
- ❌ **Manual Selection**: No user input required for track/course/direction selection

### Modified Features
- ✅ **Dummy Data Generation**: Generates simulated GPS data at ~30Hz
- ✅ **Auto-Selection**: Automatically selects random track, course, and direction
  - Track selection: 1 second delay
  - Course selection: 1 second delay
  - Direction selection: Instant (no delay)
- ✅ **Test File Naming**: Log files named `TEST-{1000-9999}.txt` instead of normal pattern
- ✅ **Default Page**: Loads Engine RPM page after track selection (for visual monitoring)
- ✅ **Error Display**: All SD card error messages remain intact on display

### Preserved Features
- ✅ **SD Card Logging**: Full SD card write functionality with error handling
- ✅ **Track/Course Loading**: Still loads track files from SD card
- ✅ **Display System**: All display pages functional
- ✅ **Tachometer Input**: Can connect wave generator to antenna port for jitter testing
- ✅ **Error Handling**: All SD card error detection and display intact

## Test Setup

1. **Hardware Required**:
   - Device with display connected
   - SD card with `/TRACKS` folder and track JSON files
   - Optional: Wave generator connected to antenna port for jitter testing

2. **Expected Behavior**:
   - Device boots and reads track list from SD card
   - Automatically selects random track (1 second delay)
   - Automatically selects random course/layout (1 second delay)
   - Automatically selects random direction (instant)
   - Loads Engine RPM page
   - Begins logging dummy data at ~30Hz to `TEST-####.txt`
   - Display shows real-time status including SD card errors

3. **What to Monitor**:
   - Watch display for SD card write errors
   - Check SD card for `TEST-####.txt` files being created
   - Verify error messages appear on display if SD card fails
   - Monitor for any crashes or hangs during extended operation

## Dummy Data Characteristics

- **Frequency**: ~30Hz (33ms interval)
- **GPS Fix**: Always reports fixed (12 satellites, 0.9 HDOP)
- **Location**: Simulated movement with sine wave variation
- **Speed**: Varies between 25-35 knots
- **Altitude**: ~250m with small variation
- **Timestamp**: Real-time system clock

## Log File Format

Files are created as `TEST-{random number 1000-9999}.txt` with standard CSV format:

```
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,rpm,exhaust_temp_c,water_temp_c
```

## Testing Procedure

1. Insert SD card with track files
2. Power on device
3. Wait for automatic selection sequence (~2 seconds)
4. Observe Engine RPM page load
5. Let run for extended period to test SD card stability
6. Monitor display for any error messages
7. Check SD card for TEST files after test

## Return to Normal Operation

To return to normal GPS-based operation, switch back to the main branch:

```bash
git checkout main
```

## Notes

- This branch should ONLY be used for SD card testing
- Do not use this branch for actual track sessions
- GPS antenna connection not required (can connect wave generator for tach testing)
- All timing and lap detection code is still present but uses dummy coordinates
