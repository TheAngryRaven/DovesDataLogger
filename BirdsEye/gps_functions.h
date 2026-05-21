#pragma once

///////////////////////////////////////////
// GPS MODULE
// SparkFun u-blox GNSS v3 (UBX-PVT binary) at 25 Hz. A TIMER3 ISR
// drains Serial1 into a 4 KB ring buffer every 10 ms to survive SD
// write stalls. The library reads from that buffer via a Stream
// wrapper. Handles V_BCKP loss with baud-rate recovery + reconfigure.
///////////////////////////////////////////

#include <SparkFun_u-blox_GNSS_v3.h>

// PVT callback fired by checkCallbacks() — populates gpsData and sets
// gpsDataFresh = true.
void onPVTReceived(UBX_NAV_PVT_data_t *pvt);

// One-shot GPS bring-up — baud autoneg, VALSET config, PVT callback,
// start of the TIMER3 serial-drain ISR. Sets gpsInitialized on
// success.
void GPS_SETUP();

// Service GPS: drain the buffer, dispatch PVT callbacks, run the
// PVT-arrival watchdog, feed CourseManager, and (when enabled) write
// the next CSV row to the open DOVEX log file.
void GPS_LOOP();

// Put the GPS module into backup mode and stop the serial-drain
// timer. RAM-config is held by V_BCKP — see GPS_WAKE() for recovery
// path when V_BCKP drops.
void GPS_SLEEP();

// Re-apply the full VALSET configuration. Idempotent. Called by
// GPS_WAKE and GPS_BAUD_RECOVERY.
void GPS_RECONFIGURE();

// Recover communication when the module reverted to 9600 baud / NMEA
// after V_BCKP loss. Returns true if reconnected at GPS_BAUD_RATE.
bool GPS_BAUD_RECOVERY();

// Wake the GPS from backup mode, restart the serial-drain timer,
// re-apply config, and arm the PVT-arrival watchdog.
void GPS_WAKE();

// Periodic sleep-mode wake to keep ephemeris fresh (called from the
// sleep loop every SLEEP_GPS_WAKE_INTERVAL ms).
void GPS_SLEEP_PERIODIC_CHECK();

// GPS time helpers (require an active fix and gpsInitialized=true).
unsigned long      getGpsTimeInMilliseconds();
unsigned long      getGpsUnixTimestamp();
unsigned long long getGpsUnixTimestampMillis();

// 1 Hz frame-rate window — populates gpsFrameRate.
void calculateGPSFrameRate();
