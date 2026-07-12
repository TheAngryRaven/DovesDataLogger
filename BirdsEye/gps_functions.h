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

// NAV-SAT callback fired by checkCallbacks() while status mode has
// NAV-SAT enabled — snapshots per-satellite CNO into gpsSatCnos for the
// GPS status page's signal bars.
void onNAVSATReceived(UBX_NAV_SAT_data_t *sat);

// GPS bring-up — backup-mode wake byte, 57600-first baud probe ladder
// (with one delayed cold-power retry), VALSET config, PVT + NAV-SAT
// callbacks, start of the TIMER3 serial-drain ISR, and arming of the
// PVT-arrival watchdog. Sets gpsInitialized on success.
void GPS_SETUP();

// Bounded re-detect while the GPS status page is showing "NO GPS":
// re-runs the fast probe ladder up to 3 times, 10 s apart. Call only
// from the status page driver — each attempt blocks ~2.5 s.
void GPS_STATUS_RETRY_LOOP();
// True once GPS_STATUS_RETRY_LOOP() has exhausted its retries — the
// status page switches to a permanent "CHECK WIRING" message.
bool gpsRetriesExhausted();

// Runtime GPS mode switches. Both set the persistent targets that
// GPS_RECONFIGURE()/GPS_WAKE()/recovery re-assert AND apply them live.
// Status mode: GPS_NAV_RATE_STATUS_HZ + NAV-SAT (boot default).
// Race mode: GPS_NAV_RATE_HZ, NAV-SAT off (menu/race steady state).
void gpsEnterStatusMode();
void gpsEnterRaceMode();

// Service GPS: drain the buffer, dispatch PVT callbacks, run the
// PVT-arrival watchdog, feed CourseManager, and (when enabled) write
// the next CSV row to the open DOVEX log file.
void GPS_LOOP();

// Put the GPS module into backup mode and stop the serial-drain
// timer. RAM-config is held by V_BCKP — see GPS_WAKE() for recovery
// path when V_BCKP drops.
void GPS_SLEEP();

// Re-apply the full VALSET configuration (at the current
// gpsNavRateTarget / gpsNavSatWanted). Idempotent. Called by GPS_WAKE
// and GPS_BAUD_RECOVERY.
void GPS_RECONFIGURE();

// Recover communication when the module reverted to 9600 baud / NMEA
// after V_BCKP loss. Returns true if reconnected at GPS_BAUD_RATE.
bool GPS_BAUD_RECOVERY();

// Wake the GPS from backup mode, restart the serial-drain timer,
// re-apply config, and arm the PVT-arrival watchdog.
void GPS_WAKE();

// GPS time helpers (require an active fix and gpsInitialized=true).
unsigned long      getGpsTimeInMilliseconds();
unsigned long      getGpsUnixTimestamp();
unsigned long long getGpsUnixTimestampMillis();

// 1 Hz frame-rate window — populates gpsFrameRate.
void calculateGPSFrameRate();
