#ifndef _DOVES_GPS_CONFIG_H
#define _DOVES_GPS_CONFIG_H

// GPS configuration constants used by GPS_SETUP()
// All module configuration is now handled via SparkFun VALSET API calls
#define GPS_BAUD_RATE      57600
#define GPS_NAV_RATE_HZ    25   // race rate (PVT only)
// Boot/status-page rate: slower nav solutions with UBX-NAV-SAT enabled so
// the GPS status page can draw per-satellite signal bars. Switched to
// GPS_NAV_RATE_HZ (NAV-SAT off) when the page exits — see gpsEnterRaceMode().
#define GPS_NAV_RATE_STATUS_HZ 5
// maxWait for the connection-probe begin() calls (probe ladder, baud
// recovery). The library's default is 1100 ms and begin() retries its
// ping 3x internally, so a failing probe blocks 3 x maxWait with no way
// to pet the ~4 s hardware watchdog in between — 1100 ms leaves a <15%
// margin. Modern modules answer in tens of ms (the library notes 250 ms
// is safe); 550 ms keeps a failed begin() at ~1.65 s, half the WDT budget.
#define GPS_PROBE_MAXWAIT_MS 550
#define GPS_SERIAL         Serial1

// TIMER3 GPS drain period (µs, 1 MHz tick). At 57600 baud ≈ 5.76
// bytes/ms line rate, a 5 ms period accumulates ~29 bytes per fire —
// under half the core Serial1 RX ring even at its stock 64 B — so a
// SoftDevice radio-ISR deferral of the drain has ~6 ms of slack before
// the core ring can overflow (~1 ms at the old 10 ms period, which is
// exactly where the scanner/camera-era PVT drops came from). The ISR
// body is µs-scale, so doubling its rate costs nothing measurable.
#define GPS_DRAIN_INTERVAL_US 5000

#endif
