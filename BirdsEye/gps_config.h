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

#endif
