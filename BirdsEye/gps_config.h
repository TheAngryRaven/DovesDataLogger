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
#define GPS_SERIAL         Serial1

#endif
