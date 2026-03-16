#ifndef _DOVES_PROJECT_H
#define _DOVES_PROJECT_H

///////////////////////////////////////////
// DovesDataLogger - Project-Wide Types & Macros
//
// This header is included FIRST in BirdsEye.ino so that custom types
// are available before the Arduino preprocessor generates function
// prototypes from the other .ino module files.
///////////////////////////////////////////

///////////////////////////////////////////
// DEBUG MACROS
///////////////////////////////////////////

#ifdef HAS_DEBUG
#define debugln Serial.println
#define debug Serial.print
#else
inline void dummy_debug(...) {
}
#define debug dummy_debug
#define debugln dummy_debug
#endif

///////////////////////////////////////////
// PROJECT LIMITS
// Defined here so structs below can reference them.
// These were previously in BirdsEye.ino — moved here for
// visibility in project-wide types.
///////////////////////////////////////////
#define MAX_LOCATIONS 200
#define MAX_LOCATION_LENGTH 13  // 13 is old DOS format for FAT16
#define MAX_LAYOUTS 10
#define MAX_LAYOUT_LENGTH 15
#define DOVEX_HEADER_SIZE 1024  // Reserved header bytes in .dovex files (1 KB)
#define TRACK_DETECT_RADIUS_MILES 5.0  // Haversine threshold for live track detection

// Sleep mode constants (ENABLE_NEW_UI only)
#define SLEEP_IDLE_TIMEOUT_MS     300000   // 5 min menu idle -> auto-sleep
#define SLEEP_LONG_PRESS_MS       5000     // 5s hold for sleep/reboot combos
#define SLEEP_GPS_WAKE_INTERVAL   86400000 // 24 hours between GPS fix attempts
#define SLEEP_GPS_FIX_TIMEOUT     120000   // 2 min max for GPS fix attempt
#define SLEEP_RPM_WAKE_THRESHOLD  100      // RPM above this wakes from sleep
#define CHARGE_DISPLAY_TIMEOUT_MS 10000    // Show charging screen for 10s then display off

///////////////////////////////////////////
// STRUCT DEFINITIONS
// Must be in a header so Arduino's auto-prototype generation
// can resolve these types in function signatures.
///////////////////////////////////////////

struct ButtonState {
  int pin = 0;
  bool pressed = false;           // Flag for "button was pressed" (consumed by handler)
  bool wasReleased = true;        // Edge detection: must release before next press
  unsigned long lastPressed = 0;
};

struct TrackLayout {
  double start_a_lat = 0.00;
  double start_a_lng = 0.00;
  double start_b_lat = 0.00;
  double start_b_lng = 0.00;

  // Sector 2 line data (optional)
  double sector_2_a_lat = 0.00;
  double sector_2_a_lng = 0.00;
  double sector_2_b_lat = 0.00;
  double sector_2_b_lng = 0.00;
  bool hasSector2 = false;

  // Sector 3 line data (optional)
  double sector_3_a_lat = 0.00;
  double sector_3_a_lng = 0.00;
  double sector_3_b_lat = 0.00;
  double sector_3_b_lng = 0.00;
  bool hasSector3 = false;
};

// Common sample struct for both DOVE and NMEA parsing
struct ReplaySample {
  unsigned long timestamp;  // milliseconds
  double lat;
  double lng;
  float speed_mph;
  float altitude;
  int rpm;  // -1 if not available
  bool valid;
};

///////////////////////////////////////////
// TRACK MANIFEST (in-RAM index for proximity detection)
// Built during buildTrackList() at boot. Each entry stores
// the filename and a representative lat/lon from the first course.
///////////////////////////////////////////
struct TrackManifestEntry {
  char filename[32];   // track filename without extension (matches locations[])
  double lat;          // first course's start_a_lat
  double lon;          // first course's start_a_lng
};

///////////////////////////////////////////
// TRACK METADATA (parsed from new JSON object format)
///////////////////////////////////////////
struct TrackMetadata {
  char longName[32];
  char shortName[16];
  char defaultCourse[MAX_LAYOUT_LENGTH];
  float courseLengthFt[MAX_LAYOUTS];  // per-course lengthFt
};

#endif
