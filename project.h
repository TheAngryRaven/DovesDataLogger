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

#endif
