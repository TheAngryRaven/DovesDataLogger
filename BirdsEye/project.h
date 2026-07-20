#ifndef _DOVES_PROJECT_H
#define _DOVES_PROJECT_H

#include <stdint.h>

///////////////////////////////////////////
// DovesDataLogger - Project-Wide Types & Macros
//
// This header is included FIRST in BirdsEye.ino so that custom types
// are available before the Arduino preprocessor generates function
// prototypes from the other .ino module files.
///////////////////////////////////////////

///////////////////////////////////////////
// FIRMWARE VERSION
//
// Single source of truth for the *base* firmware version. Exposed over BLE
// via the Device Information Service (see bluetooth.ino) so a companion
// (DovesDataViewer) can compare it against the latest GitHub release and
// decide whether an OTA update is needed. Keep this in sync with the release
// git tag (tag v2.0.0 -> "2.0.0") and the CHANGELOG.
//
// Beta/nightly builds override it from the build flag: the beta workflow
// passes -DFIRMWARE_VERSION_OVERRIDE=<base>-beta.<gitsha> as a bare token —
// the stringizer below quotes it, so there is no shell quote-escaping to get
// wrong. A normal build (prod release, plain IDE, compile-sketch CI) leaves it
// undefined and uses the literal below.
///////////////////////////////////////////
#define _BE_STRINGIFY(x) #x
#define _BE_TOSTRING(x) _BE_STRINGIFY(x)
#ifdef FIRMWARE_VERSION_OVERRIDE
  #define FIRMWARE_VERSION _BE_TOSTRING(FIRMWARE_VERSION_OVERRIDE)
#else
  #define FIRMWARE_VERSION "3.0.0"
#endif

///////////////////////////////////////////
// BOARD VARIANT
//
// Identifies which XIAO nRF52840 this image targets so the companion can
// fetch the matching OTA package. Reported over BLE as part of the DIS
// model string ("BirdsEye-" FIRMWARE_VARIANT), which equals the release
// asset prefix (BirdsEye-sense.zip / BirdsEye-nonsense.zip) so the webapp
// can map model -> download directly.
//
// Resolution order:
//   1. The build workflows pass -DBIRDSEYE_BOARD_SENSE / -DBIRDSEYE_BOARD_NONSENSE
//      per FQBN — these always win.
//   2. A plain Arduino IDE build passes no such flag, but the Seeeduino core
//      defines ARDUINO_<build.board> from the Tools -> Board selection
//      (ARDUINO_Seeed_XIAO_nRF52840_Sense vs ARDUINO_Seeed_XIAO_nRF52840), so
//      we derive the variant from the board you actually picked — no manual
//      flag needed.
//   3. If nothing matches (some other core / unknown board), default to
//      "sense" (the primary board).
// A wrong label only affects which OTA asset the companion offers — the images
// share the MCU/SoftDevice, the non-Sense image boots fine on a Sense board and
// vice-versa, and accelerometer presence is auto-detected at runtime regardless
// of this flag — so a mislabel never bricks.
///////////////////////////////////////////
#if defined(BIRDSEYE_BOARD_NONSENSE)
  #define FIRMWARE_VARIANT "nonsense"
#elif defined(BIRDSEYE_BOARD_SENSE)
  #define FIRMWARE_VARIANT "sense"
#elif defined(ARDUINO_Seeed_XIAO_nRF52840_Sense)
  #define FIRMWARE_VARIANT "sense"
#elif defined(ARDUINO_Seeed_XIAO_nRF52840)
  #define FIRMWARE_VARIANT "nonsense"
#else  // unknown board / core — fall back to the primary variant
  #define FIRMWARE_VARIANT "sense"
#endif

///////////////////////////////////////////
// BUILD-FLAG CONTRACT: GPS UART BUFFER
//
// The core's Serial1 RX ring must be 256 B (stock is 64 B, #ifndef-
// guarded in the BSP's RingBuffer.h). SoftDevice radio interrupts
// (unmaskable, prio 0–2) defer the TIMER3 GPS drain ISR (prio 3); at
// 57600 baud the stock 64 B ring gives only ~1 ms of deferral slack
// before bytes are silently dropped and 25 Hz PVT frames are lost —
// the exact field regression the drop counters were built to catch.
// A silently under-buffered build would reintroduce it, so fail loud.
//
// CI passes -DSERIAL_BUFFER_SIZE=256 (all three workflows). For a
// local Arduino IDE / arduino-cli build, see CONTRIBUTING.md
// ("Local build flags") — one-time setup via platform.local.txt or
// --build-property.
///////////////////////////////////////////
#ifndef SIM
static_assert(SERIAL_BUFFER_SIZE >= 256,
              "Build with -DSERIAL_BUFFER_SIZE=256 (see CONTRIBUTING.md, "
              "'Local build flags') — the stock 64 B Serial1 ring drops GPS "
              "bytes under BLE radio load.");
#endif

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

// Shutdown (System OFF) constants
#define SLEEP_IDLE_TIMEOUT_MS     300000   // 5 min menu idle -> auto-shutdown
#define SLEEP_LONG_PRESS_MS       5000     // 5s hold for shutdown/reboot combos
#define CHARGE_DISPLAY_TIMEOUT_MS 10000    // Show charging screen for 10s then display off
#define USB_MENU_CHARGE_IDLE_MS   60000    // USB on menu: charging loop after 60s of no buttons

///////////////////////////////////////////
// BLE RADIO OWNERSHIP
// The one SoftDevice (single advert set + single peripheral connection
// slot) is shared between the file-transfer service (bluetooth.ino) and
// the camera remote (camera_ble.ino). bleOwner (defined in BirdsEye.ino,
// documented in bluetooth.h) records which subsystem currently owns the
// advert set, so the shared connect/disconnect callbacks can route a
// peer to the right module.
///////////////////////////////////////////
enum BleOwner : uint8_t {
  BLE_OWNER_NONE = 0,      // radio idle — nobody advertising
  BLE_OWNER_TRANSFER = 1,  // file-transfer page (BLE_SETUP/BLE_STOP)
  BLE_OWNER_CAMERA = 2,    // camera remote (camera_ble)
};

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
