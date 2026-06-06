///////////////////////////////////////////
// DovesDataLogger - BirdsEye Main Sketch
//
// This is the main sketch file containing global state, includes,
// setup(), and loop(). All function implementations are split into
// separate module files (Arduino concatenates .ino files automatically):
//
//   accelerometer.ino - LSM6DS3 IMU accelerometer reads (g-force)
//   bluetooth.ino    - BLE file transfer service
//   display_pages.ino - All display page rendering functions
//   display_ui.ino   - Display setup, button handling, menu navigation
//   gps_functions.ino - GPS setup, loop, time functions, data logging
//   replay.ino       - Session replay system
//   sd_functions.ino - SD card setup, track parsing, access management
//   settings.ino     - Persistent JSON settings on SD (/SETTINGS.json)
//   tachometer.ino   - Tachometer ISR and loop processing
//
///////////////////////////////////////////

#include <avr/dtostrf.h>
#include <SPI.h>
#include <nrf_wdt.h>

// #define WOKWI
// #define HAS_DEBUG

// Hides a couple pages and changes some behavior
// todo: make dynamic in next UI version
// #define ENDURANCE_MODE

// Project-wide types and macros - MUST be included before Arduino
// auto-generates function prototypes from the other .ino files,
// otherwise custom types (ButtonState, TrackLayout, etc.) won't
// be resolved in function signatures.
#include "project.h"

// SparkFun GPS library must be included here (in the top include block)
// so that UBX_NAV_PVT_data_t is in scope when Arduino auto-generates
// function prototypes for the onPVTReceived() callback.
#include <SparkFun_u-blox_GNSS_v3.h>
#include "gps_config.h"
#include <DovesLapTimer.h>
#include <CourseManager.h>

// SdFat configuration. SD_FAT_TYPE must be defined BEFORE SdFat.h is
// processed for the first time, which means before any module header
// that pulls it in (e.g. replay.h).
//   0 = bare SdFat/File   (Wokwi only)
//   1 = SdFat32/File32    (real hardware — FAT16/FAT32)
//   2 = SdExFat/ExFile
//   3 = SdFs/FsFile
#ifdef WOKWI
#define SD_FAT_TYPE 0
#define PIN_SPI_CS -1
#else
#define SD_FAT_TYPE 1
#define PIN_SPI_CS -1  // CS is grounded on the gry-box revision
#endif
// 2 MHz SPI for EMI tolerance in ignition environments — 12.5x below
// the SdFat default (25 MHz) but still fast enough for 25 Hz logging
// and the BLE-2M file transfer ceiling.
#define SPI_SPEED SD_SCK_MHZ(2)

#include "SdFat.h"
#include "sdios.h"

// Module interfaces. Each header documents its module's public
// surface and pulls in any library types those signatures need.
#include "accelerometer.h"
#include "bluetooth.h"
#include "display_pages.h"
#include "display_ui.h"
#include "dovex_header.h"
#include "gps_functions.h"
#include "haversine.h"
#include "replay.h"
#include "sd_functions.h"
#include "settings.h"
#include "tachometer.h"

///////////////////////////////////////////
// BATTERY CONFIGURATION
///////////////////////////////////////////

// designed for seeed NRF52840 which comes with a charge circut
#define VREF 3.6
#define ADC_MAX 4096

unsigned long lastBatteryCheck;
int batteryUpdateInterval = 5000;
float lastBatteryVoltage;

float getBatteryVoltage() {
  #ifdef WOKWI
  return 3.75;
  #else
  unsigned int adcCount = analogRead(PIN_VBAT);
  float adcVoltage = adcCount * VREF / ADC_MAX;
  // Nominal divider: (1000+510)/510 = 2.9608, but reads ~2% low due to
  // resistor/VREF tolerances (4.11V observed at true 4.20V full charge).
  // Calibrated: 2.9608 * (4.20/4.11) = 3.024
  return adcVoltage * 3.024;
  #endif
}

int getBatteryPercent(float voltage) {
  // LiPo range: 3.3V (cutoff) to 4.2V (full charge)
  return constrain((int)((voltage - 3.3) / 0.9 * 100), 0, 100);
}

///////////////////////////////////////////
// LAP TIMER / SESSION STATE
///////////////////////////////////////////

double crossingThresholdMeters = 7.0;
unsigned long gpsFrameStartTime;
unsigned long gpsFrameEndTime;
unsigned long gpsFrameCounter;
float gpsFrameRate = 0.0;

CourseManager* courseManager = nullptr;
TrackConfig activeTrackConfig;
bool trackDetected = false;
int detectedTrackIndex = -1;
unsigned long idleStartTime = 0;
bool idleTimerRunning = false;
bool raceActive = false;
unsigned long raceSessionStartedAt = 0;  // For auto-idle grace period after RPM wake

// Runtime settings (loaded from SD in setup)
float settingLapDetectionDistance = 7.0;
float settingWaypointDetectionDistance = 30.0;
float settingWaypointSpeed = 30.0;
char settingDriverName[32] = "Driver";

// Track manifest for proximity detection
TrackManifestEntry trackManifest[MAX_LOCATIONS];
int trackManifestCount = 0;

// DOVEX replay globals (populated by parseDovexHeader in replay.ino)
char dovexReplayDatetime[24];
char dovexReplayDriver[32];
char dovexReplayCourseName[32];
char dovexReplayShortName[16];
char dovexReplayBestLap[16];
char dovexReplayOptimal[16];

// Sleep mode state
bool sleepModeActive = false;
bool sleepGpsWakeActive = false;         // GPS periodic fix in progress
unsigned long sleepEnteredAt = 0;
unsigned long sleepLastGpsWake = 0;
unsigned long sleepGpsWakeStartedAt = 0;
unsigned long menuIdleStartTime = 0;     // For 5-min auto-sleep
bool menuIdleTimerRunning = false;
bool chargingModeActive = false;           // USB connected during sleep
unsigned long chargeDisplayOnAt = 0;       // 0 = display off, >0 = millis() when turned on

// Button hold tracking (for long-press combos)
unsigned long btn1HoldStart = 0;
unsigned long btn2HoldStart = 0;
unsigned long btn3HoldStart = 0;
bool btn1Held = false;
bool btn2Held = false;
bool btn3Held = false;

///////////////////////////////////////////
// PROJECT DEFINES
///////////////////////////////////////////

#define SD_CARD_LOGGING_ENABLED
// MAX_LOCATIONS, MAX_LOCATION_LENGTH, MAX_LAYOUTS, MAX_LAYOUT_LENGTH
// are now defined in project.h for use by project-wide structs
#define FILEPATH_MAX 50        // "/TRACKS/" (8) + name (13) + ".json" (5) + null = 27, using 50 for safety
#include <string.h>

///////////////////////////////////////////
// BUTTON CONFIGURATION
//
// HARDWARE EMI RECOMMENDATIONS FOR BUTTONS:
// Phantom button presses can occur from EMI coupling, especially from
// the tachometer signal. To improve button reliability:
//
// 1. RC FILTER: Add 10K resistor + 100nF cap from each button pin to GND
//    This creates a ~160Hz low-pass filter that eliminates high-freq noise
// 2. WIRE ROUTING: Keep button wires away from tach/ignition wiring
// 3. SHIELDING: If buttons are on a ribbon cable, add ground wire between signals
// 4. FERRITE: Add ferrite bead on button cable near MCU for extra HF rejection
//
// The software debouncing below uses multi-sample verification to reject
// transient noise spikes that get through hardware filtering.
///////////////////////////////////////////

// ButtonState, TrackLayout structs defined in project.h
// debug/debugln macros defined in project.h

///////////////////////////////////////////
// BLUETOOTH (BLE) GLOBALS
///////////////////////////////////////////
#include <bluefruit.h>

// BLE Service & Characteristics
BLEService fileService = BLEService(0x1820);
BLECharacteristic fileListChar = BLECharacteristic(0x2A3D);
BLECharacteristic fileRequestChar = BLECharacteristic(0x2A3E);
BLECharacteristic fileDataChar = BLECharacteristic(0x2A3F);
BLECharacteristic fileStatusChar = BLECharacteristic(0x2A40);

// BLE state variables
bool bleInitialized = false;
bool bleActive = false;
bool bleConnected = false;
bool bleTransferInProgress = false;
uint32_t bleFileSize = 0;
uint32_t bleBytesTransferred = 0;
uint16_t bleNegotiatedMtu = 23;
bool bleWaitingForMTU = false;         // Deferred MTU negotiation (avoids delay in callback)
unsigned long bleMTURequestTime = 0;   // Timestamp when MTU was requested
uint16_t bleMTUConnHandle = 0;        // Connection handle for deferred MTU read
// Note: bleCurrentFile is declared after SdFat include

///////////////////////////////////////////
// TACHOMETER CONFIGURATION
//
// HARDWARE EMI RECOMMENDATIONS:
// The tach input (D0) picks up inductive kickback from ignition systems.
// To reduce phantom readings and noise coupling to other GPIO (buttons):
//
// 1. SHIELDING: Run tach signal wire in shielded cable, ground shield at MCU end only
// 2. FILTERING: Add RC low-pass filter at input: 1K resistor + 100nF cap to GND
//    This creates ~1.6kHz cutoff, plenty fast for 20,000 RPM (333Hz)
// 3. CLAMPING: Add TVS diode or zener (5.1V) from D0 to GND for spike protection
// 4. SEPARATION: Keep tach wiring physically away from button wires
// 5. PULL-DOWN: Ensure 10K pull-down on D0 to prevent floating when no signal
//
// Signal characteristics: Magneto/CDI typically produces sharp negative-going
// pulses with significant ringing. The debounce timing below filters this.
///////////////////////////////////////////

const int tachInputPin = D0;
volatile int tachLastReported = 0;  // Volatile: written by TACH_LOOP, read by display/logging/sleep
int topTachReported = 0;

// Debounce timing: ignore pulses faster than this (filters ignition ringing)
// 3000us = 3ms minimum gap, allows up to 20,000 RPM max (333Hz)
static const uint32_t tachMinPulseGapUs = 3000;
volatile uint32_t tachLastPulseUs = 0;

// Sleep wake detection: set true by ISR on any valid pulse
volatile bool tachHavePeriod = false;

// Ring buffer: ISR writes pulse timestamps, TACH_LOOP reads and computes periods.
// Single-producer (ISR writes head), single-consumer (TACH_LOOP reads tail).
// 16 entries handles up to 20k RPM with 48ms of main-loop stall margin.
static const uint8_t TACH_RING_SIZE = 16;
volatile uint32_t tachRingBuf[TACH_RING_SIZE];
volatile uint8_t  tachRingHead = 0;  // ISR write index (only ISR writes)
static uint8_t    tachRingTail = 0;  // Main-loop read index (only TACH_LOOP writes)

// Tunable constants
static const float tachRevsPerPulse = 1.0f;          // Wasted spark = 1 pulse/rev
static const uint32_t tachStopTimeoutUs = 500000;    // 500ms = engine stopped

///////////////////////////////////////////
// ACCELEROMETER GLOBALS
///////////////////////////////////////////
#include <LSM6DS3.h>
LSM6DS3 accelIMU(I2C_MODE, 0x6A);
bool accelAvailable = false;
float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;

///////////////////////////////////////////
// GPS GLOBALS
///////////////////////////////////////////
SFE_UBLOX_GNSS_SERIAL myGNSS;
bool gpsInitialized = false;  // Safety flag - true only after successful GPS init

// Cached PVT data — updated by onPVTReceived() callback from checkCallbacks()
struct GpsData {
  double latitudeDegrees;
  double longitudeDegrees;
  double altitude;       // meters
  double speed;          // knots (for DovesLapTimer compatibility)
  double HDOP;
  double heading;            // degrees (0-360), heading of motion
  double horizontalAccuracy; // meters, horizontal accuracy estimate
  int satellites;
  bool fix;
  bool timeValid;        // true only when the module reports validDate+validTime+fullyResolved
  uint16_t year;         // 2-digit (e.g. 25 for 2025) for compat with existing code
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t seconds;
  uint16_t milliseconds;
} gpsData = {};

volatile bool gpsDataFresh = false;  // Set by PVT callback, cleared by GPS_LOOP()

// GPS wake validation: tracks whether GPS is producing data after wake from sleep.
// GPS_WAKE() sets gpsWakeTime and clears gpsWakeValidated. GPS_LOOP() sets
// gpsWakeValidated=true on first PVT arrival. If 5 seconds pass without PVT,
// GPS_LOOP() triggers baud recovery and reconfiguration.
unsigned long gpsWakeTime = 0;
bool gpsWakeValidated = true;  // Start true (validated at boot by GPS_SETUP)

float gps_speed_mph = 0.0;

// GPS-lock hold: when a race session is running with the engine turning but
// the GPS has no valid time/position lock yet, we cannot name or open the
// log file (doing so produced garbage-dated files that corrupted on reboot).
// Instead of faulting, we pin the user to the tachometer page and keep
// waiting. Cleared automatically once the log file is created (lock acquired).
bool gpsLockHoldActive = false;

///////////////////////////////////////////
// LAP HISTORY
///////////////////////////////////////////
const int lapHistoryMaxLaps = 1000;
unsigned long lastLap = 0;
unsigned long lapHistory[lapHistoryMaxLaps];
int lapHistoryCount = 0;

void checkForNewLapData() {
  // Read from active timer (CourseManager owns either the course timer
  // or the Lap Anything waypoint timer).
  unsigned long activeLapTime = 0;
  if (courseManager != nullptr) {
    if (courseManager->isLapAnythingActive()) {
      activeLapTime = courseManager->getLapAnythingTimer()->getLastLapTime();
    } else if (courseManager->getActiveTimer() != nullptr) {
      activeLapTime = courseManager->getActiveTimer()->getLastLapTime();
    }
  }
  if (lapHistoryCount < lapHistoryMaxLaps && activeLapTime != 0 && activeLapTime != lastLap) {
    lastLap = activeLapTime;
    lapHistory[lapHistoryCount] = lastLap;
    lapHistoryCount++;
    debugln(F("New lap added to history..."));
  }
}

///////////////////////////////////////////
// REPLAY SYSTEM GLOBALS
///////////////////////////////////////////

// Replay file list - reduced sizes for memory constraints
#define MAX_REPLAY_FILES 20
#define MAX_REPLAY_FILENAME_LENGTH 48
char replayFiles[MAX_REPLAY_FILES][MAX_REPLAY_FILENAME_LENGTH];
int numReplayFiles = 0;
int selectedReplayFile = -1;

// Replay state (DOVEX instant-replay; populated by parseDovexHeader)
bool replayProcessingComplete = false;

///////////////////////////////////////////
// SD CARD GLOBALS
// SD_FAT_TYPE / PIN_SPI_CS / SPI_SPEED and the SdFat.h include moved
// to the top of this file so module headers (replay.h) see them.
///////////////////////////////////////////

SdFat SD;
File file; //buffer
File trackDir;
File trackFile;
File dataFile;
File replayFile;

///////////////////////////////////////////
// SD CARD ACCESS STATE MANAGEMENT
// Prevents race conditions between logging, replay, and BLE file transfers
///////////////////////////////////////////
// Note: Using #define instead of enum to avoid Arduino preprocessor issues
// (Arduino generates function prototypes before seeing enum definitions)
#define SD_ACCESS_NONE         0   // SD card not in use by any subsystem
#define SD_ACCESS_LOGGING      1   // Data logging active (dataFile in use)
#define SD_ACCESS_REPLAY       2   // Replay mode active (replayFile in use)
#define SD_ACCESS_BLE_TRANSFER 3   // BLE file transfer active (bleCurrentFile in use)
#define SD_ACCESS_TRACK_PARSE  4   // Track file parsing (temporary, should release quickly)

volatile int currentSDAccess = SD_ACCESS_NONE;

// Replay function prototypes (must be after SdFat include for File type)
bool buildReplayFileList();
bool readReplayLine(File& file, char* buffer, int bufferSize);
double haversineDistanceMiles(double lat1, double lng1, double lat2, double lng2);
void resetReplayState();
bool parseDovexHeader(const char* filename);

// SD state flags
bool sdSetupSuccess = false;
bool sdTrackSuccess = false;
bool sdDataLogInitComplete = false;
bool enableLogging = false;

unsigned long lastCardFlush = 0;
unsigned long lastLogCreateAttempt = 0;  // Throttles log-file open retries (ms)
const char trackFolder[8] = "/TRACKS";

char locations[MAX_LOCATIONS][MAX_LOCATION_LENGTH]; // 13-char FAT16 name limit
int numOfLocations = 0;

///////////////////////////////////////////
// JSON PARSING GLOBALS
///////////////////////////////////////////
#include <ArduinoJson.h>
#ifdef WOKWI
#define JSON_BUFFER_SIZE 1024
#else
// 4 KB handles tracks with up to 10 courses with full sector data
#define JSON_BUFFER_SIZE 4096
#endif

// extern matches the forward declaration in sd_functions.h so the
// constants have external linkage; otherwise their default internal
// linkage would mismatch the header.
extern const int PARSE_STATUS_GOOD = 0;
extern const int PARSE_STATUS_LOAD_FAILED = 5;
extern const int PARSE_STATUS_PARSE_FAILED = 10;

char tracks[MAX_LAYOUTS][MAX_LAYOUT_LENGTH];
TrackLayout trackLayouts[MAX_LAYOUTS];
int numOfTracks = 0;

// Track metadata (parsed from new JSON object format)
TrackMetadata activeTrackMetadata;

// trackManifest is declared with the session-state globals above

///////////////////////////////////////////
// BLE FILE HANDLE (after SdFat include)
///////////////////////////////////////////
File32 bleCurrentFile;

///////////////////////////////////////////
// BUTTON GLOBALS
///////////////////////////////////////////
ButtonState button1;
ButtonState *btn1 = &button1;
ButtonState button2;
ButtonState *btn2 = &button2;
ButtonState button3;
ButtonState *btn3 = &button3;

float epsilonPrecision = 0.001;

// Debounce settings - tuned for EMI rejection while maintaining responsiveness
// 200ms allows ~5 presses/sec which is plenty fast for menu navigation
// Edge detection ensures button must be released before registering again
int buttonPressIntv = 500;
int buttonHoldIntv = 1000;
int antiBounceIntv = 200;
const int BUTTON_SAMPLE_COUNT = 3;      // Number of samples to take
const int BUTTON_SAMPLE_DELAY_US = 500; // Microseconds between samples

bool recentlyChanged = false;

///////////////////////////////////////////
// DISPLAY GLOBALS
///////////////////////////////////////////

// uses adafruit display libraries
#include <Wire.h>

#ifdef WOKWI
// #define USE_1306_DISPLAY // remove to use SH110X oled
#endif
// #define USE_1306_DISPLAY // remove to use SH110X oled

#include "images.h"
#include "display_config.h"
int displayUpdateRateHz = 3;
unsigned long displayLastUpdate;

// Page constants
const int PAGE_BOOT = 999;
const int PAGE_TEST = 995;
const int PAGE_RC_ERROR = 990;

// main menu (shown after boot)
const int PAGE_MAIN_MENU = -1;
const int PAGE_BLUETOOTH = -2;
const int PAGE_REPLAY_FILE_SELECT = -3;
const int PAGE_REPLAY_RESULTS = -8;
const int PAGE_REPLAY_EXIT = -9;

// running menu (these must be in order)
const int GPS_DEBUG = 3;
const int GPS_STATS = 4;

#ifdef ENDURANCE_MODE
  const int GPS_SPEED = 5;
  const int GPS_LAP_TIME = 6;
  const int GPS_LAP_PACE = 7;
  const int GPS_LAP_BEST = 8;
  const int LOGGING_STOP = 9;

  const int GPS_LAP_LIST = 1002;
#else
  const int GPS_SPEED = 5;
  const int TACHOMETER = 6;
  const int GPS_LAP_TIME = 7;
  const int GPS_LAP_PACE = 8;
  const int GPS_LAP_BEST = 9;
  const int OPTIMAL_LAP = 10;
  const int GPS_LAP_LIST = 11;
  const int LOGGING_STOP = 12;
#endif

// end menu
const int LOGGING_STOP_CONFIRM = 90;
const int PAGE_INTERNAL_WARNING = 100;
const int PAGE_INTERNAL_FAULT = 105;

bool displayInverted = false;
int currentPage = PAGE_BOOT;
int lastPage = 0;

// "pageStart" defines where the UI starts, you cannot backup beyond this
#ifdef ENDURANCE_MODE
  const int runningPageStart = GPS_SPEED;
#else
  const int runningPageStart = GPS_STATS; //GPS_DEBUG;
#endif

int runningPageEnd = LOGGING_STOP; // only changes if sd:/tracks not found

// Display state
int menuSelectionIndex = 0;
bool paceFlashStatus = false;
bool notificationFlash = false;
char internalNotification[64] = "N/A";
bool calculatingFlip = false;
const int lapsPerPage = 3;
int current_lap_list_page = 0;
int lap_list_pages = 1;

///////////////////////////////////////////
// WATCHDOG TIMER
// nRF52840 hardware WDT - recovers from any lockup within ~4 seconds.
// Primary defense against I2C bus hangs, SD card stalls, etc.
///////////////////////////////////////////

void wdtSetup() {
  NRF_WDT->CONFIG = WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos;  // Keep running in sleep
  NRF_WDT->CRV = 4 * 32768;  // ~4 second timeout (32768 Hz clock)
  NRF_WDT->RREN = WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos;      // Enable reload register 0
  NRF_WDT->TASKS_START = 1;   // Start WDT (cannot be stopped once started)
}

void wdtPet() {
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;  // Feed the watchdog
}

///////////////////////////////////////////
// SETUP
///////////////////////////////////////////

void setup() {
#ifdef HAS_DEBUG
  Serial.begin(9600);
  while (!Serial);
#endif

  #ifndef WOKWI
    analogReadResolution(ADC_RESOLUTION);
    pinMode(PIN_VBAT, INPUT);
    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, LOW);
    // Enable fast charging (~100mA vs default ~50mA)
    // PIN_CHARGING_CURRENT = P0.13 = HICHG pin on BQ25100 charge IC
    pinMode(PIN_CHARGING_CURRENT, OUTPUT);
    digitalWrite(PIN_CHARGING_CURRENT, HIGH);
    lastBatteryCheck = millis();
    lastBatteryVoltage = getBatteryVoltage();
  #endif

  displaySetup();

  // setup sd card and confirm we can read track list
  sdSetupSuccess = SD_SETUP();
  sdTrackSuccess = buildTrackList();
  if(sdSetupSuccess && sdTrackSuccess) {
    debugln(F("Obtained Track List"));
    for (int i = 0; i < numOfLocations; i++) {
      char filepath[FILEPATH_MAX];
      makeFullTrackPath(locations[i], filepath);
      debugln(filepath);
    }
  }

  // Load settings from SD (creates defaults on first boot)
  SETTINGS_SETUP();

  ACCEL_SETUP();

  GPS_SETUP();

  // Read settings into runtime variables
  {
    char buf[48];
    if (getSetting("lap_detection_distance", buf, sizeof(buf))) {
      settingLapDetectionDistance = atof(buf);
      if (settingLapDetectionDistance <= 0) settingLapDetectionDistance = 7.0;
    }
    if (getSetting("waypoint_detection_distance", buf, sizeof(buf))) {
      settingWaypointDetectionDistance = atof(buf);
      if (settingWaypointDetectionDistance <= 0) settingWaypointDetectionDistance = 30.0;
    }
    if (getSetting("waypoint_speed", buf, sizeof(buf))) {
      settingWaypointSpeed = atof(buf);
      if (settingWaypointSpeed <= 0) settingWaypointSpeed = 30.0;
    }
    if (getSetting("driver_name", buf, sizeof(buf))) {
      strncpy(settingDriverName, buf, sizeof(settingDriverName) - 1);
      settingDriverName[sizeof(settingDriverName) - 1] = '\0';
    }
    crossingThresholdMeters = settingLapDetectionDistance;
    debug(F("Settings loaded: lap_dist="));
    debug(settingLapDetectionDistance);
    debug(F(" wp_dist="));
    debug(settingWaypointDetectionDistance);
    debug(F(" wp_speed="));
    debug(settingWaypointSpeed);
    debug(F(" driver="));
    debugln(settingDriverName);
  }

  if (!sdSetupSuccess) {
    strncpy(internalNotification, "SD Init failed!\n\nlogging not possible!", sizeof(internalNotification) - 1);
    internalNotification[sizeof(internalNotification) - 1] = '\0';
    switchToDisplayPage(PAGE_INTERNAL_FAULT);
  } else if (sdSetupSuccess && !sdTrackSuccess) {
    // No TRACKS folder is fine — Lap Anything will handle it
    debugln(F("No TRACKS folder — Lap Anything will activate"));
    switchToDisplayPage(PAGE_MAIN_MENU);
  } else {
    switchToDisplayPage(PAGE_MAIN_MENU);
  }

  // tachometer
  pinMode(tachInputPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(tachInputPin), TACH_COUNT_PULSE, FALLING);

  // Start hardware watchdog LAST - everything above must complete before
  // the 4-second timeout starts counting. If setup itself hangs, the
  // device won't boot-loop because WDT hasn't started yet.
  #ifndef WOKWI
  wdtSetup();
  debugln(F("Watchdog timer started (~4s timeout)"));
  #endif
}

///////////////////////////////////////////
// COURSE / TIMER HELPER FUNCTIONS
///////////////////////////////////////////

/**
 * @brief Get the active timer pointer for display/lap-history reads.
 * Returns whichever timer is active: course timer, lap anything, or nullptr.
 */
DovesLapTimer* getActiveTimerDLT() {
  if (courseManager == nullptr) return nullptr;
  if (courseManager->isLapAnythingActive()) return nullptr;
  return courseManager->getActiveTimer();
}

WaypointLapTimer* getActiveTimerWLT() {
  if (courseManager == nullptr) return nullptr;
  if (courseManager->isLapAnythingActive()) return courseManager->getLapAnythingTimer();
  return nullptr;
}

// Unified getter helpers for display pages
bool activeTimerRaceStarted() {
  DovesLapTimer* dlt = getActiveTimerDLT();
  if (dlt) return dlt->getRaceStarted();
  WaypointLapTimer* wlt = getActiveTimerWLT();
  if (wlt) return wlt->getRaceStarted();
  return false;
}

bool activeTimerCrossing() {
  DovesLapTimer* dlt = getActiveTimerDLT();
  if (dlt) return dlt->getCrossing();
  WaypointLapTimer* wlt = getActiveTimerWLT();
  if (wlt) return wlt->getCrossing();
  return false;
}

int activeTimerLaps() {
  DovesLapTimer* dlt = getActiveTimerDLT();
  if (dlt) return dlt->getLaps();
  WaypointLapTimer* wlt = getActiveTimerWLT();
  if (wlt) return wlt->getLaps();
  return 0;
}

unsigned long activeTimerCurrentLapTime() {
  DovesLapTimer* dlt = getActiveTimerDLT();
  if (dlt) return dlt->getCurrentLapTime();
  WaypointLapTimer* wlt = getActiveTimerWLT();
  if (wlt) return wlt->getCurrentLapTime();
  return 0;
}

unsigned long activeTimerLastLapTime() {
  DovesLapTimer* dlt = getActiveTimerDLT();
  if (dlt) return dlt->getLastLapTime();
  WaypointLapTimer* wlt = getActiveTimerWLT();
  if (wlt) return wlt->getLastLapTime();
  return 0;
}

unsigned long activeTimerBestLapTime() {
  DovesLapTimer* dlt = getActiveTimerDLT();
  if (dlt) return dlt->getBestLapTime();
  WaypointLapTimer* wlt = getActiveTimerWLT();
  if (wlt) return wlt->getBestLapTime();
  return 0;
}

int activeTimerBestLapNumber() {
  DovesLapTimer* dlt = getActiveTimerDLT();
  if (dlt) return dlt->getBestLapNumber();
  WaypointLapTimer* wlt = getActiveTimerWLT();
  if (wlt) return wlt->getBestLapNumber();
  return 0;
}

float activeTimerPaceDifference() {
  DovesLapTimer* dlt = getActiveTimerDLT();
  if (dlt) return dlt->getPaceDifference();
  WaypointLapTimer* wlt = getActiveTimerWLT();
  if (wlt) return wlt->getPaceDifference();
  return 0.0;
}

float activeTimerTotalDistance() {
  DovesLapTimer* dlt = getActiveTimerDLT();
  if (dlt) return dlt->getTotalDistanceTraveled();
  WaypointLapTimer* wlt = getActiveTimerWLT();
  if (wlt) return wlt->getTotalDistanceTraveled();
  return 0.0;
}

unsigned long activeTimerOptimalLapTime() {
  DovesLapTimer* dlt = getActiveTimerDLT();
  if (dlt) return dlt->getOptimalLapTime();
  return 0;
}

bool activeTimerSectorsConfigured() {
  DovesLapTimer* dlt = getActiveTimerDLT();
  if (dlt) return dlt->areSectorLinesConfigured();
  return false;
}

/**
 * @brief Scan track manifest for closest match to current GPS position
 * Creates CourseManager when a match is found within 5 miles
 */
void trackDetectionLoop() {
  if (trackDetected || !gpsData.fix || trackManifestCount == 0) return;

  double bestDist = 999999.0;
  int bestIndex = -1;

  for (int i = 0; i < trackManifestCount; i++) {
    double dist = haversineDistanceMiles(
      gpsData.latitudeDegrees, gpsData.longitudeDegrees,
      trackManifest[i].lat, trackManifest[i].lon
    );
    if (dist < bestDist) {
      bestDist = dist;
      bestIndex = i;
    }
  }

  if (bestIndex >= 0 && bestDist <= TRACK_DETECT_RADIUS_MILES) {
    debug(F("Track detected: "));
    debug(trackManifest[bestIndex].filename);
    debug(F(" ("));
    debug(bestDist, 2);
    debugln(F(" miles)"));

    detectedTrackIndex = bestIndex;

    // Use manifest filename directly to build filepath — locations[] may
    // truncate names longer than MAX_LOCATION_LENGTH (12 chars, old FAT16
    // 8.3 limit), causing strcmp mismatches that silently skip real tracks.
    {
      char filepath[FILEPATH_MAX];
      makeFullTrackPath(trackManifest[bestIndex].filename, filepath);
      int parseStatus = parseTrackFile(filepath);

      if (parseStatus == PARSE_STATUS_GOOD && numOfTracks > 0) {
        // Build TrackConfig from parsed data
        activeTrackConfig.longName = activeTrackMetadata.longName[0] ? activeTrackMetadata.longName : trackManifest[bestIndex].filename;
        activeTrackConfig.shortName = activeTrackMetadata.shortName[0] ? activeTrackMetadata.shortName : trackManifest[bestIndex].filename;

        // Populate CourseConfig entries
        // Check if any course has lengthFt > 0. CourseDetector uses
        // distance matching to identify which course the driver is on.
        // Without lengthFt, distance can never match and detection gets
        // stuck permanently — no candidates, no rejections, no fallback.
        bool anyHasLength = false;
        activeTrackConfig.courseCount = numOfTracks;
        for (int i = 0; i < numOfTracks && i < MAX_COURSES; i++) {
          activeTrackConfig.courses[i].name = tracks[i];
          activeTrackConfig.courses[i].lengthFt = activeTrackMetadata.courseLengthFt[i];
          activeTrackConfig.courses[i].startALat = trackLayouts[i].start_a_lat;
          activeTrackConfig.courses[i].startALng = trackLayouts[i].start_a_lng;
          activeTrackConfig.courses[i].startBLat = trackLayouts[i].start_b_lat;
          activeTrackConfig.courses[i].startBLng = trackLayouts[i].start_b_lng;
          activeTrackConfig.courses[i].sector2ALat = trackLayouts[i].sector_2_a_lat;
          activeTrackConfig.courses[i].sector2ALng = trackLayouts[i].sector_2_a_lng;
          activeTrackConfig.courses[i].sector2BLat = trackLayouts[i].sector_2_b_lat;
          activeTrackConfig.courses[i].sector2BLng = trackLayouts[i].sector_2_b_lng;
          activeTrackConfig.courses[i].sector3ALat = trackLayouts[i].sector_3_a_lat;
          activeTrackConfig.courses[i].sector3ALng = trackLayouts[i].sector_3_a_lng;
          activeTrackConfig.courses[i].sector3BLat = trackLayouts[i].sector_3_b_lat;
          activeTrackConfig.courses[i].sector3BLng = trackLayouts[i].sector_3_b_lng;
          activeTrackConfig.courses[i].hasSector2 = trackLayouts[i].hasSector2;
          activeTrackConfig.courses[i].hasSector3 = trackLayouts[i].hasSector3;
          if (activeTrackMetadata.courseLengthFt[i] > 0) anyHasLength = true;
        }

        // If no courses have lengthFt, CourseDetector cannot match by
        // distance and will be stuck forever. Force courseCount=0 so
        // CourseManager activates Lap Anything immediately. Track name
        // metadata is preserved so the display still shows which track.
        if (!anyHasLength) {
          debugln(F("WARNING: No courses have lengthFt — forcing Lap Anything"));
          activeTrackConfig.courseCount = 0;
        }

        // Create CourseManager (delete existing Lap Anything one if RPM-wake created it)
        if (courseManager != nullptr) {
          delete courseManager;
          courseManager = nullptr;
        }
        courseManager = new CourseManager(activeTrackConfig, crossingThresholdMeters);
        courseManager->setSpeedThresholdMph(settingWaypointSpeed);
        courseManager->setWaypointProximityMeters(settingWaypointDetectionDistance);
        courseManager->setDetectionProximityMeters(settingWaypointDetectionDistance);

        trackDetected = true;
        debugln(F("CourseManager created with track data"));
      }
    }

    // If parsing failed or no tracks, create Lap Anything fallback
    if (!trackDetected) {
      createLapAnythingCourseManager();
      trackDetected = true;
    }
  }
}

/**
 * @brief End the current race session: write DOVEX header, close file,
 * clean up CourseManager, reset state. Used by both checkAutoIdle()
 * and LOGGING_STOP_CONFIRM in display_ui.ino.
 */
void endRaceSession() {
  // Write DOVEX metadata header into the reserved region
  if (sdDataLogInitComplete && dataFile.isOpen()) {
    writeDovexHeader();
  }

  // Close log file
  if (dataFile.isOpen()) {
    dataFile.flush();
    dataFile.close();
  }
  releaseSDAccess(SD_ACCESS_LOGGING);
  enableLogging = false;
  sdDataLogInitComplete = false;

  // Clean up CourseManager
  if (courseManager != nullptr) {
    delete courseManager;
    courseManager = nullptr;
  }
  trackDetected = false;
  detectedTrackIndex = -1;
  raceActive = false;
  idleTimerRunning = false;
  idleStartTime = 0;

  // Reset lap history
  lapHistoryCount = 0;
  lastLap = 0;
  memset(lapHistory, 0, sizeof(lapHistory));
  topTachReported = 0;

  debugln(F("Race session ended"));
}

/**
 * @brief Create a fallback CourseManager with no courses (Lap Anything mode).
 * Used when no track is detected, parsing fails, or user enters race manually.
 */
void createLapAnythingCourseManager() {
  if (courseManager != nullptr) return;  // Already exists
  activeTrackConfig.longName = "Unknown";
  activeTrackConfig.shortName = "";
  activeTrackConfig.courseCount = 0;
  courseManager = new CourseManager(activeTrackConfig, crossingThresholdMeters);
  courseManager->setSpeedThresholdMph(settingWaypointSpeed);
  courseManager->setWaypointProximityMeters(settingWaypointDetectionDistance);
  debugln(F("CourseManager created (Lap Anything)"));
}

/**
 * @brief Check for auto-idle: 60s at <2mph ends the session
 */
void checkAutoIdle() {
  if (!raceActive) return;

  // Grace period: don't auto-idle within first 3 minutes of a session.
  // After RPM wake the car is often stationary (warming up, waiting for
  // track session) and GPS needs time to reacquire. Without this, the
  // 60s idle timer kills the session before the driver even moves.
  if (millis() - raceSessionStartedAt < 180000UL) return;

  if (gps_speed_mph >= 2.0) {
    idleTimerRunning = false;
    idleStartTime = 0;
    return;
  }

  if (!idleTimerRunning) {
    idleTimerRunning = true;
    idleStartTime = millis();
    return;
  }

  if (millis() - idleStartTime >= 60000) {
    debugln(F("Auto-idle: 60s at <2mph — ending session"));
    endRaceSession();
    switchToDisplayPage(PAGE_MAIN_MENU);
  }
}

/**
 * @brief Auto-enter race mode from main menu when driving
 */
void autoRaceModeCheck() {
  if (currentPage != PAGE_MAIN_MENU) return;
  if (currentPage == PAGE_BLUETOOTH || bleConnected) return;

  bool rpmTriggered = tachLastReported > 500;
  bool speedTriggered = gps_speed_mph >= 10.0;

  if (rpmTriggered || speedTriggered) {
    debugln(F("Auto-entering race mode"));
    raceActive = true;
    enableLogging = true;
    raceSessionStartedAt = millis();

    // Create a minimal CourseManager if none exists yet (no track detected)
    createLapAnythingCourseManager();

    // Show tach page if RPM triggered first, otherwise speed page
    switchToDisplayPage(rpmTriggered ? TACHOMETER : GPS_SPEED);
  }
}

/**
 * @brief Maintain the GPS-lock hold state (see gpsLockHoldActive).
 *
 * While a race session wants to log but has no valid GPS time lock yet — so
 * the log file cannot be named/created — and the engine is turning, we pin
 * the user to the tachometer instead of trying to log with a garbage date.
 * The hold latches on once the engine is seen running and clears the moment
 * the log file is created (lock acquired) or the session ends. Logging then
 * begins automatically and normal race-mode navigation resumes.
 */
void updateGpsLockHold() {
  if (raceActive && enableLogging && !sdDataLogInitComplete) {
    // No lock yet (file not created). Latch the hold once the engine turns.
    if (tachLastReported > 0) gpsLockHoldActive = true;
  } else {
    gpsLockHoldActive = false;  // lock acquired (file created) or race ended
  }
}

/**
 * @brief Write DOVEX header metadata into the reserved 1 KB at the
 *        start of the open log file. Layout + padding live in
 *        dovex_header::format(); this just wires up the I/O.
 */
void writeDovexHeader() {
  if (!dataFile.isOpen()) return;

  char datetime[24];
  snprintf(datetime, sizeof(datetime), "20%02d-%02d-%02d %02d:%02d:%02d",
           gpsData.year, gpsData.month, gpsData.day,
           gpsData.hour, gpsData.minute, gpsData.seconds);

  const char* courseName = "Lap Anything";
  const char* shortName  = "";
  if (courseManager != nullptr) {
    const char* cn = courseManager->getActiveCourseName();
    if (cn) courseName = cn;
    shortName = courseManager->getShortName();
  }

  const dovex_header::Metadata meta = {
      datetime,
      settingDriverName,
      courseName,
      shortName,
      activeTimerBestLapTime(),
      activeTimerOptimalLapTime(),
  };

  static char headerBuf[dovex_header::kHeaderSize];
  dovex_header::format(headerBuf, sizeof(headerBuf), meta,
                        lapHistory, lapHistoryCount);

  dataFile.seekSet(0);
  dataFile.write(reinterpret_cast<const uint8_t*>(headerBuf), sizeof(headerBuf));
  dataFile.flush();
  debugln(F("DOVEX header written"));
}

///////////////////////////////////////////
// SLEEP MODE
///////////////////////////////////////////

bool isUsbConnected() {
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

void enterSleepMode() {
  // 1. End race session if active (safety net)
  if (raceActive) endRaceSession();

  // 2. Stop BLE if active (prevents SoftDevice power waste during sleep)
  if (bleActive) BLE_STOP();

  // 3. Turn off display (I2C command, ~10uA sleep current)
  DISPLAY_SLEEP();

  // 3. Put GPS to backup mode
  GPS_SLEEP();

  // 4. Power down IMU (~1mA savings)
  if (accelAvailable) {
    pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
    digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);  // HIGH = disable power
  }

  // 5. Set state
  sleepModeActive = true;
  sleepEnteredAt = millis();
  sleepLastGpsWake = millis();
  // Tach ISR stays attached -- RPM pulses will wake via tachHavePeriod
}

void exitSleepMode(bool rpmWake = false) {
  // 1. Re-enable IMU
  if (accelAvailable) {
    digitalWrite(PIN_LSM6DS3TR_C_POWER, LOW);
    delay(50);
    if (accelIMU.begin() != 0) {
      debugln(F("IMU failed to reinitialize after sleep"));
      accelAvailable = false;
    }
  }

  // 2. Wake GPS
  GPS_WAKE();

  // 3. Wake display
  DISPLAY_WAKE();

  // 4. Reset state
  sleepModeActive = false;
  sleepGpsWakeActive = false;
  menuIdleTimerRunning = false;
  chargingModeActive = false;
  chargeDisplayOnAt = 0;

  // 5. RPM wake → skip main menu, go straight to race mode
  if (rpmWake) {
    debugln(F("RPM wake — entering race mode directly"));
    raceActive = true;
    enableLogging = true;
    raceSessionStartedAt = millis();
    createLapAnythingCourseManager();
    switchToDisplayPage(TACHOMETER);
  } else {
    switchToDisplayPage(PAGE_MAIN_MENU);
  }
}

///////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////

#ifdef HAS_DEBUG
unsigned long loopMaxTime = 0;
#endif

void loop() {
  #ifndef WOKWI
  wdtPet();
  #endif

  if (sleepModeActive) {
    // Check wake triggers
    bool wakeButton = anyButtonPressed();
    bool wakeRpm = tachHavePeriod;  // ISR sets this directly on any valid pulse

    if (wakeRpm) {
      exitSleepMode(true);
      return;
    }
    if (wakeButton && !chargingModeActive) {
      exitSleepMode(false);
      return;
    }

    // ---- CHARGE MODE (USB connected during sleep) ----
    if (isUsbConnected()) {
      if (!chargingModeActive) {
        // First detection of USB: show charging screen for 10s
        chargingModeActive = true;
        DISPLAY_WAKE();
        chargeDisplayOnAt = millis();
        if (chargeDisplayOnAt == 0) chargeDisplayOnAt = 1; // avoid 0 sentinel
      }

      // Button press: re-show display for another 10s
      if (wakeButton && chargeDisplayOnAt == 0) {
        DISPLAY_WAKE();
        chargeDisplayOnAt = millis();
        if (chargeDisplayOnAt == 0) chargeDisplayOnAt = 1;
      }

      // Display management
      if (chargeDisplayOnAt != 0) {
        if (millis() - chargeDisplayOnAt >= CHARGE_DISPLAY_TIMEOUT_MS) {
          DISPLAY_SLEEP();
          chargeDisplayOnAt = 0;
        } else if (millis() - displayLastUpdate > (1000 / displayUpdateRateHz)) {
          displayLastUpdate = millis();
          displayPage_sleep_charging();
        }
      }

      // Skip GPS periodic checks and WFE while charging — just loop
      return;
    }

    // USB disconnected: clean up charge mode
    if (chargingModeActive) {
      chargingModeActive = false;
      if (chargeDisplayOnAt != 0) {
        DISPLAY_SLEEP();
      }
      chargeDisplayOnAt = 0;
    }

    // Periodic GPS fix (every SLEEP_GPS_WAKE_INTERVAL)
    if (!sleepGpsWakeActive &&
        (millis() - sleepLastGpsWake >= SLEEP_GPS_WAKE_INTERVAL)) {
      GPS_SLEEP_PERIODIC_CHECK();
    }

    // GPS periodic wake: check for fix or timeout
    if (sleepGpsWakeActive) {
      myGNSS.checkUblox();
      myGNSS.checkCallbacks();
      if (gpsData.fix || (millis() - sleepGpsWakeStartedAt >= SLEEP_GPS_FIX_TIMEOUT)) {
        GPS_SLEEP();
        sleepGpsWakeActive = false;
        sleepLastGpsWake = millis();
      }
    }

    // CPU idle (SoftDevice-safe WFE — any interrupt wakes CPU)
    sd_app_evt_wait();
    return;  // Skip entire normal loop
  }

  #ifdef HAS_DEBUG
  unsigned long loopStart = millis();
  #endif

  // When BLE is active, skip GPS/tach/lap processing for better throughput
  if (bleActive) {
    BLUETOOTH_LOOP();

    // Keep battery voltage fresh for BLE BATT command and display
    if (millis() - lastBatteryCheck > batteryUpdateInterval) {
      lastBatteryCheck = millis();
      lastBatteryVoltage = getBatteryVoltage();
    }

    // Minimal button check for exit
    readButtons();
    if (btn2->pressed) {
      BLE_STOP();
      switchToDisplayPage(PAGE_MAIN_MENU);
    }
    resetButtons();

    // Reduced display rate: 5s during transfer, normal otherwise
    unsigned long displayInterval = bleTransferInProgress ? 5000 : (1000 / displayUpdateRateHz);
    if (millis() - displayLastUpdate > displayInterval) {
      displayLastUpdate = millis();
      displayPage_bluetooth();
    }

    return; // Skip GPS, tach, lap checks while BLE is active
  }

  GPS_LOOP();
  TACH_LOOP();
  ACCEL_LOOP();
  BLUETOOTH_LOOP();

  trackDetectionLoop();
  checkForNewLapData();
  checkAutoIdle();
  autoRaceModeCheck();
  updateGpsLockHold();

  // Button hold detection for sleep/reboot combos
  updateButtonHoldState();

  // Long-press left+right (5s) on main menu -> sleep
  if (currentPage == PAGE_MAIN_MENU &&
      isButtonHeld(1, SLEEP_LONG_PRESS_MS) &&
      isButtonHeld(3, SLEEP_LONG_PRESS_MS)) {
    enterSleepMode();
    return;
  }

  // Reboot combo: select + either side button held 5s (any page)
  if (isButtonHeld(2, SLEEP_LONG_PRESS_MS) &&
      (isButtonHeld(1, SLEEP_LONG_PRESS_MS) || isButtonHeld(3, SLEEP_LONG_PRESS_MS))) {
    NVIC_SystemReset();
  }

  // USB connected on main menu -> auto-sleep for efficient charging
  if (currentPage == PAGE_MAIN_MENU && isUsbConnected()) {
    enterSleepMode();
    return;
  }

  // 5-minute menu idle -> auto-sleep
  if (currentPage == PAGE_MAIN_MENU) {
    if (!menuIdleTimerRunning) {
      menuIdleTimerRunning = true;
      menuIdleStartTime = millis();
    } else if (millis() - menuIdleStartTime >= SLEEP_IDLE_TIMEOUT_MS) {
      enterSleepMode();
      return;
    }
    if (btn1->pressed || btn2->pressed || btn3->pressed) {
      menuIdleStartTime = millis();  // Reset on any button
    }
  } else {
    menuIdleTimerRunning = false;
  }

  calculateGPSFrameRate();

  readButtons();
  displayLoop();
  resetButtons();

  if (tachLastReported > topTachReported) {
    topTachReported = tachLastReported;
  }

  #ifdef HAS_DEBUG
  unsigned long loopElapsed = millis() - loopStart;
  if (loopElapsed > loopMaxTime) {
    loopMaxTime = loopElapsed;
  }
  if (loopElapsed > 100) {
    debug(F("SLOW LOOP: "));
    debug(loopElapsed);
    debug(F("ms (max: "));
    debug(loopMaxTime);
    debugln(F("ms)"));
  }
  #endif
}
