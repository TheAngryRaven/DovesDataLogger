///////////////////////////////////////////
// DovesDataLogger - BirdsEye Main Sketch
//
// This is the main sketch file containing global state, includes,
// setup(), and loop(). All function implementations are split into
// separate module files (Arduino concatenates .ino files automatically):
//
//   bluetooth.ino    - BLE file transfer service
//   display_pages.ino - All display page rendering functions
//   display_ui.ino   - Display setup, button handling, menu navigation
//   gps_functions.ino - GPS setup, loop, time functions, data logging
//   replay.ino       - Session replay system
//   sd_functions.ino - SD card setup, track parsing, access management
//   tachometer.ino   - Tachometer ISR and loop processing
//
///////////////////////////////////////////

#include <avr/dtostrf.h>
#include <SPI.h>

// #define WOKWI
// #define HAS_DEBUG

// Hides a couple pages and changes some behavior
// todo: make dynamic in next UI version
// #define ENDURANCE_MODE

// Project-wide types and macros - MUST be included before Arduino
// auto-generates function prototypes from the other .ino files,
// otherwise custom types (ButtonState, ReplaySample, etc.) won't
// be resolved in function signatures.
#include "project.h"

// SparkFun GPS library must be included here (in the top include block)
// so that UBX_NAV_PVT_data_t is in scope when Arduino auto-generates
// function prototypes for the onPVTReceived() callback.
#include <SparkFun_u-blox_GNSS_v3.h>
#include "gps_config.h"

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
  return adcVoltage * 1510 / 510;
  #endif
}

///////////////////////////////////////////
// LAP TIMER
///////////////////////////////////////////

#include <DovesLapTimer.h>
double crossingThresholdMeters = 7.0;
DovesLapTimer lapTimer(crossingThresholdMeters);
unsigned long gpsFrameStartTime;
unsigned long gpsFrameEndTime;
unsigned long gpsFrameCounter;
float gpsFrameRate = 0.0;
bool trackSelected = false;

///////////////////////////////////////////
// PROJECT DEFINES
///////////////////////////////////////////

#define SD_CARD_LOGGING_ENABLED
#define MAX_LOCATIONS 100
#define MAX_LOCATION_LENGTH 13 // 13 is old dos format for fat16
#define MAX_LAYOUTS 10
#define MAX_LAYOUT_LENGTH 15
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

// ButtonState, TrackLayout, ReplaySample structs defined in project.h

const int RACE_DIRECTION_FORWARD  = 0;
const int RACE_DIRECTION_REVERSE  = 1;

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
unsigned long tachLastUpdate = 0;
volatile int tachLastReported = 0;  // Volatile for ISR/main-loop sharing
int topTachReported = 0;

// Debounce timing: ignore pulses faster than this (filters ignition ringing)
// 3000us = 3ms minimum gap, allows up to 20,000 RPM max (333Hz)
static const uint32_t tachMinPulseGapUs = 3000;
volatile uint32_t tachLastPulseUs = 0;

// Pulse period capture for RPM calculation (written by ISR, read by loop)
volatile uint32_t tachLastPeriodUs = 0;
volatile bool tachHavePeriod = false;

// Software gate to prevent interrupt storm from noisy inductive pickup.
// After accepting a pulse, we ignore subsequent ISR calls for tachMinPulseGapUs.
// This is PURELY a volatile flag - we do NOT use noInterrupts() in the ISR
// because that would disable ALL system interrupts and cause deadlocks.
volatile bool tachInterruptShouldProcess = true;

// Filtered RPM state (updated in main loop, not ISR)
float tachRpmFiltered = 0.0f;

// Tunable settings
const int tachUpdateRateHz = 3;
static const float tachRevsPerPulse = 1.0f;     // Magneto 4T single: wasted spark = 1 pulse/rev
static const float tachFilterAlpha = 0.20f;     // EMA filter: 0=smooth, 1=instant
static const uint32_t tachStopTimeoutUs = 500000; // 500ms with no pulse = engine stopped

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
  uint16_t year;         // 2-digit (e.g. 25 for 2025) for compat with existing code
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t seconds;
  uint16_t milliseconds;
} gpsData = {};

volatile bool gpsDataFresh = false;  // Set by PVT callback, cleared by GPS_LOOP()

double crossingPointALat = 0.00;
double crossingPointALng = 0.00;
double crossingPointBLat = 0.00;
double crossingPointBLng = 0.00;
float gps_speed_mph = 0.0;

///////////////////////////////////////////
// LAP HISTORY
///////////////////////////////////////////
const int lapHistoryMaxLaps = 1000;
unsigned long lastLap = 0;
unsigned long lapHistory[lapHistoryMaxLaps];
int lapHistoryCount = 0;

void checkForNewLapData() {
  if (lapHistoryCount < lapHistoryMaxLaps && lapTimer.getLastLapTime() != lastLap) {
    lastLap = lapTimer.getLastLapTime();
    lapHistory[lapHistoryCount] = lastLap;
    // todo: watdo when too many?
    lapHistoryCount++;
    debugln(F("New lap added to history..."));
  }
}

///////////////////////////////////////////
// REPLAY SYSTEM GLOBALS
///////////////////////////////////////////

// Track detection threshold in miles (configurable)
const float REPLAY_TRACK_DETECTION_THRESHOLD_MILES = 20.0;

// Replay file list - reduced sizes for memory constraints
#define MAX_REPLAY_FILES 20
#define MAX_REPLAY_FILENAME_LENGTH 48  // Must fit longest filename (e.g. OKC_Normal_fwd_2025_1123_1459.nmea = 36 chars)
char replayFiles[MAX_REPLAY_FILES][MAX_REPLAY_FILENAME_LENGTH];
int numReplayFiles = 0;
int selectedReplayFile = -1;

// Replay state
bool replayModeActive = false;
bool replayProcessingComplete = false;
int replayDetectedTrackIndex = -1;

// Replay statistics
float replayMaxSpeed = 0.0;
int replayMaxRpm = 0;
unsigned long replayTotalSamples = 0;
unsigned long replayProcessedSamples = 0;

// Line buffer for streaming file reads (bounded memory)
// NMEA max is 82 chars, DOVE CSV lines ~100 chars, 128 is plenty
#define REPLAY_LINE_BUFFER_SIZE 128
char replayLineBuffer[REPLAY_LINE_BUFFER_SIZE];

// ReplaySample struct defined in project.h

///////////////////////////////////////////
// SD CARD GLOBALS
///////////////////////////////////////////

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#ifdef WOKWI
#define SD_FAT_TYPE 0
#define PIN_SPI_CS -1 //53
#else
#define SD_FAT_TYPE 1
#define PIN_SPI_CS -1 // grounded on gry box
// #define PIN_SPI_CS 3 // todo: ground out for A0/battery voltage...
#endif

// Reduced from 25MHz to 4MHz for better EMI resistance
// Slower speed = more robust against ignition noise
#define SPI_SPEED SD_SCK_MHZ(4)

#include "SdFat.h"
#include "sdios.h"

SdFat SD;
File file; //buffer
File trackDir;
File trackFile;
File dataFile;

// Replay file handle (must be after SdFat include)
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
bool parseDoveLine(char* line, ReplaySample& sample);  // non-const, parses in-place
bool parseNmeaSentence(char* line, ReplaySample& sample);  // non-const, parses in-place
bool extractGpsPointFromReplayFile(const char* filename, double& lat, double& lng);
bool extractGpsPointFromTrackFile(int trackIndex, double& lat, double& lng);
double haversineDistanceMiles(double lat1, double lng1, double lat2, double lng2);
int detectTrackForReplayFile(const char* filename);
void processReplayFile();
void resetReplayState();

// SD state flags
// do we really need all of these flags
bool sdSetupSuccess = false;
bool sdTrackSuccess = false;
bool sdDataLogInitComplete = false;
bool enableLogging = false;

unsigned long lastCardFlush = 0;
const char trackFolder[8] = "/TRACKS";

// could probably do all of this better
int selectedLocation = -1;
char locations[MAX_LOCATIONS][MAX_LOCATION_LENGTH]; // 13 is old dos format for fat16
int numOfLocations = 0;

///////////////////////////////////////////
// JSON PARSING GLOBALS
///////////////////////////////////////////
#include <ArduinoJson.h>
#ifdef WOKWI
#define JSON_BUFFER_SIZE 1024
#else
// might need to make bigger for more layouts, test and expiriment
#define JSON_BUFFER_SIZE 2048
#endif

const int PARSE_STATUS_GOOD = 0;
const int PARSE_STATUS_LOAD_FAILED = 5;
const int PARSE_STATUS_PARSE_FAILED = 10;

char tracks[MAX_LAYOUTS][MAX_LAYOUT_LENGTH];
TrackLayout trackLayouts[MAX_LAYOUTS];
int numOfTracks = 0;

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
const int PAGE_REPLAY_DETECTING = -4;
const int PAGE_REPLAY_SELECT_TRACK = -5;
const int PAGE_REPLAY_SELECT_DIRECTION = -6;
const int PAGE_REPLAY_PROCESSING = -7;
const int PAGE_REPLAY_RESULTS = -8;
const int PAGE_REPLAY_EXIT = -9;

// boot menu
const int PAGE_SELECT_LOCATION = 0;
const int PAGE_SELECT_TRACK = 1;
const int PAGE_SELECT_DIRECTION = 2;

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

// Track/direction selection
int selectedTrack = -1;
int selectedDirection = -1;

// Display state
int menuSelectionIndex = 0;
bool paceFlashStatus = false;
bool notificationFlash = false;
String internalNotification = "N/A";
bool calculatingFlip = false;
const int lapsPerPage = 3;
int current_lap_list_page = 0;
int lap_list_pages = 1;

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

  GPS_SETUP();

  if (!sdSetupSuccess) {
    internalNotification = "SD Init failed!\n\nlogging not possible!";
    switchToDisplayPage(PAGE_INTERNAL_FAULT);
  } else if (sdSetupSuccess && !sdTrackSuccess) {
    internalNotification = "sd:/TRACKS not found!\n\nlogging not possible!";
    switchToDisplayPage(PAGE_INTERNAL_FAULT);
  } else {
    switchToDisplayPage(PAGE_MAIN_MENU);
  }

  // tachometer
  pinMode(tachInputPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(tachInputPin), TACH_COUNT_PULSE, FALLING);
}

///////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////

void loop() {
  // When BLE transfer is active, run tight loop for maximum throughput
  if (bleTransferInProgress) {
    BLUETOOTH_LOOP();

    // Minimal button check for exit
    readButtons();
    if (btn2->pressed) {
      // Exit button during transfer
      BLE_STOP();
      switchToDisplayPage(PAGE_MAIN_MENU);
    }
    resetButtons();

    // Update display every 5s during transfer to maximize BLE throughput
    if (millis() - displayLastUpdate > 5000) {
      displayLastUpdate = millis();
      displayPage_bluetooth();
    }

    return; // Skip everything else during transfer
  }

  // Replay processing mode - tight loop for fast file processing
  if (currentPage == PAGE_REPLAY_PROCESSING && replayModeActive && !replayProcessingComplete) {
    // Process replay file
    processReplayFile();

    // Update display less frequently during processing
    if (millis() - displayLastUpdate > 200) {
      displayLastUpdate = millis();
      displayPage_replay_processing();
    }

    // Check if processing complete
    if (replayProcessingComplete) {
      replayModeActive = false;
      switchToDisplayPage(PAGE_REPLAY_RESULTS);
    }

    return; // Skip everything else during replay processing
  }

  GPS_LOOP();
  TACH_LOOP();
  BLUETOOTH_LOOP();

  #ifndef ENDURANCE_MODE
    checkForNewLapData();
  #endif
  calculateGPSFrameRate();

  // Auto-select Race mode if RPM detected on main menu
  if (currentPage == PAGE_MAIN_MENU && tachLastReported > 500) {
    debugln(F("RPM detected on main menu - auto-selecting Race"));
    switchToDisplayPage(PAGE_SELECT_LOCATION);
  }

  readButtons();
  displayLoop();
  resetButtons();

  if (tachLastReported > topTachReported) {
    topTachReported = tachLastReported;
  }
}
