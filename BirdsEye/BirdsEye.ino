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
#include <nrf_gpio.h>  // SENSE-wake pin config for System OFF shutdown
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

// Parked-transfer SPI clock. File transfers (BLE / USB mass storage) only
// happen with the motor off, so the ignition-EMI rationale for the slow 2 MHz
// clock doesn't apply — sdSetTransferSpeed(true) bumps to this for the session
// and reverts to SPI_SPEED afterward. Bump to SD_SCK_MHZ(16) if the board
// proves it can sustain it (the nRF52840 standard SPIM may clamp 16 to 8 MHz).
#define SD_SPI_SPEED_FAST SD_SCK_MHZ(8)

#include "SdFat.h"
#include "sdios.h"

// TinyUSB — provides Adafruit_USBD_MSC / TinyUSBDevice for the USB
// mass-storage transfer mode (usb_msc module). Must precede usb_msc.h.
#include <Adafruit_TinyUSB.h>

// Module interfaces. Each header documents its module's public
// surface and pulls in any library types those signatures need.
#include "accelerometer.h"
#include "bluetooth.h"
#include "camera_ble.h"
#include "display_pages.h"
#include "display_ui.h"
#include "dovex_header.h"
#include "gps_functions.h"
#include "gps_status_page.h"
#include "haversine.h"
#include "replay.h"
#include "sat_bars.h"
#include "sd_format_page.h"
#include "sd_functions.h"
#include "settings.h"
#include "tachometer.h"
#include "usb_msc.h"
#include "wake_cause.h"

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
char settingDeviceName[32] = "BirdsEye";

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

// Main-menu idle tracking (drives auto-shutdown and the USB charge-mode
// entry — see the trigger block at the end of loop()).
unsigned long menuIdleStartTime = 0;
bool menuIdleTimerRunning = false;

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

// OTA + version reporting services (set up in BLE_SETUP()):
//   bledfu - buttonless Secure DFU; a write reboots the board into the
//            bootloader's OTA mode so a companion can flash new firmware.
//   bledis - Device Information Service; publishes FIRMWARE_VERSION so the
//            companion can tell whether an update is available.
BLEDfu bledfu;
BLEDis bledis;

// BLE state variables
bool bleInitialized = false;
bool bleActive = false;
bool bleConnected = false;
// Which subsystem owns the BLE radio (advert set + peripheral slot):
// transfer service vs camera remote. Transitions only on the main loop —
// see the ownership model in bluetooth.h. volatile: read from Bluefruit
// task callbacks for routing decisions.
volatile BleOwner bleOwner = BLE_OWNER_NONE;
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

// Ring buffer: ISR writes pulse timestamps, TACH_LOOP reads and computes periods.
// Single-producer (ISR writes head), single-consumer (TACH_LOOP writes tail).
// The ISR checks full before publishing (one slot sacrificed so head==tail
// means empty) and drops + flags instead of lapping the consumer: SD GC
// stalls can block the main loop for 100 ms–2 s, far past what any sane
// ring size covers at racing RPM. tachRingTail is volatile because the ISR
// reads it for the full check.
static const uint8_t TACH_RING_SIZE = 16;
volatile uint32_t tachRingBuf[TACH_RING_SIZE];
volatile uint8_t  tachRingHead = 0;  // ISR write index (only ISR writes)
volatile uint8_t  tachRingTail = 0;  // Main-loop read index (only TACH_LOOP writes)
volatile bool     tachRingOverflow = false;  // ISR sets on drop; TACH_LOOP clears

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

// GPS nav-rate target: the rate GPS_RECONFIGURE() (and every wake/recovery
// path that calls it) re-asserts. Boot starts in status mode (5 Hz +
// NAV-SAT for the GPS status page); gpsEnterRaceMode() moves it to 25 Hz
// PVT-only when the page exits. Owned by gps_functions.ino.
uint8_t gpsNavRateTarget = GPS_NAV_RATE_STATUS_HZ;
bool gpsNavSatWanted = true;

// Why this boot happened — decoded from RESETREAS + GPIO LATCH first thing
// in setup(). Routes the GPS status page's exit (tach wake -> race mode)
// and the USB-wake charging shortcut once sleep is a full System OFF.
wake_cause::Cause bootWakeCause = wake_cause::Cause::kColdBoot;

// GPS status boot page hold/auto-close state (host-tested pure unit).
gps_status_page::State gpsStatusState;

// Per-satellite CNO snapshot for the GPS status page's signal bars.
// Written by onNAVSATReceived() (main-loop context via checkCallbacks()),
// read by displayPage_gps_status(). Selection/ordering rules live in the
// host-tested sat_bars unit.
uint8_t gpsSatCnos[sat_bars::kMaxSats];
uint8_t gpsSatCnoCount = 0;   // entries in gpsSatCnos
uint8_t gpsSatUsedCount = 0;  // satellites participating in the nav solution

// GPS PVT-arrival validation: tracks whether GPS is producing data after
// GPS_SETUP() / GPS_WAKE(). Both set gpsWakeTime and clear gpsWakeValidated;
// GPS_LOOP() sets gpsWakeValidated=true on first PVT arrival. If 5 seconds
// pass without PVT, GPS_LOOP() triggers baud recovery and reconfiguration —
// this is how a module that silently lost its config (V_BCKP drop, full
// power cycle) gets caught even when the begin() probe succeeded.
unsigned long gpsWakeTime = 0;
bool gpsWakeValidated = true;  // Armed (set false) by GPS_SETUP at boot

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
// Prevents race conditions between logging, replay, and BLE file transfers.
// The SD_ACCESS_* modes come from sd_functions.h (aliases of the host-tested
// sd_access_policy constants); transitions are made atomically by
// acquireSDAccess() / releaseSDAccess() in sd_functions.ino.
///////////////////////////////////////////
volatile int currentSDAccess = SD_ACCESS_NONE;

// Replay function prototypes (must be after SdFat include for File type)
bool buildReplayFileList();
bool readReplayLine(File& file, char* buffer, int bufferSize);
double haversineDistanceMiles(double lat1, double lng1, double lat2, double lng2);
void resetReplayState();
bool parseDovexHeader(const char* filename);

// SD state flags
bool sdSetupSuccess = false;
bool sdCardUnformatted = false;  // card answers but FAT won't mount (see SD_SETUP)
bool sdTrackSuccess = false;
bool sdDataLogInitComplete = false;
bool enableLogging = false;

// Boot format-confirm page (PAGE_SD_FORMAT): the hold-to-confirm state
// machine lives in the host-tested sd_format_page unit. displayLoop()
// only ever renders the confirm screen; the running/done screens are
// painted directly by sdPerformFormat() (which blocks the main loop).
// A failed attempt returns to the confirm page with sdFormatLastFailed
// set so the renderer can say so.
sd_format_page::State sdFormatState;
bool sdFormatLastFailed = false;

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
// GPS status boot page — every boot lands here after the splash. Outside
// the ENDURANCE_MODE-reshuffled 3-12 running block and not arrow-navigable.
const int PAGE_GPS_STATUS = 900;

// main menu (shown after boot)
const int PAGE_MAIN_MENU = -1;
const int PAGE_BLUETOOTH = -2;
const int PAGE_REPLAY_FILE_SELECT = -3;
const int PAGE_TRANSFER_MENU = -4;   // Bluetooth-vs-USB submenu
const int PAGE_USB_STORAGE = -5;     // USB mass-storage active screen
const int PAGE_PAIR_CAMERA = -6;     // Insta360 pairing / paired-status screen
const int PAGE_CAMERA_SERIAL_ENTRY = -7; // manual 6-char camera serial entry
const int PAGE_REPLAY_RESULTS = -8;
const int PAGE_REPLAY_EXIT = -9;
const int PAGE_CAMERA_TEST = -10;    // bench test menu (paired camera controls)

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
// Boot page when the SD card responds but has no mountable FAT volume
// (soldered-in module: factory-blank or corrupted). Unlike FAULT, its
// buttons stay live — driven by sdFormatPageLoop().
const int PAGE_SD_FORMAT = 106;

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
// Manual camera-serial entry state (PAGE_CAMERA_SERIAL_ENTRY): edited by the
// button handler in display_ui.ino, rendered by display_pages.ino. Cursor
// 0-5 = characters, 6 = OK, 7 = CANCEL. Reset when the page is entered.
char cameraSerialEntryBuf[7] = "AAAAAA";
int cameraSerialEntryCursor = 0;
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
// BOOT WAKE CAUSE
///////////////////////////////////////////

// Bit mask for an Arduino pin on the given GPIO port (0/1), or 0 if the
// pin lives on the other port. Uses the board variant's pin map so raw
// P-numbers never get hardcoded.
static uint32_t pinPortMask(uint32_t arduinoPin, int port) {
  const uint32_t p = g_ADigitalPinMap[arduinoPin];
  if ((int)(p >> 5) != port) return 0;
  return 1u << (p & 31);
}

// Per-port masks of the System OFF wake pins for the wake_cause decoder.
// The button pin literals mirror setupButtons() in display_ui.ino (kept in
// sync by hand) — buttons aren't assigned to the ButtonState structs until
// displaySetup(), which runs after the boot decode needs these.
static wake_cause::PinMasks shutdownWakePinMasks() {
  #ifndef WOKWI
  const uint32_t buttonPins[3] = {1, 2, 3};
  #else
  const uint32_t buttonPins[3] = {4, 5, 6};
  #endif
  wake_cause::PinMasks m = {};
  m.tach0 = pinPortMask(tachInputPin, 0);
  m.tach1 = pinPortMask(tachInputPin, 1);
  for (int i = 0; i < 3; i++) {
    m.buttons0 |= pinPortMask(buttonPins[i], 0);
    m.buttons1 |= pinPortMask(buttonPins[i], 1);
  }
  return m;
}

// Read (then clear) the sticky boot registers and decode why we booted.
// Must run before anything else touches them; RESETREAS is cumulative and
// LATCH survives System OFF, so stale bits would corrupt the next decode.
// The SoftDevice is never enabled this early (BLE init is lazy), so raw
// register access is safe.
static void captureBootWakeCause() {
  wake_cause::Regs regs;
  regs.resetreas = NRF_POWER->RESETREAS;
  regs.latch0 = NRF_P0->LATCH;
  regs.latch1 = NRF_P1->LATCH;
  NRF_POWER->RESETREAS = 0xFFFFFFFF;  // write-1-to-clear
  NRF_P0->LATCH = 0xFFFFFFFF;
  NRF_P1->LATCH = 0xFFFFFFFF;
  bootWakeCause = wake_cause::decode(regs, shutdownWakePinMasks());
}

///////////////////////////////////////////
// SETUP
///////////////////////////////////////////

void setup() {
  captureBootWakeCause();

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

  // Register the USB mass-storage callbacks (no drive presented until the
  // user enters USB transfer mode). Needs a working SD card for block I/O.
  if (sdSetupSuccess) {
    USB_MSC_SETUP();
  }

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
    if (getSetting("device_name", buf, sizeof(buf))) {
      strncpy(settingDeviceName, buf, sizeof(settingDeviceName) - 1);
      settingDeviceName[sizeof(settingDeviceName) - 1] = '\0';
    }
    crossingThresholdMeters = settingLapDetectionDistance;
    debug(F("Settings loaded: lap_dist="));
    debug(settingLapDetectionDistance);
    debug(F(" wp_dist="));
    debug(settingWaypointDetectionDistance);
    debug(F(" wp_speed="));
    debug(settingWaypointSpeed);
    debug(F(" driver="));
    debug(settingDriverName);
    debug(F(" device="));
    debugln(settingDeviceName);
  }

  // Camera auto-record: load the persisted Insta360 serial + init the FSM
  CAMERA_SETUP();

  if (!sdSetupSuccess && sdCardUnformatted) {
    // Card answers but no FAT volume mounts: soldered-in module out of the
    // factory, or a corrupted filesystem. The card can't be pulled to fix
    // it on a PC, so offer the on-device format (hold Select to confirm).
    // Deliberately outranks the USB-wake charging branch — a device with an
    // unusable card should say so; the page's idle timeout still lands in
    // enterShutdown(), whose VBUS handling enters the charging loop anyway.
    sd_format_page::begin(sdFormatState, millis());
    switchToDisplayPage(PAGE_SD_FORMAT);
  } else if (!sdSetupSuccess) {
    strncpy(internalNotification, "SD Init failed!\n\nlogging not possible!", sizeof(internalNotification) - 1);
    internalNotification[sizeof(internalNotification) - 1] = '\0';
    switchToDisplayPage(PAGE_INTERNAL_FAULT);
  } else if (bootWakeCause == wake_cause::Cause::kUsbWake) {
    // Plugged in while off: VBUS woke the chip so software can hold the
    // fast-charge pin. Skip the GPS status page and drop straight into
    // the charging loop; a button press there resumes to the main menu
    // (in which case setup() continues below), unplugging powers back off.
    enterShutdown();
  } else {
    // A missing TRACKS folder is auto-created by buildTrackList(); a
    // false here means even that failed — Lap Anything will handle it.
    if (!sdTrackSuccess) {
      debugln(F("No usable TRACKS folder — Lap Anything will activate"));
    }
    // Every boot lands on the GPS status page (MyChron-style): hold until
    // a stable lock (or a button press), then continue to the menu — or
    // straight into race mode when the tach woke us / the engine runs.
    gps_status_page::begin(gpsStatusState, millis());
    switchToDisplayPage(PAGE_GPS_STATUS);
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

  // Throttle the scan to 1 Hz. Each haversineDistanceMiles() is several
  // software-emulated double libm calls (the M4F FPU is single-precision
  // only); at the 200-entry manifest ceiling a full scan costs multiple
  // milliseconds. gpsData.fix stays true BETWEEN PVT updates, so without
  // this gate the scan ran every ~250 Hz loop iteration — collapsing the
  // loop rate — for an answer that changes at driving pace.
  static unsigned long lastManifestScan = 0;
  if (millis() - lastManifestScan < 1000) return;
  lastManifestScan = millis();

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
  // Deliberately NO camera notification here: checkAutoIdle() ends the
  // log session on speed alone (engine ignored), but the camera must
  // keep recording through a stationary grid idle — its own
  // stationary-AND-engine-off rule decides the recording stop. The
  // camera is stopped explicitly where the user means "I'm done":
  // the manual stop confirm (display_ui.ino) and shutdown entry
  // (CAMERA_SLEEP() in enterShutdown()).

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

  // Yield to an active camera recording: while the camera is recording, IT owns
  // the end (30 s engine-off -> cameraConsumeAutoStop() above), so the
  // speed-based idle must not cut the log out from under it during a stationary
  // but engine-running stint (grid/paddock). No camera, or not recording, keeps
  // the original speed-only behavior below.
  if (cameraActivelyRecording()) return;

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
 * @brief Drive the GPS status boot page: bounded GPS re-detect, then step
 *        the hold/auto-close state machine (host-tested gps_status_page
 *        unit) and act on its exit verdict. Runs between readButtons()
 *        and displayLoop() so it consumes this frame's presses — the page
 *        has an explicit no-op branch in displayLoop()'s button handling.
 */
void gpsStatusPageLoop() {
  if (currentPage != PAGE_GPS_STATUS) return;

  // GPS missing at boot? Re-probe a few times while the page is up.
  GPS_STATUS_RETRY_LOOP();

  gps_status_page::Inputs in;
  in.fix = gpsData.fix;
  in.timeValid = gpsData.timeValid;
  in.buttonPressed = btn1->pressed || btn2->pressed || btn3->pressed;
  in.tachWakeBoot = (bootWakeCause == wake_cause::Cause::kTachWake);
  in.engineRunning = tachLastReported > 500;  // autoRaceModeCheck threshold
  in.nowMs = millis();

  const gps_status_page::Exit verdict = gps_status_page::step(gpsStatusState, in);
  if (verdict != gps_status_page::Exit::kStay) {
    // The press that skipped this page must not also drive the
    // destination page's button handling in displayLoop() this frame.
    resetButtons();
  }

  switch (verdict) {
    case gps_status_page::Exit::kToMenu:
      gpsEnterRaceMode();  // 25 Hz PVT-only is the steady state off this page
      switchToDisplayPage(PAGE_MAIN_MENU);
      break;
    case gps_status_page::Exit::kToRace:
      // Engine is (or was, at wake) running — straight into race mode with
      // logging, mirroring the old RPM-wake path.
      gpsEnterRaceMode();
      debugln(F("GPS status page -> race mode"));
      raceActive = true;
      enableLogging = true;
      raceSessionStartedAt = millis();
      createLapAnythingCourseManager();
      switchToDisplayPage(TACHOMETER);
      break;
    case gps_status_page::Exit::kToShutdown:
      // Nothing locked, engine silent for the idle timeout — a spurious
      // wake or a shelf queen. Power back down.
      enterShutdown();
      break;
    case gps_status_page::Exit::kStay:
      break;
  }
}

/**
 * @brief Drive the SD format-confirm boot page: step the hold-to-confirm
 *        state machine (host-tested sd_format_page unit) and act on its
 *        exit verdict. Runs between readButtons() and displayLoop(), like
 *        gpsStatusPageLoop() — the page has an explicit no-op branch in
 *        displayLoop()'s button handling.
 */
void sdFormatPageLoop() {
  if (currentPage != PAGE_SD_FORMAT) return;

  sd_format_page::Inputs in;
  in.selectHeld = isButtonHeld(2, 0);  // live level, updated by updateButtonHoldState()
  // A held side button disarms the confirm — the user is going for the
  // global Select+side reboot combo (5 s), which must outrank a 3 s erase.
  in.otherButtonHeld = isButtonHeld(1, 0) || isButtonHeld(3, 0);
  in.otherButtonPressed = btn1->pressed || btn3->pressed;
  in.engineRunning = tachLastReported > 500;  // autoRaceModeCheck threshold
  in.nowMs = millis();

  const sd_format_page::Exit verdict = sd_format_page::step(sdFormatState, in);
  if (verdict != sd_format_page::Exit::kStay) {
    resetButtons();
  }

  switch (verdict) {
    case sd_format_page::Exit::kFormat:
      // Blocking. Reboots the device on success; returns to this page's
      // confirm screen on failure (fresh full hold required to retry).
      sdPerformFormat();
      break;
    case sd_format_page::Exit::kToShutdown:
      // Unformatted card and nobody home — don't drain the pack.
      enterShutdown();
      break;
    case sd_format_page::Exit::kStay:
      break;
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
      settingDeviceName,
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
// SHUTDOWN (System OFF)
//
// "Sleep" is a full power-down: tear everything down, arm GPIO SENSE on
// the tach + buttons as wake sources, and enter nRF52 System OFF (~µA).
// Wake is a chip reset — setup() runs fresh and captureBootWakeCause()
// reads why (tach pulse -> the GPS status page exits into race mode).
//
// The ONE soft exception is charging: while VBUS is present the device
// must stay alive because the HICHG fast-charge pin is software-held, so
// enterShutdown() parks in a live charging loop instead and only drops
// to System OFF when the cable is pulled. VBUS itself is a System OFF
// wake source (always armed on nRF52840), so plugging in a dark device
// boots it straight back into that loop.
///////////////////////////////////////////

bool isUsbConnected() {
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

// Raw multi-sampled poll until every button is released (or timeout).
// A held button at System OFF entry = SENSE already satisfied = instant
// wake-reset, so the entry combo must be released before we commit.
static void waitAllButtonsReleased(unsigned long timeoutMs) {
  unsigned long start = millis();
  while (anyButtonPressed() && millis() - start < timeoutMs) {
    wdtPet();
    delay(10);
  }
}

// CPU idle for the charging loop. sd_app_evt_wait() is an SVC into the
// SoftDevice and hard-faults when it isn't enabled — BLE is lazily
// initialized, so ask the SoftDevice itself (the old sleep loop called
// it unconditionally, a latent fault).
static void shutdownIdleWait() {
  uint8_t sdEnabled = 0;
  (void)sd_softdevice_is_enabled(&sdEnabled);
  if (sdEnabled) {
    sd_app_evt_wait();
  } else {
    __WFE();
  }
}

// Configure wake sources and enter System OFF. Does not return: wake is
// a full reset (RESETREAS.OFF + the pin's LATCH bit record the cause),
// and the WDT stops with every other clock — wdtSetup() re-arms on the
// fresh boot (nRF52840 PS: System OFF halts all clocks/peripherals).
static void shutdownSystemOff() {
  #ifdef WOKWI
  // Simulator has no System OFF emulation worth trusting — plain reset.
  NVIC_SystemReset();
  #else
  waitAllButtonsReleased(3000);
  delay(50);  // contact settle
  wdtPet();

  // Wake sources: SENSE-LOW with pull-up on the tach (idle-high, pulse =
  // falling) and all three buttons (active-low). Pull + SENSE config is
  // retained in System OFF. P-numbers via the board variant's pin map.
  nrf_gpio_cfg_sense_input(g_ADigitalPinMap[tachInputPin],
                           NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  nrf_gpio_cfg_sense_input(g_ADigitalPinMap[btn1->pin],
                           NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  nrf_gpio_cfg_sense_input(g_ADigitalPinMap[btn2->pin],
                           NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  nrf_gpio_cfg_sense_input(g_ADigitalPinMap[btn3->pin],
                           NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
  // VBUS wake needs no configuration on the nRF52840 — always armed.

  // A set LATCH bit is a pending DETECT = instant re-wake; clear last,
  // right before entering OFF. (Boot already cleared RESETREAS, but the
  // SoftDevice path below clears again to be unambiguous.)
  NRF_P0->LATCH = 0xFFFFFFFF;
  NRF_P1->LATCH = 0xFFFFFFFF;

  // Cortex-M4F: a pending FPU exception can inhibit low-power entry —
  // clear lazily-stacked FP state before OFF (standard Nordic guidance).
  __set_FPSCR(__get_FPSCR() & ~0x9F);
  NVIC_ClearPendingIRQ(FPU_IRQn);

  // NRF_POWER is a restricted peripheral while the SoftDevice is enabled
  // (BLE is lazy — it may or may not be up). GPREGRET is deliberately
  // untouched: register 0 belongs to the OTA/bootloader handoff.
  uint8_t sdEnabled = 0;
  (void)sd_softdevice_is_enabled(&sdEnabled);
  if (sdEnabled) {
    sd_power_reset_reason_clr(0xFFFFFFFF);
    (void)sd_power_system_off();
  } else {
    NRF_POWER->RESETREAS = 0xFFFFFFFF;
    NRF_POWER->SYSTEMOFF = 1;
  }
  // Only reachable in emulated System OFF (debugger attached).
  while (true) { __WFE(); }
  #endif
}

// The charging loop — runs after full teardown whenever VBUS is present
// at shutdown (or woke us). Shows the charging screen for 10 s, then
// display off; ANY button press is a full wake back to the main menu
// (the USB-on-menu trigger waits USB_MENU_CHARGE_IDLE_MS before pulling
// the device back here, so wake doesn't bounce). Returns true to resume
// to the menu, false when the cable was pulled (caller enters System OFF).
static bool runChargingShutdownLoop() {
  debugln(F("Charging loop (VBUS present)"));
  DISPLAY_WAKE();
  waitAllButtonsReleased(2000);  // shutdown entry combo may still be held
  unsigned long shownAt = millis();
  if (shownAt == 0) shownAt = 1;  // 0 is the display-off sentinel

  while (true) {
    wdtPet();

    if (!isUsbConnected()) {
      if (shownAt != 0) DISPLAY_SLEEP();
      return false;  // unplugged — fall through to System OFF
    }

    if (anyButtonPressed()) {
      waitAllButtonsReleased(2000);
      return true;  // full wake to the main menu
    }

    if (shownAt != 0 && millis() - shownAt >= CHARGE_DISPLAY_TIMEOUT_MS) {
      DISPLAY_SLEEP();
      shownAt = 0;
    }
    if (shownAt != 0 &&
        millis() - displayLastUpdate > (1000 / displayUpdateRateHz)) {
      displayLastUpdate = millis();
      displayPage_sleep_charging();
    }

    shutdownIdleWait();
  }
}

// Bring the subsystems back after the charging loop — the chip never
// powered down, so this is the minimal mirror of the cold-boot bring-up.
// BLE and the camera stay down (both come up lazily on demand).
static void softResumeFromCharging() {
  if (accelAvailable) {
    digitalWrite(PIN_LSM6DS3TR_C_POWER, LOW);
    delay(50);
    if (accelIMU.begin() != 0) {
      debugln(F("IMU failed to reinitialize after charging"));
      accelAvailable = false;
    }
  }

  // The menu's steady state is race config. Set the targets directly so
  // GPS_WAKE()'s GPS_RECONFIGURE() applies them (a shutdown from the GPS
  // status page would otherwise resume at 5 Hz with NAV-SAT on).
  gpsNavRateTarget = GPS_NAV_RATE_HZ;
  gpsNavSatWanted = false;
  gpsSatCnoCount = 0;
  gpsSatUsedCount = 0;
  GPS_WAKE();

  DISPLAY_WAKE();
  menuIdleTimerRunning = false;
  if (!sdSetupSuccess && sdCardUnformatted) {
    // The format page's idle timeout landed us in the charging loop; the
    // card still has no FAT, so resume back to the format offer — the main
    // menu is useless (and misleading) when nothing can mount.
    sd_format_page::begin(sdFormatState, millis());
    switchToDisplayPage(PAGE_SD_FORMAT);
    return;
  }
  switchToDisplayPage(PAGE_MAIN_MENU);
}

// Full shutdown: tear down every subsystem, then System OFF — or the
// charging loop when VBUS is present. May return (charging resume);
// callers in loop() must `return` right after so the frame restarts.
void enterShutdown() {
  debugln(F("Shutdown"));

  // End race session if active (safety net — may write the DOVEX header)
  if (raceActive) endRaceSession();
  wdtPet();

  // Camera power-off streams a synchronous 3 s ce82 hold — the longest
  // single teardown step, bracketed by pets against the ~4 s WDT.
  CAMERA_SLEEP();
  wdtPet();

  // Stop BLE if active (advertising/connection teardown)
  if (bleActive) BLE_STOP();

  // Display off (I2C command, ~10 µA panel sleep)
  DISPLAY_SLEEP();

  // GPS to software backup mode (µA; config retained while powered) and
  // TIMER3 serial-drain stopped
  GPS_SLEEP();

  // IMU rail off (HIGH = power disabled)
  if (accelAvailable) {
    pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
    digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  }
  wdtPet();

  // Charging exception: never System OFF while VBUS is present — the
  // HICHG charge-rate pin needs software alive.
  if (isUsbConnected()) {
    if (runChargingShutdownLoop()) {
      softResumeFromCharging();
      return;
    }
    // Cable pulled during the charging loop — power down for real.
  }

  shutdownSystemOff();  // does not return
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

  // When USB mass-storage is active the host PC owns the SD card over MSC.
  // Park the loop exactly like the BLE branch so the firmware never drives
  // the FAT concurrently with the host — GPS/tach/lap/SD processing is all
  // skipped. (The SD-access policy would deny those acquires anyway, but
  // parking here is the real guarantee rather than a side effect.)
  if (usbMscActive) {
    // Cable pulled without using the on-device Exit (the natural way to
    // "finish") — tear down so the SD lock, the fast SPI clock, and the
    // media-ready state don't leak into a later driving session and
    // silently refuse to log. USB_MSC_DISABLE() reboots and does not return.
    if (!isUsbConnected()) {
      USB_MSC_DISABLE();
    }

    // Minimal button check for the on-device Exit (Select).
    readButtons();
    if (btn2->pressed) {
      USB_MSC_DISABLE();  // does not return (NVIC_SystemReset)
    }
    resetButtons();

    // Refresh the status page at the normal rate.
    if (millis() - displayLastUpdate > (1000 / displayUpdateRateHz)) {
      displayLastUpdate = millis();
      displayPage_usb_storage();
    }

    return;  // host owns the card — skip GPS/tach/lap/SD entirely
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
  CAMERA_LOOP();  // step the Insta360 auto-record FSM (GPS/tach fresh above)

  // Camera auto-stopped recording (30 s engine-off): end + save the race
  // session and return to the menu — the camera stays connected in WATCHING,
  // ready to re-record if the engine restarts. endRaceSession() is idempotent
  // and does not switch pages itself, so we do (mirrors checkAutoIdle). A
  // manual logging-stop does not set this — that path already ended the log.
  if (raceActive && cameraConsumeAutoStop()) {
    endRaceSession();
    switchToDisplayPage(PAGE_MAIN_MENU);
  }

  // Button hold detection for shutdown/reboot combos
  updateButtonHoldState();

  // Long-press left+right (5s) on main menu -> shutdown
  if (currentPage == PAGE_MAIN_MENU &&
      isButtonHeld(1, SLEEP_LONG_PRESS_MS) &&
      isButtonHeld(3, SLEEP_LONG_PRESS_MS)) {
    enterShutdown();
    return;
  }

  // Reboot combo: select + either side button held 5s (any page)
  if (isButtonHeld(2, SLEEP_LONG_PRESS_MS) &&
      (isButtonHeld(1, SLEEP_LONG_PRESS_MS) || isButtonHeld(3, SLEEP_LONG_PRESS_MS))) {
    NVIC_SystemReset();
  }

  // Main-menu idle tracking. Two consumers:
  //  - USB present: enter the charging loop after USB_MENU_CHARGE_IDLE_MS
  //    of no button activity. Not immediate — a charging-loop button wake
  //    lands here, and an instant re-entry would bounce it right back.
  //    The window is what lets replay/transfer be used while plugged in.
  //  - No USB: full shutdown after SLEEP_IDLE_TIMEOUT_MS.
  if (currentPage == PAGE_MAIN_MENU) {
    if (!menuIdleTimerRunning) {
      menuIdleTimerRunning = true;
      menuIdleStartTime = millis();
    }
    if (btn1->pressed || btn2->pressed || btn3->pressed) {
      menuIdleStartTime = millis();  // Reset on any button
    }
    unsigned long idleFor = millis() - menuIdleStartTime;
    if ((isUsbConnected() && idleFor >= USB_MENU_CHARGE_IDLE_MS) ||
        idleFor >= SLEEP_IDLE_TIMEOUT_MS) {
      enterShutdown();
      return;
    }
  } else {
    menuIdleTimerRunning = false;
  }

  calculateGPSFrameRate();

  readButtons();
  gpsStatusPageLoop();  // boot status page: consume presses, hold/auto-close
  sdFormatPageLoop();   // boot format-confirm page: hold Select 3s to format
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
