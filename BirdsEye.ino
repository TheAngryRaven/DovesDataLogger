#include <avr/dtostrf.h>
#include <SPI.h>

// #define WOKWI
// #define HAS_DEBUG

// Hides a couple pages and changes some behavior
// todo: make dynamic in next UI version
// #define ENDURANCE_MODE

// designed for seeed NRF52840 which comes with a charge circut
#define VREF 3.6
#define ADC_MAX 4096

unsigned long lastBatteryCheck;
int batteryUpdateInterval = 5000;
float lastBatteryVoltage;

#include <DovesLapTimer.h>
double crossingThresholdMeters = 7.0;
DovesLapTimer lapTimer(crossingThresholdMeters);
unsigned long gpsFrameStartTime;
unsigned long gpsFrameEndTime;
unsigned long gpsFrameCounter;
float gpsFrameRate = 0.0;
bool trackSelected = false;

// TODO: setup header for project
#define SD_CARD_LOGGING_ENABLED
#define MAX_LOCATIONS 100
#define MAX_LOCATION_LENGTH 13 // 13 is old dos format for fat16
#define MAX_LAYOUTS 10
#define MAX_LAYOUT_LENGTH 15
#define FILEPATH_MAX 50        // "/TRACKS/" (8) + name (13) + ".json" (5) + null = 27, using 50 for safety
#include <string.h>
struct ButtonState {
  int pin = 0;
  bool pressed = false;
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

const int RACE_DIRECTION_FORWARD  = 0;
const int RACE_DIRECTION_REVERSE  = 1;

#ifdef HAS_DEBUG
#define debugln Serial.println
#define debug Serial.print
#else
void dummy_debug(...) {
}
#define debug dummy_debug
#define debugln dummy_debug
#endif

///////////////////////////////////////////
// BLUETOOTH (BLE) File Transfer
///////////////////////////////////////////
#include <bluefruit.h>

// BLE Service & Characteristics
BLEService fileService = BLEService(0x1820);
BLECharacteristic fileListChar = BLECharacteristic(0x2A3D);
BLECharacteristic fileRequestChar = BLECharacteristic(0x2A3E);
BLECharacteristic fileDataChar = BLECharacteristic(0x2A3F);
BLECharacteristic fileStatusChar = BLECharacteristic(0x2A40);
BLECharacteristic fileUploadChar = BLECharacteristic(0x2A41);

// BLE state variables
bool bleInitialized = false;
bool bleActive = false;
bool bleConnected = false;
bool bleTransferInProgress = false;
uint32_t bleFileSize = 0;
uint32_t bleBytesTransferred = 0;
uint16_t bleNegotiatedMtu = 23;
// Upload state variables
bool bleUploadInProgress = false;
uint32_t bleUploadExpectedSize = 0;
uint32_t bleUploadBytesReceived = 0;
File32 bleUploadFile;
// Note: bleCurrentFile is declared after SdFat include

///////////////////////////////////////////
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

/**
 * Tachometer ISR - called on rising edge of tach signal
 *
 * CRITICAL: This ISR must be fast and must NOT call noInterrupts().
 * Calling noInterrupts() here would disable ALL system interrupts,
 * and if the main loop is delayed (e.g., SD write), interrupts would
 * stay disabled forever, freezing button input and causing system lockup.
 *
 * Instead, we use a volatile flag gate (tachInterruptShouldProcess) that
 * the main loop re-enables after the debounce period.
 */
void TACH_COUNT_PULSE() {
  // Exit immediately if we're in the debounce window
  if (!tachInterruptShouldProcess) return;

  uint32_t now = micros();
  uint32_t dt = now - tachLastPulseUs;

  // Secondary debounce check (belt and suspenders with the flag)
  if (dt < tachMinPulseGapUs) return;

  // Record this pulse
  tachLastPulseUs = now;
  tachLastPeriodUs = dt;
  tachHavePeriod = true;

  // Gate off further interrupts until main loop re-enables
  // DO NOT call noInterrupts() here - that causes system-wide deadlock!
  tachInterruptShouldProcess = false;
}

/**
 * Tachometer main loop processing
 *
 * Re-enables the ISR gate after debounce period, reads pulse data,
 * applies exponential moving average filter, and handles timeout.
 */
void TACH_LOOP() {
  // Re-enable interrupt processing after debounce window expires
  if (!tachInterruptShouldProcess) {
    uint32_t elapsed = micros() - tachLastPulseUs;
    if (elapsed >= tachMinPulseGapUs) {
      tachInterruptShouldProcess = true;
      // Note: We don't call interrupts() here anymore - they were never disabled!
    }
  }

  // Atomically read the pulse period captured by ISR
  // On ARM Cortex-M4 (NRF52840), 32-bit aligned reads are atomic,
  // but we use noInterrupts() briefly for the read-modify-clear sequence
  uint32_t periodUs = 0;
  bool havePeriod = false;

  noInterrupts();
  havePeriod = tachHavePeriod;
  if (havePeriod) {
    periodUs = tachLastPeriodUs;
    tachHavePeriod = false;  // Clear flag so we don't re-process
  }
  interrupts();

  // Apply exponential moving average filter to smooth RPM
  if (havePeriod && periodUs > 0) {
    float rpmInst = (60.0e6f * tachRevsPerPulse) / (float)periodUs;
    tachRpmFiltered += tachFilterAlpha * (rpmInst - tachRpmFiltered);
  }

  // Timeout: if no pulses for tachStopTimeoutUs, engine is stopped
  uint32_t lastPulseUs;
  noInterrupts();
  lastPulseUs = tachLastPulseUs;
  interrupts();

  if ((uint32_t)(micros() - lastPulseUs) > tachStopTimeoutUs) {
    tachRpmFiltered = 0.0f;
  }

  // Update display/log value at configured rate
  if (millis() - tachLastUpdate > (1000 / tachUpdateRateHz)) {
    tachLastUpdate = millis();
    tachLastReported = (int)(tachRpmFiltered + 0.5f);
  }
}

///////////////////////////////////////////
#include <Adafruit_GPS.h>
#include "gps_config.h"
Adafruit_GPS* gps = NULL;
bool gpsInitialized = false;  // Safety flag - true only after successful GPS init

#define GPS_SERIAL Serial1
#ifndef GPS_CONFIGURATION
  /**
   * @brief Returns the GPS time since midnight in milliseconds
   *
   * @return unsigned long The time since midnight in milliseconds, or 0 if GPS unavailable
   */
  unsigned long getGpsTimeInMilliseconds() {
    if (!gpsInitialized || gps == NULL) return 0;

    unsigned long timeInMillis = 0;
    timeInMillis += gps->hour * 3600000ULL;   // Convert hours to milliseconds
    timeInMillis += gps->minute * 60000ULL;   // Convert minutes to milliseconds
    timeInMillis += gps->seconds * 1000ULL;   // Convert seconds to milliseconds
    timeInMillis += gps->milliseconds;        // Add the milliseconds part

    return timeInMillis;
  }

  /**
   * @brief Converts GPS date/time to Unix timestamp (seconds since Jan 1, 1970)
   *
   * @return unsigned long Unix timestamp in seconds, or 0 if GPS unavailable
   */
  unsigned long getGpsUnixTimestamp() {
    if (!gpsInitialized || gps == NULL) return 0;

    // Days in each month (non-leap year)
    const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    uint16_t year = 2000 + gps->year;
    uint8_t month = gps->month;
    uint8_t day = gps->day;

    // Calculate days since Unix epoch (Jan 1, 1970)
    unsigned long days = 0;

    // Add days for complete years since 1970
    for (uint16_t y = 1970; y < year; y++) {
      if ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0)) {
        days += 366; // Leap year
      } else {
        days += 365;
      }
    }

    // Add days for complete months this year
    for (uint8_t m = 1; m < month; m++) {
      days += daysInMonth[m - 1];
      // Add leap day if February and leap year
      if (m == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
        days++;
      }
    }

    // Add remaining days
    days += day - 1;

    // Convert to seconds and add time of day
    unsigned long timestamp = days * 86400UL;
    timestamp += gps->hour * 3600UL;
    timestamp += gps->minute * 60UL;
    timestamp += gps->seconds;

    return timestamp;
  }

  /**
   * @brief Converts GPS date/time to Unix timestamp with millisecond precision
   *
   * @return unsigned long long Unix timestamp in milliseconds since Jan 1, 1970
   */
  unsigned long long getGpsUnixTimestampMillis() {
    if (!gpsInitialized || gps == NULL) return 0;

    // Get the Unix timestamp in seconds
    unsigned long long timestampMillis = (unsigned long long)getGpsUnixTimestamp() * 1000ULL;

    // Add the milliseconds from GPS
    timestampMillis += gps->milliseconds;

    return timestampMillis;
  }

  /**
   * @brief Sends a GPS configuration command stored in program memory to the GPS module via [GPS_SERIAL].
   *
   * This function reads a configuration command from PROGMEM (program memory) and sends it byte by byte to the GPS module using the [GPS_SERIAL] interface.
   * The function also prints the configuration command in hexadecimal format for debugging purposes.
   *
   * @note This function contains blocking code and should be used during setup only.
   *
   * @param Progmem_ptr Pointer to the PROGMEM (program memory) containing the GPS configuration command.
   * @param arraysize Size of the configuration command stored in PROGMEM.
   */
  void GPS_SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize) {
    uint8_t byteread, index;

    debug(F("GPSSend  "));

    for (index = 0; index < arraysize; index++)
    {
      byteread = pgm_read_byte_near(Progmem_ptr++);
      if (byteread < 0x10)
      {
        debug(F("0"));
      }
      debug(byteread, HEX);
      debug(F(" "));
    }

    debugln();
    //set Progmem_ptr back to start
    Progmem_ptr = Progmem_ptr - arraysize;

    for (index = 0; index < arraysize; index++)
    {
      byteread = pgm_read_byte_near(Progmem_ptr++);
      GPS_SERIAL.write(byteread);
    }
    delay(200);
  }

  void GPS_SETUP() {
    gpsInitialized = false;  // Reset flag at start of setup

    #ifndef WOKWI
      debugln(F("ACTUAL GPS SETUP"));
      // first try serial at 9600 baud
      GPS_SERIAL.begin(9600);
      // wait for the GPS to boot
      delay(2250);
      if (GPS_SERIAL) {
        GPS_SendConfig(uart115200NmeaOnly, 28);
        GPS_SERIAL.end();
      }

      // reconnect at proper baud
      gps = new Adafruit_GPS(&GPS_SERIAL);
      if (gps == NULL) {
        debugln(F("ERROR: GPS allocation failed!"));
        return;  // Leave gpsInitialized = false
      }

      GPS_SERIAL.begin(115200);
      // wait for the GPS to boot
      delay(2250);
      // Send GPS Configurations
      if (GPS_SERIAL) {
        GPS_SendConfig(NMEAVersion23, 28);
        GPS_SendConfig(FullPower, 16);

        GPS_SendConfig(GPGLLOff, 16);
        GPS_SendConfig(GPVTGOff, 16);
        GPS_SendConfig(GPGSVOff, 16);
        GPS_SendConfig(GPGSAOff, 16);
        // GPS_SendConfig(GPGGAOn5, 16); // for 10hz
        GPS_SendConfig(GPGGAOn10, 16); // for 18hz
        GPS_SendConfig(NavTypeAutomobile, 44);
        // GPS_SendConfig(ENABLE_GPS_ONLY, 68);
        GPS_SendConfig(ENABLE_GPS_ONLY_M10, 60);
        // GPS_SendConfig(Navrate10hz, 14);
        // GPS_SendConfig(Navrate18hz, 14);
        // GPS_SendConfig(Navrate20hz, 14);
        GPS_SendConfig(Navrate25hz, 14);

        gpsInitialized = true;  // Success!
        debugln(F("GPS initialized successfully"));
      } else {
        debugln(F("ERROR: GPS Serial not available!"));
      }
    #else
      debugln(F("WOKWI GPS SETUP"));
      // reconnect at proper baud
      gps = new Adafruit_GPS(&GPS_SERIAL);
      if (gps == NULL) {
        debugln(F("ERROR: GPS allocation failed!"));
        return;
      }
      GPS_SERIAL.begin(19200);
      gpsInitialized = true;
    #endif
  }
#endif

double crossingPointALat = 0.00;
double crossingPointALng = 0.00;
double crossingPointBLat = 0.00;
double crossingPointBLng = 0.00;
float gps_speed_mph = 0.0;

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
    // lapHistoryCount = (lapHistoryCount + 1) % lapHistoryMaxLaps;
    lapHistoryCount++;
    debugln(F("New lap added to history..."));
  }
}

///////////////////////////////////////////
// REPLAY SYSTEM
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

// Note: File-dependent replay variables and function prototypes
// are declared after SdFat.h include below

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

void makeFullTrackPath(const char* trackName, char* filepath) {
  // Use snprintf for bounds safety - prevents buffer overflow
  // Caller MUST provide buffer of at least FILEPATH_MAX bytes
  snprintf(filepath, FILEPATH_MAX, "/TRACKS/%s.json", trackName);
}

bool SD_SETUP() {
  // TODO: FAT32 CHECK
  // Try multiple times - EMI from ignition can cause init failures
  for (int attempt = 0; attempt < 3; attempt++) {
    if (SD.begin(PIN_SPI_CS, SPI_SPEED)) {
      debugln(F("SD Card initialized successfully"));
      return true;
    }
    debugln(F("SD init attempt failed, retrying..."));
    delay(100);  // Brief delay between attempts
  }
  debugln(F("Card initialization failed after 3 attempts."));
  return false;
}

bool buildTrackList() {
  if (!SD.exists(trackFolder)) {
    debugln(F("TRACKS folder does not exist."));
    return false;
  }

  // reset list
  numOfLocations = 0;

  // If the TRACKS directory exists, open it
  trackDir.open(trackFolder);
  
  // Reset the file to the first position in the directory
  trackDir.rewind();
  
  // Loop through each file in the directory
  while (file.openNext(&trackDir, O_READ)) {
    // Create a buffer to store the filename
    char filename[25];
    
    // Get the filename
    file.getName(filename, sizeof(filename));
    
    // Find the last dot in the filename
    char *dot = strrchr(filename, '.');
    
    // If a dot was found, replace it with a null character to end the string there
    if(dot) *dot = '\0';
    
    // Print the filename without the extension
    // debugln(filename);

    // Add the file to the array
    strncpy(locations[numOfLocations], filename, sizeof(filename));

    // Increment the numOfLocations
    numOfLocations++;
    
    // Close the file to free up any memory it's using
    file.close();
  }

  // Close the directory to free up any memory it's using
  trackDir.close();

  return true;
}

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

int parseTrackFile(char* filepath) {
  debug(F("ParseTrackFile:"));
  debugln(filepath);

  // double check the SD is active
  if (!sdSetupSuccess) {
    debugln(F("ParseTrackFile: failed to initialize SD card"));
    return PARSE_STATUS_LOAD_FAILED;
  }

  // load file
  trackFile.open(filepath, O_READ);
  if (!trackFile) {
    debugln(F("ParseTrackFile: failed to LOAD file"));
    return PARSE_STATUS_LOAD_FAILED;
  }

  // Create a buffer to store file content
  char buffer[JSON_BUFFER_SIZE];
  int bytesRead = trackFile.read(buffer, sizeof(buffer));
  
  // Check if read was successful
  if (bytesRead == -1) {
    debugln(F("ParseTrackFile: failed to READ file"));
    return PARSE_STATUS_LOAD_FAILED;
  }

  // Null-terminate the buffer
  if (bytesRead < sizeof(buffer)) {
    buffer[bytesRead] = '\0';
  } else {
    buffer[sizeof(buffer) - 1] = '\0';
  }

  // Parse JSON
  StaticJsonDocument<JSON_BUFFER_SIZE> trackJson;
  DeserializationError error = deserializeJson(trackJson, buffer);
  if (error != DeserializationError::Ok) {
    // todo: add better parsing error handing
    if (error == DeserializationError::EmptyInput) {
      debugln(F("DeserializationError::EmptyInput"));
    } else if (error == DeserializationError::IncompleteInput) {
      debugln(F("DeserializationError::IncompleteInput"));
    } else if (error == DeserializationError::InvalidInput) {
      debugln(F("DeserializationError::InvalidInput"));
    } else if (error == DeserializationError::NoMemory) {
      debugln(F("DeserializationError::NoMemory"));
    } else if (error == DeserializationError::TooDeep) {
      debugln(F("DeserializationError::TooDeep"));
    } else {
      debugln(F("DeserializationError::UNKNOWN"));
    }

    trackFile.close();
    return PARSE_STATUS_PARSE_FAILED;
  }

  // handle parsed data
  JsonArray array = trackJson.as<JsonArray>();
  debugln(F("Generating Layout List..."));
  for(JsonVariant layout : array) {
    // debug is bugged lol
    #ifdef HAS_DEBUG
    const char* layoutName = layout["name"];
    debug(F("Layout Name: "));
    debugln(layoutName);
    #endif

    if(numOfTracks < MAX_LAYOUTS) {
      // add name to array of strings to display to the user
      strncpy(tracks[numOfTracks], layout["name"], sizeof(tracks[numOfTracks]) - 1);
      tracks[numOfTracks][sizeof(tracks[numOfTracks]) - 1] = '\0'; // Ensure null-termination

      // add data to array of layouts for later use
      trackLayouts[numOfTracks].start_a_lat = layout["start_a_lat"];
      trackLayouts[numOfTracks].start_a_lng = layout["start_a_lng"];
      trackLayouts[numOfTracks].start_b_lat = layout["start_b_lat"];
      trackLayouts[numOfTracks].start_b_lng = layout["start_b_lng"];

      // Check if sector 2 data is present (optional)
      if (layout.containsKey("sector_2_a_lat") && layout.containsKey("sector_2_a_lng") &&
          layout.containsKey("sector_2_b_lat") && layout.containsKey("sector_2_b_lng")) {
        trackLayouts[numOfTracks].sector_2_a_lat = layout["sector_2_a_lat"];
        trackLayouts[numOfTracks].sector_2_a_lng = layout["sector_2_a_lng"];
        trackLayouts[numOfTracks].sector_2_b_lat = layout["sector_2_b_lat"];
        trackLayouts[numOfTracks].sector_2_b_lng = layout["sector_2_b_lng"];
        trackLayouts[numOfTracks].hasSector2 = true;
        #ifdef HAS_DEBUG
        debugln(F("  Sector 2 data loaded"));
        #endif
      } else {
        trackLayouts[numOfTracks].hasSector2 = false;
      }

      // Check if sector 3 data is present (optional)
      if (layout.containsKey("sector_3_a_lat") && layout.containsKey("sector_3_a_lng") &&
          layout.containsKey("sector_3_b_lat") && layout.containsKey("sector_3_b_lng")) {
        trackLayouts[numOfTracks].sector_3_a_lat = layout["sector_3_a_lat"];
        trackLayouts[numOfTracks].sector_3_a_lng = layout["sector_3_a_lng"];
        trackLayouts[numOfTracks].sector_3_b_lat = layout["sector_3_b_lat"];
        trackLayouts[numOfTracks].sector_3_b_lng = layout["sector_3_b_lng"];
        trackLayouts[numOfTracks].hasSector3 = true;
        #ifdef HAS_DEBUG
        debugln(F("  Sector 3 data loaded"));
        #endif
      } else {
        trackLayouts[numOfTracks].hasSector3 = false;
      }

      numOfTracks++;
    }
  }

  // make sure to close before logging
  debugln(F("ParseTrackFile: SUCCESS"));
  trackFile.close();
  return PARSE_STATUS_GOOD;
}

///////////////////////////////////////////
// BLUETOOTH FUNCTIONS (after SdFat include)
///////////////////////////////////////////

File32 bleCurrentFile;

void bleConnectCallback(uint16_t conn_handle) {
  debugln(F("BLE: Device connected!"));
  bleConnected = true;

  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  debug(F("BLE: Initial MTU: "));
  debugln(connection->getMtu());

  // Request MTU exchange
  debugln(F("BLE: Requesting MTU exchange to 247..."));
  if (connection->requestMtuExchange(247)) {
    debugln(F("BLE: MTU exchange requested successfully"));
  } else {
    debugln(F("BLE: MTU exchange request failed!"));
  }

  // Wait for MTU negotiation
  delay(500);

  bleNegotiatedMtu = connection->getMtu();
  debug(F("BLE: Negotiated MTU: "));
  debugln(bleNegotiatedMtu);

  debug(F("BLE: Connection interval: "));
  debug(connection->getConnectionInterval() * 1.25);
  debugln(F("ms"));
}

void bleDisconnectCallback(uint16_t conn_handle, uint8_t reason) {
  debugln(F("BLE: Disconnected!"));
  bleConnected = false;
  bleNegotiatedMtu = 23; // Reset to default
  if (bleCurrentFile) {
    bleCurrentFile.close();
  }
  bleTransferInProgress = false;
}

// Forward declaration for callback
void bleFileRequestCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);
void bleUploadDataCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);

void bleSetupFileService() {
  fileService.begin();

  // File List Characteristic
  fileListChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  fileListChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  fileListChar.setMaxLen(244);
  fileListChar.begin();

  // File Request Characteristic (commands)
  fileRequestChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  fileRequestChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  fileRequestChar.setMaxLen(128);  // Increased for longer commands like UPLOAD:{path}:{size}
  fileRequestChar.setWriteCallback(bleFileRequestCallback);
  fileRequestChar.begin();

  // File Data Characteristic (download data - device to client)
  fileDataChar.setProperties(CHR_PROPS_NOTIFY);
  fileDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  fileDataChar.setMaxLen(244);
  fileDataChar.begin();

  // File Status Characteristic
  fileStatusChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  fileStatusChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  fileStatusChar.setMaxLen(64);
  fileStatusChar.begin();

  // File Upload Characteristic (upload data - client to device)
  fileUploadChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  fileUploadChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  fileUploadChar.setMaxLen(244);
  fileUploadChar.setWriteCallback(bleUploadDataCallback);
  fileUploadChar.begin();
}

void bleStartAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(fileService);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void bleSendFileList(String directory = "/") {
  // Ensure directory starts with /
  if (!directory.startsWith("/")) {
    directory = "/" + directory;
  }

  String fileList = "";
  File32 root = SD.open(directory.c_str());

  if (!root) {
    debug(F("BLE: Failed to open directory: "));
    debugln(directory);
    fileListChar.notify((uint8_t*)"ERROR", 5);
    return;
  }

  while (true) {
    File32 entry = root.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory()) {
      char name[256];
      entry.getName(name, sizeof(name));

      if (fileList.length() > 0) fileList += "|";
      fileList += String(name) + ":" + String(entry.size());
    }
    entry.close();
  }
  root.close();

  debug(F("BLE: Sending file list for "));
  debug(directory);
  debug(F(" ("));
  debug(fileList.length());
  debugln(F(" bytes)"));

  uint16_t maxChunk = bleNegotiatedMtu - 3;
  uint16_t offset = 0;

  while (offset < fileList.length()) {
    uint16_t chunkSize = min(maxChunk, (uint16_t)(fileList.length() - offset));

    char chunkBuf[600];
    fileList.substring(offset, offset + chunkSize).toCharArray(chunkBuf, chunkSize + 1);

    fileListChar.notify((uint8_t*)chunkBuf, chunkSize);

    offset += chunkSize;
    delay(10);
  }

  fileListChar.notify((uint8_t*)"END", 3);
  debugln(F("BLE: File list sent!"));
}

void bleStartFileTransfer(String filename) {
  if (bleCurrentFile) bleCurrentFile.close();

  debug(F("BLE: Opening file: ["));
  debug(filename);
  debugln(F("]"));

  bleCurrentFile = SD.open(filename.c_str(), FILE_READ);

  if (!bleCurrentFile) {
    debugln(F("BLE: Failed to open file!"));
    fileStatusChar.notify((uint8_t*)"ERROR", 5);
    return;
  }

  bleFileSize = bleCurrentFile.size();
  bleBytesTransferred = 0;
  bleTransferInProgress = true;

  debug(F("BLE: File size: "));
  debug(bleFileSize);
  debug(F(" bytes, MTU: "));
  debugln(bleNegotiatedMtu);

  char sizeMsg[32];
  snprintf(sizeMsg, sizeof(sizeMsg), "SIZE:%lu", bleFileSize);
  fileStatusChar.notify((uint8_t*)sizeMsg, strlen(sizeMsg));

  delay(50);
}

void bleDeleteFile(String filename) {
  debug(F("BLE: Deleting file: ["));
  debug(filename);
  debugln(F("]"));

  if (SD.exists(filename.c_str())) {
    if (SD.remove(filename.c_str())) {
      debugln(F("BLE: File deleted successfully"));
      fileStatusChar.notify((uint8_t*)"DELETED", 7);
    } else {
      debugln(F("BLE: Failed to delete file"));
      fileStatusChar.notify((uint8_t*)"DEL_ERR", 7);
    }
  } else {
    debugln(F("BLE: File not found"));
    fileStatusChar.notify((uint8_t*)"NOT_FOUND", 9);
  }
}

void bleStartUpload(String filepath, uint32_t expectedSize) {
  // Close any existing upload
  if (bleUploadFile) {
    bleUploadFile.close();
  }

  debug(F("BLE: Starting upload: ["));
  debug(filepath);
  debug(F("] size: "));
  debugln(expectedSize);

  // Ensure path starts with /
  if (!filepath.startsWith("/")) {
    filepath = "/" + filepath;
  }

  // Create parent directories if needed
  int lastSlash = filepath.lastIndexOf('/');
  if (lastSlash > 0) {
    String parentDir = filepath.substring(0, lastSlash);
    if (!SD.exists(parentDir.c_str())) {
      debug(F("BLE: Creating directory: "));
      debugln(parentDir);
      SD.mkdir(parentDir.c_str());
    }
  }

  // Open file for writing (will overwrite if exists)
  bleUploadFile = SD.open(filepath.c_str(), FILE_WRITE);

  if (!bleUploadFile) {
    debugln(F("BLE: Failed to open file for upload!"));
    fileStatusChar.notify((uint8_t*)"UPLOAD_ERR", 10);
    return;
  }

  bleUploadExpectedSize = expectedSize;
  bleUploadBytesReceived = 0;
  bleUploadInProgress = true;

  debugln(F("BLE: Ready to receive upload data"));
  fileStatusChar.notify((uint8_t*)"READY", 5);
}

void bleUploadDataCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  if (!bleUploadInProgress || !bleUploadFile) {
    debugln(F("BLE: Upload data received but no upload in progress"));
    return;
  }

  size_t written = bleUploadFile.write(data, len);
  bleUploadBytesReceived += written;

  // Progress update every 10KB
  if (bleUploadBytesReceived % 10000 < len) {
    debug(F("BLE: Upload progress: "));
    debug(bleUploadBytesReceived);
    debug(F(" / "));
    debug(bleUploadExpectedSize);
    debug(F(" ("));
    debug((bleUploadBytesReceived * 100) / bleUploadExpectedSize);
    debugln(F("%)"));
  }
}

void bleFinishUpload() {
  if (!bleUploadInProgress) {
    debugln(F("BLE: No upload in progress to finish"));
    fileStatusChar.notify((uint8_t*)"UPLOAD_ERR", 10);
    return;
  }

  bleUploadFile.flush();
  bleUploadFile.close();
  bleUploadInProgress = false;

  debug(F("BLE: Upload complete! Received "));
  debug(bleUploadBytesReceived);
  debug(F(" of "));
  debug(bleUploadExpectedSize);
  debugln(F(" bytes"));

  if (bleUploadBytesReceived == bleUploadExpectedSize) {
    fileStatusChar.notify((uint8_t*)"UPLOADED", 8);
  } else {
    debugln(F("BLE: Warning - size mismatch!"));
    fileStatusChar.notify((uint8_t*)"UPLOADED", 8);  // Still report success, let client validate
  }
}

void bleCancelUpload() {
  if (bleUploadFile) {
    bleUploadFile.close();
  }
  bleUploadInProgress = false;
  bleUploadBytesReceived = 0;
  bleUploadExpectedSize = 0;

  debugln(F("BLE: Upload canceled"));
  fileStatusChar.notify((uint8_t*)"UPLOAD_CANCELED", 15);
}

void bleFileRequestCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  char buffer[129];
  memset(buffer, 0, sizeof(buffer));
  memcpy(buffer, data, min(len, (uint16_t)128));

  String command = String(buffer);
  command.trim();

  debug(F("BLE: Received command: ["));
  debug(command);
  debugln(F("]"));

  if (command == "LIST") {
    // List root directory (datalogs)
    bleSendFileList("/");
  } else if (command.startsWith("LIST:")) {
    // List specified directory
    String directory = command.substring(5);
    directory.trim();
    bleSendFileList(directory);
  } else if (command.startsWith("GET:")) {
    String filename = command.substring(4);
    filename.trim();
    bleStartFileTransfer(filename);
  } else if (command.startsWith("DELETE:")) {
    String filename = command.substring(7);
    filename.trim();
    bleDeleteFile(filename);
  } else if (command.startsWith("UPLOAD:")) {
    // Format: UPLOAD:{filepath}:{size}
    String params = command.substring(7);
    int colonPos = params.lastIndexOf(':');
    if (colonPos > 0) {
      String filepath = params.substring(0, colonPos);
      String sizeStr = params.substring(colonPos + 1);
      filepath.trim();
      sizeStr.trim();
      uint32_t size = sizeStr.toInt();
      bleStartUpload(filepath, size);
    } else {
      debugln(F("BLE: Invalid UPLOAD command format"));
      fileStatusChar.notify((uint8_t*)"UPLOAD_ERR", 10);
    }
  } else if (command == "UPLOAD_DONE") {
    bleFinishUpload();
  } else if (command == "UPLOAD_CANCEL") {
    bleCancelUpload();
  }
}

void BLE_SETUP() {
  if (bleInitialized) {
    // Already initialized, just start advertising
    debugln(F("BLE: Restarting advertising..."));
    Bluefruit.autoConnLed(true);
    Bluefruit.setConnLedInterval(250); // Blink every 250ms when connected
    Bluefruit.Advertising.start(0);
    bleActive = true;
    return;
  }

  debugln(F("BLE: Initializing Bluetooth..."));

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("DovesLapTimer");

  // Enable connection LED
  Bluefruit.autoConnLed(true);
  Bluefruit.setConnLedInterval(250);

  Bluefruit.Periph.setConnectCallback(bleConnectCallback);
  Bluefruit.Periph.setDisconnectCallback(bleDisconnectCallback);

  // Set connection interval (7.5-15ms)
  Bluefruit.Periph.setConnInterval(6, 12);

  bleSetupFileService();
  bleStartAdvertising();

  bleInitialized = true;
  bleActive = true;

  debugln(F("BLE: Ready for connection!"));
}

void BLE_STOP() {
  if (!bleActive) return;

  debugln(F("BLE: Stopping Bluetooth..."));

  // Close any open download file
  if (bleCurrentFile) {
    bleCurrentFile.close();
  }
  bleTransferInProgress = false;

  // Close any open upload file
  if (bleUploadFile) {
    bleUploadFile.close();
  }
  bleUploadInProgress = false;
  bleUploadBytesReceived = 0;
  bleUploadExpectedSize = 0;

  // Disconnect any connected device
  if (Bluefruit.connected()) {
    Bluefruit.disconnect(Bluefruit.connHandle());
    delay(100);
  }

  // Stop advertising
  Bluefruit.Advertising.stop();

  // Turn off the BLE LED
  Bluefruit.autoConnLed(false);
  Bluefruit.setConnLedInterval(0);
  digitalWrite(LED_BLUE, HIGH);

  bleConnected = false;
  bleActive = false;

  debugln(F("BLE: Bluetooth stopped"));
}

void BLUETOOTH_LOOP() {
  if (!bleActive) return;

  if (bleTransferInProgress && bleCurrentFile && Bluefruit.connected()) {
    // Use actual negotiated MTU
    uint16_t maxChunk = bleNegotiatedMtu - 3;

    uint8_t buffer[524];

    size_t chunkSize = min(maxChunk, (uint16_t)244);
    size_t bytesRead = bleCurrentFile.read(buffer, chunkSize);

    if (bytesRead > 0) {
      if (fileDataChar.notify(buffer, bytesRead)) {
        bleBytesTransferred += bytesRead;

        // Progress update every 10KB
        if (bleBytesTransferred % 10000 == 0) {
          debug(F("BLE: Progress: "));
          debug(bleBytesTransferred);
          debug(F(" / "));
          debug(bleFileSize);
          debug(F(" ("));
          debug((bleBytesTransferred * 100) / bleFileSize);
          debugln(F("%)"));
        }
      }
    } else {
      // Transfer complete
      bleCurrentFile.close();
      bleTransferInProgress = false;

      debugln(F("BLE: Transfer complete!"));

      fileStatusChar.notify((uint8_t*)"DONE", 4);
    }
  }
}

///////////////////////////////////////////
// REPLAY FUNCTIONS IMPLEMENTATION
///////////////////////////////////////////

/**
 * @brief Reset replay state to initial values
 */
void resetReplayState() {
  replayModeActive = false;
  replayProcessingComplete = false;
  replayDetectedTrackIndex = -1;
  replayMaxSpeed = 0.0;
  replayMaxRpm = 0;
  replayTotalSamples = 0;
  replayProcessedSamples = 0;

  // Reset lap timer and history for replay
  lapTimer.reset();
  lapHistoryCount = 0;
  lastLap = 0;
  memset(lapHistory, 0, sizeof(lapHistory));

  // Close replay file if open
  if (replayFile.isOpen()) {
    replayFile.close();
    debugln(F("Replay: Closed replay file"));
  }
}

/**
 * @brief Build list of .dove and .nmea files from SD card root
 * @return true if files found, false otherwise
 */
bool buildReplayFileList() {
  numReplayFiles = 0;

  if (!sdSetupSuccess) {
    debugln(F("Replay: SD not initialized"));
    return false;
  }

  // Defensive cleanup - ensure no stale file handles interfere with SD operations
  if (replayFile.isOpen()) {
    debugln(F("Replay: Closing stale replay file handle"));
    replayFile.close();
  }
  if (file.isOpen()) {
    debugln(F("Replay: Closing stale file handle"));
    file.close();
  }
  if (trackDir.isOpen()) {
    debugln(F("Replay: Closing stale trackDir handle"));
    trackDir.close();
  }
  if (trackFile.isOpen()) {
    debugln(F("Replay: Closing stale trackFile handle"));
    trackFile.close();
  }

  // Warn if data logging is active (potential conflict)
  if (dataFile.isOpen()) {
    debugln(F("Replay: WARNING - dataFile is open, may cause SD conflicts"));
  }

  // Delay to let SD card settle after any previous operations
  delay(50);

  File32 root = SD.open("/");
  if (!root) {
    debugln(F("Replay: Failed to open root"));
    return false;
  }

  debugln(F("Replay: Root opened, scanning files..."));

  int filesScanned = 0;
  while (numReplayFiles < MAX_REPLAY_FILES) {
    File32 entry = root.openNextFile();
    if (!entry) {
      debug(F("Replay: openNextFile returned null after "));
      debug(filesScanned);
      debugln(F(" files"));
      break;
    }
    filesScanned++;

    // Use larger buffer for getName - SdFat may fail silently with small buffers
    char name[64];
    entry.getName(name, sizeof(name));

    // Debug: show every file/dir found
    debug(F("Replay: ["));
    debug(filesScanned);
    debug(F("] "));
    debug(entry.isDirectory() ? F("DIR: ") : F("FILE: "));
    debug(name);
    debug(F(" (len="));
    debug(strlen(name));
    debugln(F(")"));

    if (!entry.isDirectory()) {
      // Check for .dove or .nmea extension (case insensitive)
      int len = strlen(name);
      if (len > 5) {
        char* ext = name + len - 5;
        bool isDove = (strcasecmp(ext, ".dove") == 0);
        bool isNmea = (strcasecmp(ext, ".nmea") == 0);

        if (isDove || isNmea) {
          strncpy(replayFiles[numReplayFiles], name, MAX_REPLAY_FILENAME_LENGTH - 1);
          replayFiles[numReplayFiles][MAX_REPLAY_FILENAME_LENGTH - 1] = '\0';
          numReplayFiles++;
          debug(F("Replay: Found file: "));
          debugln(name);
        }
      }
    }
    entry.close();;
  }

  root.close();

  debug(F("Replay: Total files found: "));
  debugln(numReplayFiles);

  return numReplayFiles > 0;
}

/**
 * @brief Read a single line from file into buffer using block reads for speed
 * @param file File handle
 * @param buffer Output buffer
 * @param bufferSize Size of output buffer
 * @return true if line read successfully, false on EOF or error
 */
bool readReplayLine(File& file, char* buffer, int bufferSize) {
  int pos = 0;

  while (pos < bufferSize - 1) {
    int c = file.read();

    if (c < 0) {
      // EOF
      if (pos > 0) {
        buffer[pos] = '\0';
        return true;
      }
      return false;
    }

    if (c == '\n') {
      buffer[pos] = '\0';
      return true;
    }

    if (c == '\r') {
      // Skip carriage return
      continue;
    }

    buffer[pos++] = (char)c;
  }

  // Line too long, truncate
  buffer[bufferSize - 1] = '\0';

  // Skip rest of line
  while (true) {
    int c = file.read();
    if (c < 0 || c == '\n') break;
  }

  return true;
}

/**
 * @brief Parse a DOVE CSV line into a ReplaySample (parses in-place, modifies line)
 * Format: timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,rpm,exhaust_temp_c,water_temp_c
 * @param line Input CSV line (will be modified by strtok)
 * @param sample Output sample struct
 * @return true if parsed successfully
 */
bool parseDoveLine(char* line, ReplaySample& sample) {
  sample.valid = false;
  sample.rpm = -1;

  // Skip header line
  if (strstr(line, "timestamp") != NULL) {
    return false;
  }

  // Skip empty lines
  if (strlen(line) < 10) {
    return false;
  }

  // Parse CSV fields in-place (no copy needed - saves 128+ bytes of stack)
  char* token;
  int fieldIndex = 0;

  token = strtok(line, ",");
  while (token != NULL && fieldIndex < 10) {
    switch (fieldIndex) {
      case 0: // timestamp
        sample.timestamp = strtoul(token, NULL, 10);
        break;
      case 1: // sats - skip
        break;
      case 2: // hdop - skip
        break;
      case 3: // lat
        sample.lat = atof(token);
        break;
      case 4: // lng
        sample.lng = atof(token);
        break;
      case 5: // speed_mph
        sample.speed_mph = atof(token);
        break;
      case 6: // altitude_m
        sample.altitude = atof(token);
        break;
      case 7: // rpm
        sample.rpm = atoi(token);
        break;
      // 8, 9: exhaust_temp_c, water_temp_c - skip
    }
    fieldIndex++;
    token = strtok(NULL, ",");
  }

  // Validate we got minimum required fields
  if (fieldIndex >= 6 && sample.lat != 0.0 && sample.lng != 0.0) {
    sample.valid = true;
    return true;
  }

  return false;
}

/**
 * @brief Parse an NMEA sentence into a ReplaySample (parses in-place, modifies line)
 * Supports GPGGA, GNGGA, GPRMC, GNRMC
 * @param line Input NMEA sentence (will be modified by strtok)
 * @param sample Output sample struct
 * @return true if parsed successfully
 */
bool parseNmeaSentence(char* line, ReplaySample& sample) {
  sample.valid = false;
  sample.rpm = -1;  // NMEA has no RPM
  sample.altitude = 0.0;

  // Check for valid NMEA sentence
  if (line[0] != '$') {
    return false;
  }

  // Check sentence type (do this before strtok modifies the line)
  bool isGGA = (strstr(line, "GGA") != NULL);
  bool isRMC = (strstr(line, "RMC") != NULL);

  if (!isGGA && !isRMC) {
    return false;
  }

  // Parse NMEA sentence in-place (no copy needed - saves 128+ bytes of stack)
  char* token;
  int fieldIndex = 0;

  double latDeg = 0, latMin = 0;
  double lngDeg = 0, lngMin = 0;
  char latDir = 'N', lngDir = 'W';
  float speedKnots = 0;

  token = strtok(line, ",");
  while (token != NULL) {
    if (isGGA) {
      switch (fieldIndex) {
        case 1: // Time - can be used for timestamp
          {
            // Parse HHMMSS.sss format
            if (strlen(token) >= 6) {
              int hour = (token[0] - '0') * 10 + (token[1] - '0');
              int min = (token[2] - '0') * 10 + (token[3] - '0');
              int sec = (token[4] - '0') * 10 + (token[5] - '0');
              int ms = 0;
              if (strlen(token) > 7) {
                ms = atoi(token + 7);
              }
              sample.timestamp = hour * 3600000UL + min * 60000UL + sec * 1000UL + ms;
            }
          }
          break;
        case 2: // Latitude DDMM.MMMM
          if (strlen(token) >= 4) {
            latDeg = (token[0] - '0') * 10 + (token[1] - '0');
            latMin = atof(token + 2);
          }
          break;
        case 3: // N/S
          latDir = token[0];
          break;
        case 4: // Longitude DDDMM.MMMM
          if (strlen(token) >= 5) {
            lngDeg = (token[0] - '0') * 100 + (token[1] - '0') * 10 + (token[2] - '0');
            lngMin = atof(token + 3);
          }
          break;
        case 5: // E/W
          lngDir = token[0];
          break;
        case 9: // Altitude
          sample.altitude = atof(token);
          break;
      }
    } else if (isRMC) {
      switch (fieldIndex) {
        case 1: // Time
          {
            if (strlen(token) >= 6) {
              int hour = (token[0] - '0') * 10 + (token[1] - '0');
              int min = (token[2] - '0') * 10 + (token[3] - '0');
              int sec = (token[4] - '0') * 10 + (token[5] - '0');
              int ms = 0;
              if (strlen(token) > 7) {
                ms = atoi(token + 7);
              }
              sample.timestamp = hour * 3600000UL + min * 60000UL + sec * 1000UL + ms;
            }
          }
          break;
        case 3: // Latitude
          if (strlen(token) >= 4) {
            latDeg = (token[0] - '0') * 10 + (token[1] - '0');
            latMin = atof(token + 2);
          }
          break;
        case 4: // N/S
          latDir = token[0];
          break;
        case 5: // Longitude
          if (strlen(token) >= 5) {
            lngDeg = (token[0] - '0') * 100 + (token[1] - '0') * 10 + (token[2] - '0');
            lngMin = atof(token + 3);
          }
          break;
        case 6: // E/W
          lngDir = token[0];
          break;
        case 7: // Speed in knots
          speedKnots = atof(token);
          break;
      }
    }

    fieldIndex++;
    token = strtok(NULL, ",*");
  }

  // Convert to decimal degrees
  sample.lat = latDeg + (latMin / 60.0);
  if (latDir == 'S') sample.lat = -sample.lat;

  sample.lng = lngDeg + (lngMin / 60.0);
  if (lngDir == 'W') sample.lng = -sample.lng;

  // Convert knots to mph (1 knot = 1.15078 mph)
  sample.speed_mph = speedKnots * 1.15078;

  // Validate
  if (sample.lat != 0.0 && sample.lng != 0.0) {
    sample.valid = true;
    return true;
  }

  return false;
}

/**
 * @brief Calculate haversine distance between two GPS points in miles
 */
double haversineDistanceMiles(double lat1, double lng1, double lat2, double lng2) {
  const double R = 3958.8; // Earth radius in miles
  // Note: DEG_TO_RAD is already defined by Arduino

  double dLat = (lat2 - lat1) * DEG_TO_RAD;
  double dLng = (lng2 - lng1) * DEG_TO_RAD;

  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * DEG_TO_RAD) * cos(lat2 * DEG_TO_RAD) *
             sin(dLng / 2) * sin(dLng / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));

  return R * c;
}

/**
 * @brief Extract a single GPS point from a replay file (first valid point)
 */
bool extractGpsPointFromReplayFile(const char* filename, double& lat, double& lng) {
  File file;
  if (!file.open(filename, O_READ)) {
    debug(F("Replay: Cannot open file: "));
    debugln(filename);
    return false;
  }

  // Determine file type
  int len = strlen(filename);
  bool isDove = (len > 5 && strcasecmp(filename + len - 5, ".dove") == 0);

  char lineBuffer[REPLAY_LINE_BUFFER_SIZE];
  int linesChecked = 0;
  const int maxLinesToCheck = 100; // Don't scan too much

  while (linesChecked < maxLinesToCheck && readReplayLine(file, lineBuffer, sizeof(lineBuffer))) {
    linesChecked++;

    ReplaySample sample;
    bool parsed = false;

    if (isDove) {
      parsed = parseDoveLine(lineBuffer, sample);
    } else {
      parsed = parseNmeaSentence(lineBuffer, sample);
    }

    if (parsed && sample.valid) {
      lat = sample.lat;
      lng = sample.lng;
      file.close();
      debug(F("Replay: Extracted point: "));
      debug(lat, 6);
      debug(F(", "));
      debugln(lng, 6);
      return true;
    }
  }

  file.close();
  debugln(F("Replay: No valid GPS point found in file"));
  return false;
}

/**
 * @brief Extract a single GPS point from a track file (start line midpoint)
 */
bool extractGpsPointFromTrackFile(int trackIndex, double& lat, double& lng) {
  if (trackIndex < 0 || trackIndex >= numOfLocations) {
    return false;
  }

  // Parse the track file to get coordinates
  char filepath[50];
  makeFullTrackPath(locations[trackIndex], filepath);

  File trackFileTemp;
  trackFileTemp.open(filepath, O_READ);
  if (!trackFileTemp) {
    debug(F("Replay: Cannot open track: "));
    debugln(filepath);
    return false;
  }

  // Read JSON content
  char buffer[JSON_BUFFER_SIZE];
  int bytesRead = trackFileTemp.read(buffer, sizeof(buffer) - 1);
  trackFileTemp.close();

  if (bytesRead <= 0) {
    return false;
  }
  buffer[bytesRead] = '\0';

  // Parse JSON
  StaticJsonDocument<JSON_BUFFER_SIZE> trackJson;
  DeserializationError error = deserializeJson(trackJson, buffer);
  if (error != DeserializationError::Ok) {
    return false;
  }

  // Get first layout's start line midpoint
  JsonArray array = trackJson.as<JsonArray>();
  if (array.size() == 0) {
    return false;
  }

  JsonVariant firstLayout = array[0];
  double a_lat = firstLayout["start_a_lat"];
  double a_lng = firstLayout["start_a_lng"];
  double b_lat = firstLayout["start_b_lat"];
  double b_lng = firstLayout["start_b_lng"];

  // Calculate midpoint
  lat = (a_lat + b_lat) / 2.0;
  lng = (a_lng + b_lng) / 2.0;

  debug(F("Replay: Track "));
  debug(locations[trackIndex]);
  debug(F(" point: "));
  debug(lat, 6);
  debug(F(", "));
  debugln(lng, 6);

  return true;
}

/**
 * @brief Detect which track a replay file belongs to
 * @param filename The replay file name
 * @return Track index if found, -1 otherwise
 */
int detectTrackForReplayFile(const char* filename) {
  double fileLat, fileLng;

  if (!extractGpsPointFromReplayFile(filename, fileLat, fileLng)) {
    debugln(F("Replay: Failed to extract GPS point from file"));
    return -1;
  }

  // Compare against each track
  for (int i = 0; i < numOfLocations; i++) {
    double trackLat, trackLng;

    if (extractGpsPointFromTrackFile(i, trackLat, trackLng)) {
      double distance = haversineDistanceMiles(fileLat, fileLng, trackLat, trackLng);

      debug(F("Replay: Distance to "));
      debug(locations[i]);
      debug(F(": "));
      debug(distance, 2);
      debugln(F(" miles"));

      if (distance <= REPLAY_TRACK_DETECTION_THRESHOLD_MILES) {
        debug(F("Replay: Matched track: "));
        debugln(locations[i]);
        return i;
      }
    }
  }

  debugln(F("Replay: No matching track found"));
  return -1;
}

/**
 * @brief Process the selected replay file through the lap timer
 * Call this from loop() when in replay processing state
 */
void processReplayFile() {
  // Use isOpen() for proper SdFat file checking
  if (!replayFile.isOpen()) {
    debugln(F("Replay: File not open, marking complete"));
    replayProcessingComplete = true;
    return;
  }

  // Process multiple lines per call to speed things up
  const int linesPerCall = 50;

  // Determine file type once per batch (optimization)
  int len = strlen(replayFiles[selectedReplayFile]);
  bool isDove = (len > 5 && strcasecmp(replayFiles[selectedReplayFile] + len - 5, ".dove") == 0);

  for (int i = 0; i < linesPerCall; i++) {
    if (!readReplayLine(replayFile, replayLineBuffer, sizeof(replayLineBuffer))) {
      // EOF - processing complete
      replayFile.close();
      replayProcessingComplete = true;
      debugln(F("Replay: Processing complete"));
      debug(F("Replay: Total samples: "));
      debugln(replayProcessedSamples);
      debug(F("Replay: Max speed: "));
      debugln(replayMaxSpeed, 2);
      debug(F("Replay: Max RPM: "));
      debugln(replayMaxRpm);
      debug(F("Replay: Laps: "));
      debugln(lapTimer.getLaps());
      return;
    }

    replayTotalSamples++;

    ReplaySample sample;
    bool parsed = false;

    if (isDove) {
      parsed = parseDoveLine(replayLineBuffer, sample);
    } else {
      parsed = parseNmeaSentence(replayLineBuffer, sample);
    }

    if (parsed && sample.valid) {
      replayProcessedSamples++;

      // Update lap timer with this sample
      lapTimer.updateCurrentTime(sample.timestamp);
      lapTimer.loop(sample.lat, sample.lng, sample.altitude, sample.speed_mph / 1.15078); // Convert mph to knots for lapTimer

      // Track max values
      if (sample.speed_mph > replayMaxSpeed) {
        replayMaxSpeed = sample.speed_mph;
      }
      if (sample.rpm > replayMaxRpm) {
        replayMaxRpm = sample.rpm;
      }

      // Check for new lap
      checkForNewLapData();
    }
  }
}

///////////////////////////////////////////

ButtonState button1;
ButtonState *btn1 = &button1;
ButtonState button2;
ButtonState *btn2 = &button2;
ButtonState button3;
ButtonState *btn3 = &button3;

float epsilonPrecision = 0.001;

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

int buttonPressIntv = 500;
int buttonHoldIntv = 1000;
int antiBounceIntv = 500;

bool recentlyChanged = false;

void setupButtons() {
  #ifndef WOKWI
  // blackbox
  // btn1->pin = 1;
  // btn2->pin = 2;
  // btn3->pin = 0;
  // greybox
  btn1->pin = 1;
  btn2->pin = 2;
  btn3->pin = 3;

  #else
  btn1->pin = 4;
  btn2->pin = 5;
  btn3->pin = 6;
  #endif

  pinMode(btn1->pin, INPUT_PULLUP);
  pinMode(btn2->pin, INPUT_PULLUP);
  pinMode(btn3->pin, INPUT_PULLUP);
}
void readButtons() {
  checkButton(btn1);
  checkButton(btn2);
  checkButton(btn3);

  //force update when button pressed
  if (
    btn1->pressed ||
    btn2->pressed ||
    btn3->pressed
  ) {
    forceDisplayRefresh();
  }
}
void resetButtons() {
  resetButton(btn1);
  resetButton(btn2);
  resetButton(btn3);
}
void resetButton(ButtonState* button) {
  button->pressed = false;
}
void checkButton(ButtonState* button) {
  bool btnPressed = digitalRead(button->pin) == LOW;
  bool btnReady = millis() - button->lastPressed >= antiBounceIntv;
  if (btnReady && btnPressed) {
    button->lastPressed = millis();
    button->pressed = true;
  }
}

//////////////////////////////////////////
// TODO: make display into own class??
int menuSelectionIndex = 0;
void resetDisplay() {
  if (currentPage != lastPage) {
    lastPage = currentPage;
    recentlyChanged = true;
    menuSelectionIndex = 0;
  } else {
    recentlyChanged = false;
  }
  display.setTextWrap(false);
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.setTextColor(DISPLAY_TEXT_WHITE);
}

void displayPage_boot() {
  resetDisplay();
  
  display.setTextSize(2);
  display.println(F("   Doves\n MagicBox"));
  display.setTextSize(1);
  display.println(F(""));
  display.println(F(" Timer + Data Logger"));
  display.println(F("\n    Initializing..."));

  display.display();
}

///////////////////////////////////////////
int selectedTrack = -1;
int selectedDirection = -1;

void displayPage_select_location() {
  resetDisplay();
  if (recentlyChanged && selectedLocation >= 0) {
    menuSelectionIndex = selectedLocation;
  }

  display.print(F("Select Track: "));
  display.print(menuSelectionIndex+1);
  display.print(F("/"));
  display.println(numOfLocations);
  display.println();
  display.setTextSize(2);

  if (numOfLocations < 3) {
    display.println();
    // small menu
    if (numOfLocations >= 1) {
      if (menuSelectionIndex == 0) {
        display.print(F("->"));
      } else {
        display.print(F("  "));
      }
      display.println(locations[0]);
    }
    if (numOfLocations >= 2) {
      if (menuSelectionIndex == 1) {
        display.print(F("->"));
      } else {
        display.print(F("  "));
      }
      display.println(locations[1]);
    }
  } else {
    // scrolling menu
    int indexA = menuSelectionIndex == numOfLocations - 1 ? 0 : menuSelectionIndex + 1;
    int indexB = menuSelectionIndex;
    int indexC = menuSelectionIndex == 0 ? numOfLocations - 1 : menuSelectionIndex - 1;
    display.print(F("  "));
    display.println(locations[indexA]);
    display.print(F("->"));
    display.println(locations[indexB]);
    display.print(F("  "));
    display.println(locations[indexC]);
  }

  display.display();
}

void displayPage_select_track() {
  resetDisplay();
  if (recentlyChanged && selectedTrack >= 0) {
    menuSelectionIndex = selectedTrack;
  }
  display.print(F("Select Layout: "));
  display.print(menuSelectionIndex+1);
  display.print(F("/"));
  display.println(numOfTracks);
  display.println(locations[selectedLocation]);
  display.setTextSize(2);

  if (numOfTracks < 3) {
    display.println();
    // small menu
    if (numOfTracks >= 1) {
      if (menuSelectionIndex == 0) {
        display.print(F("->"));
      } else {
        display.print(F("  "));
      }
      display.println(tracks[0]);
    }
    if (numOfTracks >= 2) {
      if (menuSelectionIndex == 1) {
        display.print(F("->"));
      } else {
        display.print(F("  "));
      }
      display.println(tracks[1]);
    }
  } else {
    // scrolling menu
    int indexA = menuSelectionIndex == numOfTracks - 1 ? 0 : menuSelectionIndex + 1;
    int indexB = menuSelectionIndex;
    int indexC = menuSelectionIndex == 0 ? numOfTracks - 1 : menuSelectionIndex - 1;
    display.print(F("  "));
    display.println(tracks[indexA]);
    display.print(F("->"));
    display.println(tracks[indexB]);
    display.print(F("  "));
    display.println(tracks[indexC]);
  }
  
  display.display();
}

void displayPage_select_direction() {
  resetDisplay();
  if (recentlyChanged && selectedDirection >= 0) {
    menuSelectionIndex = selectedDirection;
  }

  display.print(F("Select Direction"));
  display.println();
  display.setTextSize(2);

  display.println(F(""));
  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("Forward"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("Reverse"));

  display.display();
}

void displayPage_main_menu() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("   Doves MagicBox"));
  display.setTextSize(1);
  display.println();
  display.setTextSize(2);

  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("Race"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("Review"));
  display.print(menuSelectionIndex == 2 ? "->" : "  ");
  display.println(F("Transfer"));

  display.display();
}

void displayPage_bluetooth() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F(" Bluetooth Connection"));
  display.println();

  display.setTextSize(2);
  if (bleConnected) {
    display.println(F(" Connected"));
  } else {
    display.println(F("  Waiting"));
  }

  display.setTextSize(1);
  display.println();

  if (bleTransferInProgress) {
    display.print(F("Transfer: "));
    display.print((bleBytesTransferred * 100) / bleFileSize);
    display.println(F("%"));
  } else {
    display.println();
  }

  display.println();
  display.setTextSize(1);
  display.println(F("->Exit"));

  display.display();
}

void displayPage_replay_file_select() {
  resetDisplay();

  display.print(F("Select Session: "));
  display.print(menuSelectionIndex + 1);
  display.print(F("/"));
  display.println(numReplayFiles);
  display.println();
  display.setTextSize(1);

  if (numReplayFiles == 0) {
    display.println();
    display.println(F("No .dove or .nmea"));
    display.println(F("files found!"));
    display.println();
    display.println(F("Press any key"));
    display.println(F("to go back"));
  } else if (numReplayFiles < 3) {
    // Small menu - show all files
    for (int i = 0; i < numReplayFiles; i++) {
      if (menuSelectionIndex == i) {
        display.print(F("->"));
      } else {
        display.print(F("  "));
      }
      // Split long filenames across two lines
      int fileNameLen = strlen(replayFiles[i]);
      char displayName[20];

      // First line: first 19 characters
      strncpy(displayName, replayFiles[i], 19);
      displayName[19] = '\0';
      display.println(displayName);

      // Second line: next 19 characters if filename is longer
      if (fileNameLen > 19) {
        display.print(F("  "));  // Indent to align with first line
        strncpy(displayName, replayFiles[i] + 19, 19);
        displayName[19] = '\0';
        display.println(displayName);
      } else {
        display.println();  // Blank line if no wrap needed
      }
    }
  } else {
    // Scrolling menu
    int indexA = menuSelectionIndex == numReplayFiles - 1 ? 0 : menuSelectionIndex + 1;
    int indexB = menuSelectionIndex;
    int indexC = menuSelectionIndex == 0 ? numReplayFiles - 1 : menuSelectionIndex - 1;

    char displayName[20];
    int fileNameLen;

    // First item
    display.print(F("  "));
    fileNameLen = strlen(replayFiles[indexA]);
    strncpy(displayName, replayFiles[indexA], 19);
    displayName[19] = '\0';
    display.println(displayName);
    if (fileNameLen > 19) {
      display.print(F("  "));
      strncpy(displayName, replayFiles[indexA] + 19, 19);
      displayName[19] = '\0';
      display.println(displayName);
    } else {
      display.println();
    }

    // Second item (selected)
    display.print(F("->"));
    fileNameLen = strlen(replayFiles[indexB]);
    strncpy(displayName, replayFiles[indexB], 19);
    displayName[19] = '\0';
    display.println(displayName);
    if (fileNameLen > 19) {
      display.print(F("  "));
      strncpy(displayName, replayFiles[indexB] + 19, 19);
      displayName[19] = '\0';
      display.println(displayName);
    } else {
      display.println();
    }

    // Third item
    display.print(F("  "));
    fileNameLen = strlen(replayFiles[indexC]);
    strncpy(displayName, replayFiles[indexC], 19);
    displayName[19] = '\0';
    display.println(displayName);
    if (fileNameLen > 19) {
      display.print(F("  "));
      strncpy(displayName, replayFiles[indexC] + 19, 19);
      displayName[19] = '\0';
      display.println(displayName);
    } else {
      display.println();
    }
  }

  display.display();
}

void displayPage_replay_detecting() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("  Detecting Track"));
  display.println();

  display.setTextSize(2);
  display.println(F(" Please"));
  display.println(F("   Wait..."));

  display.display();
}

void displayPage_replay_select_track() {
  resetDisplay();

  if (recentlyChanged && selectedTrack >= 0) {
    menuSelectionIndex = selectedTrack;
  }

  display.print(F("Replay Layout: "));
  display.print(menuSelectionIndex + 1);
  display.print(F("/"));
  display.println(numOfTracks);
  display.println(locations[selectedLocation]);
  display.setTextSize(2);

  if (numOfTracks < 3) {
    display.println();
    if (numOfTracks >= 1) {
      display.print(menuSelectionIndex == 0 ? "->" : "  ");
      display.println(tracks[0]);
    }
    if (numOfTracks >= 2) {
      display.print(menuSelectionIndex == 1 ? "->" : "  ");
      display.println(tracks[1]);
    }
  } else {
    int indexA = menuSelectionIndex == numOfTracks - 1 ? 0 : menuSelectionIndex + 1;
    int indexB = menuSelectionIndex;
    int indexC = menuSelectionIndex == 0 ? numOfTracks - 1 : menuSelectionIndex - 1;
    display.print(F("  "));
    display.println(tracks[indexA]);
    display.print(F("->"));
    display.println(tracks[indexB]);
    display.print(F("  "));
    display.println(tracks[indexC]);
  }

  display.display();
}

void displayPage_replay_select_direction() {
  resetDisplay();

  if (recentlyChanged && selectedDirection >= 0) {
    menuSelectionIndex = selectedDirection;
  }

  display.println(F("Replay Direction"));
  display.println();
  display.setTextSize(2);

  display.println(F(""));
  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("Forward"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("Reverse"));

  display.display();
}

void displayPage_replay_processing() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("  Processing Replay"));
  display.println();

  // Show file name (truncated)
  char displayName[22];
  strncpy(displayName, replayFiles[selectedReplayFile], 21);
  displayName[21] = '\0';
  display.println(displayName);
  display.println();

  // Show progress
  display.setTextSize(2);
  if (replayTotalSamples > 0) {
    display.print(F(" "));
    display.print(replayProcessedSamples);
    display.println();
    display.setTextSize(1);
    display.println(F(" samples processed"));
  } else {
    display.println(F(" Starting..."));
  }

  display.setTextSize(1);
  display.println();
  if (lapTimer.getLaps() > 0) {
    display.print(F("Laps found: "));
    display.println(lapTimer.getLaps());
  }

  display.display();
}

void displayPage_replay_results() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("   Replay Results"));

  display.print(F("Laps: "));
  display.println(lapTimer.getLaps());

  if (lapTimer.getLaps() > 0) {
    // Best lap with lap number
    display.print(F("Best: "));
    unsigned long bestTime = lapTimer.getBestLapTime();
    unsigned long minutes = bestTime / 60000;
    unsigned long seconds = (bestTime % 60000) / 1000;
    unsigned long milliseconds = bestTime % 1000;
    if (minutes > 0) {
      display.print(minutes);
      display.print(F(":"));
    }
    if (seconds < 10 && minutes > 0) display.print(F("0"));
    display.print(seconds);
    display.print(F("."));
    if (milliseconds < 100) display.print(F("0"));
    if (milliseconds < 10) display.print(F("0"));
    display.print(milliseconds);
    display.print(F(" (L"));
    display.print(lapTimer.getBestLapNumber());
    display.println(F(")"));

    // Optimal lap with sector numbers (only if track has sectors)
    if (trackLayouts[selectedTrack].hasSector2 || trackLayouts[selectedTrack].hasSector3) {
      display.print(F("Opt: "));
      unsigned long optTime = lapTimer.getOptimalLapTime();
      minutes = optTime / 60000;
      seconds = (optTime % 60000) / 1000;
      milliseconds = optTime % 1000;
      if (minutes > 0) {
        display.print(minutes);
        display.print(F(":"));
      }
      if (seconds < 10 && minutes > 0) display.print(F("0"));
      display.print(seconds);
      display.print(F("."));
      if (milliseconds < 100) display.print(F("0"));
      if (milliseconds < 10) display.print(F("0"));
      display.print(milliseconds);
      display.print(F(" ("));
      display.print(lapTimer.getBestSector1LapNumber());
      display.print(F(","));
      display.print(lapTimer.getBestSector2LapNumber());
      display.print(F(","));
      display.print(lapTimer.getBestSector3LapNumber());
      display.println(F(")"));
    }
  }

  display.print(F("Max Spd: "));
  display.print(replayMaxSpeed, 1);
  display.println(F(" mph"));

  if (replayMaxRpm > 0) {
    display.print(F("Max RPM: "));
    display.println(replayMaxRpm);
  } else {
    display.println();
  }

  display.println();
  display.println(F("<- Laps       Exit ->"));

  display.display();
}

void displayPage_replay_exit() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F("   Exit Replay?"));
  display.println();

  display.setTextSize(2);
  display.println(F(""));
  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("Back"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("Exit"));

  display.display();
}

void displayPage_gps_stats() {
  resetDisplay();

  // Safety: GPS stats page requires GPS to be initialized
  if (!gpsInitialized || gps == NULL) {
    display.println(F("GPS not\ninitialized"));
    display.display();
    return;
  }

  // display.println(F("   Doves Magic Box\n"));

  if (millis() - lastBatteryCheck > batteryUpdateInterval) {
    lastBatteryCheck = millis();
    lastBatteryVoltage = getBatteryVoltage();
  }
  // display.println(F("Battery  : [#####]"));
  display.print(F("Battery  : "));
  display.print("[");
  if (lastBatteryVoltage > 3.7) {
    display.print("#");
  } else {
    display.print(" ");
  }
  if (lastBatteryVoltage >= 3.8) {
    display.print("#");
  } else {
    display.print(" ");
  }
  if (lastBatteryVoltage >= 3.9) {
    display.print("#");
  } else {
    display.print(" ");
  }
  if (lastBatteryVoltage >= 4.0) {
    display.print("#");
  } else {
    display.print(" ");
  }
  if (lastBatteryVoltage >= 4.1) {
    display.print("#");
  } else {
    display.print(" ");
  }
  display.println("]");


  display.print(F("Sats     : "));
  display.println(gps->satellites);
  
  display.print(F("Rate     : "));
  if (gps->fix) {
    display.print(gpsFrameRate, 1);
    display.println(F("Hz"));
  } else {
    display.println(F("NO FIX"));
  }

  display.print(F("HDOP     : "));
  if (gps->fix) {
    display.println(gps->HDOP, 1);
  } else {
    display.println(F("NO FIX"));
  }

  display.print(F("SDCard   : "));
  if (!sdSetupSuccess) {
    display.println(F("Bad Init"));
  } else if (enableLogging == true && sdDataLogInitComplete == false) {
    display.println(F("Waiting..."));
  } else if (enableLogging == true && sdDataLogInitComplete == true) {
    display.println(F("Logging"));
  } else if (enableLogging == false && sdDataLogInitComplete == true) {
    display.println(F("Bad File"));
  }

  display.println();

  if (sdSetupSuccess && sdTrackSuccess) {
    display.print(F("Track : "));
    display.println(locations[selectedLocation]);
    display.print(F("Layout: "));
    display.print(selectedDirection == RACE_DIRECTION_FORWARD ? "->" : "<-");
    display.print(F(" "));
    display.println(tracks[selectedTrack]);
  } else {
    display.print(F("Autologging\nShut off to stop"));
  }
  
  display.display();
}

void displayPage_gps_speed() {
  resetDisplay();

  display.println(F("SPEED"));

  if (sdSetupSuccess && sdTrackSuccess) {
    int currentLap = lapTimer.getLaps() + (lapTimer.getRaceStarted() ? 1 : 0);
    if (currentLap > 0) {
      display.println(F("\nLAP"));
      if (currentLap < 100) {
        display.setTextSize(3);
      } else {
        display.setTextSize(2);
      }
      display.print(currentLap);
    }
  }

  display.setCursor(40, 5);
  display.setTextSize(7);
  // Safety check for GPS access
  if (gpsInitialized && gps != NULL && gps->fix) {
    display.println(round(gps_speed_mph));
  } else {
    display.println(F("--"));
  }

  display.display();
}

void displayPage_gps_lap_time() {
  resetDisplay();

  display.println(F("  Current Lap Time"));

  ///////////////////////////////////////////////////
  // todo: fancier layout.
  /*
  display.println();
  const int lineHeight = 21;
  int leftMargin = 0;//29;//0;//29;
  if (leftMargin == 29) {
    display.setCursor(0, lineHeight);
    display.setTextSize(4);
    display.print(F("5"));
    display.setCursor(17, lineHeight + 4);
    display.setTextSize(3);
    display.print(F(":"));
  }

  display.setCursor(leftMargin, lineHeight);
  display.setTextSize(4);
  display.print(F("54"));
  display.setCursor(leftMargin + 42, lineHeight + 7);
  // display.setCursor(leftMargin + 42 + 5, lineHeight + 7);
  display.setTextSize(3);
  display.print(F("."));
  display.setCursor(leftMargin + 55, lineHeight);
  // display.setCursor(leftMargin + 55 + 5, lineHeight);
  display.setTextSize(4);
  display.print(F("173"));
  */
  ///////////////////////////////////////////////////

  display.print(F("\n\n"));
  display.setTextSize(3);
  if (lapTimer.getRaceStarted()) {
    unsigned long minutes = lapTimer.getCurrentLapTime() / 60000;
    unsigned long seconds = (lapTimer.getCurrentLapTime() % 60000) / 1000;
    unsigned long milliseconds = lapTimer.getCurrentLapTime() % 1000;
    if (minutes > 0) {
      display.print(minutes);
      display.print(":");
    } else {
      display.print(" ");
    }

    if (seconds < 10) {
      display.print(" ");
    }
    display.print(seconds);
    display.print(".");
    display.print(milliseconds);
    if (milliseconds < 10) {
      display.print("00");
    } else if (milliseconds < 100) {
      display.print("0");
    }
  } else {
    display.print("  N/A");
  }
  
  display.display();
}

bool paceFlashStatus = false;
void displayPage_gps_pace() {
  resetDisplay();

  display.println(F("  Current Lap Pace"));

  // animation
  if (lapTimer.getLaps() >= 1 && lapTimer.getPaceDifference() < (-1)) {
    if (paceFlashStatus) {
      paceFlashStatus = false;
      display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
      display.print(F("           "));
      display.setTextColor(DISPLAY_TEXT_WHITE);
      display.println(F("           "));
    } else {
      paceFlashStatus = true;
      display.setTextColor(DISPLAY_TEXT_WHITE);
      display.print(F("           "));
      display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
      display.println(F("           "));
    }
  }

  // main page into
  display.setTextColor(DISPLAY_TEXT_WHITE);
  const int lineHeight = 21;
  if (lapTimer.getRaceStarted() && lapTimer.getLaps() >= 1) {
    display.setCursor(0, lineHeight);
    display.setTextSize(4);
    if (lapTimer.getPaceDifference() > 0) {
      display.print(F("+"));
    }
    display.print(lapTimer.getPaceDifference());
  } else {
    display.setTextSize(2);
    display.println();
    display.setTextSize(3);
    display.print(F("  N/A"));
  }

  // animation
  display.println();
  display.setTextSize(1);
  
  if (lapTimer.getLaps() >= 1 && lapTimer.getPaceDifference() < (-1)) {
    if (paceFlashStatus) {
      // paceFlashStatus = false;
      display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
      display.print(F("           "));
      display.setTextColor(DISPLAY_TEXT_WHITE);
      display.println(F("           "));
    } else {
      // paceFlashStatus = true;
      display.setTextColor(DISPLAY_TEXT_WHITE);
      display.print(F("           "));
      display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
      display.println(F("           "));
    }
  }
  
  
  display.display();
}

void displayPage_gps_best_lap() {
  resetDisplay();

  display.println(F("      Best Lap"));
  display.print(F("\n"));
  if (
    lapTimer.getRaceStarted() &&
    lapTimer.getLaps() > 0
  ) {
    unsigned long minutes = lapTimer.getBestLapTime() / 60000;
    unsigned long seconds = (lapTimer.getBestLapTime() % 60000) / 1000;
    unsigned long milliseconds = lapTimer.getBestLapTime() % 1000;
    display.setTextSize(3);
    if (minutes > 0) {
      display.print(minutes);
      display.print(":");
    } else {
      display.print(" ");
    }

    if (seconds < 10) {
      display.print(" ");
    }
    display.print(seconds);
    display.print(".");
    display.print(milliseconds);
    if (milliseconds < 10) {
      display.print("00");
    } else if (milliseconds < 100) {
      display.print("0");
    }

    display.setTextSize(2);
    display.print(F("\n\n"));
    display.print(F("Lap: "));
    display.print(lapTimer.getBestLapNumber());
  } else {
    display.print(F("\n"));
    display.setTextSize(3);
    display.print("  N/A");
  }
  
  display.display();
}

void displayPage_tachometer() {
  resetDisplay();

  if (tachLastReported > 9999) {
    display.println(F("Engine RPM *OVER REV*"));
  } else {
    display.println(F("     Engine RPM"));
  }

  display.setCursor(5, 20);
  display.setTextSize(4);
  if (tachLastReported < 10000) {
    display.print(F(" "));
  }
  if (tachLastReported < 1000) {
    display.print(F(" "));
  }
  if (tachLastReported < 100) {
    display.print(F(" "));
  }
  if (tachLastReported < 10) {
    display.print(F(" "));
  }
  display.println(tachLastReported);


  display.setTextSize(1);
  display.setCursor(0, 55);
  display.print(F("     max: "));
  display.print(topTachReported);

  display.display();
}

void displayPage_optimal_lap() {
  resetDisplay();

  display.println(F("     Optimal Lap"));
  if (
    lapTimer.getRaceStarted() &&
    lapTimer.getLaps() > 0
  ) {
    const int lineHeight = 15;
    display.setCursor(0, lineHeight);
    display.setTextSize(2);

    unsigned long minutes = lapTimer.getOptimalLapTime() / 60000;
    unsigned long seconds = (lapTimer.getOptimalLapTime() % 60000) / 1000;
    unsigned long milliseconds = lapTimer.getOptimalLapTime() % 1000;

    if (minutes > 0) {
      display.print(minutes);
      display.print(":");
    } else {
      display.print(" ");
    }

    if (seconds < 10) {
      display.print(" ");
    }
    display.print(seconds);
    display.print(".");
    display.print(milliseconds);
    if (milliseconds < 10) {
      display.print("00");
    } else if (milliseconds < 100) {
      display.print("0");
    }

    display.setCursor(0, lineHeight+20);
    display.setTextSize(1);
    display.println(F("     Lap Numbers"));
    display.setCursor(0, lineHeight+35);
    display.setTextSize(2);
    display.print(F("  "));
    display.print(lapTimer.getBestSector1LapNumber());
    display.print(F("  "));
    display.print(lapTimer.getBestSector2LapNumber());
    display.print(F("  "));
    display.print(lapTimer.getBestSector3LapNumber());
  } else {
    display.print(F("\n\n"));
    display.setTextSize(3);
    display.print("  N/A");
  }
  
  display.display();
}

// TODO: this page probably needs some kind of delayed rendering?
const int lapsPerPage = 3;
int current_lap_list_page = 0;
int lap_list_pages = 1;
void displayPage_gps_lap_list() {
  resetDisplay();
  if (recentlyChanged) {
    current_lap_list_page = 0;
  }
  lap_list_pages = ceil((double)lapHistoryCount / (double)lapsPerPage);

  if (lapHistoryCount >= 1) {
    display.print(F("   Lap History   "));
    display.print(current_lap_list_page + 1);
    display.print(F("/"));
    display.print(lap_list_pages);
    display.println(F("\n"));
    display.setTextSize(2);

    int pageStart = current_lap_list_page * lapsPerPage;
    int pageEnd = pageStart + lapsPerPage;
    for (int lap = pageStart; lap < pageEnd; ++lap) {
      if (lap < lapHistoryCount) {
        unsigned long minutes = lapHistory[lap] / 60000;
        unsigned long seconds = (lapHistory[lap] % 60000) / 1000;
        unsigned long milliseconds = lapHistory[lap] % 1000;

        int actualLap = lap + 1;
        if (actualLap < 10) {
          display.print(F(" "));
        }
        display.print(actualLap);
        display.setTextSize(1);
        display.print(F(" "));
        display.setTextSize(2);
        display.print(minutes);
        display.print(F(":"));
        display.print(seconds);
        display.print(F("."));
        display.print(milliseconds);
        display.println();
      }
    }
  } else {
    display.println(F("     Lap History     "));
    display.setTextSize(2);
    display.println();
    display.setTextSize(3);
    display.print(F("  N/A"));
  }
  
  display.display();
}

void displayPage_stop_logging() {
  resetDisplay();

  display.setTextSize(2);
  display.println();
  display.println(F(" END RACE"));
  display.setTextSize(1);
  display.println();
  display.println(F(" press middle button"));
  
  display.display();
}

void displayPage_stop_logging_confirm() {
  resetDisplay();

  display.println(F("Stop Logging?"));
  display.println();
  display.setTextSize(2);

  display.println(F(""));
  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("BACK"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("END RACE"));
  
  display.display();
}

void displayPage_gps_debug() {
  resetDisplay();
  display.println(F("GPS LapTimer Debug"));

  // Safety check for GPS access
  if (!gpsInitialized || gps == NULL) {
    display.println(F("\nGPS not available"));
    display.display();
    return;
  }

  double dist2Line = lapTimer.pointLineSegmentDistance(gps->latitudeDegrees, gps->longitudeDegrees, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  display.print(F("DistToLine: "));
  display.println(dist2Line, 2);

  display.print(F("Laps: "));
  display.print(lapTimer.getLaps());
  display.print(F(" | od:"));
  display.println(lapTimer.getTotalDistanceTraveled());
  display.print(F("Strt: "));
  display.print(lapTimer.getRaceStarted() ? F("T") : F("F"));
  display.print(F(" | Xing: "));
  display.println(lapTimer.getCrossing() ? F("T") : F("F"));

  display.print(F("Current: "));
  display.println(lapTimer.getCurrentLapTime());
  display.print(F("Last   : "));
  display.println(lapTimer.getLastLapTime());
  display.print(F("Best: "));
  display.print(lapTimer.getBestLapNumber());
  display.print(F(": "));
  display.println(lapTimer.getBestLapTime());
  display.print(F("Pace   : "));
  display.println(lapTimer.getPaceDifference());

  display.display();
}

bool notificationFlash = false;
String internalNotification = "N/A";

void displayPage_internal_fault() {
  resetDisplay();
  display.setCursor(0, 0);
  notificationFlash = notificationFlash == true ? false : true;
  display.setTextSize(2);

  if (notificationFlash) {
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
  }
  display.println(F("   FAULT  "));
  display.setTextWrap(true);
  display.setTextColor(DISPLAY_TEXT_WHITE);
  display.setTextSize(1);
  display.println(F(" Please Reboot Device"));
  display.println(F(""));
  // display.println(F("MSG:"));
  display.println(internalNotification);
  display.display();
}
void displayPage_internal_warning() {
  resetDisplay();
  notificationFlash = notificationFlash == true ? false : true;

  display.setTextSize(2);
  if (notificationFlash) {
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
  }
  display.println(F("  WARNING  "));
  display.setTextWrap(true);
  display.setTextColor(DISPLAY_TEXT_WHITE);
  display.setTextSize(1);
  display.println(F("Continue With Caution"));
  display.println(F(""));
  // display.println(F("MSG:"));
  display.println(internalNotification);
  display.display();
}

///////////////////////////////////////////
bool calculatingFlip = false;
void displayCrossing() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  #ifndef ENDURANCE_MODE
    // Draw bitmap on the screen
    calculatingFlip = calculatingFlip == true ? false : true;
    if (calculatingFlip) {
      display.drawBitmap(0, 0, image_data_calculating1, 128, 64, 1);
    } else {
      display.drawBitmap(0, 0, image_data_calculating2, 128, 64, 1);
    }
  #else
  #endif

  display.display();
}
void forceDisplayRefresh() {
  // why does adding work but not subtracting?
  displayLastUpdate += 5000;
}
void switchToDisplayPage(int newDisplayPage) {
  currentPage = newDisplayPage;
  forceDisplayRefresh();
}
///////////////////////////////////////////

void displaySetup() {
  debugln(F("SETTING UP DISPLAY"));
  delay(250); // wait for the OLED to power up
#ifdef USE_1306_DISPLAY
  display.begin(SSD1306_SWITCHCAPVCC, I2C_DISPLAY_ADDRESS);
#else
  display.begin(I2C_DISPLAY_ADDRESS, true);
#endif

  display.setTextColor(DISPLAY_TEXT_WHITE);
  display.setTextWrap(false);

  setupButtons();

  displayLastUpdate = millis();

  currentPage = PAGE_BOOT;

  // silly boot splash, maybe anim?
  resetDisplay();
  display.drawBitmap(0, 0, image_data_bird1, 128, 64, 1);
  display.display();
  delay(750);

  displayPage_boot();
}

void handleMenuPageSelection() {
  if (currentPage == PAGE_MAIN_MENU) {
    if (menuSelectionIndex == 0) {
      // Race selected
      debugln(F("Main Menu: Race selected"));
      switchToDisplayPage(PAGE_SELECT_LOCATION);
    } else if (menuSelectionIndex == 1) {
      // Replay selected
      debugln(F("Main Menu: Replay selected"));
      resetReplayState();
      if (buildReplayFileList()) {
        switchToDisplayPage(PAGE_REPLAY_FILE_SELECT);
      } else {
        internalNotification = "No .dove or .nmea\nfiles found on SD!";
        switchToDisplayPage(PAGE_INTERNAL_WARNING);
      }
    } else {
      // Bluetooth selected
      debugln(F("Main Menu: Bluetooth selected"));
      BLE_SETUP();
      switchToDisplayPage(PAGE_BLUETOOTH);
    }
  } else if (currentPage == PAGE_REPLAY_FILE_SELECT) {
    if (numReplayFiles == 0) {
      // No files - go back
      switchToDisplayPage(PAGE_MAIN_MENU);
    } else {
      selectedReplayFile = menuSelectionIndex;
      debug(F("Replay: Selected file: "));
      debugln(replayFiles[selectedReplayFile]);

      // Show detecting page
      switchToDisplayPage(PAGE_REPLAY_DETECTING);
      forceDisplayRefresh();
      displayPage_replay_detecting();

      // Detect track
      replayDetectedTrackIndex = detectTrackForReplayFile(replayFiles[selectedReplayFile]);

      if (replayDetectedTrackIndex >= 0) {
        // Track found - parse it and go to layout selection
        selectedLocation = replayDetectedTrackIndex;
        char filepath[50];
        makeFullTrackPath(locations[selectedLocation], filepath);
        numOfTracks = 0; // Reset before parsing
        int parseStatus = parseTrackFile(filepath);

        if (parseStatus == PARSE_STATUS_GOOD) {
          switchToDisplayPage(PAGE_REPLAY_SELECT_TRACK);
        } else {
          internalNotification = "Failed to parse\ntrack file!";
          switchToDisplayPage(PAGE_INTERNAL_FAULT);
        }
      } else {
        // No track detected - let user select manually
        internalNotification = "Track not detected!\nSelect manually.";
        // Go to normal track selection
        switchToDisplayPage(PAGE_SELECT_LOCATION);
        replayModeActive = true; // Set flag so we know we're in replay mode
      }
    }
  } else if (currentPage == PAGE_REPLAY_SELECT_TRACK) {
    selectedTrack = menuSelectionIndex;
    switchToDisplayPage(PAGE_REPLAY_SELECT_DIRECTION);
    debug(F("Replay: Selected layout: "));
    debugln(tracks[selectedTrack]);

    // Configure lap timer with selected track
    crossingPointALat = trackLayouts[selectedTrack].start_a_lat;
    crossingPointALng = trackLayouts[selectedTrack].start_a_lng;
    crossingPointBLat = trackLayouts[selectedTrack].start_b_lat;
    crossingPointBLng = trackLayouts[selectedTrack].start_b_lng;

    lapTimer.setStartFinishLine(crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

    if (trackLayouts[selectedTrack].hasSector2) {
      lapTimer.setSector2Line(
        trackLayouts[selectedTrack].sector_2_a_lat,
        trackLayouts[selectedTrack].sector_2_a_lng,
        trackLayouts[selectedTrack].sector_2_b_lat,
        trackLayouts[selectedTrack].sector_2_b_lng
      );
    }

    if (trackLayouts[selectedTrack].hasSector3) {
      lapTimer.setSector3Line(
        trackLayouts[selectedTrack].sector_3_a_lat,
        trackLayouts[selectedTrack].sector_3_a_lng,
        trackLayouts[selectedTrack].sector_3_b_lat,
        trackLayouts[selectedTrack].sector_3_b_lng
      );
    }

    lapTimer.forceLinearInterpolation();
    lapTimer.reset();
  } else if (currentPage == PAGE_REPLAY_SELECT_DIRECTION) {
    selectedDirection = menuSelectionIndex;
    debug(F("Replay: Selected direction: "));
    debugln(selectedDirection == RACE_DIRECTION_FORWARD ? "Forward" : "Reverse");

    // Open the replay file and start processing
    if (replayFile.open(replayFiles[selectedReplayFile], O_READ)) {
      replayModeActive = true;
      replayProcessingComplete = false;
      switchToDisplayPage(PAGE_REPLAY_PROCESSING);
    } else {
      internalNotification = "Failed to open\nreplay file!";
      switchToDisplayPage(PAGE_INTERNAL_FAULT);
    }
  } else if (currentPage == PAGE_REPLAY_EXIT) {
    if (menuSelectionIndex == 0) {
      // Back - return to results
      switchToDisplayPage(PAGE_REPLAY_RESULTS);
    } else {
      // Exit - return to main menu
      resetReplayState();
      switchToDisplayPage(PAGE_MAIN_MENU);
    }
  } else if (currentPage == PAGE_BLUETOOTH) {
    // Exit button pressed - go back to main menu and disable bluetooth
    debugln(F("Bluetooth: Exit selected"));
    BLE_STOP();
    switchToDisplayPage(PAGE_MAIN_MENU);
  } else if (currentPage == PAGE_SELECT_LOCATION) {
    selectedLocation = menuSelectionIndex;

    char filepath[50];  // Must fit "/TRACKS/" + name + ".json" (was 13, caused overflow!)
    makeFullTrackPath(locations[selectedLocation], filepath);
    int parseStatus = parseTrackFile(filepath);

    if (parseStatus == PARSE_STATUS_GOOD) {
      switchToDisplayPage(PAGE_SELECT_TRACK);
    } else if (parseStatus == PARSE_STATUS_LOAD_FAILED) {
      internalNotification = "Could not load file!";
      switchToDisplayPage(PAGE_INTERNAL_FAULT);
    } else if (parseStatus == PARSE_STATUS_PARSE_FAILED) {
      internalNotification = "JSON Parsing Failed!";
      switchToDisplayPage(PAGE_INTERNAL_FAULT);
    }

    debug(F("Selected Location: "));
    debugln(locations[selectedLocation]);
  } else if (currentPage == PAGE_SELECT_TRACK) {
    selectedTrack = menuSelectionIndex;

    // Check if we're in replay mode (manual track selection after auto-detect failed)
    if (replayModeActive) {
      switchToDisplayPage(PAGE_REPLAY_SELECT_DIRECTION);
    } else {
      switchToDisplayPage(PAGE_SELECT_DIRECTION);
    }

    debug(F("Selected Track: "));
    debugln(tracks[selectedTrack]);
    debug(F("start_a_lat: "));
    debugln(trackLayouts[selectedTrack].start_a_lat, 8);
    debug(F("start_a_lng: "));
    debugln(trackLayouts[selectedTrack].start_a_lng, 8);
    debug(F("start_b_lat: "));
    debugln(trackLayouts[selectedTrack].start_b_lat, 8);
    debug(F("start_b_lng: "));
    debugln(trackLayouts[selectedTrack].start_b_lng, 8);

    crossingPointALat = trackLayouts[selectedTrack].start_a_lat;
    crossingPointALng = trackLayouts[selectedTrack].start_a_lng;
    crossingPointBLat = trackLayouts[selectedTrack].start_b_lat;
    crossingPointBLng = trackLayouts[selectedTrack].start_b_lng;

    // initialize laptimer class
    lapTimer.setStartFinishLine(crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);

    // Set sector lines if available
    if (trackLayouts[selectedTrack].hasSector2) {
      lapTimer.setSector2Line(
        trackLayouts[selectedTrack].sector_2_a_lat,
        trackLayouts[selectedTrack].sector_2_a_lng,
        trackLayouts[selectedTrack].sector_2_b_lat,
        trackLayouts[selectedTrack].sector_2_b_lng
      );
      #ifdef HAS_DEBUG
      debugln(F("Sector 2 line configured"));
      #endif
    }

    if (trackLayouts[selectedTrack].hasSector3) {
      lapTimer.setSector3Line(
        trackLayouts[selectedTrack].sector_3_a_lat,
        trackLayouts[selectedTrack].sector_3_a_lng,
        trackLayouts[selectedTrack].sector_3_b_lat,
        trackLayouts[selectedTrack].sector_3_b_lng
      );
      #ifdef HAS_DEBUG
      debugln(F("Sector 3 line configured"));
      #endif
    }

    lapTimer.forceLinearInterpolation();
    lapTimer.reset();
  } else if (currentPage == PAGE_SELECT_DIRECTION) {
    selectedDirection = menuSelectionIndex;
    switchToDisplayPage(GPS_SPEED);
    debug(F("Selected Direction: "));
    debugln(selectedDirection == RACE_DIRECTION_FORWARD ? "Forward" : "Reverse");
    enableLogging = true;
    trackSelected = true;
  } else if (currentPage == LOGGING_STOP_CONFIRM) {
    if (menuSelectionIndex == 0) {
      switchToDisplayPage(GPS_SPEED);
    } else {
      // LOGGING STOP
      switchToDisplayPage(PAGE_SELECT_TRACK);
      enableLogging = false;
      sdDataLogInitComplete = false;
      trackSelected = false;
      dataFile.flush();
      dataFile.close();
      lapTimer.reset();

      // reset lap history
      lapHistoryCount = 0;
      lastLap = 0;
      memset(lapHistory, 0, sizeof(lapHistory));
    }
    debug(F("Stop Logging?: "));
    debugln(menuSelectionIndex == 0 ? "NO" : "YES");
  }
}

void handleRunningPageSelection() {
  if (currentPage == LOGGING_STOP) {
    switchToDisplayPage(LOGGING_STOP_CONFIRM);
  } else if (currentPage == GPS_LAP_LIST) {
    // Middle button cycles lap list pages (both live and replay mode)
    current_lap_list_page = current_lap_list_page == (lap_list_pages-1) ? 0 : current_lap_list_page + 1;
    forceDisplayRefresh();
  } else if (currentPage == PAGE_REPLAY_RESULTS) {
    // Middle button on results does nothing (use left/right for navigation)
    // Could optionally go to lap list here
  } else {
    // wokwi doesnt like fancy page switcher
    // inverted eats too much power
    #ifdef WOKWI
      if(displayInverted == true) {
        displayInverted = false;
        display.invertDisplay(false);
      } else {
        displayInverted = true;
        display.invertDisplay(true);
      }
    #else
      if (gps_speed_mph <= 5.0) {
        currentPage = GPS_LAP_BEST;
      } else {
        currentPage = GPS_LAP_PACE;
      }
      switchToDisplayPage(currentPage);
    #endif
  }
}

void displayLoop() {
  // todo: better page handling
  if (millis() - displayLastUpdate > (1000 / displayUpdateRateHz)) {
    displayLastUpdate = millis();

    #ifdef ENDURANCE_MODE
      bool inEndurance = true;
    #else
      bool inEndurance = false;
    #endif

    if (
      currentPage != GPS_STATS &&
      currentPage != GPS_DEBUG &&
      currentPage != LOGGING_STOP &&
      currentPage != LOGGING_STOP_CONFIRM &&
      currentPage != PAGE_INTERNAL_FAULT &&
      currentPage != PAGE_INTERNAL_WARNING &&
      lapTimer.getCrossing() &&
      inEndurance == false
    ) {
      displayCrossing();
      lastPage = 999;
    } else if (currentPage == PAGE_MAIN_MENU) {
      displayPage_main_menu();
    } else if (currentPage == PAGE_BLUETOOTH) {
      displayPage_bluetooth();
    } else if (currentPage == PAGE_REPLAY_FILE_SELECT) {
      displayPage_replay_file_select();
    } else if (currentPage == PAGE_REPLAY_DETECTING) {
      displayPage_replay_detecting();
    } else if (currentPage == PAGE_REPLAY_SELECT_TRACK) {
      displayPage_replay_select_track();
    } else if (currentPage == PAGE_REPLAY_SELECT_DIRECTION) {
      displayPage_replay_select_direction();
    } else if (currentPage == PAGE_REPLAY_PROCESSING) {
      displayPage_replay_processing();
    } else if (currentPage == PAGE_REPLAY_RESULTS) {
      displayPage_replay_results();
    } else if (currentPage == PAGE_REPLAY_EXIT) {
      displayPage_replay_exit();
    } else if (currentPage == PAGE_SELECT_LOCATION) {
      displayPage_select_location();
    } else if (currentPage == PAGE_SELECT_TRACK) {
      displayPage_select_track();
    } else if (currentPage == PAGE_SELECT_DIRECTION) {
      displayPage_select_direction();
    } else if (currentPage == GPS_STATS) {
      displayPage_gps_stats();
    } else if (currentPage == GPS_SPEED) {
      displayPage_gps_speed();
    } else if (currentPage == TACHOMETER) {
      displayPage_tachometer();
    } else if (currentPage == GPS_LAP_TIME) {
      displayPage_gps_lap_time();
    } else if (currentPage == GPS_LAP_PACE) {
      displayPage_gps_pace();
    } else if (currentPage == GPS_LAP_BEST) {
      #ifndef ENDURANCE_MODE
        displayPage_gps_best_lap();
      #else
        if (gps_speed_mph <= 10.0) {
          displayPage_gps_best_lap();
        } else {
          if (lastPage == (GPS_LAP_BEST - 1)) {
            currentPage = GPS_SPEED;
          } else {
            currentPage = (GPS_LAP_BEST - 1);
          }
          switchToDisplayPage(currentPage);
        }
      #endif
    } else if (currentPage == OPTIMAL_LAP) {
      displayPage_optimal_lap();
    } else if (currentPage == GPS_LAP_LIST) {
      displayPage_gps_lap_list();
    } else if (currentPage == LOGGING_STOP) {
      // dont let us stop logging while were moving, duh
      if (gps_speed_mph <= 2.0) {
        displayPage_stop_logging();
      } else {
        // todo: not super friendly when expanding pages, assuming "logging stop" is always the "last" page
        if (lastPage == (LOGGING_STOP - 1)) {
          currentPage = runningPageStart;
        } else {
          currentPage = LOGGING_STOP - 1;
        }
        switchToDisplayPage(currentPage);
      }
    } else if (currentPage == LOGGING_STOP_CONFIRM) {
      displayPage_stop_logging_confirm();
    } else if (currentPage == GPS_DEBUG) {
      displayPage_gps_debug();
    } else if (currentPage == PAGE_INTERNAL_FAULT) {
      displayPage_internal_fault();
    } else if (currentPage == PAGE_INTERNAL_WARNING) {
      displayPage_internal_warning();
    }
  }

  // todo: better button handling
  bool insideMenu = false;
  bool buttonsDisabled = false;
  int menuLimit = 0;

  if (
    currentPage == PAGE_MAIN_MENU ||
    currentPage == PAGE_BLUETOOTH ||
    currentPage == PAGE_SELECT_LOCATION ||
    currentPage == PAGE_SELECT_TRACK ||
    currentPage == PAGE_SELECT_DIRECTION ||
    currentPage == LOGGING_STOP_CONFIRM ||
    currentPage == PAGE_REPLAY_FILE_SELECT ||
    currentPage == PAGE_REPLAY_SELECT_TRACK ||
    currentPage == PAGE_REPLAY_SELECT_DIRECTION ||
    currentPage == PAGE_REPLAY_EXIT
  ) {
    insideMenu = true;
    if (currentPage == PAGE_MAIN_MENU) {
      menuLimit = 3; // Race, Replay, Download
    } else if (currentPage == PAGE_BLUETOOTH) {
      menuLimit = 1; // Only "Exit" option
    } else if (currentPage == PAGE_SELECT_LOCATION) {
      menuLimit = numOfLocations;
    } else if (currentPage == PAGE_SELECT_TRACK) {
      menuLimit = numOfTracks;
    } else if (
      currentPage == PAGE_SELECT_DIRECTION ||
      currentPage == LOGGING_STOP_CONFIRM ||
      currentPage == PAGE_REPLAY_EXIT
    ) {
      menuLimit = 2;
    } else if (currentPage == PAGE_REPLAY_FILE_SELECT) {
      menuLimit = numReplayFiles > 0 ? numReplayFiles : 1;
    } else if (currentPage == PAGE_REPLAY_SELECT_TRACK) {
      menuLimit = numOfTracks;
    } else if (currentPage == PAGE_REPLAY_SELECT_DIRECTION) {
      menuLimit = 2;
    }
  }

  if (
    currentPage == PAGE_INTERNAL_FAULT ||
    currentPage == PAGE_REPLAY_PROCESSING ||
    currentPage == PAGE_REPLAY_DETECTING
  ) {
    buttonsDisabled = true;
  }
    
  // menu operator
  if (insideMenu && !buttonsDisabled) {
    // we are in a menu do weird menu things
    // Main menu has static display (items top-to-bottom = index 0,1,2)
    // so button direction needs to be reversed compared to scrolling menus
    bool reverseDirection = (currentPage == PAGE_MAIN_MENU);

    // BUTTON UP (or DOWN for reversed menus)
    if (btn1->pressed) {
      // debugln(F("Button Up"));
      if (reverseDirection) {
        // Move UP visually = decrease index
        if (menuSelectionIndex == 0) {
          menuSelectionIndex = menuLimit-1;
        } else {
          menuSelectionIndex--;
        }
      } else {
        if (menuSelectionIndex == menuLimit-1) {
          menuSelectionIndex = 0;
        } else {
          menuSelectionIndex++;
        }
      }
      debug(F("menu number: "));
      debugln(menuSelectionIndex);
      forceDisplayRefresh();
    }
    // BUTTON ENTER
    if (btn2->pressed) {
      debugln(F("Button Enter"));
      handleMenuPageSelection();
    }
    // BUTTON DOWN (or UP for reversed menus)
    if (btn3->pressed) {
      // debugln(F("Button Down"));
      if (reverseDirection) {
        // Move DOWN visually = increase index
        if (menuSelectionIndex == menuLimit-1) {
          menuSelectionIndex = 0;
        } else {
          menuSelectionIndex++;
        }
      } else {
        if (menuSelectionIndex == 0) {
          menuSelectionIndex = menuLimit-1;
        } else {
          menuSelectionIndex--;
        }
      }
      debug(F("menu number: "));
      debugln(menuSelectionIndex);
      forceDisplayRefresh();
    }
  } else if (!buttonsDisabled){
    // page up/down/enter
    // BUTTON LEFT
    if (btn1->pressed) {
      debugln(F("Button Left"));
      // Special handling for replay results - left goes to lap list
      if (currentPage == PAGE_REPLAY_RESULTS) {
        if (lapTimer.getLaps() > 0) {
          current_lap_list_page = 0;
          switchToDisplayPage(GPS_LAP_LIST);
        }
      // Special handling for lap list during replay - left goes back to results
      } else if (currentPage == GPS_LAP_LIST && replayProcessingComplete) {
        switchToDisplayPage(PAGE_REPLAY_RESULTS);
      } else {
        if (currentPage <= runningPageStart) {
          currentPage = runningPageEnd;
        } else {
          currentPage--;
        }
        debug(F("running menu number: "));
        debugln(currentPage);
        switchToDisplayPage(currentPage);
      }
    }
    // BUTTON ENTER
    if (btn2->pressed) {
      debugln(F("Button Middle (running)"));
      handleRunningPageSelection();
    }
    // BUTTON DOWN/RIGHT
    if (btn3->pressed) {
      debugln(F("Button Right"));
      // Special handling for replay results - right goes to exit page
      if (currentPage == PAGE_REPLAY_RESULTS) {
        switchToDisplayPage(PAGE_REPLAY_EXIT);
      // Special handling for lap list during replay - right goes back to results
      } else if (currentPage == GPS_LAP_LIST && replayProcessingComplete) {
        switchToDisplayPage(PAGE_REPLAY_RESULTS);
      } else {
        if (currentPage >= runningPageEnd) {
          currentPage = runningPageStart;
        } else {
          currentPage++;
        }
        debug(F("running menu number: "));
        debugln(currentPage);
        switchToDisplayPage(currentPage);
      }
    }
  }
}

///////////////////////////////////////////

void GPS_LOOP() {
  // Safety check: skip if GPS not initialized (prevents null pointer crash)
  if (!gpsInitialized || gps == NULL) {
    return;
  }

  char c = gps->read();

  if (gps->newNMEAreceived() && gps->parse(gps->lastNMEA())) {
    // char *lastNMEA = gps->lastNMEA();
    // char *nmeaSearch = "$GPRMC";
    // bool isRMCPacket = strstr(lastNMEA, nmeaSearch) != NULL;
    // if (isRMCPacket) {
      gpsFrameCounter++;
    // }

    // update the timer loop everytime we have fixed data
    if (trackSelected && gps->fix) {
      lapTimer.updateCurrentTime(getGpsTimeInMilliseconds());
      lapTimer.loop(gps->latitudeDegrees, gps->longitudeDegrees, gps->altitude, gps->speed);
    }

    #ifdef SD_CARD_LOGGING_ENABLED
      if (trackSelected && gps->fix && sdSetupSuccess && sdDataLogInitComplete && enableLogging) {

        /////////////////////////////////////////////////////////////////
        // Only log GPGGA packets (contains all the data we need, ~25Hz at our GPS settings)
        const char* nmea = gps->lastNMEA();

        // Check if this is a GPGGA or GNGGA packet
        if (strstr(nmea, "$GPGGA") != NULL || strstr(nmea, "$GNGGA") != NULL) {

          // Build CSV line: timestamp,sats,hdop,lat,lng,speed_mph,alt_m,rpm,egt,water_temp,reserved1,reserved2
          char csvLine[256];

          // Convert floats to strings with proper precision
          char latStr[16], lngStr[16], hdopStr[8], speedStr[12], altStr[12];

          // 8 decimals for lat/lng (racing precision)
          dtostrf(gps->latitudeDegrees, 1, 8, latStr);
          dtostrf(gps->longitudeDegrees, 1, 8, lngStr);

          // 1 decimal for HDOP
          dtostrf(gps->HDOP, 1, 1, hdopStr);

          // 2 decimals for speed and altitude
          dtostrf(gps_speed_mph, 1, 2, speedStr);
          dtostrf(gps->altitude, 1, 2, altStr);

          // Build the complete CSV line
          snprintf(csvLine, sizeof(csvLine), "%llu,%d,%s,%s,%s,%s,%s,%d,0,0",
                   getGpsUnixTimestampMillis(), // timestamp (Unix milliseconds)
                   (int)gps->satellites,        // sats
                   hdopStr,                     // hdop
                   latStr,                      // lat
                   lngStr,                      // lng
                   speedStr,                    // speed_mph
                   altStr,                      // alt_m
                   tachLastReported             // rpm
                   // egt, water_temp, reserved1, reserved2 are all 0
          );

          // Write with error checking - EMI can corrupt writes
          size_t written = dataFile.println(csvLine);
          if (written == 0) {
            debugln(F("SD write failed - disabling logging"));
            enableLogging = false;
            sdDataLogInitComplete = false;
            dataFile.close();
            internalNotification = "SD Write Failed!\nCheck card/connections";
            switchToDisplayPage(PAGE_INTERNAL_FAULT);
          }
        }
        /////////////////////////////////////////////////////////////////

        // Flush more frequently to avoid data loss - every 2 seconds
        if (millis() - lastCardFlush > 2000) {
          lastCardFlush = millis();
          dataFile.flush();
        }
      } else if (trackSelected && gps->fix && sdSetupSuccess && !sdDataLogInitComplete && enableLogging && gps->day > 0) {
        debugln(F("Attempt to initialize logfile"));
        
        // all this could be much cleaner.....
        // todo: timezones?
        String gpsYear = "20" + String(gps->year);
        String gpsMonth = gps->month < 10 ? "0" + String(gps->month) : String(gps->month);
        String gpsDay = gps->day < 10 ? "0" + String(gps->day) : String(gps->day);
        String gpsHour = gps->hour < 10 ? "0" + String(gps->hour) : String(gps->hour);
        String gpsMinute = gps->minute < 10 ? "0" + String(gps->minute) : String(gps->minute);

        String trackLocation = locations[selectedLocation];
        String trackLayout= tracks[selectedTrack];
        String layoutDirection = selectedDirection == RACE_DIRECTION_FORWARD ? "fwd" : "rev";

        String dataFileNameS = trackLocation + "_" + trackLayout + "_" + layoutDirection + "_" + gpsYear + "_" + gpsMonth + gpsDay + "_" + gpsHour + gpsMinute + ".dove";

        // const char* dataFileName = dataFileNameS.c_str();

        debug(F("dataFileNameS: ["));
        debug(dataFileNameS.c_str());
        debugln(F("]"));

        // Open for writing - removed O_NONBLOCK as it doesn't work on SD
        dataFile.open(dataFileNameS.c_str(), O_CREAT | O_WRITE);

        if (!dataFile) {
          debugln(F("Error opening log file"));

          // hmmm
          String errorMessage = String("Error saving log:\n") + dataFileNameS;
          internalNotification = errorMessage;
          // internalNotification = errorMessage.c_str();
          // int errorMessageLength = errorMessage.length() + 1;
          // char errorMessageCharArray[errorMessageLength];
          // errorMessage.toCharArray(errorMessageCharArray, errorMessageLength);
          // internalNotification = "Error creating log!";

          switchToDisplayPage(PAGE_INTERNAL_FAULT);

          enableLogging = false;
        } else {
          // Write CSV header as first line
          dataFile.println(F("timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,rpm,exhaust_temp_c,water_temp_c"));
          debugln(F("CSV header written"));
        }
        sdDataLogInitComplete = true;
      }
      if (gps->fix) {
        // debug(lastNMEA);
      }
    #endif
  }

  // update globals for display
  gps_speed_mph = gps->speed * 1.15078;
}

void calculateGPSFrameRate() {
  // calculate actual GPS fix frequency
  gpsFrameEndTime = millis();
  // Check if the update interval has passed
  if (gpsFrameEndTime - gpsFrameStartTime >= 1000) {
    // Calculate the frame rate (loops per second)
    gpsFrameRate = (float)gpsFrameCounter / ((gpsFrameEndTime - gpsFrameStartTime) / 1000.0);
    // Reset the loop counter and start time for the next interval
    gpsFrameCounter = 0;
    gpsFrameStartTime = millis();
  }
}

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
      char filepath[50];
      makeFullTrackPath(locations[i], filepath);
      debugln(filepath);
    }
  }

  // for debuggin error menus
  // sdSetupSuccess = true;
  // sdTrackSuccess = false;

  GPS_SETUP();

  if (!sdSetupSuccess) {
    internalNotification = "SD Init failed!\n\nlogging not possible!";
    switchToDisplayPage(PAGE_INTERNAL_FAULT);
  } else if (sdSetupSuccess && !sdTrackSuccess) {
    // todo: auto logging 
    // internalNotification = "sd:/TRACKS not found!\nOnly Speed Available!\n\n!!AUTO LOGGING DATA!!";
    // runningPageEnd = GPS_SPEED;
    // switchToDisplayPage(PAGE_INTERNAL_WARNING);

    internalNotification = "sd:/TRACKS not found!\n\nlogging not possible!";
    switchToDisplayPage(PAGE_INTERNAL_FAULT);
  } else {
    switchToDisplayPage(PAGE_MAIN_MENU);
  }

  // tachometer
  // pinMode(tachInputPin, INPUT_PULLDOWN);
  // attachInterrupt(digitalPinToInterrupt(tachInputPin), TACH_COUNT_PULSE, RISING);
  pinMode(tachInputPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(tachInputPin), TACH_COUNT_PULSE, FALLING);
}

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

    // Update display periodically during transfer (every 333ms)
    if (millis() - displayLastUpdate > (1000 / displayUpdateRateHz)) {
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