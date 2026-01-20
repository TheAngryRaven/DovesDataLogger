#include <avr/dtostrf.h>
#include <SPI.h>

// #define WOKWI
//#define HAS_DEBUG

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

// BLE state variables
bool bleInitialized = false;
bool bleActive = false;
bool bleConnected = false;
bool bleTransferInProgress = false;
uint32_t bleFileSize = 0;
uint32_t bleBytesTransferred = 0;
uint16_t bleNegotiatedMtu = 23;
File32 bleCurrentFile;

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
// NEW TACHOMETER settings

const int tachInputPin = D0;
unsigned long tachLastUpdate = 0;
volatile int tachLastReported = 0;  // FIXED: Made volatile for thread safety
int topTachReported = 0;

static const uint32_t tachMinPulseGapUs = 3000; // ignore bounces/noise faster than this
volatile uint32_t tachLastPulseUs = 0;

// capture last pulse period so loop can compute RPM
volatile uint32_t tachLastPeriodUs = 0;
volatile bool tachHavePeriod = false;

// Gate to reduce interrupt storm from noisy inductive pickup
// After accepting a pulse, we ignore ALL interrupts for tachMinPulseGapUs
// This prevents the ISR from running hundreds of times per ignition event
volatile bool tachInterruptShouldProcess = true;

// filtered RPM state (updated in loop, not ISR)
float tachRpmFiltered = 0.0f;

// tunable settings
const int tachUpdateRateHz = 3;
static const float tachRevsPerPulse = 1.0f;     // magneto 4T single is usually wasted spark (1 pulse / rev)
static const float tachFilterAlpha = 0.20f;     // 0..1, higher = snappier, lower = smoother
static const uint32_t tachStopTimeoutUs = 500000; // if no pulses for this long, force RPM to 0

// interrupt
void TACH_COUNT_PULSE() {
  // CRITICAL: Exit immediately if we're in the dead-time window
  // This prevents interrupt storm from noisy inductive pickup
  if (!tachInterruptShouldProcess) return;

  uint32_t now = micros();
  uint32_t dt = now - tachLastPulseUs;

  if (dt < tachMinPulseGapUs) return;

  tachLastPulseUs = now;
  tachLastPeriodUs = dt;
  tachHavePeriod = true;

  // Disable interrupt processing until TACH_LOOP re-enables it
  // This blocks the interrupt storm that follows each ignition event
  tachInterruptShouldProcess = false;
  noInterrupts();  // Disable interrupts at hardware level
}

void TACH_LOOP() {
  // Re-enable interrupt processing after the dead-time window
  // Keep interrupts disabled until we're ready for the next pulse
  if (!tachInterruptShouldProcess) {
    uint32_t elapsed = micros() - tachLastPulseUs;

    if (elapsed >= tachMinPulseGapUs) {
      tachInterruptShouldProcess = true;
      interrupts();  // Only re-enable when ready for next pulse
    }
  }

  // Update filtered RPM (highest "real" update rate)
  uint32_t periodUs = 0;
  bool havePeriod = false;

  noInterrupts();
  havePeriod = tachHavePeriod;
  if (havePeriod) {
    periodUs = tachLastPeriodUs;
    tachHavePeriod = false;
  }
  interrupts();

  if (havePeriod && periodUs > 0) {
    float rpmInst = (60.0e6f * tachRevsPerPulse) / (float)periodUs;
    tachRpmFiltered += tachFilterAlpha * (rpmInst - tachRpmFiltered);
  }

  // Timeout to zero if signal disappears
  uint32_t lastPulseUs;
  noInterrupts();
  lastPulseUs = tachLastPulseUs;
  interrupts();

  if ((uint32_t)(micros() - lastPulseUs) > tachStopTimeoutUs) {
    tachRpmFiltered = 0.0f;
  }

  // Snapshot for display/log at any chosen rate (can be high; value stays "latest filtered RPM")
  if (millis() - tachLastUpdate > (1000 / tachUpdateRateHz)) {
    tachLastUpdate = millis();
    tachLastReported = (int)(tachRpmFiltered + 0.5f);
  }
}

///////////////////////////////////////////
// BLUETOOTH FUNCTIONS
///////////////////////////////////////////

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

void bleSetupFileService() {
  fileService.begin();

  // File List Characteristic
  fileListChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  fileListChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  fileListChar.setMaxLen(244);
  fileListChar.begin();

  // File Request Characteristic
  fileRequestChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  fileRequestChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  fileRequestChar.setMaxLen(64);
  fileRequestChar.setWriteCallback(bleFileRequestCallback);
  fileRequestChar.begin();

  // File Data Characteristic
  fileDataChar.setProperties(CHR_PROPS_NOTIFY);
  fileDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  fileDataChar.setMaxLen(244);
  fileDataChar.begin();

  // File Status Characteristic
  fileStatusChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  fileStatusChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  fileStatusChar.setMaxLen(64);
  fileStatusChar.begin();
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

void bleFileRequestCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  char buffer[65];
  memset(buffer, 0, sizeof(buffer));
  memcpy(buffer, data, min(len, (uint16_t)64));

  String command = String(buffer);
  command.trim();

  debug(F("BLE: Received command: ["));
  debug(command);
  debugln(F("]"));

  if (command.startsWith("LIST")) {
    bleSendFileList();
  } else if (command.startsWith("GET:")) {
    String filename = command.substring(4);
    filename.trim();
    bleStartFileTransfer(filename);
  } else if (command.startsWith("DELETE:")) {
    String filename = command.substring(7);
    filename.trim();
    bleDeleteFile(filename);
  }
}

void bleSendFileList() {
  String fileList = "";
  File32 root = SD.open("/");

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

  debug(F("BLE: Sending file list ("));
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

void BLE_SETUP() {
  if (bleInitialized) {
    // Already initialized, just start advertising
    debugln(F("BLE: Restarting advertising..."));
    Bluefruit.Advertising.start(0);
    bleActive = true;
    return;
  }

  debugln(F("BLE: Initializing Bluetooth..."));

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("DovesLapTimer");

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

  // Close any open file
  if (bleCurrentFile) {
    bleCurrentFile.close();
  }
  bleTransferInProgress = false;

  // Disconnect any connected device
  if (Bluefruit.connected()) {
    Bluefruit.disconnect(Bluefruit.connHandle());
    delay(100);
  }

  // Stop advertising
  Bluefruit.Advertising.stop();

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
#include <Adafruit_GPS.h>
#include "gps_config.h"
Adafruit_GPS* gps = NULL;

#define GPS_SERIAL Serial1
#ifndef GPS_CONFIGURATION
  /**
   * @brief Returns the GPS time since midnight in milliseconds
   *
   * @return unsigned long The time since midnight in milliseconds
   */
  unsigned long getGpsTimeInMilliseconds() {
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
   * @return unsigned long Unix timestamp in seconds
   */
  unsigned long getGpsUnixTimestamp() {
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
      } else {
        debugln("No GPS????");
      }
    #else
      debugln(F("WOKWI GPS SETUP"));
      // reconnect at proper baud
      gps = new Adafruit_GPS(&GPS_SERIAL);
      GPS_SERIAL.begin(19200);
      // GPS_SERIAL.begin(14400);
      // GPS_SERIAL.begin(9600);
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

void makeFullTrackPath(char* trackName, char* filepath) {
  char basePath[9] = "/TRACKS/";
  char fileExtension[6] = ".json";

  // Start with the base path
  strcpy(filepath, basePath);

  // Append the selected file name
  strcat(filepath, trackName);

  // Append the file extension
  strcat(filepath, fileExtension);
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

  display.setTextSize(2);
  display.println(F("   Doves\n MagicBox"));
  display.setTextSize(1);
  display.println();
  display.setTextSize(2);

  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println(F("Race"));
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println(F("Bluetooth"));

  display.display();
}

void displayPage_bluetooth() {
  resetDisplay();

  display.setTextSize(1);
  display.println(F(" Bluetooth Download"));
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
  display.setTextSize(2);
  display.println(F("->Exit"));

  display.display();
}

void displayPage_gps_stats() {
  resetDisplay();

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
  // display.println(F("69"));
  if (gps->fix) {
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
    } else {
      // Bluetooth selected
      debugln(F("Main Menu: Bluetooth selected"));
      BLE_SETUP();
      switchToDisplayPage(PAGE_BLUETOOTH);
    }
  } else if (currentPage == PAGE_BLUETOOTH) {
    // Exit button pressed - go back to main menu and disable bluetooth
    debugln(F("Bluetooth: Exit selected"));
    BLE_STOP();
    switchToDisplayPage(PAGE_MAIN_MENU);
  } else if (currentPage == PAGE_SELECT_LOCATION) {
    selectedLocation = menuSelectionIndex;

    char filepath[13];
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
    switchToDisplayPage(PAGE_SELECT_DIRECTION);
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
    current_lap_list_page = current_lap_list_page == (lap_list_pages-1) ? 0 : current_lap_list_page + 1;
    forceDisplayRefresh();
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
    currentPage == LOGGING_STOP_CONFIRM
  ) {
    insideMenu = true;
    if (currentPage == PAGE_MAIN_MENU) {
      menuLimit = 2;
    } else if (currentPage == PAGE_BLUETOOTH) {
      menuLimit = 1; // Only "Exit" option
    } else if (currentPage == PAGE_SELECT_LOCATION) {
      menuLimit = numOfLocations;
    } else if (currentPage == PAGE_SELECT_TRACK) {
      menuLimit = numOfTracks;
    } else if (
      currentPage == PAGE_SELECT_DIRECTION ||
      currentPage == LOGGING_STOP_CONFIRM
    ) {
      menuLimit = 2;
    }
  }

  if (
    currentPage == PAGE_INTERNAL_FAULT
  ) {
    buttonsDisabled = true;
  }
    
  // menu operator
  if (insideMenu && !buttonsDisabled) {
    // we are in a menu do weird menu things
    // BUTTON UP
    if (btn1->pressed) {
      // debugln(F("Button Up"));
      if (menuSelectionIndex == menuLimit-1) {
        menuSelectionIndex = 0;
      } else {
        menuSelectionIndex++;
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
    // BUTTON DOWN
    if (btn3->pressed) {
      // debugln(F("Button Down"));
      if (menuSelectionIndex == 0) {
        menuSelectionIndex = menuLimit-1;
      } else {
        menuSelectionIndex--;
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
      if (currentPage <= runningPageStart) {
        currentPage = runningPageEnd;
      } else {
        currentPage--;
      }      
      debug(F("running menu number: "));
      debugln(currentPage);
      switchToDisplayPage(currentPage);
    }
    // BUTTON ENTER
    if (btn2->pressed) {
      debugln(F("Button Middle (running)"));
      handleRunningPageSelection();
    }
    // BUTTON DOWN
    if (btn3->pressed) {
      debugln(F("Button Right"));
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

///////////////////////////////////////////

void GPS_LOOP() {
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
  GPS_LOOP();
  TACH_LOOP();
  BLUETOOTH_LOOP();

  #ifndef ENDURANCE_MODE
    checkForNewLapData();
  #endif
  calculateGPSFrameRate();

  // Auto-select Race mode if RPM detected while in Bluetooth menu
  if (currentPage == PAGE_BLUETOOTH && tachLastReported > 500) {
    debugln(F("RPM detected in Bluetooth mode - switching to Race"));
    BLE_STOP();
    switchToDisplayPage(PAGE_SELECT_LOCATION);
  }

  readButtons();
  displayLoop();
  resetButtons();

  if (tachLastReported > topTachReported) {
    topTachReported = tachLastReported;
  }
}