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

#define SPI_SPEED SD_SCK_MHZ(25)

#include <SPI.h>
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
  if (!SD.begin(PIN_SPI_CS, SPI_SPEED)) {
    debugln(F("Card initialization failed."));
    return false;
  }
  return true;
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
#define JSON_BUFFER_SIZE 1024
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
#define USE_1306_DISPLAY // remove to use SH110X oled
#endif
#define USE_1306_DISPLAY // remove to use SH110X oled

#include "images.h"
#include "display_config.h"
int displayUpdateRateHz = 3;
unsigned long displayLastUpdate;

const int PAGE_BOOT = 999;
const int PAGE_TEST = 995;
const int PAGE_RC_ERROR = 990;
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
  const int GPS_LAP_TIME = 6;
  const int GPS_LAP_PACE = 7;
  const int GPS_LAP_BEST = 8;
  const int GPS_LAP_LIST = 9;
  const int LOGGING_STOP = 10;
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
  btn1->pin = 3;
  btn2->pin = 2;
  btn3->pin = 1;

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
  display.println(F("  GPS SDCard Logger"));
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
  display.setTextSize(8);
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
  notificationFlash = notificationFlash == true ? false : true;
  display.setTextSize(2);

  if (notificationFlash) {
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
  }
  display.println(F("   FAULT   "));
  display.setTextWrap(true);
  display.setTextColor(DISPLAY_TEXT_WHITE);
  display.setTextSize(1);
  display.println(F("  Must Reboot Device"));
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

  // Draw bitmap on the screen
  calculatingFlip = calculatingFlip == true ? false : true;
  if (calculatingFlip) {
    display.drawBitmap(0, 0, image_data_calculating1, 128, 64, 1);
  } else {
    display.drawBitmap(0, 0, image_data_calculating2, 128, 64, 1);
  }

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
  if (currentPage == PAGE_SELECT_LOCATION) {
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

    if (
      currentPage != GPS_STATS &&
      currentPage != GPS_DEBUG &&
      currentPage != LOGGING_STOP &&
      currentPage != LOGGING_STOP_CONFIRM &&
      currentPage != PAGE_INTERNAL_FAULT &&
      currentPage != PAGE_INTERNAL_WARNING &&
      lapTimer.getCrossing()
    ) {
      displayCrossing();
      lastPage = 999;
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
    currentPage == PAGE_SELECT_LOCATION ||
    currentPage == PAGE_SELECT_TRACK ||
    currentPage == PAGE_SELECT_DIRECTION ||
    currentPage == LOGGING_STOP_CONFIRM
  ) {
    insideMenu = true;
    if (currentPage == PAGE_SELECT_LOCATION) {
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
      // debugln(F("Button Left"));
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
      debugln(F("Button Enter (running)"));
      handleRunningPageSelection();
    }
    // BUTTON DOWN
    if (btn3->pressed) {
      // debugln(F("Button Down"));
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
        dataFile.print(gps->lastNMEA());
        // flush once in a while
        if (millis() - lastCardFlush > 10000) {
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

        String dataFileNameS = trackLocation + "_" + trackLayout + "_" + layoutDirection + "_" + gpsYear + "_" + gpsMonth + gpsDay + "_" + gpsHour + gpsMinute + ".nmea";

        // const char* dataFileName = dataFileNameS.c_str();

        debug(F("dataFileNameS: ["));
        debug(dataFileNameS.c_str());
        debugln(F("]"));

        #ifdef WOKWI
        dataFile.open(dataFileNameS.c_str(), O_CREAT | O_WRITE);
        #else
        dataFile.open(dataFileNameS.c_str(), O_CREAT | O_WRITE | O_NONBLOCK);
        #endif

        if (!dataFile) {
          debugln(F("Error opening log file"));
          
          // hmmm
          String errorMessage = String("Error saving log:\n") + dataFileNameS;
          internalNotification = errorMessage;
          // internalNotification = errorMessage.c_str();
          // int errorMessageLength = errorMessage.length() + 1;
          // char errorMessageCharArray[errorMessageLength];
          // errorMessage.toCharArray(errorMessageCharArray, errorMessageLength);
          // internalNotification = errorMessageCharArray;
          // internalNotification = "Error creating log!";

          switchToDisplayPage(PAGE_INTERNAL_FAULT);

          enableLogging = false;
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
    switchToDisplayPage(PAGE_SELECT_LOCATION);
  }
}

void loop() {
  GPS_LOOP();
  #ifndef ENDURANCE_MODE
    checkForNewLapData();
  #endif
  calculateGPSFrameRate();

  readButtons();
  displayLoop();
  resetButtons();
}