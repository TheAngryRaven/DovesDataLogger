#include <string.h>
struct ButtonState {
  int pin = 0;
  bool pressed = false;
  unsigned long lastPressed = 0;
};

#define HAS_DEBUG
#ifdef HAS_DEBUG
  #define debugln Serial.println
  #define debug Serial.print
#else
  void dummy_debug(...) {
  }
  #define debug dummy_debug
  #define debugln dummy_debug
#endif

/////////////////////////////////////////////////

#include <SPI.h>
#include "SdFat.h"
#include "sdios.h"

SdFat SD;
File file;
File trackDir;
const int chipSelect = 53; // Modify this according to your setup
const char* trackFolder = "/TRACKS";

// could do better
char locations[25][13]; // 13 is old dos format for fat16
int numOfLocations = 0;

void makeFullTrackPath(char* trackName, char* filepath) {
  char* basePath = "/TRACKS/";
  char* fileExtension = ".JSON";

  // Start with the base path
  strcpy(filepath, basePath);

  // Append the selected file name
  strcat(filepath, trackName);

  // Append the file extension
  strcat(filepath, fileExtension);
}

bool SD_SETUP() {
  if (!SD.begin(chipSelect, SD_SCK_MHZ(50))) {
    debugln("Card initialization failed.");
    return false;
  }
  return true;
}

bool buildTrackList() {
  if (!SD.exists(trackFolder)) {
    debugln("TRACKS folder does not exist.");
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

//////////////////////////////////////////////////////////////

ButtonState button1;
ButtonState *btn1 = &button1;
ButtonState button2;
ButtonState *btn2 = &button2;
ButtonState button3;
ButtonState *btn3 = &button3;

float epsilonPrecision = 0.001;

// uses adafruit display libraries
#include <Wire.h>

#define USE_1306_DISPLAY // remove to use SH110X oled
#include "images.h"
#include "display_config.h"
int displayUpdateRateHz = 3;
unsigned long displayLastUpdate;

const int PAGE_BOOT = 999;
const int PAGE_TEST = 995;
const int PAGE_RC_ERROR = 990;
// boot menu
const int PAGE_START = 0;
const int PAGE_SELECT_TRACK = 1;
const int PAGE_SELECT_DIRECTION = 2;
// running menu
const int GPS_STATS = 3;
const int GPS_SPEED = 4;
const int GPS_LAP_TIME = 5;
const int GPS_LAP_PACE = 6;
const int GPS_LAP_BEST = 7;
const int GPS_LAP_LIST = 8;
const int LOGGING_STOP = 9;
const int GPS_DEBUG = 10;
// end menu
const int LOGGING_STOP_CONFIRM = 90;

bool displayInverted = false;
int currentPage = PAGE_BOOT;
int lastPage = 0;

const int runningPageStart = GPS_STATS;
const int runningPageEnd = GPS_DEBUG;//LOGGING_STOP;

int buttonPressIntv = 500;
int buttonHoldIntv = 1000;
int antiBounceIntv = 500;

bool recentlyChanged = false;

void setupButtons() {
  btn1->pin = 4;
  btn2->pin = 5;
  btn3->pin = 6;

  pinMode(btn1->pin, INPUT_PULLUP);
  pinMode(btn2->pin, INPUT_PULLUP);
  pinMode(btn3->pin, INPUT_PULLUP);
}
void readButtons() {
  checkButton(btn1);
  checkButton(btn2);
  checkButton(btn3);
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
  bool btnReady =
    millis() - button->lastPressed >= antiBounceIntv
    // millis() - button->lastReleased >= buttonPressIntv
  ;
  if (btnReady && btnPressed) {
    button->lastPressed = millis();
    button->pressed = true;
  }
}

//////////////
// TODO: mode display into own class??
int menuSelectionIndex = 0;
void resetDisplay() {
  if (currentPage != lastPage) {
    lastPage = currentPage;
    recentlyChanged = true;
    // for partial refreshes
    // display.clearDisplay();
    menuSelectionIndex = 0;
  } else {
    recentlyChanged = false;
  }
  // for full refreshes
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.setTextColor(DISPLAY_TEXT_WHITE);
}

void displayPage_boot() {
  resetDisplay();
  
  display.setTextSize(2);
  display.println("   Doves\n MagicBox");
  display.setTextSize(1);
  display.println("");
  display.println("  GPS SDCard Logger");
  display.println("\n    Initializing...");

  // resetDisplay();
  // display.drawBitmap(0, 0, image_data_bird1, 128, 64, 1);
  display.display();
}

/////////////////////////
// TESTING SHIT

int selectedLocation = -1;
// // TODO: READ FROM SDCARD
// const int numOfLocations = 5;
// char* locations[numOfLocations] = {
//   "OKC",
//   "Piquet",
//   "Jacksonville",
//   "Miami",
//   "Bushnell",
// };

const int numOfTracks = 5;
int selectedTrack = -1;
char* tracks[numOfTracks] = {
  "Normal",
  "Short",
  "Super Short",
  "Long",
  "Extra Long"
};

int selectedDirection = -1;

void displayPage_select_location() {
  resetDisplay();
  if (recentlyChanged && selectedLocation >= 0) {
    menuSelectionIndex = selectedLocation;
  }

  display.print("Select Track: ");
  display.print(menuSelectionIndex+1);
  display.print("/");
  display.println(numOfLocations);
  display.println();
  display.setTextSize(2);

  display.print("  ");
  display.println(locations[menuSelectionIndex == numOfLocations - 1 ? 0 : menuSelectionIndex + 1]);
  display.print("->");
  display.println(locations[menuSelectionIndex]);
  display.print("  ");
  display.println(locations[menuSelectionIndex == 0 ? numOfLocations - 1 : menuSelectionIndex - 1]);  
  
  display.display();
}

void displayPage_select_track() {
  resetDisplay();
  if (recentlyChanged && selectedTrack >= 0) {
    menuSelectionIndex = selectedTrack;
  }
  display.print("Select Layout: ");
  display.print(menuSelectionIndex+1);
  display.print("/");
  display.println(numOfTracks);
  display.println(locations[selectedLocation]);
  display.setTextSize(2);

  display.print("  ");
  display.println(tracks[menuSelectionIndex == numOfTracks - 1 ? 0 : menuSelectionIndex + 1]);
  display.print("->");
  display.println(tracks[menuSelectionIndex]);
  display.print("  ");
  display.println(tracks[menuSelectionIndex == 0 ? numOfTracks - 1 : menuSelectionIndex - 1]);
  
  display.display();
}

void displayPage_select_direction() {
  resetDisplay();
  if (recentlyChanged && selectedDirection >= 0) {
    menuSelectionIndex = selectedDirection;
  }

  display.print("Select Direction");
  display.println();
  display.setTextSize(2);

  display.println("");
  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println("Forward");
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println("Reverse");
  
  display.display();
}

void displayPage_gps_stats() {
  resetDisplay();

  // display.println("   Doves Magic Box\n");
  display.println("Battery  : [#####]");
  display.println("Sats/Rate: 10  24.7hz");
  display.println("HDOP     : 0.75");
  display.println("SDCard   : Logging...");

  display.println();
  display.println();

  display.print("Track : ");
  display.println(locations[selectedLocation]);
  display.print("Layout: ");
  display.print(selectedDirection == 0 ? "->" : "<-");
  display.print(" ");
  display.println(tracks[selectedTrack]);  
  
  display.display();
}

void displayPage_gps_speed() {
  resetDisplay();

  display.println("SPEED");
  display.println("\nLAP");
  display.setTextSize(3);
  display.print("4");
  display.setCursor(40, 5);
  display.setTextSize(8);
  display.println("69");
  
  display.display();
}

void displayPage_gps_lap_time() {
  resetDisplay();

  display.println("Current Lap Time");
  // display.println();

  /////////////////
  const int lineHeight = 21;
  //tmp obviously
  int leftMargin = 0;//29;//0;//29;

  if (leftMargin == 29) {
    display.setCursor(0, lineHeight);
    display.setTextSize(4);
    display.print("5");
    display.setCursor(17, lineHeight + 4);
    display.setTextSize(3);
    display.print(":");
  }

  display.setCursor(leftMargin, lineHeight);
  display.setTextSize(4);
  display.print("54");
  display.setCursor(leftMargin + 42, lineHeight + 7);
  // display.setCursor(leftMargin + 42 + 5, lineHeight + 7);
  display.setTextSize(3);
  display.print(".");
  display.setCursor(leftMargin + 55, lineHeight);
  // display.setCursor(leftMargin + 55 + 5, lineHeight);
  display.setTextSize(4);
  display.print("173");

  
  display.display();
}

bool paceFlashStatus = false;
void displayPage_gps_pace() {
  resetDisplay();

  display.println("Current Lap Pace");
  // display.println();
  // display.println();
  // display.setTextSize(4);
  // display.println("+1.308");

  // animation
  if (paceFlashStatus) {
    paceFlashStatus = false;
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
    display.print("           ");
    display.setTextColor(DISPLAY_TEXT_WHITE);
    display.println("           ");
  } else {
    paceFlashStatus = true;
    display.setTextColor(DISPLAY_TEXT_WHITE);
    display.print("           ");
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
    display.println("           ");
  }
  // display.println("                      ");

  // main page into
  display.setTextColor(DISPLAY_TEXT_WHITE);
  const int lineHeight = 21;
  display.setCursor(0, lineHeight);
  display.setTextSize(4);
  display.print("- 1");
  display.setCursor(63, lineHeight + 7);
  display.setTextSize(3);
  display.print(".");
  display.setCursor(80, lineHeight);
  display.setTextSize(4);
  display.print("27");

  // animation
  display.println();
  display.setTextSize(1);
  
  if (paceFlashStatus) {
    // paceFlashStatus = false;
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
    display.print("           ");
    display.setTextColor(DISPLAY_TEXT_WHITE);
    display.println("           ");
  } else {
    // paceFlashStatus = true;
    display.setTextColor(DISPLAY_TEXT_WHITE);
    display.print("           ");
    display.setTextColor(DISPLAY_TEXT_BLACK, DISPLAY_TEXT_WHITE);
    display.println("           ");
  }
  // display.println("   !!! GO GO GO !!!   ");
  // display.println("                      ");
  
  
  display.display();
}

void displayPage_gps_best_lap() {
  resetDisplay();

  display.println("Best Lap");
  display.println();
  // display.println();
  display.setTextSize(3);
  display.println("0:54:275");
  display.setTextSize(1);
  display.println();
  display.println("Lap: 4 : 0:54:275");
  
  display.display();
}

int current_lap_list_page = 0;
int lap_list_pages = 2;
void displayPage_gps_lap_list() {
  resetDisplay();
  if (recentlyChanged) {
    current_lap_list_page = 0;
  }

  if (current_lap_list_page == 0) {
    display.println(" 1 0:54:27  9 0:54:27");
    display.println(" 2 0:54:27 10 0:54:27");
    display.println(" 3 0:54:27 11 0:54:27");
    display.println(" 4 0:54:27<12 0:54:27");
    display.println(" 5 0:54:27 13 0:54:27");
    display.println(" 6 0:54:27 14 0:54:27");
    display.println(" 7 0:54:27 15 0:54:27");
    display.println(" 8 0:54:27 16 0:54:27");
  } else if (current_lap_list_page == 1) {
    display.println("17 0:54:27 25 0:54:27");
    display.println("18 0:54:27 26 0:54:27");
    display.println("19 0:54:27 27 0:54:27");
    display.println("20 0:54:27 28 0:54:27");
    display.println("21 0:54:27");
    display.println("21 0:54:27");
    display.println("23 0:54:27");
    display.println("24 0:54:27");
  }
  
  display.display();
}

void displayPage_stop_logging() {
  resetDisplay();

  display.setTextSize(2);
  display.println();
  display.println(" END RACE");
  display.setTextSize(1);
  display.println();
  display.println(" press middle button");
  
  display.display();
}

void displayPage_stop_logging_confirm() {
  resetDisplay();

  display.println("Stop Logging?");
  display.println();
  display.setTextSize(2);

  display.println("");
  display.print(menuSelectionIndex == 0 ? "->" : "  ");
  display.println("BACK");
  display.print(menuSelectionIndex == 1 ? "->" : "  ");
  display.println("END RACE");
  
  display.display();
}

void displayPage_gps_debug() {
  resetDisplay();
  display.println("GPS LapTimer Debug");
  
  // double dist2Line = lapTimer.pointLineSegmentDistance(gps->latitudeDegrees, gps->longitudeDegrees, crossingPointALat, crossingPointALng, crossingPointBLat, crossingPointBLng);
  double dist2Line = 420.69;
  display.print("DistToLine: ");
  display.println(dist2Line, 2);

  display.print("Laps: ");
  display.print(5);
  display.print(" | od:");
  display.println(69.2);
  display.print("Strt: ");
  display.print(true ? "T" : "F");
  display.print(" | Xing: ");
  display.println(false ? "T" : "F");

  display.print("Current: ");
  display.println(42069);
  display.print("Last   : ");
  display.println(42069);
  display.print("Best: ");
  display.print(4);
  display.print(": ");
  display.println(42069);
  display.print("Pace   : ");
  display.println(278);

  display.display();
}

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

void displaySetup() {
  debugln("SETTING UP DISPLAY");
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
  //

  displayPage_boot();
}

void displayLoop() {
  // todo: better page handling
  if (millis() - displayLastUpdate > (1000 / displayUpdateRateHz)) {
    displayLastUpdate = millis();
    if (currentPage == PAGE_START) {
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
      displayPage_gps_best_lap();
    } else if (currentPage == GPS_LAP_LIST) {
      displayPage_gps_lap_list();
    } else if (currentPage == LOGGING_STOP) {
      displayPage_stop_logging();
    } else if (currentPage == LOGGING_STOP_CONFIRM) {
      displayPage_stop_logging_confirm();
    } else if (currentPage == GPS_DEBUG) {
      displayPage_gps_debug();
      // displayCrossing();
    }
  }

  // how2button
  bool insideMenu = false;
  int menuLimit = 0;
  if (
    currentPage == PAGE_START ||
    currentPage == PAGE_SELECT_TRACK ||
    currentPage == PAGE_SELECT_DIRECTION ||
    currentPage == LOGGING_STOP_CONFIRM
  ) {
    insideMenu = true;
    if (currentPage == PAGE_START) {
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
    
  // menu operator
  if (insideMenu) {
    // we are in a menu do weird menu things
    // BUTTON UP
    if (btn1->pressed) {
      // debugln("Button Up");
      if (menuSelectionIndex == menuLimit-1) {
        menuSelectionIndex = 0;
      } else {
        menuSelectionIndex++;
      }
      debug("menu number: ");
      debugln(menuSelectionIndex);
    }
    // BUTTON ENTER
    if (btn2->pressed) {
      debugln("Button Enter");
      if (currentPage == PAGE_START) {
        selectedLocation = menuSelectionIndex;
        currentPage = PAGE_SELECT_TRACK;
        displayLastUpdate += 5000;
        debug("Selected Location: ");
        debugln(locations[selectedLocation]);
      } else if (currentPage == PAGE_SELECT_TRACK) {
        selectedTrack = menuSelectionIndex;
        currentPage = PAGE_SELECT_DIRECTION;
        displayLastUpdate += 5000;
        debug("Selected Track: ");
        debugln(tracks[selectedTrack]);
      } else if (currentPage == PAGE_SELECT_DIRECTION) {
        selectedDirection = menuSelectionIndex;
        currentPage = GPS_SPEED;
        displayLastUpdate += 5000;
        debug("Selected Direction: ");
        debugln(selectedDirection == 0 ? "Forward" : "Reverse");
      } else if (currentPage == LOGGING_STOP_CONFIRM) {
        selectedDirection = menuSelectionIndex;
        if (menuSelectionIndex == 0) {
          currentPage = GPS_SPEED;
        } else {
          // currentPage = PAGE_START;
          currentPage = PAGE_SELECT_TRACK;
        }
        displayLastUpdate += 5000;
        debug("Stop Logging?: ");
        debugln(selectedDirection == 0 ? "NO" : "YES");
      }
    }
    // BUTTON DOWN
    if (btn3->pressed) {
      // debugln("Button Down");
      if (menuSelectionIndex == 0) {
        menuSelectionIndex = menuLimit-1;
      } else {
        menuSelectionIndex--;
      }
      debug("menu number: ");
      debugln(menuSelectionIndex);
    }
  } else {
    // page up/down/enter
    // BUTTON UP

    if (btn1->pressed) {
      // debugln("Button Up");
      if (currentPage == runningPageEnd) {
        currentPage = runningPageStart;
      } else {
        currentPage++;
      }
      debug("running menu number: ");
      debugln(currentPage);
    }
    // BUTTON ENTER
    if (btn2->pressed) {
      debugln("Button Enter (running)");
      if (currentPage == LOGGING_STOP) {
        currentPage = LOGGING_STOP_CONFIRM;
        displayLastUpdate += 5000;
      } else if (currentPage == GPS_LAP_LIST) {
        current_lap_list_page = current_lap_list_page == (lap_list_pages-1) ? 0 : current_lap_list_page+1;
      } else {
        if(displayInverted == true) {
          displayInverted = false;
          display.invertDisplay(false);
        } else {
          displayInverted = true;
          display.invertDisplay(true);
        }
      }
    }
    // BUTTON DOWN
    if (btn3->pressed) {
      // debugln("Button Down");
      if (currentPage == runningPageStart) {
        currentPage = runningPageEnd;
      } else {
        currentPage--;
      }
      debug("running menu number: ");
      debugln(currentPage);
    }
  }
}
//////////////

void setup() {
  randomSeed(analogRead(0));
  #ifdef HAS_DEBUG
      Serial.begin(9600);
      while (!Serial);
  #endif

  //setupSD();

  bool sdSetupSuccess = SD_SETUP();
  if(sdSetupSuccess && buildTrackList()) {
    debugln("Obtained Track List");
    for (int i = 0; i < numOfLocations; i++) {
      char filepath[50];
      makeFullTrackPath(locations[i], filepath);
      debugln(filepath);
    }
  }

  displaySetup();

  delay(2000);
  //setupGPS();

  currentPage = PAGE_START;
  displayLastUpdate += 5000;
}

void loop() {
  readButtons();
  //force update when button pressed
  if (
    btn1->pressed ||
    btn2->pressed ||
    btn3->pressed
  ) {
    displayLastUpdate += 5000;
  }
  displayLoop();
  resetButtons();
}