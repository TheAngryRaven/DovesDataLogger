///////////////////////////////////////////
// SD CARD MODULE
// SD card setup, access management, track list building, and JSON track parsing
///////////////////////////////////////////

#include "sd_functions.h"

/**
 * @brief Attempt to acquire SD card access for a subsystem
 * @param mode The access mode being requested (SD_ACCESS_*)
 * @return true if access granted, false if SD busy with another operation
 *
 * The check-then-set must be atomic: the main loop and the Bluefruit
 * callback task can both call this, and a plain `volatile int` test+store
 * is a TOCTOU that lets two tasks each believe they own the card.
 * taskENTER_CRITICAL() masks interrupts only up to
 * configMAX_SYSCALL_INTERRUPT_PRIORITY (BASEPRI), so the SoftDevice's
 * high-priority radio interrupts are never starved. The grant/deny rules
 * live in the host-tested sd_access_policy pure unit.
 */
bool acquireSDAccess(int mode) {
  bool granted;
  taskENTER_CRITICAL();
  granted = sd_access_policy::canAcquire(currentSDAccess, mode);
  if (granted) {
    currentSDAccess = mode;
  }
  taskEXIT_CRITICAL();

  if (!granted) {
    // SD busy with another subsystem
    debug(F("SD access denied, current mode: "));
    debugln(currentSDAccess);
  }
  return granted;
}

/**
 * @brief Release SD card access
 * @param mode The access mode being released (must match current)
 */
void releaseSDAccess(int mode) {
  taskENTER_CRITICAL();
  if (sd_access_policy::releaseClears(currentSDAccess, mode)) {
    currentSDAccess = SD_ACCESS_NONE;
  }
  taskEXIT_CRITICAL();
}

/**
 * @brief Force release all SD access (use during cleanup/error recovery)
 */
void forceReleaseSDAccess() {
  taskENTER_CRITICAL();
  currentSDAccess = SD_ACCESS_NONE;
  taskEXIT_CRITICAL();
}

void makeFullTrackPath(const char* trackName, char* filepath) {
  // Use snprintf for bounds safety - prevents buffer overflow
  // Caller MUST provide buffer of at least FILEPATH_MAX bytes
  snprintf(filepath, FILEPATH_MAX, "/TRACKS/%s.json", trackName);
}

// (Re)initialize the SD card at a given SPI clock. Re-calling SD.begin() is
// the supported way to change the SdFat SPI speed at runtime — it re-inits the
// card and remounts the volume. Retried because EMI from ignition can cause
// init failures. Returns true on success.
bool sdSetSpiClock(uint32_t maxSck) {
  for (int attempt = 0; attempt < 3; attempt++) {
    if (SD.begin(PIN_SPI_CS, maxSck)) {
      return true;
    }
    debugln(F("SD init attempt failed, retrying..."));
    delay(100);  // Brief delay between attempts
  }
  return false;
}

// Switch the SD SPI clock between the parked-transfer fast clock and the
// EMI-safe normal clock. Only safe to call when no SD file is open (the
// BLE/USB transfer entry/exit points). Falls back to the normal clock if the
// fast re-init fails, so a flaky fast clock can never leave the card unusable.
void sdSetTransferSpeed(bool fast) {
  uint32_t target = fast ? (uint32_t)SD_SPI_SPEED_FAST : (uint32_t)SPI_SPEED;
  if (sdSetSpiClock(target)) {
    return;
  }
  if (fast) {
    debugln(F("SD: fast clock re-init failed, falling back to normal speed"));
    sdSetSpiClock(SPI_SPEED);
  }
}

bool SD_SETUP() {
  sdCardUnformatted = false;
  if (sdSetSpiClock(SPI_SPEED)) {
    debugln(F("SD Card initialized successfully"));
    return true;
  }
  debugln(F("Card initialization failed after 3 attempts."));
  // SD.begin() = cardBegin() && volumeBegin(). Distinguish "card answers
  // but no FAT volume mounts" (factory-blank or corrupted soldered-in
  // module — recoverable via the on-device format page) from a dead or
  // absent card (keeps the existing FAULT dead-end).
  if (SD.cardBegin(SdSpiConfig(PIN_SPI_CS, SHARED_SPI, SPI_SPEED)) &&
      SD.card() && SD.card()->sectorCount() > 0) {
    sdCardUnformatted = true;
    debugln(F("SD card responds but has no FAT volume — format candidate"));
  }
  return false;
}

///////////////////////////////////////////
// ON-DEVICE FORMAT (blank/corrupt soldered-in module)
///////////////////////////////////////////

// FatFormatter reports progress through a Print*: a few text messages plus
// one '.' per fatSize/32 FAT sectors written. Each write pets the ~4 s WDT
// (armed since the end of setup(); the format blocks the main loop) and
// repaints a throttled progress screen. 4-64 GB cards emit a dot every
// ~240 sectors — comfortably inside the WDT budget even at 2 MHz SPI.
class SdFormatProgress : public Print {
 public:
  size_t write(uint8_t) override {
    tick();
    return 1;
  }
  size_t write(const uint8_t*, size_t n) override {
    tick();
    return n;
  }

 private:
  void tick() {
    wdtPet();
    if (millis() - lastPaintMs >= 250) {
      lastPaintMs = millis();
      displayPage_sd_format();
    }
  }
  unsigned long lastPaintMs = 0;
};

void sdPerformFormat() {
  // Nothing else can hold the card here (no FAT ever mounted this boot),
  // but hold the mutex anyway so no other subsystem can sneak in mid-erase.
  if (!acquireSDAccess(SD_ACCESS_FORMAT)) {
    return;  // impossible in practice; stay on the confirm page
  }

  sdFormatPhase = SD_FORMAT_RUNNING;
  displayPage_sd_format();
  wdtPet();

  SdFormatProgress progress;
  bool formatted = false;
  // The device is parked at boot (no session, motor normally off), so use
  // the fast transfer clock first; retry the whole format once at the
  // EMI-safe clock — a tach-wake boot means the engine may be running.
  if (SD.cardBegin(SdSpiConfig(PIN_SPI_CS, SHARED_SPI, SD_SPI_SPEED_FAST))) {
    formatted = SD.format(&progress);
  }
  if (!formatted) {
    wdtPet();
    debugln(F("SD format at fast clock failed, retrying at normal speed"));
    if (SD.cardBegin(SdSpiConfig(PIN_SPI_CS, SHARED_SPI, SPI_SPEED))) {
      formatted = SD.format(&progress);
    }
  }

  if (!formatted) {
    releaseSDAccess(SD_ACCESS_FORMAT);
    sdFormatPhase = SD_FORMAT_CONFIRM;
    debugln(F("SD format failed"));
    strncpy(internalNotification, "SD format failed!\n\ncard may be dead", sizeof(internalNotification) - 1);
    internalNotification[sizeof(internalNotification) - 1] = '\0';
    switchToDisplayPage(PAGE_INTERNAL_FAULT);
    return;
  }

  // Mount the fresh volume and provision /TRACKS now — even though the
  // fresh boot's buildTrackList() would also create it — so an interrupted
  // reboot still leaves a usable card.
  wdtPet();
  if (SD.begin(PIN_SPI_CS, SPI_SPEED)) {
    SD.mkdir(trackFolder);
  }

  debugln(F("SD format complete, rebooting"));
  sdFormatPhase = SD_FORMAT_DONE;
  displayPage_sd_format();
  // Let the "FORMAT OK" frame be seen; WDT stays fed. The reboot re-runs
  // SD_SETUP() (mounts clean) and SETTINGS_SETUP() (creates defaults) —
  // mirrors the BLE-disconnect / USB-MSC-exit reset precedent.
  for (int i = 0; i < 3; i++) {
    delay(500);
    wdtPet();
  }
  NVIC_SystemReset();
}

// Static buffers for JSON parsing — saves ~4KB of stack per call.
// Only one parseTrackFile() call can be active at a time (single-threaded).
// Also reused by buildTrackList() for manifest extraction.
static char jsonFileBuffer[JSON_BUFFER_SIZE];
static StaticJsonDocument<JSON_BUFFER_SIZE> trackJson;

bool buildTrackList() {
  // Take the SD mutex for the whole directory walk. Every other SD consumer
  // arbitrates through it; without this, a BLE track upload/delete completing
  // while logging is being torn down could hit SdFat from two tasks at once.
  // Both BLE callers (processTrackUpload/processTrackDelete) and setup()
  // release/hold no lock before calling, so this can't self-deadlock.
  if (!acquireSDAccess(SD_ACCESS_TRACK_PARSE)) {
    debugln(F("buildTrackList: SD busy, skipping rebuild."));
    return false;
  }

  if (!SD.exists(trackFolder)) {
    // Soldered-in module: first boot is a blank card, and SdFat's open()
    // never creates parent directories — without this, every BLE TPUT
    // would fail until the folder exists. Self-provision it.
    if (!SD.mkdir(trackFolder)) {
      debugln(F("TRACKS folder missing and could not be created."));
      releaseSDAccess(SD_ACCESS_TRACK_PARSE);
      return false;
    }
    debugln(F("TRACKS folder created (blank card)."));
  }

  // reset lists
  numOfLocations = 0;
  trackManifestCount = 0;

  // If the TRACKS directory exists, open it
  if (!trackDir.open(trackFolder)) {
    debugln(F("Failed to open TRACKS folder."));
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return false;
  }

  // Reset the file to the first position in the directory
  trackDir.rewind();

  // Loop through each file in the directory
  while (file.openNext(&trackDir, O_READ)) {
    if (numOfLocations >= MAX_LOCATIONS) {
      file.close();
      break;
    }

    // Create a buffer to store the filename
    char filename[25];

    // Get the filename
    file.getName(filename, sizeof(filename));

    // Find the last dot in the filename
    char *dot = strrchr(filename, '.');

    // If a dot was found, replace it with a null character to end the string there
    if(dot) *dot = '\0';

    // Add the file to the array (use destination size, not source size!)
    strncpy(locations[numOfLocations], filename, MAX_LOCATION_LENGTH - 1);
    locations[numOfLocations][MAX_LOCATION_LENGTH - 1] = '\0';  // Ensure null-termination

    // Build track manifest entry — extract first lat/lon from JSON
    // Reuses the static JSON buffer (safe: single-threaded, one file at a time)
    if (trackManifestCount < MAX_LOCATIONS) {
      int bytesRead = file.read(jsonFileBuffer, sizeof(jsonFileBuffer) - 1);
      if (bytesRead > 0) {
        jsonFileBuffer[bytesRead] = '\0';
        trackJson.clear();
        DeserializationError err = deserializeJson(trackJson, jsonFileBuffer);
        if (err == DeserializationError::Ok) {
          double firstLat = 0, firstLon = 0;
          if (trackJson.is<JsonObject>()) {
            // New format: object with "courses" array
            JsonArray courses = trackJson["courses"];
            if (courses.size() > 0) {
              firstLat = courses[0]["start_a_lat"];
              firstLon = courses[0]["start_a_lng"];
            }
          } else if (trackJson.is<JsonArray>()) {
            // Legacy format: bare array of courses
            JsonArray arr = trackJson.as<JsonArray>();
            if (arr.size() > 0) {
              firstLat = arr[0]["start_a_lat"];
              firstLon = arr[0]["start_a_lng"];
            }
          }
          if (firstLat != 0 || firstLon != 0) {
            strncpy(trackManifest[trackManifestCount].filename, filename, sizeof(trackManifest[0].filename) - 1);
            trackManifest[trackManifestCount].filename[sizeof(trackManifest[0].filename) - 1] = '\0';
            trackManifest[trackManifestCount].lat = firstLat;
            trackManifest[trackManifestCount].lon = firstLon;
            trackManifestCount++;
          }
        }
      }
    }

    // Increment the numOfLocations
    numOfLocations++;

    // Close the file to free up any memory it's using
    file.close();
  }

  // Close the directory to free up any memory it's using
  trackDir.close();

  debug(F("Tracks found: "));
  debugln(numOfLocations);
  debug(F("Manifest entries: "));
  debugln(trackManifestCount);

  releaseSDAccess(SD_ACCESS_TRACK_PARSE);
  return true;
}

int parseTrackFile(char* filepath) {
  debug(F("ParseTrackFile:"));
  debugln(filepath);

  // Reset track count so we don't accumulate stale entries from prior calls
  numOfTracks = 0;

  // double check the SD is active
  if (!sdSetupSuccess) {
    debugln(F("ParseTrackFile: failed to initialize SD card"));
    return PARSE_STATUS_LOAD_FAILED;
  }

  // Acquire SD access for track parsing
  if (!acquireSDAccess(SD_ACCESS_TRACK_PARSE)) {
    debugln(F("ParseTrackFile: SD busy"));
    return PARSE_STATUS_LOAD_FAILED;
  }

  // load file
  trackFile.open(filepath, O_READ);
  if (!trackFile) {
    debugln(F("ParseTrackFile: failed to LOAD file"));
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return PARSE_STATUS_LOAD_FAILED;
  }

  // Read file into static buffer (not stack-allocated)
  int bytesRead = trackFile.read(jsonFileBuffer, sizeof(jsonFileBuffer));

  // Check if read was successful
  if (bytesRead == -1) {
    debugln(F("ParseTrackFile: failed to READ file"));
    trackFile.close();
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return PARSE_STATUS_LOAD_FAILED;
  }

  // Null-terminate the buffer
  if (bytesRead < (int)sizeof(jsonFileBuffer)) {
    jsonFileBuffer[bytesRead] = '\0';
  } else {
    jsonFileBuffer[sizeof(jsonFileBuffer) - 1] = '\0';
  }

  // Parse JSON (using file-scope static document to save stack)
  trackJson.clear();
  DeserializationError error = deserializeJson(trackJson, jsonFileBuffer);
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
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return PARSE_STATUS_PARSE_FAILED;
  }

  // Detect JSON root type: object (new format) or array (legacy format)
  JsonArray coursesArray;
  if (trackJson.is<JsonObject>()) {
    // New format: { "longName": "...", "shortName": "...", "courses": [...] }
    debugln(F("ParseTrackFile: New object format detected"));

    const char* longName = trackJson["longName"] | "";
    const char* shortName = trackJson["shortName"] | "";
    const char* defaultCourse = trackJson["defaultCourse"] | "";

    strncpy(activeTrackMetadata.longName, longName, sizeof(activeTrackMetadata.longName) - 1);
    activeTrackMetadata.longName[sizeof(activeTrackMetadata.longName) - 1] = '\0';
    strncpy(activeTrackMetadata.shortName, shortName, sizeof(activeTrackMetadata.shortName) - 1);
    activeTrackMetadata.shortName[sizeof(activeTrackMetadata.shortName) - 1] = '\0';
    strncpy(activeTrackMetadata.defaultCourse, defaultCourse, sizeof(activeTrackMetadata.defaultCourse) - 1);
    activeTrackMetadata.defaultCourse[sizeof(activeTrackMetadata.defaultCourse) - 1] = '\0';

    coursesArray = trackJson["courses"];

    debug(F("  longName: "));
    debugln(longName);
    debug(F("  shortName: "));
    debugln(shortName);
  } else if (trackJson.is<JsonArray>()) {
    // Legacy format: bare array of course objects
    debugln(F("ParseTrackFile: Legacy array format detected"));
    coursesArray = trackJson.as<JsonArray>();

    // Derive metadata from filename — leave blank, caller knows the filename
    activeTrackMetadata.longName[0] = '\0';
    activeTrackMetadata.shortName[0] = '\0';
    activeTrackMetadata.defaultCourse[0] = '\0';
  } else {
    debugln(F("ParseTrackFile: Unknown JSON format"));
    trackFile.close();
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return PARSE_STATUS_PARSE_FAILED;
  }

  // Parse courses array (common to both formats)
  debugln(F("Generating Layout List..."));
  for(JsonVariant layout : coursesArray) {
    #ifdef HAS_DEBUG
    const char* layoutName = layout["name"];
    debug(F("Layout Name: "));
    debugln(layoutName);
    #endif

    if(numOfTracks < MAX_LAYOUTS) {
      // add name to array of strings to display to the user
      strncpy(tracks[numOfTracks], layout["name"], sizeof(tracks[numOfTracks]) - 1);
      tracks[numOfTracks][sizeof(tracks[numOfTracks]) - 1] = '\0';

      // Store lengthFt per course (new format field, defaults to 0)
      activeTrackMetadata.courseLengthFt[numOfTracks] = layout["lengthFt"] | 0.0f;

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
  releaseSDAccess(SD_ACCESS_TRACK_PARSE);
  return PARSE_STATUS_GOOD;
}
