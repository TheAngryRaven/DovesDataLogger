///////////////////////////////////////////
// SD CARD MODULE
// SD card setup, access management, track list building, and JSON track parsing
///////////////////////////////////////////

#include "sd_functions.h"

#include "track_json.h"

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
    // Normally the requester takes ownership; a track parse nested under a
    // logging hold leaves logging as the owner (see sd_access_policy.h).
    currentSDAccess = sd_access_policy::ownerAfterAcquire(currentSDAccess, mode);
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

void makeFullTrackPath(const char* trackName, char* filepath, uint8_t kind) {
  // Use snprintf for bounds safety - prevents buffer overflow
  // Caller MUST provide buffer of at least FILEPATH_MAX bytes
  const char* folder = (kind == TRACK_KIND_SPRINT) ? trackFolderSprint : trackFolder;
  snprintf(filepath, FILEPATH_MAX, "%s/%s.json", folder, trackName);
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
  // absent card (keeps the existing FAULT dead-end). The volumeBegin()
  // check is what makes the diagnosis safe: if the three begin() attempts
  // failed at the CARD layer (EMI, marginal contact) the FAT may be
  // perfectly healthy — offering a format then would invite the user to
  // erase real data. If the volume mounts here, the card just recovered.
  if (SD.cardBegin(SdSpiConfig(PIN_SPI_CS, SHARED_SPI, SPI_SPEED)) &&
      SD.card() && SD.card()->sectorCount() > 0) {
    if (SD.volumeBegin()) {
      debugln(F("SD recovered on probe (transient card-init failure)"));
      return true;
    }
    sdCardUnformatted = true;
    debugln(F("SD card responds but has no FAT volume — format candidate"));
  }
  return false;
}

// Make sure /TRACKS and /TRACKS/SPRINT exist (blank soldered-in card;
// SdFat's open() never creates parent directories). Caller must already
// hold the SD mutex. Returns true when the circuit folder exists or was
// created — the sprint subfolder is best-effort (a failure there must
// not take out circuit operation).
bool sdEnsureTracksFolder() {
  if (!SD.exists(trackFolder) && !SD.mkdir(trackFolder)) {
    return false;
  }
  if (!SD.exists(trackFolderSprint) && !SD.mkdir(trackFolderSprint)) {
    debugln(F("WARNING: could not create /TRACKS/SPRINT"));
  }
  return true;
}

///////////////////////////////////////////
// ON-DEVICE FORMAT (blank/corrupt soldered-in module)
///////////////////////////////////////////

// FatFormatter reports progress through a Print*: a few text messages plus
// one '.' per ~(2*fatSize+spc)/32 FAT sectors written. Each write pets the
// ~4 s WDT (armed since the end of setup(); the format blocks the main
// loop). A 64 GB card at the 2 MHz EMI clock leaves ~1.4 s between pets,
// and an SD-internal GC stall can add up to ~2 s — so a huge card on the
// slow clock CAN brush the WDT. That is accepted deliberately: a WDT reset
// mid-format reboots straight back to this page (the card is still
// unmountable — idempotent, not a brick), which is a better backstop than
// a timer-fed WDT that would mask a true SPI hang. The static "FORMATTING"
// screen is painted once before the format starts; repainting it here
// would only add I2C traffic to the erase window.
class SdFormatProgress : public Print {
 public:
  size_t write(uint8_t) override {
    wdtPet();
    return 1;
  }
  size_t write(const uint8_t*, size_t n) override {
    wdtPet();
    return n;
  }
};

void sdPerformFormat() {
  // Nothing else can hold the card here (no FAT ever mounted this boot),
  // but hold the mutex anyway so no other subsystem can sneak in mid-erase.
  if (!acquireSDAccess(SD_ACCESS_FORMAT)) {
    return;  // impossible in practice; stay on the confirm page
  }

  // A paired camera may be recording (camera control is independent of SD
  // state — a tach-wake boot can have started it). The format blocks the
  // main loop for minutes (CAMERA_LOOP() and the ce82 GPS heartbeat stop)
  // and ends in a hard reset with no shutdown teardown, which would leave
  // the X4 recording until its battery dies. Stop and power it off first —
  // a no-op when the camera stack never came up.
  CAMERA_SLEEP();
  wdtPet();

  displayPage_sd_format_progress(F(" Formatting card..."), F(" DO NOT POWER OFF"));
  wdtPet();

  // Clock policy: the fast parked-transfer clock is only safe with the
  // engine off — ignition EMI corrupts writes SILENTLY (the card ACKs
  // them and SD.format() has no read-back verify), so a "successful"
  // 8 MHz format under RPM could produce an unmountable FAT and loop the
  // user right back here. Engine turning → EMI-safe clock only.
  SdFormatProgress progress;
  const bool engineTurning = tachLastReported > 0;
  const uint32_t clocks[2] = {
      engineTurning ? (uint32_t)SPI_SPEED : (uint32_t)SD_SPI_SPEED_FAST,
      (uint32_t)SPI_SPEED};
  bool formatted = false;
  for (int i = 0; i < 2 && !formatted; i++) {
    if (SD.cardBegin(SdSpiConfig(PIN_SPI_CS, SHARED_SPI, clocks[i]))) {
      formatted = SD.format(&progress);
    }
    wdtPet();
  }

  if (!formatted) {
    releaseSDAccess(SD_ACCESS_FORMAT);
    debugln(F("SD format failed"));
    // Stay on the confirm page rather than the buttons-dead FAULT page: an
    // engine-on EMI failure is transient and retryable, and the confirm
    // page keeps the idle-timeout battery protection FAULT lacks (this
    // sealed device has no power switch to escape a dead-end screen).
    // Re-begin the state machine so a still-held Select can't instantly
    // re-fire — a fresh release + full 3 s hold is required to retry.
    sdFormatLastFailed = true;
    sd_format_page::begin(sdFormatState, millis());
    return;
  }

  // Mount the fresh volume and provision /TRACKS now — even though the
  // fresh boot's buildTrackList() would also create it — so an interrupted
  // reboot still leaves a usable card.
  wdtPet();
  if (SD.begin(PIN_SPI_CS, SPI_SPEED)) {
    sdEnsureTracksFolder();
  }

  debugln(F("SD format complete, rebooting"));
  displayPage_sd_format_progress(F(" Format OK"), F(" Rebooting..."));
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

  // Soldered-in module: first boot is a blank card — self-provision the
  // folder so BLE track uploads work without hand-populating the card.
  if (!sdEnsureTracksFolder()) {
    debugln(F("TRACKS folder missing and could not be created."));
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return false;
  }

  // reset lists
  numOfLocations = 0;
  trackManifestCount = 0;

  // Circuit tracks are mandatory (the original folder); the sprint scan is
  // best-effort — a missing/unreadable /TRACKS/SPRINT must never take out
  // circuit operation. Both share the locations[]/manifest arrays and caps.
  if (!scanTrackDir(trackFolder, TRACK_KIND_CIRCUIT)) {
    debugln(F("Failed to open TRACKS folder."));
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return false;
  }
  scanTrackDir(trackFolderSprint, TRACK_KIND_SPRINT);

  debug(F("Tracks found: "));
  debugln(numOfLocations);
  debug(F("Manifest entries: "));
  debugln(trackManifestCount);

  releaseSDAccess(SD_ACCESS_TRACK_PARSE);
  return true;
}

// Walk one track folder, appending to locations[] and trackManifest[]
// (shared caps). Caller must hold the SD mutex. Returns false only when
// the directory cannot be opened.
bool scanTrackDir(const char* folder, uint8_t kind) {
  if (!trackDir.open(folder)) {
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

    // Skip sub-directories (the /TRACKS scan would otherwise list the
    // SPRINT folder itself as a "track").
    if (file.isDir()) {
      file.close();
      continue;
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
            trackManifest[trackManifestCount].kind = kind;
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
    // Track-level type marker ("sprint"). Redundant with the folder the
    // file lives in — the folder is authoritative — but parsed as cheap
    // validation / future-proofing (plan 0002 keeps modes an open enum).
    const char* trackType = trackJson["type"] | "";
    activeTrackMetadata.isSprint = (strcasecmp(trackType, "sprint") == 0);

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
    activeTrackMetadata.isSprint = false;
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

      // Sprint-only: separate finish line (optional — same idiom as sectors)
      if (layout.containsKey("finish_a_lat") && layout.containsKey("finish_a_lng") &&
          layout.containsKey("finish_b_lat") && layout.containsKey("finish_b_lng")) {
        trackLayouts[numOfTracks].finish_a_lat = layout["finish_a_lat"];
        trackLayouts[numOfTracks].finish_a_lng = layout["finish_a_lng"];
        trackLayouts[numOfTracks].finish_b_lat = layout["finish_b_lat"];
        trackLayouts[numOfTracks].finish_b_lng = layout["finish_b_lng"];
        trackLayouts[numOfTracks].hasFinish = true;
        #ifdef HAS_DEBUG
        debugln(F("  Finish line data loaded (sprint)"));
        #endif
      } else {
        trackLayouts[numOfTracks].hasFinish = false;
      }

      // Sprint-only: sortable ISO date_created (drives newest-course pick)
      const char* dateCreated = layout["date_created"] | "";
      strncpy(trackLayouts[numOfTracks].date_created, dateCreated,
              sizeof(trackLayouts[numOfTracks].date_created) - 1);
      trackLayouts[numOfTracks].date_created[sizeof(trackLayouts[numOfTracks].date_created) - 1] = '\0';

      numOfTracks++;
    }
  }

  // make sure to close before logging
  debugln(F("ParseTrackFile: SUCCESS"));
  trackFile.close();
  releaseSDAccess(SD_ACCESS_TRACK_PARSE);
  return PARSE_STATUS_GOOD;
}

///////////////////////////////////////////
// TRACK WRITING (on-device course creator, plan 0002 §5)
//
// This is the firmware's first track-file WRITER — until the course
// creator it only ever read them. The JSON text itself is built by the
// host-tested track_json unit; everything here is file plumbing.
///////////////////////////////////////////

// Scratch for one serialized course. A course is four coordinate lines at
// ~28 bytes a field plus keys — comfortably under 700 bytes even with all
// four lines and a date stamp. Static, not stack: the sketch keeps big
// buffers out of the main loop's stack (same reason jsonFileBuffer is).
static char courseJsonBuffer[768];
// A course parsed back into its own document so it can be grafted into the
// existing track. Sized to hold that same one course.
static StaticJsonDocument<768> courseJson;

/**
 * @brief Append a course to an existing track file.
 *
 * Read-modify-write, the same shape settings.ino uses, with one addition:
 * the rewrite goes to a temp file that REPLACES the original only once it
 * is safely closed. Writing in place would mean a power loss (or a card
 * yank) mid-serialize leaves a truncated file where a working track used
 * to be — and this runs in a field, on a battery, at an event.
 *
 * Caller must hold the SD mutex.
 */
static SdCourseWriteResult appendCourseToTrackFile(const char* filepath,
                                                   const char* courseText) {
  trackFile.open(filepath, O_READ);
  if (!trackFile) {
    debugln(F("SaveCourse: append target missing"));
    return SD_COURSE_WRITE_NO_TRACK;
  }
  const int bytesRead = trackFile.read(jsonFileBuffer, sizeof(jsonFileBuffer) - 1);
  trackFile.close();
  if (bytesRead <= 0) return SD_COURSE_WRITE_NO_TRACK;
  jsonFileBuffer[bytesRead] = '\0';

  trackJson.clear();
  if (deserializeJson(trackJson, jsonFileBuffer) != DeserializationError::Ok) {
    debugln(F("SaveCourse: existing track will not parse"));
    return SD_COURSE_WRITE_NO_TRACK;
  }

  // Both on-disk shapes are appendable: the object format's "courses"
  // array, and the legacy bare array which IS the course list.
  JsonArray courses;
  if (trackJson.is<JsonObject>()) {
    courses = trackJson["courses"];
    if (courses.isNull()) courses = trackJson.createNestedArray("courses");
  } else if (trackJson.is<JsonArray>()) {
    courses = trackJson.as<JsonArray>();
  } else {
    return SD_COURSE_WRITE_NO_TRACK;
  }
  if (courses.isNull()) return SD_COURSE_WRITE_NO_TRACK;

  if ((int)courses.size() >= MAX_LAYOUTS) {
    // The device only ever loads MAX_LAYOUTS courses, so an appended one
    // past the cap would be written and then silently ignored.
    debugln(F("SaveCourse: track already holds MAX_LAYOUTS courses"));
    return SD_COURSE_WRITE_TOO_BIG;
  }

  courseJson.clear();
  if (deserializeJson(courseJson, courseText) != DeserializationError::Ok) {
    return SD_COURSE_WRITE_TOO_BIG;
  }
  const size_t before = courses.size();
  courses.add(courseJson);
  // The whole file has to fit the 4 KB parse budget on the next boot, so a
  // grafted course that overflowed the document must not reach the card.
  if (courses.size() != before + 1 || trackJson.overflowed()) {
    debugln(F("SaveCourse: track file is full"));
    return SD_COURSE_WRITE_TOO_BIG;
  }

  char tempPath[FILEPATH_MAX];
  snprintf(tempPath, sizeof(tempPath), "%s.tmp", filepath);
  SD.remove(tempPath);  // a temp left by an interrupted earlier attempt

  File32 outFile;
  outFile.open(tempPath, O_CREAT | O_WRITE | O_TRUNC);
  if (!outFile) return SD_COURSE_WRITE_IO;
  const size_t written = serializeJson(trackJson, outFile);
  outFile.sync();
  outFile.close();

  if (written == 0) {
    SD.remove(tempPath);
    return SD_COURSE_WRITE_IO;
  }

  // Swap only now that the replacement is complete on the card.
  if (!SD.remove(filepath)) {
    SD.remove(tempPath);
    return SD_COURSE_WRITE_IO;
  }
  if (!SD.rename(tempPath, filepath)) {
    debugln(F("SaveCourse: rename failed"));
    return SD_COURSE_WRITE_IO;
  }
  return SD_COURSE_WRITE_OK;
}

SdCourseWriteResult sdSaveCreatedCourse(const CreatedCourseWrite& req) {
  if (req.course == nullptr) return SD_COURSE_WRITE_IO;

  const uint8_t kind = (req.course->kind == course_creator::CourseKind::kSprint)
                           ? TRACK_KIND_SPRINT
                           : TRACK_KIND_CIRCUIT;

  const int courseLen = track_json::formatCourse(
      courseJsonBuffer, sizeof(courseJsonBuffer),
      *req.course, req.courseName, req.dateCreated);
  if (courseLen < 0) return SD_COURSE_WRITE_TOO_BIG;

  if (!acquireSDAccess(SD_ACCESS_TRACK_PARSE)) {
    debugln(F("SaveCourse: SD busy"));
    return SD_COURSE_WRITE_BUSY;
  }

  // Both folders, since SdFat's open() never creates parents and a course
  // can be the very first thing ever written to a blank soldered-in card.
  if (!sdEnsureTracksFolder() ||
      (kind == TRACK_KIND_SPRINT && !SD.exists(trackFolderSprint))) {
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return SD_COURSE_WRITE_IO;
  }

  char filepath[FILEPATH_MAX];
  makeFullTrackPath(req.trackName, filepath, kind);

  SdCourseWriteResult result;
  if (req.newTrack) {
    if (SD.exists(filepath)) {
      // Names carry the GPS clock down to the minute, so this means the
      // same minute twice — refuse rather than overwrite someone's course.
      debugln(F("SaveCourse: track file already exists"));
      result = SD_COURSE_WRITE_EXISTS;
    } else {
      const int fileLen = track_json::formatTrackFile(
          jsonFileBuffer, sizeof(jsonFileBuffer),
          req.trackName, req.shortName,
          *req.course, req.courseName, req.dateCreated);
      if (fileLen < 0) {
        result = SD_COURSE_WRITE_TOO_BIG;
      } else {
        File32 outFile;
        outFile.open(filepath, O_CREAT | O_WRITE | O_TRUNC);
        if (!outFile) {
          result = SD_COURSE_WRITE_IO;
        } else {
          const size_t written = outFile.write(jsonFileBuffer, (size_t)fileLen);
          outFile.sync();
          outFile.close();
          result = (written == (size_t)fileLen) ? SD_COURSE_WRITE_OK : SD_COURSE_WRITE_IO;
          if (result != SD_COURSE_WRITE_OK) SD.remove(filepath);
        }
      }
    }
  } else {
    result = appendCourseToTrackFile(filepath, courseJsonBuffer);
  }

  releaseSDAccess(SD_ACCESS_TRACK_PARSE);

  // The manifest drives proximity detection, so a course that isn't in it
  // doesn't exist as far as the next session is concerned.
  if (result == SD_COURSE_WRITE_OK) buildTrackList();
  return result;
}
