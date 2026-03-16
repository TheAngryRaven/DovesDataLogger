///////////////////////////////////////////
// SD CARD MODULE
// SD card setup, access management, track list building, and JSON track parsing
///////////////////////////////////////////

/**
 * @brief Attempt to acquire SD card access for a subsystem
 * @param mode The access mode being requested (SD_ACCESS_*)
 * @return true if access granted, false if SD busy with another operation
 */
bool acquireSDAccess(int mode) {
  // Allow re-acquiring same mode (idempotent)
  if (currentSDAccess == mode) return true;

  // Only allow acquisition if currently free or track parsing (which is temporary)
  if (currentSDAccess == SD_ACCESS_NONE || currentSDAccess == SD_ACCESS_TRACK_PARSE) {
    currentSDAccess = mode;
    return true;
  }

  // SD busy with another subsystem
  debug(F("SD access denied, current mode: "));
  debugln(currentSDAccess);
  return false;
}

/**
 * @brief Release SD card access
 * @param mode The access mode being released (must match current)
 */
void releaseSDAccess(int mode) {
  if (currentSDAccess == mode) {
    currentSDAccess = SD_ACCESS_NONE;
  }
}

/**
 * @brief Force release all SD access (use during cleanup/error recovery)
 */
void forceReleaseSDAccess() {
  currentSDAccess = SD_ACCESS_NONE;
}

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

// Static buffers for JSON parsing — saves ~4KB of stack per call.
// Only one parseTrackFile() call can be active at a time (single-threaded).
// Also reused by buildTrackList() for manifest extraction.
static char jsonFileBuffer[JSON_BUFFER_SIZE];
static StaticJsonDocument<JSON_BUFFER_SIZE> trackJson;

bool buildTrackList() {
  if (!SD.exists(trackFolder)) {
    debugln(F("TRACKS folder does not exist."));
    return false;
  }

  // reset lists
  numOfLocations = 0;
  #ifdef ENABLE_NEW_UI
  trackManifestCount = 0;
  #endif

  // If the TRACKS directory exists, open it
  trackDir.open(trackFolder);

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
    #ifdef ENABLE_NEW_UI
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
    #endif

    // Increment the numOfLocations
    numOfLocations++;

    // Close the file to free up any memory it's using
    file.close();
  }

  // Close the directory to free up any memory it's using
  trackDir.close();

  debug(F("Tracks found: "));
  debugln(numOfLocations);
  #ifdef ENABLE_NEW_UI
  debug(F("Manifest entries: "));
  debugln(trackManifestCount);
  #endif

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

    // Derive metadata from filename (already stored in locations[selectedLocation])
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
