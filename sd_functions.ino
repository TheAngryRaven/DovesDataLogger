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

    // Add the file to the array (use destination size, not source size!)
    strncpy(locations[numOfLocations], filename, MAX_LOCATION_LENGTH - 1);
    locations[numOfLocations][MAX_LOCATION_LENGTH - 1] = '\0';  // Ensure null-termination

    // Increment the numOfLocations
    numOfLocations++;

    // Close the file to free up any memory it's using
    file.close();
  }

  // Close the directory to free up any memory it's using
  trackDir.close();

  return true;
}

// Static buffers for JSON parsing — saves ~4KB of stack per call.
// Only one parseTrackFile() call can be active at a time (single-threaded).
static char jsonFileBuffer[JSON_BUFFER_SIZE];
static StaticJsonDocument<JSON_BUFFER_SIZE> trackJson;

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

  // Read file into static buffer (not stack-allocated)
  int bytesRead = trackFile.read(jsonFileBuffer, sizeof(jsonFileBuffer));

  // Check if read was successful
  if (bytesRead == -1) {
    debugln(F("ParseTrackFile: failed to READ file"));
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
