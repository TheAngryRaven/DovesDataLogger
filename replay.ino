///////////////////////////////////////////
// REPLAY MODULE
// Replay system: file list building, NMEA/DOVE parsing,
// track detection, haversine distance, and replay processing
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

  // Close replay file if open and release SD access
  if (replayFile.isOpen()) {
    replayFile.close();
    debugln(F("Replay: Closed replay file"));
  }
  releaseSDAccess(SD_ACCESS_REPLAY);
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
 * Format: timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm
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
      case 7: // heading_deg - skip (replay doesn't use it)
        break;
      case 8: // h_acc_m - skip (replay doesn't use it)
        break;
      case 9: // rpm
        sample.rpm = atoi(token);
        break;
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
  char filepath[FILEPATH_MAX];
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
