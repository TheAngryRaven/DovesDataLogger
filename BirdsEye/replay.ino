///////////////////////////////////////////
// REPLAY MODULE
// Instant DOVEX session replay: parses the reserved header at the
// start of a .dovex file (datetime, driver, course, lap times) and
// populates lapHistory[] for the results page.
//
// haversineDistanceMiles() also lives here — used by the auto-track
// detection loop in BirdsEye.ino.
///////////////////////////////////////////

#include "replay.h"

/**
 * @brief Reset replay state to initial values
 */
void resetReplayState() {
  replayProcessingComplete = false;

  // Reset lap history for replay
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
 * @brief Build list of .dovex replay files from SD card root
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

    char name[64];
    entry.getName(name, sizeof(name));

    debug(F("Replay: ["));
    debug(filesScanned);
    debug(F("] "));
    debug(entry.isDirectory() ? F("DIR: ") : F("FILE: "));
    debug(name);
    debug(F(" (len="));
    debug(strlen(name));
    debugln(F(")"));

    if (!entry.isDirectory()) {
      int len = strlen(name);
      if (len > 6) {
        char* ext = name + len - 6;
        if (strcasecmp(ext, ".dovex") == 0) {
          strncpy(replayFiles[numReplayFiles], name, MAX_REPLAY_FILENAME_LENGTH - 1);
          replayFiles[numReplayFiles][MAX_REPLAY_FILENAME_LENGTH - 1] = '\0';
          numReplayFiles++;
          debug(F("Replay: Found file: "));
          debugln(name);
        }
      }
    }
    entry.close();
  }

  root.close();

  debug(F("Replay: Total files found: "));
  debugln(numReplayFiles);

  return numReplayFiles > 0;
}

/**
 * @brief Read a single line from file into buffer (drops \r, stops at \n or EOF)
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

///////////////////////////////////////////
// DOVEX REPLAY (header-only, instant)
///////////////////////////////////////////

// dovexReplay* globals are declared in BirdsEye.ino (must be visible to display_pages.ino)

/**
 * @brief Parse a DOVEX file's header (first 1KB) for instant replay results.
 * Line 1: column labels (datetime,driver,course,short_name,best_lap_ms,optimal_ms)
 * Line 2: metadata values
 * Line 3: column label (laps_ms)
 * Line 4: lap1_ms,lap2_ms,lap3_ms,...
 * @return true if header parsed successfully
 */
bool parseDovexHeader(const char* filename) {
  File replayDovex;
  if (!replayDovex.open(filename, O_READ)) {
    debugln(F("DOVEX Replay: Cannot open file"));
    return false;
  }

  // Line 1 is the column header label — skip it
  char headerBuf[256];
  if (!readReplayLine(replayDovex, headerBuf, sizeof(headerBuf))) {
    replayDovex.close();
    debugln(F("DOVEX Replay: Cannot read line 1"));
    return false;
  }

  // Check for empty/null header (crash recovery — metadata never written)
  if (headerBuf[0] == '\0' || headerBuf[0] == '\n' || headerBuf[0] == '\r') {
    replayDovex.close();
    debugln(F("DOVEX Replay: Empty header (incomplete session)"));
    return false;
  }

  // Read line 2 (metadata values)
  if (!readReplayLine(replayDovex, headerBuf, sizeof(headerBuf))) {
    replayDovex.close();
    debugln(F("DOVEX Replay: Cannot read line 2"));
    return false;
  }

  // Parse line 2: datetime, driver, course, short_name, best_lap, optimal
  char* tok = strtok(headerBuf, ",");
  if (tok) { strncpy(dovexReplayDatetime, tok, sizeof(dovexReplayDatetime) - 1); dovexReplayDatetime[sizeof(dovexReplayDatetime) - 1] = '\0'; }
  tok = strtok(NULL, ",");
  if (tok) { strncpy(dovexReplayDriver, tok, sizeof(dovexReplayDriver) - 1); dovexReplayDriver[sizeof(dovexReplayDriver) - 1] = '\0'; }
  tok = strtok(NULL, ",");
  if (tok) { strncpy(dovexReplayCourseName, tok, sizeof(dovexReplayCourseName) - 1); dovexReplayCourseName[sizeof(dovexReplayCourseName) - 1] = '\0'; }
  tok = strtok(NULL, ",");
  if (tok) { strncpy(dovexReplayShortName, tok, sizeof(dovexReplayShortName) - 1); dovexReplayShortName[sizeof(dovexReplayShortName) - 1] = '\0'; }
  tok = strtok(NULL, ",");
  if (tok) { strncpy(dovexReplayBestLap, tok, sizeof(dovexReplayBestLap) - 1); dovexReplayBestLap[sizeof(dovexReplayBestLap) - 1] = '\0'; }
  tok = strtok(NULL, ",");
  if (tok) { strncpy(dovexReplayOptimal, tok, sizeof(dovexReplayOptimal) - 1); dovexReplayOptimal[sizeof(dovexReplayOptimal) - 1] = '\0'; }

  // Skip line 3 (lap column header)
  static char lapBuf[800];
  readReplayLine(replayDovex, lapBuf, sizeof(lapBuf));

  // Read line 4 (lap times)
  if (!readReplayLine(replayDovex, lapBuf, sizeof(lapBuf))) {
    // No lap times — session with no laps
    replayDovex.close();
    lapHistoryCount = 0;
    return true;
  }

  replayDovex.close();

  // Parse comma-separated lap times
  lapHistoryCount = 0;
  lastLap = 0;
  tok = strtok(lapBuf, ",");
  while (tok != NULL && lapHistoryCount < lapHistoryMaxLaps) {
    unsigned long lapMs = strtoul(tok, NULL, 10);
    if (lapMs > 0) {
      lapHistory[lapHistoryCount++] = lapMs;
      lastLap = lapMs;
    }
    tok = strtok(NULL, ",");
  }

  debug(F("DOVEX Replay: Parsed "));
  debug(lapHistoryCount);
  debugln(F(" laps from header"));

  return true;
}
