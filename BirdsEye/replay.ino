///////////////////////////////////////////
// REPLAY MODULE
// Instant DOVEX session replay: parses the reserved header at the
// start of a .dovex file (datetime, driver, course, lap times) and
// populates lapHistory[] for the results page.
///////////////////////////////////////////

#include "replay.h"
#include "dovex_header.h"

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

///////////////////////////////////////////
// DOVEX REPLAY (header-only, instant)
///////////////////////////////////////////

// dovexReplay* globals are declared in BirdsEye.ino (must be visible to display_pages.ino)

/**
 * @brief Parse a DOVEX file's header (first 1KB) for instant replay results.
 *
 * Reads the reserved 1KB region into a stack buffer, then hands it to
 * dovex_header::parse() (the testable pure function). On success, copies
 * the parsed strings into the dovexReplay* globals and writes the lap
 * times directly into lapHistory[].
 *
 * @return true if header parsed successfully
 */
bool parseDovexHeader(const char* filename) {
  File replayDovex;
  if (!replayDovex.open(filename, O_READ)) {
    debugln(F("DOVEX Replay: Cannot open file"));
    return false;
  }

  static char buf[dovex_header::kHeaderSize];
  const int bytesRead = replayDovex.read(buf, sizeof(buf));
  replayDovex.close();

  if (bytesRead < (int)sizeof(buf)) {
    debugln(F("DOVEX Replay: Header too short"));
    return false;
  }

  dovex_header::ParsedHeader meta;
  size_t lapCount = 0;
  if (!dovex_header::parse(buf, sizeof(buf), meta,
                           lapHistory, lapHistoryMaxLaps, lapCount)) {
    debugln(F("DOVEX Replay: Empty/incomplete header"));
    return false;
  }

  // Copy parsed metadata into module globals consumed by the display pages.
  strncpy(dovexReplayDatetime,   meta.datetime,  sizeof(dovexReplayDatetime)   - 1);
  dovexReplayDatetime[sizeof(dovexReplayDatetime)   - 1] = '\0';
  strncpy(dovexReplayDriver,     meta.driver,    sizeof(dovexReplayDriver)     - 1);
  dovexReplayDriver[sizeof(dovexReplayDriver)       - 1] = '\0';
  strncpy(dovexReplayCourseName, meta.course,    sizeof(dovexReplayCourseName) - 1);
  dovexReplayCourseName[sizeof(dovexReplayCourseName) - 1] = '\0';
  strncpy(dovexReplayShortName,  meta.shortName, sizeof(dovexReplayShortName)  - 1);
  dovexReplayShortName[sizeof(dovexReplayShortName)   - 1] = '\0';
  strncpy(dovexReplayBestLap,    meta.bestLap,   sizeof(dovexReplayBestLap)    - 1);
  dovexReplayBestLap[sizeof(dovexReplayBestLap)     - 1] = '\0';
  strncpy(dovexReplayOptimal,    meta.optimal,   sizeof(dovexReplayOptimal)    - 1);
  dovexReplayOptimal[sizeof(dovexReplayOptimal)     - 1] = '\0';

  lapHistoryCount = (int)lapCount;
  lastLap = (lapCount > 0) ? lapHistory[lapCount - 1] : 0;

  debug(F("DOVEX Replay: Parsed "));
  debug(lapHistoryCount);
  debugln(F(" laps from header"));

  return true;
}
