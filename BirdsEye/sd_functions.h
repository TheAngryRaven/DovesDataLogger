#pragma once

///////////////////////////////////////////
// SD CARD MODULE
// SD init, access arbitration (mutex), track-list building, and
// track-JSON parsing. All cross-task SD users must go through the
// access mutex to avoid corrupting SdFat's internal state.
///////////////////////////////////////////

// SD access modes — used by acquireSDAccess / releaseSDAccess.
// TRACK_PARSE is treated as "preemptible" by other acquirers since it's
// always brief.
#define SD_ACCESS_NONE         0
#define SD_ACCESS_LOGGING      1
#define SD_ACCESS_REPLAY       2
#define SD_ACCESS_BLE_TRANSFER 3
#define SD_ACCESS_TRACK_PARSE  4

// JSON parser status codes returned by parseTrackFile(). Defined in
// BirdsEye.ino; redeclared here so callers in other .ino files can
// reference them without depending on declaration order.
extern const int PARSE_STATUS_GOOD;
extern const int PARSE_STATUS_LOAD_FAILED;
extern const int PARSE_STATUS_PARSE_FAILED;

// Current SD owner. volatile because BLE callback task and main loop
// both read it. Writes are gated by the acquire/release helpers.
extern volatile int currentSDAccess;

// Attempt to acquire SD for a given mode. Idempotent for the same
// mode. Returns false if SD is busy with a different mode.
bool acquireSDAccess(int mode);

// Release a previously-acquired SD access. No-op if `mode` is not
// the current holder (safe to call on error paths).
void releaseSDAccess(int mode);

// Force the SD mutex back to NONE — last-resort recovery after an
// error path forgets to release.
void forceReleaseSDAccess();

// Build "/TRACKS/<trackName>.json" into the caller's filepath buffer.
// Caller MUST provide at least FILEPATH_MAX bytes.
void makeFullTrackPath(const char* trackName, char* filepath);

// Initialize the SD card (with EMI-tolerant retries). Returns true
// on success. Populates the global SD object.
bool SD_SETUP();

// Scan /TRACKS/ and populate locations[] + trackManifest[] (one entry
// per .json file). Returns true if the folder existed.
bool buildTrackList();

// Parse one track JSON file from disk into activeTrackMetadata and
// trackLayouts[]. Auto-detects new (object) vs legacy (bare array)
// JSON format. Returns one of the PARSE_STATUS_* codes.
int parseTrackFile(char* filepath);
