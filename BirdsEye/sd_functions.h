#pragma once

///////////////////////////////////////////
// SD CARD MODULE
// SD init, access arbitration (mutex), track-list building, and
// track-JSON parsing. All cross-task SD users must go through the
// access mutex to avoid corrupting SdFat's internal state.
///////////////////////////////////////////

#include "sd_access_policy.h"

// SD access modes — used by acquireSDAccess / releaseSDAccess. Aliases of
// the sd_access_policy constants (the single source of truth for the values
// and the grant/deny rules — TRACK_PARSE is preemptible, same-mode
// re-acquire is idempotent; see sd_access_policy.h).
#define SD_ACCESS_NONE         sd_access_policy::kNone
#define SD_ACCESS_LOGGING      sd_access_policy::kLogging
#define SD_ACCESS_REPLAY       sd_access_policy::kReplay
#define SD_ACCESS_BLE_TRANSFER sd_access_policy::kBleTransfer
#define SD_ACCESS_TRACK_PARSE  sd_access_policy::kTrackParse
#define SD_ACCESS_USB_MSC      sd_access_policy::kUsbMsc
#define SD_ACCESS_FORMAT       sd_access_policy::kFormat

// JSON parser status codes returned by parseTrackFile(). Defined in
// BirdsEye.ino; redeclared here so callers in other .ino files can
// reference them without depending on declaration order.
extern const int PARSE_STATUS_GOOD;
extern const int PARSE_STATUS_LOAD_FAILED;
extern const int PARSE_STATUS_PARSE_FAILED;

// Current SD owner. volatile because BLE callback task and main loop
// both read it. Writes are gated by the acquire/release helpers.
extern volatile int currentSDAccess;

// Set by SD_SETUP() when the card answers at the SPI level but its FAT
// volume will not mount (factory-blank or corrupted soldered-in module).
// Routes boot to the format-confirm page instead of the FAULT dead-end.
extern bool sdCardUnformatted;

// Attempt to acquire SD for a given mode. Idempotent for the same
// mode. Returns false if SD is busy with a different mode.
bool acquireSDAccess(int mode);

// Release a previously-acquired SD access. No-op if `mode` is not
// the current holder (safe to call on error paths).
void releaseSDAccess(int mode);

// Force the SD mutex back to NONE — last-resort recovery after an
// error path forgets to release.
void forceReleaseSDAccess();

// Build "/TRACKS/<trackName>.json" (TRACK_KIND_CIRCUIT) or
// "/TRACKS/SPRINT/<trackName>.json" (TRACK_KIND_SPRINT) into the caller's
// filepath buffer. Caller MUST provide at least FILEPATH_MAX bytes.
void makeFullTrackPath(const char* trackName, char* filepath, uint8_t kind);

// Walk one track folder, appending entries to locations[] and
// trackManifest[] (shared caps) with the given TRACK_KIND_*. Caller must
// hold the SD mutex. Returns false only if the directory can't be opened.
bool scanTrackDir(const char* folder, uint8_t kind);

// Initialize the SD card (with EMI-tolerant retries). Returns true
// on success. Populates the global SD object. On failure, probes the
// raw card and sets sdCardUnformatted when it responds without a
// mountable FAT volume.
bool SD_SETUP();

// Format the card FAT16/32 (blocking; pets the WDT via the formatter's
// progress callbacks). Reboots the device on success; on failure returns
// to the confirm page with sdFormatLastFailed set (a fresh full hold is
// required to retry). Only call from the PAGE_SD_FORMAT confirm flow.
void sdPerformFormat();

// (Re)initialize the SD card at a specific SPI clock (EMI-tolerant retries).
bool sdSetSpiClock(uint32_t maxSck);

// Switch the SD SPI clock between the fast parked-transfer clock (true) and the
// EMI-safe normal clock (false). Only call when no SD file is open. Falls back
// to the normal clock if the fast re-init fails.
void sdSetTransferSpeed(bool fast);

// Make sure /TRACKS exists, creating it when missing (blank soldered-in
// card). Caller must already hold the SD mutex. Returns true when the
// folder exists or was created.
bool sdEnsureTracksFolder();

// Scan /TRACKS/ and populate locations[] + trackManifest[] (one entry
// per .json file). Creates the folder when missing (blank soldered-in
// card). Returns true if the folder existed or was created.
bool buildTrackList();

// Parse one track JSON file from disk into activeTrackMetadata and
// trackLayouts[]. Auto-detects new (object) vs legacy (bare array)
// JSON format. Returns one of the PARSE_STATUS_* codes.
int parseTrackFile(char* filepath);
