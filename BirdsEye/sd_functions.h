#pragma once

///////////////////////////////////////////
// SD CARD MODULE
// SD init, access arbitration (mutex), track-list building, and
// track-JSON parsing. All cross-task SD users must go through the
// access mutex to avoid corrupting SdFat's internal state.
///////////////////////////////////////////

#include <stdint.h>

#include "course_creator.h"  // CreatedCourseWrite carries a walked course
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

// SdFat's card-level error code from the last failed boot probe or
// post-format mount (SdBase::sdErrorCode(); 0 = none). Shown on the
// format page's diagnostic line and the SD FAULT page so a wiring or
// card-state failure can be told apart from a blank card in the field.
extern uint8_t sdLastErrorCode;

// Why the last on-device format attempt bounced back to the confirm page
// (sdFormatFailure). NONE = no attempt yet / last one rebooted clean.
// ERASE = SD.format() itself failed (retryable, usually engine-on EMI).
// MOUNT = the format reported success but the fresh volume would not
// mount afterwards: re-formatting will not help, the card or its wiring
// is the problem, and a power cycle is the recovery (the card cannot be
// reset by firmware — CS is grounded and there is no power switch).
constexpr uint8_t SD_FORMAT_FAIL_NONE = 0;
constexpr uint8_t SD_FORMAT_FAIL_ERASE = 1;
constexpr uint8_t SD_FORMAT_FAIL_MOUNT = 2;
extern uint8_t sdFormatFailure;

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
// raw card layer by layer (host-tested sd_probe rules) and sets
// sdCardUnformatted only when it answers consistently, across a settle,
// without a mountable FAT volume. Records sdLastErrorCode.
bool SD_SETUP();

// Format the card FAT16/32 (blocking; pets the WDT via the formatter's
// progress callbacks). Verifies the fresh volume mounts, then reboots.
// On failure returns to the confirm page with sdFormatFailure set (a
// fresh full hold is required to retry). Only call from the
// PAGE_SD_FORMAT confirm flow.
void sdPerformFormat();

// (Re)initialize the SD card at a specific SPI clock (EMI-tolerant retries).
bool sdSetSpiClock(uint32_t maxSck);

// Switch the SD SPI clock between the fast parked-transfer clock (true) and the
// EMI-safe normal clock (false). Only call when no SD file is open. Falls back
// to the normal clock if the fast re-init fails.
void sdSetTransferSpeed(bool fast);

// The SPI clock the card is actually running at (Hz), i.e. the last one
// SD.begin() accepted — 0 before the first successful init. Not the same as
// the clock that was requested: sdSetTransferSpeed(true) silently falls back
// to the normal clock when the fast re-init fails, and a transfer session
// running at 2 MHz instead of 8 MHz is worth being able to see.
uint32_t sdActiveSpiHz();

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

///////////////////////////////////////////
// TRACK WRITING (on-device course creator, plan 0002 §5)
///////////////////////////////////////////

// Why a course write failed, so the creator can say something more useful
// than "error" on a 128x64 screen.
enum SdCourseWriteResult : uint8_t {
  SD_COURSE_WRITE_OK = 0,
  SD_COURSE_WRITE_BUSY,        // another subsystem holds the card
  SD_COURSE_WRITE_NO_TRACK,    // append target missing or unparseable
  SD_COURSE_WRITE_TOO_BIG,     // the course would not fit JSON_BUFFER_SIZE
  SD_COURSE_WRITE_IO,          // open/write/rename failed
  SD_COURSE_WRITE_EXISTS,      // a track file of that name is already there
};

// One walked course, ready to be written.
struct CreatedCourseWrite {
  const course_creator::State* course = nullptr;
  bool newTrack = false;      // create a track file vs append to an existing one
  const char* trackName = ""; // file basename, without folder or .json
  const char* shortName = ""; // new tracks only
  const char* courseName = "";
  const char* dateCreated = ""; // sprint only; "" on circuit courses
};

// Write a freshly-walked course to the card.
//
// A NEW track becomes /TRACKS/<name>.json (or /TRACKS/SPRINT/<name>.json)
// holding exactly this course. An APPEND parses the existing file, adds the
// course to its "courses" array, and rewrites it — via a temp file and a
// rename, so a power loss mid-write cannot leave a half-written track file
// where a working one used to be.
//
// `dropOldest` (plan 0005) removes that many existing courses before the
// append, best candidate first per `course_prune::dropOrder`. 0 keeps every
// course, which is the behaviour every caller had before pruning existed.
// Ignored when creating a new track — there is nothing there to drop.
//
// Takes the SD mutex itself; the caller must not hold it.
SdCourseWriteResult sdSaveCreatedCourse(const CreatedCourseWrite& req,
                                        uint8_t dropOldest = 0);

// What it would take to fit one more course into a full sprint track.
struct SdSprintPrunePlan {
  // How many existing courses have to go. 0 when it already fits.
  uint8_t dropCount = 0;
  // True when at least one of them still carries the name the DEVICE gave it,
  // so it has never been through the webapp and this card may be the only
  // place it exists. That is the difference between doing it quietly and
  // asking first.
  bool needsConfirm = false;
  // False when even dropping everything droppable would not make room — the
  // one course being saved is simply too big for the buffer.
  bool possible = false;
};

// Work out what `sdSaveCreatedCourse` would have to drop, WITHOUT touching the
// card. Reads the track file and discards its own working copy.
//
// Only meaningful for an append to an existing sprint track; a new track has
// nothing to prune. Takes the SD mutex itself.
SdCourseWriteResult sdPlanSprintPrune(const CreatedCourseWrite& req,
                                      SdSprintPrunePlan& out);
