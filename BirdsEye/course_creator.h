#pragma once

#include <stddef.h>
#include <stdint.h>

///////////////////////////////////////////
// ON-DEVICE COURSE CREATOR — MODEL & NAVIGATION (pure, host-tested)
//
// Sprint entrants walk the course before an event (autocross re-lays the
// cones every time), so the device itself has to be able to create a
// course: stand at each cone, capture a GPS position. Plan 0002 §5.
//
// HARD RULE — no text entry on-device, ever. Every name is generated
// from the GPS clock and renamed later in the webapp, which is why
// `generatedName()` below is the only naming path.
//
// Three nested screens (plan 0002 §5 steps 3-5):
//   LINE MENU    one row per timing line, plus Save / Cancel
//   LINE DETAIL  Point A / Point B / Save / Back for one line
//   POINT        "Save current pos" — a timed averaging hold
// preceded by the track prompt (here / new) and the type picker.
//
// This unit owns the MODEL — which rows exist, what is captured, whether
// the course may be saved, and the point-averaging math. Navigation
// *input* stays with the sketch's existing menu machinery
// (`menuSelectionIndex` / `menuLimit`), which asks this unit for the row
// count and hands back the chosen index. Rendering and all SD access stay
// in the sketch.
//
// No Arduino headers — compiled into both the firmware and the host test
// harness (tests/course_creator_test.cpp).
///////////////////////////////////////////

namespace course_creator {

///////////////////////////////////////////
// NAMING
//
// Generated names are "N" + YYMMDD + "_" + HHMM, e.g. "N260803_1432":
// 12 characters, so it fits the track browser's 13-char window
// (MAX_LOCATION_LENGTH) WITHOUT truncation. Plan 0002 §5 originally
// proposed a literal `NEWTRACK_` / `NEWCOURSE_` prefix and flagged that
// the browser would truncate it — every same-day creation would render
// identically on-device, which defeats picking the right one. Favouring
// the timestamp over the prefix is that note resolved: unique to the
// minute, sorts chronologically, and still obviously machine-generated.
///////////////////////////////////////////

// "N" + 6 + "_" + 4 + NUL.
constexpr size_t kNameSize = 13;
// Short name for a generated track: MMDDHHMM — exactly the 8 characters
// the webapp's Track.shortName budget allows, and the key its device-sync
// merge uses, so two tracks walked on the same day cannot collide.
constexpr size_t kShortNameSize = 9;
// "20YY-MM-DDTHH:MM" + NUL — the sortable stamp sprint course selection
// compares byte-wise (see sprint_select.h).
constexpr size_t kDateCreatedSize = 17;

///////////////////////////////////////////
// POINT CAPTURE
//
// "Save current pos" averages instead of snapshotting (plan 0002 §5,
// decided): the user is standing at the cone anyway, so a hold costs
// nothing and buys a big accuracy win over a single fix.
///////////////////////////////////////////

// How long the averaging hold runs.
constexpr uint32_t kCaptureHoldMs = 3000;
// Fixes below this and the hold FAILS rather than returning a mean of
// two samples — at 25 Hz a healthy 3 s hold collects ~75.
constexpr uint16_t kCaptureMinFixes = 8;
// Fixes worse than this are dropped on the floor; a point built from
// them would be a cone-width off.
constexpr float kCaptureMaxHAccM = 10.0f;
// Above this the renderer warns but the fix still counts.
constexpr float kCaptureWarnHAccM = 5.0f;

///////////////////////////////////////////
// MODEL
///////////////////////////////////////////

enum class CourseKind : uint8_t {
  kCircuit = 0,  // one line is both start and finish
  kSprint = 1,   // start line + a separate finish line
};

// Timing lines, in the order they are stored in the track JSON. Sprint
// uses the same sector_2 / sector_3 slots for its optional splits — the
// firmware's SprintTimer reads them from exactly those fields.
enum class LineId : uint8_t {
  kStart = 0,
  kSector2 = 1,
  kSector3 = 2,
  kFinish = 3,  // sprint only
};
constexpr uint8_t kLineCount = 4;

// A timing line under construction. Both endpoints must be captured
// before the line counts as done.
struct Line {
  double aLat = 0.0;
  double aLon = 0.0;
  double bLat = 0.0;
  double bLon = 0.0;
  bool hasA = false;
  bool hasB = false;
};

inline bool lineDone(const Line& l) { return l.hasA && l.hasB; }
inline bool lineEmpty(const Line& l) { return !l.hasA && !l.hasB; }

// Averaging accumulator for one point.
struct Capture {
  double latSum = 0.0;
  double lonSum = 0.0;
  uint16_t fixes = 0;
  uint16_t rejected = 0;   // fixes dropped for poor accuracy (renderer hint)
  uint32_t startedMs = 0;
  bool active = false;
};

enum class CaptureResult : uint8_t {
  kIdle,     // no hold running
  kRunning,  // still collecting
  kDone,     // hold complete with enough fixes — commit it
  kFailed,   // hold elapsed but too few usable fixes — tell the user to retry
};

enum class Screen : uint8_t {
  kTrackPrompt,   // "Are you at X?"  Here / New Track
  kTypeSelect,    // Circuit / Sprint
  kLineMenu,      // one row per line + Save + Cancel
  kLineDetail,    // Point A / Point B / Save / Back
  kPointCapture,  // Save current pos / Back
};

// Row identities. The sketch renders and navigates by these rather than
// by raw indices, so inserting a row can't silently re-map an action.
enum class Row : uint8_t {
  kTrackHere,     // use the detected track
  kTrackNew,      // start a new track file
  kTypeCircuit,
  kTypeSprint,
  kLine,          // carries a LineId
  kSave,          // commit the course (line menu)
  kCancel,        // discard everything
  kPointA,
  kPointB,
  kLineSave,      // commit the scratch line back into the course
  kLineBack,      // discard the scratch line
  kCaptureNow,    // start the averaging hold
  kCaptureBack,
};

struct RowRef {
  Row row = Row::kCancel;
  LineId line = LineId::kStart;  // meaningful only when row == kLine
};

// What the sketch must DO after a selection. Screen changes are handled
// inside step()/select(); these are the side effects only.
enum class Action : uint8_t {
  kNone,
  kBeginCapture,  // start the averaging hold (GPS must be feeding fixes)
  kSaveCourse,    // write the course to SD
  kExit,          // discard and leave the creator
};

// Why Save is refused. Surfaced on the line menu so the row can say what
// is missing instead of just failing silently.
enum class SaveBlock : uint8_t {
  kNone,
  kStartMissing,   // start (or start/finish) line incomplete
  kFinishMissing,  // sprint: no finish line, so the run cannot be timed
  kSectorPair,     // circuit: sectors are all-or-nothing (see saveBlocked)
  kSplitOrder,     // sprint: sector 3 captured without sector 2
};

struct State {
  Screen screen = Screen::kTrackPrompt;
  CourseKind kind = CourseKind::kCircuit;
  bool newTrack = false;      // true = write a new track file
  bool trackChoiceOffered = true;  // false when no track was detected nearby

  Line lines[kLineCount];     // committed lines
  Line scratch;               // the line currently being edited
  LineId editing = LineId::kStart;
  bool editingPointB = false;

  Capture capture;
  bool captureFailed = false;  // last hold ended with too few fixes
};

///////////////////////////////////////////
// LIFECYCLE
///////////////////////////////////////////

// Enter the creator. `trackDetected` false means no known track is within
// range, so the prompt has nothing to offer and the flow starts on the
// type picker with newTrack already true.
void begin(State& s, bool trackDetected);

///////////////////////////////////////////
// ROWS — what the current screen shows
///////////////////////////////////////////

// Number of selectable rows on the current screen (the sketch's menuLimit).
uint8_t rowCount(const State& s);

// The row at `index` on the current screen. Out-of-range indices clamp to
// the last row rather than returning garbage — a stale menuSelectionIndex
// left over from a bigger menu must not be able to fire the wrong action.
RowRef rowAt(const State& s, uint8_t index);

// Human label for a timing line. Circuit's start line is also its finish,
// and says so; sprint's is just the start.
const char* lineLabel(LineId line, CourseKind kind);

// True when the course cannot be timed without this line.
bool lineRequired(LineId line, CourseKind kind);

// The committed line (not the scratch copy).
const Line& lineOf(const State& s, LineId line);

///////////////////////////////////////////
// VALIDATION
///////////////////////////////////////////

// Why the course may not be saved yet, or kNone when it is ready.
//
// Beyond the obvious required lines, two rules keep device-created courses
// loadable by the webapp's editor:
//   - CIRCUIT sectors are all-or-nothing. The webapp's validator accepts
//     zero sectors or exactly three majors (start/finish + two); a course
//     with only sector 2 would load but could never be saved again there.
//   - SPRINT splits fill in order. The webapp maps splits POSITIONALLY
//     into sector_2 / sector_3, so a lone sector 3 would come back as a
//     sector 2 on the next sync — a silent edit nobody made.
SaveBlock saveBlocked(const State& s);

inline bool canSave(const State& s) { return saveBlocked(s) == SaveBlock::kNone; }

///////////////////////////////////////////
// NAVIGATION
///////////////////////////////////////////

// Act on the row the user selected. Mutates `s` (screen, scratch, choice)
// and returns the side effect the sketch must perform.
Action select(State& s, uint8_t index);

// Abandon the averaging hold (Back on the capture screen).
void captureCancel(State& s);

///////////////////////////////////////////
// POINT CAPTURE
///////////////////////////////////////////

void captureBegin(State& s, uint32_t nowMs);

// Feed one GPS fix. Returns false when it was rejected for accuracy.
// Safe to call when no hold is running (returns false, changes nothing).
bool captureAddFix(State& s, double lat, double lon, float hAccM, uint32_t nowMs);

// Where the hold stands. kDone/kFailed are terminal — the caller commits
// or reports and then the hold is over.
CaptureResult capturePoll(const State& s, uint32_t nowMs);

// 0-100 progress for the renderer, by elapsed time.
uint8_t capturePercent(const State& s, uint32_t nowMs);

// Commit a completed hold into the scratch line's A or B endpoint and end
// the hold. Returns false (and captureFailed is set) when the hold did not
// gather enough fixes.
bool captureCommit(State& s, uint32_t nowMs);

///////////////////////////////////////////
// NAME GENERATION
///////////////////////////////////////////

// "N260803_1432" — the track filename and course name. `year` is the
// 2-digit form the GPS layer carries (25 == 2025).
bool generatedName(char* out, size_t outSize,
                   uint16_t year, uint8_t month, uint8_t day,
                   uint8_t hour, uint8_t minute);

// "08031432" — the 8-char webapp short name for a generated track.
bool generatedShortName(char* out, size_t outSize,
                        uint8_t month, uint8_t day,
                        uint8_t hour, uint8_t minute);

// "2026-08-03T14:32" — the sortable stamp sprint selection compares.
bool generatedDateCreated(char* out, size_t outSize,
                          uint16_t year, uint8_t month, uint8_t day,
                          uint8_t hour, uint8_t minute);

}  // namespace course_creator
