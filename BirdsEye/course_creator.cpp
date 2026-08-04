#include "course_creator.h"

#include <stdio.h>
#include <string.h>

namespace course_creator {

namespace {

// Rows shown on the line menu, in order. Sprint appends the finish line;
// everything before it is shared, so the two layouts can't drift apart.
constexpr LineId kCircuitLines[] = {LineId::kStart, LineId::kSector2, LineId::kSector3};
constexpr uint8_t kCircuitLineRows = 3;
constexpr uint8_t kSprintLineRows = 4;  // + kFinish

uint8_t lineRowsFor(CourseKind kind) {
  return kind == CourseKind::kSprint ? kSprintLineRows : kCircuitLineRows;
}

LineId lineRowAt(CourseKind kind, uint8_t i) {
  if (kind == CourseKind::kSprint && i == kCircuitLineRows) return LineId::kFinish;
  if (i >= kCircuitLineRows) i = kCircuitLineRows - 1;
  return kCircuitLines[i];
}

Line& mutableLine(State& s, LineId line) {
  return s.lines[static_cast<uint8_t>(line)];
}

// Zero-padded two-digit write. `out` must have room for 2 chars.
void writeTwo(char* out, unsigned value) {
  out[0] = static_cast<char>('0' + (value / 10) % 10);
  out[1] = static_cast<char>('0' + value % 10);
}

}  // namespace

///////////////////////////////////////////
// LIFECYCLE
///////////////////////////////////////////

void begin(State& s, bool trackDetected) {
  s = State{};
  s.trackChoiceOffered = trackDetected;
  if (trackDetected) {
    s.screen = Screen::kTrackPrompt;
  } else {
    // Nothing nearby to attach to — there is no choice to offer, so skip
    // straight to the type picker with a new track already implied.
    s.newTrack = true;
    s.screen = Screen::kTypeSelect;
  }
}

///////////////////////////////////////////
// ROWS
///////////////////////////////////////////

uint8_t rowCount(const State& s) {
  switch (s.screen) {
    case Screen::kTrackPrompt:  return 2;                       // Here, New Track
    case Screen::kTypeSelect:   return 2;                       // Circuit, Sprint
    case Screen::kLineMenu:     return lineRowsFor(s.kind) + 2; // + Save, Cancel
    case Screen::kLineDetail:   return 4;                       // A, B, Save, Back
    case Screen::kPointCapture: return 2;                       // Capture, Back
  }
  return 1;
}

RowRef rowAt(const State& s, uint8_t index) {
  const uint8_t count = rowCount(s);
  // A stale menuSelectionIndex left over from a larger menu must never be
  // able to fire an action it wasn't pointing at — clamp, don't wrap.
  if (index >= count) index = static_cast<uint8_t>(count - 1);

  RowRef ref;
  switch (s.screen) {
    case Screen::kTrackPrompt:
      ref.row = (index == 0) ? Row::kTrackHere : Row::kTrackNew;
      return ref;

    case Screen::kTypeSelect:
      ref.row = (index == 0) ? Row::kTypeCircuit : Row::kTypeSprint;
      return ref;

    case Screen::kLineMenu: {
      const uint8_t lineRows = lineRowsFor(s.kind);
      if (index < lineRows) {
        ref.row = Row::kLine;
        ref.line = lineRowAt(s.kind, index);
      } else if (index == lineRows) {
        ref.row = Row::kSave;
      } else {
        ref.row = Row::kCancel;
      }
      return ref;
    }

    case Screen::kLineDetail:
      ref.line = s.editing;
      if (index == 0)      ref.row = Row::kPointA;
      else if (index == 1) ref.row = Row::kPointB;
      else if (index == 2) ref.row = Row::kLineSave;
      else                 ref.row = Row::kLineBack;
      return ref;

    case Screen::kPointCapture:
      ref.line = s.editing;
      ref.row = (index == 0) ? Row::kCaptureNow : Row::kCaptureBack;
      return ref;
  }
  return ref;
}

const char* lineLabel(LineId line, CourseKind kind) {
  switch (line) {
    case LineId::kStart:
      // On a circuit the one line is both, and saying so is the difference
      // between the user walking one line or two.
      return kind == CourseKind::kSprint ? "Start" : "Start/Fin";
    case LineId::kSector2: return "Sector 2";
    case LineId::kSector3: return "Sector 3";
    case LineId::kFinish:  return "Finish";
  }
  return "";
}

bool lineRequired(LineId line, CourseKind kind) {
  if (line == LineId::kStart) return true;
  if (line == LineId::kFinish) return kind == CourseKind::kSprint;
  return false;
}

const Line& lineOf(const State& s, LineId line) {
  return s.lines[static_cast<uint8_t>(line)];
}

///////////////////////////////////////////
// VALIDATION
///////////////////////////////////////////

SaveBlock saveBlocked(const State& s) {
  const Line& start = lineOf(s, LineId::kStart);
  if (!lineDone(start)) return SaveBlock::kStartMissing;

  const Line& s2 = lineOf(s, LineId::kSector2);
  const Line& s3 = lineOf(s, LineId::kSector3);

  if (s.kind == CourseKind::kSprint) {
    if (!lineDone(lineOf(s, LineId::kFinish))) return SaveBlock::kFinishMissing;
    // Splits are optional and independent in principle, but the webapp
    // stores them as an ORDERED list and re-exports them positionally, so
    // a lone sector 3 would come back as a sector 2 after one sync round
    // trip. Capture them in order and that can't happen.
    if (lineDone(s3) && !lineDone(s2)) return SaveBlock::kSplitOrder;
    return SaveBlock::kNone;
  }

  // Circuit sectors are all-or-nothing: the webapp's course validator
  // accepts zero sectors or exactly three majors (start/finish + two).
  // Writing just one would produce a course the app can load but can
  // never save again.
  const bool has2 = lineDone(s2);
  const bool has3 = lineDone(s3);
  if (has2 != has3) return SaveBlock::kSectorPair;

  // A half-captured sector (one endpoint) is the same problem in a
  // different disguise — it never became a line but the user thinks it did.
  if ((!lineEmpty(s2) && !has2) || (!lineEmpty(s3) && !has3)) {
    return SaveBlock::kSectorPair;
  }
  return SaveBlock::kNone;
}

///////////////////////////////////////////
// NAVIGATION
///////////////////////////////////////////

Action select(State& s, uint8_t index) {
  const RowRef ref = rowAt(s, index);

  switch (ref.row) {
    case Row::kTrackHere:
      s.newTrack = false;
      s.screen = Screen::kTypeSelect;
      return Action::kNone;

    case Row::kTrackNew:
      s.newTrack = true;
      s.screen = Screen::kTypeSelect;
      return Action::kNone;

    case Row::kTypeCircuit:
    case Row::kTypeSprint:
      s.kind = (ref.row == Row::kTypeSprint) ? CourseKind::kSprint : CourseKind::kCircuit;
      s.screen = Screen::kLineMenu;
      return Action::kNone;

    case Row::kLine:
      // Editing works on a scratch copy so Back is a real undo — re-capture
      // point A, decide you stood in the wrong place, and walk away.
      s.editing = ref.line;
      s.scratch = lineOf(s, ref.line);
      s.screen = Screen::kLineDetail;
      return Action::kNone;

    case Row::kSave:
      // Refused rather than partially written: an unsaveable course on the
      // card is worse than none, because the device would load it.
      if (!canSave(s)) return Action::kNone;
      return Action::kSaveCourse;

    case Row::kCancel:
      return Action::kExit;

    case Row::kPointA:
    case Row::kPointB:
      s.editingPointB = (ref.row == Row::kPointB);
      s.captureFailed = false;
      s.capture = Capture{};
      s.screen = Screen::kPointCapture;
      return Action::kNone;

    case Row::kLineSave:
      mutableLine(s, s.editing) = s.scratch;
      s.screen = Screen::kLineMenu;
      return Action::kNone;

    case Row::kLineBack:
      s.scratch = Line{};
      s.screen = Screen::kLineMenu;
      return Action::kNone;

    case Row::kCaptureNow:
      return Action::kBeginCapture;

    case Row::kCaptureBack:
      captureCancel(s);
      s.screen = Screen::kLineDetail;
      return Action::kNone;
  }
  return Action::kNone;
}

void captureCancel(State& s) {
  s.capture = Capture{};
  s.captureFailed = false;
}

///////////////////////////////////////////
// POINT CAPTURE
///////////////////////////////////////////

void captureBegin(State& s, uint32_t nowMs) {
  s.capture = Capture{};
  s.capture.active = true;
  s.capture.startedMs = nowMs;
  s.captureFailed = false;
}

bool captureAddFix(State& s, double lat, double lon, float hAccM, uint32_t nowMs) {
  if (!s.capture.active) return false;
  // Past the window the hold is decided; late fixes must not shift a mean
  // the user has already been shown.
  if (nowMs - s.capture.startedMs >= kCaptureHoldMs) return false;
  // A fix this loose puts the point most of a cone away. Counting it would
  // quietly poison an average the whole point of which is precision.
  if (!(hAccM > 0.0f) || hAccM > kCaptureMaxHAccM) {
    if (s.capture.rejected < UINT16_MAX) s.capture.rejected++;
    return false;
  }
  s.capture.latSum += lat;
  s.capture.lonSum += lon;
  if (s.capture.fixes < UINT16_MAX) s.capture.fixes++;
  return true;
}

CaptureResult capturePoll(const State& s, uint32_t nowMs) {
  if (!s.capture.active) return CaptureResult::kIdle;
  if (nowMs - s.capture.startedMs < kCaptureHoldMs) return CaptureResult::kRunning;
  return s.capture.fixes >= kCaptureMinFixes ? CaptureResult::kDone : CaptureResult::kFailed;
}

uint8_t capturePercent(const State& s, uint32_t nowMs) {
  if (!s.capture.active) return 0;
  const uint32_t elapsed = nowMs - s.capture.startedMs;
  if (elapsed >= kCaptureHoldMs) return 100;
  return static_cast<uint8_t>((elapsed * 100u) / kCaptureHoldMs);
}

bool captureCommit(State& s, uint32_t nowMs) {
  if (capturePoll(s, nowMs) != CaptureResult::kDone) {
    // Ending a failed hold here (rather than leaving it armed) is what lets
    // the screen say "retry" instead of sitting at 100% forever.
    if (s.capture.active) {
      s.capture = Capture{};
      s.captureFailed = true;
    }
    return false;
  }

  const double lat = s.capture.latSum / s.capture.fixes;
  const double lon = s.capture.lonSum / s.capture.fixes;
  if (s.editingPointB) {
    s.scratch.bLat = lat;
    s.scratch.bLon = lon;
    s.scratch.hasB = true;
  } else {
    s.scratch.aLat = lat;
    s.scratch.aLon = lon;
    s.scratch.hasA = true;
  }

  s.capture = Capture{};
  s.captureFailed = false;
  s.screen = Screen::kLineDetail;
  return true;
}

///////////////////////////////////////////
// NAME GENERATION
///////////////////////////////////////////

bool generatedName(char* out, size_t outSize,
                   uint16_t year, uint8_t month, uint8_t day,
                   uint8_t hour, uint8_t minute) {
  if (out == nullptr || outSize < kNameSize) return false;
  out[0] = 'N';
  writeTwo(out + 1, year % 100u);
  writeTwo(out + 3, month);
  writeTwo(out + 5, day);
  out[7] = '_';
  writeTwo(out + 8, hour);
  writeTwo(out + 10, minute);
  out[12] = '\0';
  return true;
}

bool generatedShortName(char* out, size_t outSize,
                        uint8_t month, uint8_t day,
                        uint8_t hour, uint8_t minute) {
  if (out == nullptr || outSize < kShortNameSize) return false;
  writeTwo(out + 0, month);
  writeTwo(out + 2, day);
  writeTwo(out + 4, hour);
  writeTwo(out + 6, minute);
  out[8] = '\0';
  return true;
}

bool generatedDateCreated(char* out, size_t outSize,
                          uint16_t year, uint8_t month, uint8_t day,
                          uint8_t hour, uint8_t minute) {
  if (out == nullptr || outSize < kDateCreatedSize) return false;
  out[0] = '2';
  out[1] = '0';
  writeTwo(out + 2, year % 100u);
  out[4] = '-';
  writeTwo(out + 5, month);
  out[7] = '-';
  writeTwo(out + 8, day);
  out[10] = 'T';
  writeTwo(out + 11, hour);
  out[13] = ':';
  writeTwo(out + 14, minute);
  out[16] = '\0';
  return true;
}

}  // namespace course_creator
