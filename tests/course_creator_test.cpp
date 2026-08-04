#include "doctest.h"

#include <string.h>

#include <string>

#include "course_creator.h"

using namespace course_creator;

namespace {

// Drive the flow up to the line menu for a given course type.
State atLineMenu(CourseKind kind, bool trackDetected = true) {
  State s;
  begin(s, trackDetected);
  if (trackDetected) select(s, 0);            // "Here"
  select(s, kind == CourseKind::kSprint ? 1 : 0);  // type
  return s;
}

// Index of a line's row on the line menu.
uint8_t rowOf(const State& s, LineId want) {
  for (uint8_t i = 0; i < rowCount(s); i++) {
    const RowRef r = rowAt(s, i);
    if (r.row == Row::kLine && r.line == want) return i;
  }
  return 0;
}

// Capture both endpoints of a line, the way the UI would.
void captureLine(State& s, LineId line, double lat, double lon) {
  select(s, rowOf(s, line));  // open the line detail
  uint32_t t = 1000;
  for (uint8_t point = 0; point < 2; point++) {
    select(s, point);  // Point A / Point B
    captureBegin(s, t);
    for (uint16_t i = 0; i < kCaptureMinFixes; i++) {
      captureAddFix(s, lat + point * 0.0001, lon, 1.0f, t + i);
    }
    t += kCaptureHoldMs;
    REQUIRE(captureCommit(s, t));
  }
  select(s, 2);  // Save (commit the scratch line)
}

}  // namespace

TEST_CASE("begin offers the track prompt only when a track is nearby") {
  State s;
  begin(s, true);
  CHECK(s.screen == Screen::kTrackPrompt);
  CHECK(s.trackChoiceOffered);
  CHECK_FALSE(s.newTrack);

  // Nothing in range: there is no choice to make, so don't ask one.
  begin(s, false);
  CHECK(s.screen == Screen::kTypeSelect);
  CHECK(s.newTrack);
}

TEST_CASE("the track prompt records which track the course lands in") {
  State s;
  begin(s, true);
  select(s, 0);
  CHECK_FALSE(s.newTrack);

  begin(s, true);
  select(s, 1);
  CHECK(s.newTrack);
  CHECK(s.screen == Screen::kTypeSelect);
}

TEST_CASE("sprint gets a finish row, circuit does not") {
  const State circuit = atLineMenu(CourseKind::kCircuit);
  const State sprint = atLineMenu(CourseKind::kSprint);

  // 3 lines + Save + Cancel vs 4 lines + Save + Cancel.
  CHECK(rowCount(circuit) == 5);
  CHECK(rowCount(sprint) == 6);

  bool circuitHasFinish = false;
  for (uint8_t i = 0; i < rowCount(circuit); i++) {
    const RowRef r = rowAt(circuit, i);
    if (r.row == Row::kLine && r.line == LineId::kFinish) circuitHasFinish = true;
  }
  CHECK_FALSE(circuitHasFinish);

  CHECK(rowAt(sprint, 3).row == Row::kLine);
  CHECK(rowAt(sprint, 3).line == LineId::kFinish);
  CHECK(rowAt(sprint, 4).row == Row::kSave);
  CHECK(rowAt(sprint, 5).row == Row::kCancel);
}

TEST_CASE("an out-of-range row clamps instead of wrapping") {
  const State s = atLineMenu(CourseKind::kCircuit);
  // A menuSelectionIndex left over from a longer menu must not be able to
  // fire an action it was never pointing at — Cancel is the last row, and
  // landing there is at worst a discard, never a bad save.
  CHECK(rowAt(s, 99).row == Row::kCancel);
}

TEST_CASE("the start line's label says whether it is also the finish") {
  CHECK(std::string(lineLabel(LineId::kStart, CourseKind::kCircuit)) == "Start/Fin");
  CHECK(std::string(lineLabel(LineId::kStart, CourseKind::kSprint)) == "Start");
}

TEST_CASE("required lines depend on the course type") {
  CHECK(lineRequired(LineId::kStart, CourseKind::kCircuit));
  CHECK(lineRequired(LineId::kStart, CourseKind::kSprint));
  CHECK(lineRequired(LineId::kFinish, CourseKind::kSprint));
  CHECK_FALSE(lineRequired(LineId::kFinish, CourseKind::kCircuit));
  CHECK_FALSE(lineRequired(LineId::kSector2, CourseKind::kSprint));
  CHECK_FALSE(lineRequired(LineId::kSector3, CourseKind::kCircuit));
}

// ─── Validation ─────────────────────────────────────────────────────────────

TEST_CASE("a circuit course needs its start/finish line") {
  State s = atLineMenu(CourseKind::kCircuit);
  CHECK(saveBlocked(s) == SaveBlock::kStartMissing);
  CHECK_FALSE(canSave(s));

  captureLine(s, LineId::kStart, 35.1, -97.1);
  CHECK(saveBlocked(s) == SaveBlock::kNone);
  CHECK(canSave(s));
}

TEST_CASE("circuit sectors are all-or-nothing") {
  State s = atLineMenu(CourseKind::kCircuit);
  captureLine(s, LineId::kStart, 35.1, -97.1);

  // One sector alone would load in the webapp but could never be saved
  // there again — its validator wants zero sectors or exactly three majors.
  captureLine(s, LineId::kSector2, 35.2, -97.2);
  CHECK(saveBlocked(s) == SaveBlock::kSectorPair);

  captureLine(s, LineId::kSector3, 35.3, -97.3);
  CHECK(saveBlocked(s) == SaveBlock::kNone);
}

TEST_CASE("a half-captured circuit sector blocks the save too") {
  State s = atLineMenu(CourseKind::kCircuit);
  captureLine(s, LineId::kStart, 35.1, -97.1);
  captureLine(s, LineId::kSector2, 35.2, -97.2);
  captureLine(s, LineId::kSector3, 35.3, -97.3);
  REQUIRE(canSave(s));

  // Re-open sector 3, capture only point A, and save the line: the user
  // believes they placed a line, so silently dropping it would be worse
  // than refusing.
  select(s, rowOf(s, LineId::kSector3));
  s.scratch = Line{};
  select(s, 0);  // Point A
  captureBegin(s, 5000);
  for (uint16_t i = 0; i < kCaptureMinFixes; i++) captureAddFix(s, 35.4, -97.4, 1.0f, 5000 + i);
  REQUIRE(captureCommit(s, 5000 + kCaptureHoldMs));
  select(s, 2);  // Save the line

  CHECK(saveBlocked(s) == SaveBlock::kSectorPair);
}

TEST_CASE("a sprint course needs a separate finish line") {
  State s = atLineMenu(CourseKind::kSprint);
  captureLine(s, LineId::kStart, 35.1, -97.1);
  CHECK(saveBlocked(s) == SaveBlock::kFinishMissing);

  captureLine(s, LineId::kFinish, 35.5, -97.5);
  CHECK(saveBlocked(s) == SaveBlock::kNone);
}

TEST_CASE("sprint splits must be captured in order") {
  State s = atLineMenu(CourseKind::kSprint);
  captureLine(s, LineId::kStart, 35.1, -97.1);
  captureLine(s, LineId::kFinish, 35.5, -97.5);
  REQUIRE(canSave(s));

  // The webapp stores splits as an ordered list and re-exports them
  // positionally, so a lone sector 3 comes back as a sector 2.
  captureLine(s, LineId::kSector3, 35.3, -97.3);
  CHECK(saveBlocked(s) == SaveBlock::kSplitOrder);

  captureLine(s, LineId::kSector2, 35.2, -97.2);
  CHECK(saveBlocked(s) == SaveBlock::kNone);
}

TEST_CASE("one sprint split is legal on its own") {
  State s = atLineMenu(CourseKind::kSprint);
  captureLine(s, LineId::kStart, 35.1, -97.1);
  captureLine(s, LineId::kFinish, 35.5, -97.5);
  captureLine(s, LineId::kSector2, 35.2, -97.2);
  CHECK(saveBlocked(s) == SaveBlock::kNone);
}

// ─── Navigation ─────────────────────────────────────────────────────────────

TEST_CASE("Save is refused while the course is incomplete") {
  State s = atLineMenu(CourseKind::kSprint);
  const uint8_t saveRow = 4;
  REQUIRE(rowAt(s, saveRow).row == Row::kSave);
  // Nothing captured — pressing Save must do nothing at all rather than
  // write a course the device would later load and fail to time.
  CHECK(select(s, saveRow) == Action::kNone);

  captureLine(s, LineId::kStart, 35.1, -97.1);
  captureLine(s, LineId::kFinish, 35.5, -97.5);
  CHECK(select(s, saveRow) == Action::kSaveCourse);
}

TEST_CASE("Cancel exits the creator") {
  State s = atLineMenu(CourseKind::kCircuit);
  CHECK(select(s, 4) == Action::kExit);
}

TEST_CASE("Back on a line discards the scratch edits") {
  State s = atLineMenu(CourseKind::kCircuit);
  captureLine(s, LineId::kStart, 35.1, -97.1);
  const double committed = lineOf(s, LineId::kStart).aLat;

  select(s, rowOf(s, LineId::kStart));
  select(s, 0);  // Point A
  captureBegin(s, 9000);
  for (uint16_t i = 0; i < kCaptureMinFixes; i++) captureAddFix(s, 40.0, -100.0, 1.0f, 9000 + i);
  REQUIRE(captureCommit(s, 9000 + kCaptureHoldMs));
  select(s, 3);  // Back

  CHECK(s.screen == Screen::kLineMenu);
  CHECK(lineOf(s, LineId::kStart).aLat == doctest::Approx(committed));
}

TEST_CASE("opening a line seeds the scratch copy from what is committed") {
  State s = atLineMenu(CourseKind::kCircuit);
  captureLine(s, LineId::kStart, 35.1, -97.1);

  select(s, rowOf(s, LineId::kStart));
  // Re-opening a done line shows both points already captured, so a user
  // can re-walk just one endpoint.
  CHECK(s.scratch.hasA);
  CHECK(s.scratch.hasB);
  CHECK(s.scratch.aLat == doctest::Approx(35.1));
}

TEST_CASE("the capture screen targets the point that opened it") {
  State s = atLineMenu(CourseKind::kSprint);
  select(s, rowOf(s, LineId::kStart));

  select(s, 0);
  CHECK(s.screen == Screen::kPointCapture);
  CHECK_FALSE(s.editingPointB);

  select(s, 1);  // Back
  select(s, 1);  // Point B
  CHECK(s.editingPointB);
  CHECK(select(s, 0) == Action::kBeginCapture);
}

// ─── Point capture ──────────────────────────────────────────────────────────

TEST_CASE("a capture averages the fixes it collected") {
  State s = atLineMenu(CourseKind::kCircuit);
  select(s, rowOf(s, LineId::kStart));
  select(s, 0);

  captureBegin(s, 1000);
  // Symmetric spread around 35.0 / -97.0 — the mean is the centre.
  captureAddFix(s, 34.9, -97.1, 1.0f, 1010);
  captureAddFix(s, 35.1, -96.9, 1.0f, 1020);
  for (uint16_t i = 0; i < kCaptureMinFixes; i++) {
    captureAddFix(s, 35.0, -97.0, 1.0f, 1100 + i);
  }

  CHECK(capturePoll(s, 2000) == CaptureResult::kRunning);
  CHECK(capturePoll(s, 1000 + kCaptureHoldMs) == CaptureResult::kDone);
  REQUIRE(captureCommit(s, 1000 + kCaptureHoldMs));

  CHECK(s.scratch.hasA);
  CHECK(s.scratch.aLat == doctest::Approx(35.0));
  CHECK(s.scratch.aLon == doctest::Approx(-97.0));
  CHECK(s.screen == Screen::kLineDetail);
}

TEST_CASE("fixes worse than the accuracy limit are dropped") {
  State s;
  begin(s, false);
  captureBegin(s, 0);

  CHECK_FALSE(captureAddFix(s, 35.0, -97.0, kCaptureMaxHAccM + 0.1f, 10));
  CHECK_FALSE(captureAddFix(s, 35.0, -97.0, 0.0f, 20));  // no accuracy reported
  CHECK(s.capture.fixes == 0);
  CHECK(s.capture.rejected == 2);

  CHECK(captureAddFix(s, 35.0, -97.0, kCaptureMaxHAccM, 30));
  CHECK(s.capture.fixes == 1);
}

TEST_CASE("a hold that gathers too few fixes fails instead of averaging noise") {
  State s = atLineMenu(CourseKind::kCircuit);
  select(s, rowOf(s, LineId::kStart));
  select(s, 0);

  captureBegin(s, 0);
  for (uint16_t i = 0; i < kCaptureMinFixes - 1; i++) {
    captureAddFix(s, 35.0, -97.0, 1.0f, i);
  }

  CHECK(capturePoll(s, kCaptureHoldMs) == CaptureResult::kFailed);
  CHECK_FALSE(captureCommit(s, kCaptureHoldMs));
  CHECK(s.captureFailed);
  CHECK_FALSE(s.scratch.hasA);
  // The hold is over, so the screen can offer a retry rather than sitting
  // pinned at 100%.
  CHECK_FALSE(s.capture.active);
}

TEST_CASE("fixes arriving after the window do not shift the average") {
  State s;
  begin(s, false);
  captureBegin(s, 0);
  for (uint16_t i = 0; i < kCaptureMinFixes; i++) captureAddFix(s, 35.0, -97.0, 1.0f, i);

  CHECK_FALSE(captureAddFix(s, 80.0, 10.0, 1.0f, kCaptureHoldMs));
  CHECK(s.capture.fixes == kCaptureMinFixes);
}

TEST_CASE("capture progress tracks elapsed time") {
  State s;
  begin(s, false);
  CHECK(capturePercent(s, 0) == 0);  // nothing running

  captureBegin(s, 1000);
  CHECK(capturePercent(s, 1000) == 0);
  CHECK(capturePercent(s, 1000 + kCaptureHoldMs / 2) == 50);
  CHECK(capturePercent(s, 1000 + kCaptureHoldMs) == 100);
  CHECK(capturePercent(s, 1000 + kCaptureHoldMs * 2) == 100);
}

TEST_CASE("adding a fix with no hold running changes nothing") {
  State s;
  begin(s, false);
  CHECK_FALSE(captureAddFix(s, 35.0, -97.0, 1.0f, 0));
  CHECK(capturePoll(s, 0) == CaptureResult::kIdle);
}

TEST_CASE("Back on the capture screen abandons the hold") {
  State s = atLineMenu(CourseKind::kCircuit);
  select(s, rowOf(s, LineId::kStart));
  select(s, 0);
  captureBegin(s, 0);
  captureAddFix(s, 35.0, -97.0, 1.0f, 10);

  select(s, 1);  // Back
  CHECK(s.screen == Screen::kLineDetail);
  CHECK_FALSE(s.capture.active);
  CHECK(s.capture.fixes == 0);
}

// ─── Name generation ────────────────────────────────────────────────────────

TEST_CASE("generated names fit the track browser without truncation") {
  char name[kNameSize];
  REQUIRE(generatedName(name, sizeof(name), 26, 8, 3, 14, 32));
  CHECK(std::string(name) == "N260803_1432");
  // MAX_LOCATION_LENGTH is 13 including the NUL slot, which is exactly
  // what this fits — the whole reason it isn't the spec's "NEWTRACK_".
  CHECK(strlen(name) == 12);
}

TEST_CASE("generated names zero-pad every field") {
  char name[kNameSize];
  REQUIRE(generatedName(name, sizeof(name), 26, 1, 2, 3, 4));
  CHECK(std::string(name) == "N260102_0304");
}

TEST_CASE("a generated short name is exactly the webapp's 8-char budget") {
  char shortName[kShortNameSize];
  REQUIRE(generatedShortName(shortName, sizeof(shortName), 8, 3, 14, 32));
  CHECK(std::string(shortName) == "08031432");
  // The webapp's device-sync merge keys on (kind, shortName), so two
  // tracks walked the same day must not collide here.
  CHECK(strlen(shortName) == 8);
}

TEST_CASE("date_created is the sortable stamp sprint selection compares") {
  char stamp[kDateCreatedSize];
  REQUIRE(generatedDateCreated(stamp, sizeof(stamp), 26, 8, 3, 14, 32));
  CHECK(std::string(stamp) == "2026-08-03T14:32");

  // Lexicographic order must match chronological order — that is the whole
  // contract sprint_select relies on.
  char earlier[kDateCreatedSize];
  REQUIRE(generatedDateCreated(earlier, sizeof(earlier), 26, 8, 3, 9, 5));
  CHECK(std::string(earlier) < std::string(stamp));
}

TEST_CASE("name generation refuses a buffer that is too small") {
  char tiny[4];
  CHECK_FALSE(generatedName(tiny, sizeof(tiny), 26, 8, 3, 14, 32));
  CHECK_FALSE(generatedShortName(tiny, sizeof(tiny), 8, 3, 14, 32));
  CHECK_FALSE(generatedDateCreated(tiny, sizeof(tiny), 26, 8, 3, 14, 32));
  CHECK_FALSE(generatedName(nullptr, 32, 26, 8, 3, 14, 32));
}
