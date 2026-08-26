#include <string.h>

#include <initializer_list>

#include "doctest.h"
#include "drag_tree.h"
#include "led_frame.h"

using drag_tree::Effects;
using drag_tree::Stage;

// ---------------------------------------------------------------------------
// Harness: a rig holding the live input levels; step() snapshots them,
// mirroring how dragStagingLoop() builds Inputs each loop iteration.
// ---------------------------------------------------------------------------
namespace {

struct Rig {
  drag_tree::State s;
  uint32_t now = 100000;
  bool fix = true;
  float speed = 0.0f;
  bool staged = false;
  bool runActive = false;
  int runs = 0;
  bool selectHeld = false;
  bool otherHeld = false;

  Rig() { drag_tree::begin(s, now); }

  Effects step(bool button = false) {
    drag_tree::Inputs in;
    in.nowMs = now;
    in.fix = fix;
    in.speedMph = speed;
    in.timerStaged = staged;
    in.runActive = runActive;
    in.runs = runs;
    in.buttonPressed = button;
    in.selectHeld = selectHeld;
    in.otherButtonHeld = otherHeld;
    return drag_tree::step(s, in);
  }

  // Advance in 20 ms ticks for `totalMs`; returns true if any exit fired.
  bool tick(uint32_t totalMs, uint32_t dtMs = 20) {
    bool exited = false;
    for (uint32_t t = 0; t < totalMs; t += dtMs) {
      now += dtMs;
      if (step().exitSession) exited = true;
    }
    return exited;
  }

  Stage stage() const { return s.stage; }

  // Drive a fresh rig to STAGED-on-the-tree's-terms (kPreStage entry).
  void toPreStage() {
    step(true);  // arm
    REQUIRE(stage() == Stage::kWaitStop);
    staged = true;
    now += 20;
    step();
    REQUIRE(stage() == Stage::kPreStage);
  }

  void toYellow1() {
    toPreStage();
    now += drag_tree::kPreStageHoldMs;
    step();
    REQUIRE(stage() == Stage::kYellow1);
  }

  Effects toGreen() {
    toYellow1();
    now += drag_tree::kTreeStepMs;
    step();
    REQUIRE(stage() == Stage::kYellow2);
    now += drag_tree::kTreeStepMs;
    step();
    REQUIRE(stage() == Stage::kYellow3);
    now += drag_tree::kTreeStepMs;
    Effects fx = step();
    REQUIRE(stage() == Stage::kGreen);
    return fx;
  }
};

bool sameRgb(const led_frame::Rgb& p, uint8_t r, uint8_t g, uint8_t b) {
  return p.r == r && p.g == g && p.b == b;
}
bool isOff(const led_frame::Rgb& p) { return sameRgb(p, 0, 0, 0); }
bool isRed(const led_frame::Rgb& p) { return sameRgb(p, 255, 0, 0); }
bool isGreen(const led_frame::Rgb& p) { return sameRgb(p, 0, 255, 0); }
bool isWhite(const led_frame::Rgb& p) { return sameRgb(p, 255, 255, 255); }
bool isYellow(const led_frame::Rgb& p) {
  return sameRgb(p, led_frame::kYellow.r, led_frame::kYellow.g,
                 led_frame::kYellow.b);
}

// Compare a rendered strip against a pattern string: '-' off, 'w' white,
// 'y' yellow, 'g' green, 'r' red. The user's spec is literally these
// strings, so the tests read the same as the requirement.
bool matches(Stage st, uint32_t tMs, const char* pattern) {
  led_frame::Rgb out[led_frame::kStripCount];
  drag_tree::renderStrip(st, tMs, out);
  REQUIRE(strlen(pattern) == (size_t)led_frame::kStripCount);
  for (int i = 0; i < led_frame::kStripCount; i++) {
    bool ok = false;
    switch (pattern[i]) {
      case '-': ok = isOff(out[i]); break;
      case 'w': ok = isWhite(out[i]); break;
      case 'y': ok = isYellow(out[i]); break;
      case 'g': ok = isGreen(out[i]); break;
      case 'r': ok = isRed(out[i]); break;
    }
    if (!ok) return false;
  }
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// The happy path, with exact stage-entry times
// ---------------------------------------------------------------------------

TEST_CASE("happy path: arm -> stage -> tree -> green -> run -> results") {
  Rig r;
  CHECK(r.stage() == Stage::kAwaitArm);

  Effects fx = r.step(true);
  CHECK(fx.consumedButton);
  CHECK(r.stage() == Stage::kWaitStop);

  r.staged = true;
  r.now += 20;
  r.step();
  CHECK(r.stage() == Stage::kPreStage);
  const uint32_t preStageAt = r.now;

  // One tick shy of the pre-stage hold: still white pip.
  r.now = preStageAt + drag_tree::kPreStageHoldMs - 1;
  r.step();
  CHECK(r.stage() == Stage::kPreStage);
  r.now = preStageAt + drag_tree::kPreStageHoldMs;
  r.step();
  CHECK(r.stage() == Stage::kYellow1);

  // 500 ms cadence, boundary-exact.
  const uint32_t y1At = r.now;
  r.now = y1At + drag_tree::kTreeStepMs - 1;
  r.step();
  CHECK(r.stage() == Stage::kYellow1);
  r.now = y1At + drag_tree::kTreeStepMs;
  r.step();
  CHECK(r.stage() == Stage::kYellow2);
  r.now += drag_tree::kTreeStepMs;
  r.step();
  CHECK(r.stage() == Stage::kYellow3);
  r.now += drag_tree::kTreeStepMs;
  fx = r.step();
  CHECK(r.stage() == Stage::kGreen);
  CHECK(fx.greenEdge);

  // Launch: physics runActive rises.
  r.speed = 20.0f;
  r.runActive = true;
  r.now += 200;
  fx = r.step();
  CHECK(fx.runStartEdge);
  CHECK(r.stage() == Stage::kRunning);

  // Finish: runs increments.
  r.runs = 1;
  r.runActive = false;
  r.now += 11000;
  r.step();
  CHECK(r.stage() == Stage::kResults);

  // Any button re-arms.
  fx = r.step(true);
  CHECK(fx.consumedButton);
  CHECK(r.stage() == Stage::kWaitStop);
}

// ---------------------------------------------------------------------------
// Fouls and silent resets
// ---------------------------------------------------------------------------

TEST_CASE("movement during any yellow is a red light") {
  for (int stageNum = 0; stageNum < 3; stageNum++) {
    CAPTURE(stageNum);
    Rig r;
    r.toYellow1();
    for (int i = 0; i < stageNum; i++) {
      r.now += drag_tree::kTreeStepMs;
      r.step();
    }
    r.speed = 5.0f;  // >= drag_timer::kLaunchMinMph
    r.now += 20;
    r.step();
    CHECK(r.stage() == Stage::kRedLight);
    // Any button re-stages.
    r.speed = 0.0f;
    Effects fx = r.step(true);
    CHECK(fx.consumedButton);
    CHECK(r.stage() == Stage::kWaitStop);
  }
}

TEST_CASE("unstage without movement during the tree is silent, not a foul") {
  Rig r;
  r.toYellow1();
  r.staged = false;  // fix gap consumed the physics anchor
  r.now += 20;
  r.step();
  CHECK(r.stage() == Stage::kWaitStop);
}

TEST_CASE("fix loss during the tree re-stages silently") {
  Rig r;
  r.toYellow1();
  r.fix = false;
  r.now += 20;
  r.step();
  CHECK(r.stage() == Stage::kWaitStop);
}

TEST_CASE("pre-stage unstage bounces back to wait-stop") {
  Rig r;
  r.toPreStage();
  r.staged = false;
  r.now += 20;
  r.step();
  CHECK(r.stage() == Stage::kWaitStop);
}

TEST_CASE("failed to launch: green + 5 s still") {
  Rig r;
  r.toGreen();
  const uint32_t greenAt = r.now;
  r.now = greenAt + drag_tree::kFailedLaunchMs - 1;
  r.step();
  CHECK(r.stage() == Stage::kGreen);
  r.now = greenAt + drag_tree::kFailedLaunchMs;
  r.step();
  CHECK(r.stage() == Stage::kFailedLaunch);
  Effects fx = r.step(true);
  CHECK(fx.consumedButton);
  CHECK(r.stage() == Stage::kWaitStop);
}

TEST_CASE("green + moving but physics never launched aborts") {
  Rig r;
  r.toGreen();
  r.speed = 10.0f;
  r.staged = false;   // degenerate: anchor was lost right at green
  const uint32_t greenAt = r.now;
  r.now = greenAt + drag_tree::kGreenNoRunMs;
  r.step();
  CHECK(r.stage() == Stage::kAborted);
}

TEST_CASE("mid-run physics abort surfaces as RUN ABORTED") {
  Rig r;
  r.toGreen();
  r.speed = 30.0f;
  r.runActive = true;
  r.now += 100;
  r.step();
  REQUIRE(r.stage() == Stage::kRunning);
  r.runActive = false;  // silent physics abort, runs unchanged
  r.now += 20;
  r.step();
  CHECK(r.stage() == Stage::kAborted);
  Effects fx = r.step(true);
  CHECK(fx.consumedButton);
  CHECK(r.stage() == Stage::kWaitStop);
}

// ---------------------------------------------------------------------------
// Exit hold
// ---------------------------------------------------------------------------

TEST_CASE("select hold ends the session from waiting stages") {
  Rig r;
  r.step();  // one released frame arms selectReleasedSeen
  r.selectHeld = true;
  CHECK_FALSE(r.tick(drag_tree::kExitHoldMs - 40));
  CHECK(r.tick(100));  // crosses the threshold
}

TEST_CASE("select hold works from the results screen, surviving the re-arm press") {
  Rig r;
  r.toGreen();
  r.speed = 30.0f;
  r.runActive = true;
  r.now += 100;
  r.step();
  r.runs = 1;
  r.runActive = false;
  r.now += 8000;
  r.step();
  REQUIRE(r.stage() == Stage::kResults);

  // Press Select and KEEP holding: the edge consumes as re-arm, the
  // continuing hold still exits ~2 s later.
  r.selectHeld = true;
  Effects fx = r.step(true);
  CHECK(fx.consumedButton);
  CHECK(r.stage() == Stage::kWaitStop);
  CHECK(r.tick(drag_tree::kExitHoldMs + 40));
}

TEST_CASE("exit hold never fires mid-run") {
  Rig r;
  r.toGreen();
  r.speed = 30.0f;
  r.runActive = true;
  r.now += 100;
  r.step();
  REQUIRE(r.stage() == Stage::kRunning);
  r.selectHeld = true;
  CHECK_FALSE(r.tick(drag_tree::kExitHoldMs * 3));
  CHECK(r.stage() == Stage::kRunning);
}

TEST_CASE("a held side button disarms the exit hold (reboot combo wins)") {
  Rig r;
  r.step();
  r.selectHeld = true;
  r.tick(1500);
  r.otherHeld = true;  // user is going for the Select+side combo
  r.tick(100);
  r.otherHeld = false;
  // The tracking restarted — 1 s more is not enough...
  CHECK_FALSE(r.tick(1000));
  // ...a full fresh window is.
  CHECK(r.tick(drag_tree::kExitHoldMs));
}

TEST_CASE("the press that entered the session cannot become the exit hold") {
  Rig r;
  r.selectHeld = true;  // held from the very first frame, never released
  CHECK_FALSE(r.tick(drag_tree::kExitHoldMs * 3));
}

// ---------------------------------------------------------------------------
// Renderers
// ---------------------------------------------------------------------------

TEST_CASE("strip frames match the spec patterns") {
  const uint32_t t = 123456;
  CHECK(matches(Stage::kAwaitArm, t, "---------"));
  CHECK(matches(Stage::kWaitStop, t, "---------"));
  CHECK(matches(Stage::kResults,  t, "---------"));
  CHECK(matches(Stage::kPreStage, t, "----w----"));
  CHECK(matches(Stage::kYellow1,  t, "---ywy---"));
  CHECK(matches(Stage::kYellow2,  t, "--yywyy--"));
  CHECK(matches(Stage::kYellow3,  t, "-yyywyyy-"));
  CHECK(matches(Stage::kGreen,    t, "gyyywyyyg"));
}

TEST_CASE("foul stages flash whole-strip red on the shared phase") {
  for (Stage st : {Stage::kRedLight, Stage::kFailedLaunch, Stage::kAborted}) {
    CAPTURE((int)st);
    // Two half-periods apart: exactly one phase is all red, the other
    // all off — and the two frames differ.
    const bool aRed = matches(st, 0, "rrrrrrrrr");
    const bool aOff = matches(st, 0, "---------");
    const bool bRed = matches(st, drag_tree::kFoulFlashHalfMs, "rrrrrrrrr");
    const bool bOff = matches(st, drag_tree::kFoulFlashHalfMs, "---------");
    CHECK(aRed != bRed);
    CHECK(aOff != bOff);
    CHECK((aRed || aOff));
    CHECK((bRed || bOff));
  }
}

TEST_CASE("renderStrip is deterministic") {
  led_frame::Rgb a[led_frame::kStripCount];
  led_frame::Rgb b[led_frame::kStripCount];
  drag_tree::renderStrip(Stage::kYellow2, 777, a);
  drag_tree::renderStrip(Stage::kYellow2, 777, b);
  for (int i = 0; i < led_frame::kStripCount; i++) {
    CHECK(sameRgb(a[i], b[i].r, b[i].g, b[i].b));
  }
}

TEST_CASE("countdownDigit and stripActive maps") {
  CHECK(drag_tree::countdownDigit(Stage::kYellow1) == '3');
  CHECK(drag_tree::countdownDigit(Stage::kYellow2) == '2');
  CHECK(drag_tree::countdownDigit(Stage::kYellow3) == '1');
  CHECK(drag_tree::countdownDigit(Stage::kGreen) == 0);
  CHECK(drag_tree::countdownDigit(Stage::kAwaitArm) == 0);

  CHECK(drag_tree::stripActive(Stage::kAwaitArm));
  CHECK(drag_tree::stripActive(Stage::kGreen));
  CHECK(drag_tree::stripActive(Stage::kRedLight));
  CHECK_FALSE(drag_tree::stripActive(Stage::kRunning));
}
