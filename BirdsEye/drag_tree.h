#pragma once

#include <stdint.h>

#include "drag_timer.h"  // kLaunchMinMph — the ONE movement threshold
#include "led_frame.h"

///////////////////////////////////////////
// MANUAL DRAG STAGING TREE (plan 0016)
// The christmas-tree staging sequence for manual drag mode: white
// staging pip -> three yellows at the sportsman-tree cadence -> green,
// with red-light fouls, a failed-to-launch timeout, and press-any-button
// re-arm between runs. This unit is the ONE state machine driving BOTH
// the 9-px LED strip and the OLED staging screen, so the two can never
// disagree — every glyph, message, and flash phase decision lives here
// (flash phases via led_modes::flashOn on timestamps, never a per-render
// toggle: the display refreshes at 3 Hz and would drift from the strip).
//
// The tree never duplicates physics: staging detection is drag_timer's
// own standstill logic (observed via Inputs.timerStaged), the run is
// drag_timer's launch/finish (runActive / runs), and the movement that
// constitutes a red-light foul is the same threshold that would launch
// the physics timer (drag_timer::kLaunchMinMph). The glue holds the
// physics launch gate closed (DragTimer::setLaunchEnabled) except while
// this unit shows green.
//
// Pure logic — no Arduino headers — host-tested (step FSM per the
// gps_status_page pattern, LED frames per the led_animations golden
// pattern).
///////////////////////////////////////////

namespace drag_tree {

///////////////////////////////////////////
// Tunables
///////////////////////////////////////////

// Yellow cadence — 500 ms per bulb, the NHRA sportsman tree.
constexpr uint32_t kTreeStepMs = 500;

// Staged (drag_timer's own <=1 mph held 1 s) must hold this much LONGER
// before the tree starts — the requirement's "stopped for a couple
// seconds" is on top of the physics promotion (total stop->tree ~3 s).
// Deliberately NOT a reuse of drag_timer::kStageHoldMs, which is the
// physics standstill promotion, a different thing.
constexpr uint32_t kPreStageHoldMs = 2000;

// Green with no movement this long -> FAILED TO LAUNCH.
constexpr uint32_t kFailedLaunchMs = 5000;

// Green, moving, but the physics never launched (degenerate: a fix gap
// consumed the staged anchor right at green) -> RUN ABORTED rather than
// a hang in green. 3 s is far beyond any real rollout time.
constexpr uint32_t kGreenNoRunMs = 3000;

// Select held this long (any non-running stage) ends the session. Short
// enough to be discoverable from the on-screen hint, long enough that a
// press-any-button tap can never trip it.
constexpr uint32_t kExitHoldMs = 2000;

// Red foul flash / screen GO flash half-periods. 500 ms, NOT the LED
// alert modes' 100-250 ms: the OLED refreshes at 3 Hz (~333 ms), and a
// half-period near the render period aliases into an irregular flicker.
// At 500 ms each phase gets one or two renders, so the screen blinks a
// clean ~1 Hz in step with the strip.
constexpr uint16_t kFoulFlashHalfMs = 500;
constexpr uint16_t kGoFlashHalfMs = 500;

enum class Stage : uint8_t {
  kAwaitArm,      // session entry: "press any button", strip dark
  kWaitStop,      // armed: waiting for the physics to stage
  kPreStage,      // staged: white center pip, pre-stage hold running
  kYellow1,       // ---ywy---   screen 3
  kYellow2,       // --yywyy--   screen 2
  kYellow3,       // -yyywyyy-   screen 1
  kGreen,         // gyyywyyyg   screen GO (flashing); launch gate open
  kRunning,       // physics run live: strip -> normal compose, screen -> ET
  kResults,       // ET/trap/0-60/RT, press any button -> kWaitStop
  kRedLight,      // moved during the yellows: foul
  kFailedLaunch,  // green + 5 s still
  kAborted,       // run died mid-pass, or green-no-run
};

// Snapshot built fresh by the sketch each loop iteration.
struct Inputs {
  uint32_t nowMs = 0;          // millis()
  bool  fix = false;           // gpsData.fix
  float speedMph = 0.0f;       // gps_speed_mph (meaningful only with fix)
  bool  timerStaged = false;   // dragTimer->staged()
  bool  runActive = false;     // dragTimer->runActive()
  int   runs = 0;              // dragTimer->runs()
  bool  buttonPressed = false; // any debounced press edge this frame
  bool  selectHeld = false;    // live level, isButtonHeld(2, 0)
  bool  otherButtonHeld = false;  // side buttons — disarms the exit hold
                                  // so the Select+side reboot combo wins
};

struct State {
  Stage    stage = Stage::kAwaitArm;
  uint32_t stageEnteredMs = 0;
  uint32_t greenAtMs = 0;
  int      lastRuns = 0;        // latched entering kRunning
  bool     prevRunActive = false;
  bool     selectReleasedSeen = false;  // the press that started the
                                        // session can never be the exit
  bool     selectHoldTracking = false;
  uint32_t selectHoldSinceMs = 0;
};

// One-shot side effects for the glue; all default false.
struct Effects {
  bool consumedButton = false;  // -> resetButtons()
  bool greenEdge = false;       // -> latch the green epoch for RT
  bool runStartEdge = false;    // -> compute RT from runStartEpochMs()
  bool exitSession = false;     // -> endRaceSession() + main menu
};

void begin(State& s, uint32_t nowMs);

// Advance one step. The glue keeps the physics launch gate =
// (stage == kGreen) after every call.
Effects step(State& s, const Inputs& in);

// The 9 strip pixels for a stage (kRunning excepted — see stripActive).
// Clears then paints; brightness capping stays npxPushFrame's job.
void renderStrip(Stage st, uint32_t nowMs,
                 led_frame::Rgb out[led_frame::kStripCount]);

// '3' / '2' / '1' for the yellow stages, 0 otherwise — the OLED's big
// countdown digit comes from here so screen and bulbs can't disagree.
char countdownDigit(Stage st);

// The shared GO/foul flash phase for the OLED renderer — the SAME clock
// as the strip's red flash (led_modes::flashOn under the hood), so the
// screen and the LEDs blink in lockstep instead of drifting at the
// display's 3 Hz refresh.
bool flashPhase(uint32_t nowMs);

// False only for kRunning: the strip returns to the normal race compose
// (RPM/speed scale) while the pass is being driven.
bool stripActive(Stage st);

}  // namespace drag_tree
