#include "drag_tree.h"

#include "led_modes.h"  // flashOn — THE flash-phase definition

namespace drag_tree {

namespace {

bool foulStage(Stage st) {
  return st == Stage::kRedLight || st == Stage::kFailedLaunch ||
         st == Stage::kAborted;
}

void enter(State& s, Stage st, uint32_t nowMs) {
  s.stage = st;
  s.stageEnteredMs = nowMs;
}

}  // namespace

void begin(State& s, uint32_t nowMs) {
  s = State{};
  s.stageEnteredMs = nowMs;
}

Effects step(State& s, const Inputs& in) {
  Effects fx;
  const bool moving =
      in.fix && in.speedMph >= drag_timer::kLaunchMinMph;

  // ---- Exit hold (every stage except kRunning) --------------------------
  // The press that entered the session may still be held on the first
  // steps — require one observed release before the hold can arm. A held
  // side button disarms it so the Select+side reboot combo stays
  // reachable (sd_format_page precedent). The hold deliberately survives
  // stage transitions: pressing Select on the results screen re-arms via
  // the any-button edge below, and KEEPING it held still exits 2 s later.
  if (!in.selectHeld) s.selectReleasedSeen = true;
  if (s.stage != Stage::kRunning && s.selectReleasedSeen &&
      in.selectHeld && !in.otherButtonHeld) {
    if (!s.selectHoldTracking) {
      s.selectHoldTracking = true;
      s.selectHoldSinceMs = in.nowMs;
    } else if (in.nowMs - s.selectHoldSinceMs >= kExitHoldMs) {
      fx.exitSession = true;
      s.prevRunActive = in.runActive;
      return fx;
    }
  } else {
    s.selectHoldTracking = false;
  }

  switch (s.stage) {
    case Stage::kAwaitArm:
      if (in.buttonPressed) {
        fx.consumedButton = true;
        enter(s, Stage::kWaitStop, in.nowMs);
      }
      break;

    case Stage::kWaitStop:
      // drag_timer's own standstill logic IS the staging detector —
      // this unit never duplicates physics.
      if (in.timerStaged) enter(s, Stage::kPreStage, in.nowMs);
      break;

    case Stage::kPreStage:
      if (!in.timerStaged) {
        enter(s, Stage::kWaitStop, in.nowMs);  // silent — not a foul yet
      } else if (in.nowMs - s.stageEnteredMs >= kPreStageHoldMs) {
        enter(s, Stage::kYellow1, in.nowMs);
      }
      break;

    case Stage::kYellow1:
    case Stage::kYellow2:
    case Stage::kYellow3:
      if (moving) {
        // Jumped the tree.
        enter(s, Stage::kRedLight, in.nowMs);
      } else if (!in.timerStaged || !in.fix) {
        // Physics un-staged without movement (fix gap / fix lost) —
        // silent re-stage, a GPS hiccup is not a foul.
        enter(s, Stage::kWaitStop, in.nowMs);
      } else if (in.nowMs - s.stageEnteredMs >= kTreeStepMs) {
        const Stage next = (s.stage == Stage::kYellow1) ? Stage::kYellow2
                           : (s.stage == Stage::kYellow2)
                               ? Stage::kYellow3
                               : Stage::kGreen;
        enter(s, next, in.nowMs);
        if (next == Stage::kGreen) {
          s.greenAtMs = in.nowMs;
          fx.greenEdge = true;
        }
      }
      break;

    case Stage::kGreen:
      if (in.runActive && !s.prevRunActive) {
        // The physics launched — the pass is on.
        fx.runStartEdge = true;
        s.lastRuns = in.runs;
        enter(s, Stage::kRunning, in.nowMs);
      } else if (!moving && in.nowMs - s.greenAtMs >= kFailedLaunchMs) {
        enter(s, Stage::kFailedLaunch, in.nowMs);
      } else if (moving && !in.runActive &&
                 in.nowMs - s.greenAtMs >= kGreenNoRunMs) {
        // Moving but the physics never launched (staged anchor lost at
        // exactly the wrong moment) — abort rather than hang in green.
        enter(s, Stage::kAborted, in.nowMs);
      }
      break;

    case Stage::kRunning:
      if (in.runs > s.lastRuns) {
        enter(s, Stage::kResults, in.nowMs);
      } else if (!in.runActive && s.prevRunActive) {
        // The physics abandoned the run (mid-run standstill, fix gap,
        // prove-out, time step) — surface it instead of staying silent
        // on a pinned screen showing a frozen ET.
        enter(s, Stage::kAborted, in.nowMs);
      }
      break;

    case Stage::kResults:
    case Stage::kRedLight:
    case Stage::kFailedLaunch:
    case Stage::kAborted:
      if (in.buttonPressed) {
        fx.consumedButton = true;
        enter(s, Stage::kWaitStop, in.nowMs);
      }
      break;
  }

  s.prevRunActive = in.runActive;
  return fx;
}

void renderStrip(Stage st, uint32_t nowMs,
                 led_frame::Rgb out[led_frame::kStripCount]) {
  for (int i = 0; i < led_frame::kStripCount; i++) out[i] = led_frame::kOff;

  if (foulStage(st)) {
    if (led_modes::flashOn(nowMs, kFoulFlashHalfMs)) {
      for (int i = 0; i < led_frame::kStripCount; i++) {
        out[i] = led_frame::kRed;
      }
    }
    return;
  }

  const int c = led_frame::kStripCenter;  // 4
  switch (st) {
    case Stage::kGreen:
      // Green keeps the full yellow ladder lit; the ladder builds down
      // through the fallthroughs, and the center pip stays white all
      // the way from pre-stage to launch.
      out[0] = led_frame::kGreen;
      out[led_frame::kStripCount - 1] = led_frame::kGreen;
      [[fallthrough]];
    case Stage::kYellow3:
      out[c - 3] = led_frame::kYellow;
      out[c + 3] = led_frame::kYellow;
      [[fallthrough]];
    case Stage::kYellow2:
      out[c - 2] = led_frame::kYellow;
      out[c + 2] = led_frame::kYellow;
      [[fallthrough]];
    case Stage::kYellow1:
      out[c - 1] = led_frame::kYellow;
      out[c + 1] = led_frame::kYellow;
      [[fallthrough]];
    case Stage::kPreStage:
      out[c] = led_frame::kWhite;
      break;
    default:
      break;  // kAwaitArm / kWaitStop / kResults: strip dark
  }
}

bool flashPhase(uint32_t nowMs) {
  return led_modes::flashOn(nowMs, kFoulFlashHalfMs);
}

char countdownDigit(Stage st) {
  switch (st) {
    case Stage::kYellow1: return '3';
    case Stage::kYellow2: return '2';
    case Stage::kYellow3: return '1';
    default: return 0;
  }
}

bool stripActive(Stage st) { return st != Stage::kRunning; }

}  // namespace drag_tree
