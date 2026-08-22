#include "neopixel.h"
#include "project.h"

#if BIRDSEYE_ENABLE_NEOPIXEL

#include <Adafruit_NeoPixel.h>

#include "led_animations.h"
#include "led_frame.h"
#include "led_modes.h"
#include "led_status.h"
#include "local_time.h"
#include "nan_bits.h"
#include "camera_ble.h"
#include "sector_purple.h"
#include "sensoregg.h"
#include "tachometer.h"

// 30 Hz: smooth for a moving pip and cheap (~0.4 ms show() ≈ 1% CPU).
// The display's 3 Hz gate is a different clock — never share it.
static const uint32_t NPX_FRAME_INTERVAL_MS = 33;

// Boost converter settle before first data after EN rises.
static const uint32_t NPX_BOOST_SETTLE_MS = 5;

static Adafruit_NeoPixel npxStrip(led_frame::kPixelCount, NEOPIXEL_PIN_DATA,
                                  NEO_GRB + NEO_KHZ800);

///////////////////////////////////////////
// BOOST EN OWNERSHIP
//
// In a normal build this module owns NEOPIXEL_PIN_BOOST_EN outright:
// HIGH raises the 5 V rail, LOW drops it, and the LOW is what actually
// keeps it down through System OFF (GPIO levels are retained there —
// the "blue conn LED stays on after sleep" precedent).
//
// A PROFILING build (BIRDSEYE_ENABLE_PROFILING, plan 0011) takes that
// pin away for its scope output, so all three helpers become no-ops and
// the regulator is left at its hardware default (EN pulled up = rail
// on). That is a real, accepted cost, spelled out in project.h: the
// rail can no longer be switched off, including at shutdown. The pin
// cannot serve both purposes, and profiling builds are bench builds.
//
// Everything else in this file is unchanged by the flag — the strip
// still renders, because its DATA pin is untouched and the rail it
// needs is up.
///////////////////////////////////////////
#if BIRDSEYE_ENABLE_PROFILING
static inline void npxBoostPinInit() {}
static inline void npxBoostEnable() {}
static inline void npxBoostDisable() {}
#else
static inline void npxBoostPinInit() {
  pinMode(NEOPIXEL_PIN_BOOST_EN, OUTPUT);
  digitalWrite(NEOPIXEL_PIN_BOOST_EN, LOW);
}
static inline void npxBoostEnable() {
  digitalWrite(NEOPIXEL_PIN_BOOST_EN, HIGH);
}
static inline void npxBoostDisable() {
  pinMode(NEOPIXEL_PIN_BOOST_EN, OUTPUT);
  digitalWrite(NEOPIXEL_PIN_BOOST_EN, LOW);
}
#endif

static bool npxReady = false;  // strip powered + begun; LOOP is live
static uint32_t npxLastFrameMs = 0;

// Animation clocks: active while *Active, rendered from (now - start).
static bool npxBootAnimActive = false;
static uint32_t npxBootAnimStartMs = 0;
static uint32_t npxBootAnimSeed = 0;
static bool npxPurpleActive = false;
static bool npxPurpleIsLap = false;  // which of the two animations is running
static uint32_t npxPurpleStartMs = 0;
static uint32_t npxPurpleSeed = 0;

// Lap/sector close-edge monitor + per-status-LED state.
static sector_purple::State npxPurpleMon;
static led_status::State npxLeftState;
static led_status::State npxRightState;
static led_modes::StatusState npxOverrevState;

// The last verdict each indicator mode reports, HELD until the next
// close edge — that hold is the whole point: the driver looks down some
// seconds after the line, not at the instant of it. Cleared when the
// session ends so the next one starts blank.
static sector_purple::Verdict npxLapVerdict = sector_purple::Verdict::kNone;
static sector_purple::Verdict npxSectorVerdict = sector_purple::Verdict::kNone;

// Overrev (plan 0007): past this the engine is BROKEN, not just at its
// ceiling — the whole 11-px chain flashes red. Latch clears down at the
// normal rev limit's clear point so a spike leaves a visible flash.
// threshold/clearBelow filled from settings at NEOPIXEL_SETUP();
// disabled (source kNone) while overrev_limit is 0.
static led_modes::StatusAction npxOverrevAction = {
    led_modes::Source::kNone, 0.0f, 0.0f, led_frame::kRed,
    led_modes::kRevFlashHalfPeriodMs, led_frame::kOff};

/**
 * @brief One-time NFC->GPIO conversion. UICR->NFCPINS bit 0 set means
 * the pads still belong to NFCT; program it to 0 (a 1->0-only word
 * write, no erase needed) and reset — NFCPINS latches only at reset.
 * ONE-WAY: undoing this needs a full chip erase (bootloader reflash).
 * Direct NVMC access, so this MUST run before the SoftDevice is
 * enabled (NEOPIXEL_SETUP is called before SENSOREGG_SETUP's
 * bleCoreEnsureInit) and it runs before wdtSetup() arms the watchdog,
 * so the reset can't race it. Every boot after the first skips this.
 */
static void npxEnsureNfcPinsAreGpio() {
  if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) == 0) {
    return;  // already GPIO
  }
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
  while (!NRF_NVMC->READY) {}
  NRF_UICR->NFCPINS = 0xFFFFFFFE;  // clear PROTECT, leave the rest erased
  while (!NRF_NVMC->READY) {}
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
  while (!NRF_NVMC->READY) {}
  debugln(F("NFC pads converted to GPIO — one-time reset"));
  delay(10);  // let the debug byte drain
  NVIC_SystemReset();
}

void NEOPIXEL_SETUP() {
  npxEnsureNfcPinsAreGpio();

  // Hold both pins in a defined LOW state first — EN low keeps the 5 V
  // rail off, data low can't back-power an unpowered strip. (On a
  // profiling build the EN half is a no-op; see the block above.)
  npxBoostPinInit();
  pinMode(NEOPIXEL_PIN_DATA, OUTPUT);
  digitalWrite(NEOPIXEL_PIN_DATA, LOW);

  if (settingLedBrightness == 0) {
    debugln(F("NeoPixel: brightness 0 — LEDs disabled, boost held off"));
    return;
  }

  // Overrev: fire at the problem limit, release only once back under
  // the TARGET RPM's clear point — the whole band stays latched.
  // (The per-status-LED thresholds are no longer built here: each
  // assignable mode derives its own StatusAction from the settings
  // every frame in led_status::evalMode. This one stays because it
  // drives the whole chain, not a status pixel.)
  if (settingOverrevLimit > 0) {
    npxOverrevAction.source = led_modes::Source::kRpm;
    npxOverrevAction.threshold = (float)settingOverrevLimit;
    npxOverrevAction.clearBelow =
        (float)settingTargetRpm * led_modes::kRevClearFrac;
  }

  npxBoostEnable();
  delay(NPX_BOOST_SETTLE_MS);
  npxStrip.begin();
  npxStrip.clear();
  npxStrip.show();
  npxReady = true;

  sector_purple::reset(npxPurpleMon);
  npxBootAnimActive = true;
  npxBootAnimStartMs = millis();
  npxBootAnimSeed = micros();  // house entropy rule: never analogRead()
}

void neopixelNotifyPurpleSector() {
  if (!npxReady) {
    return;
  }
  // A lap celebration already running outranks a sector one — the last
  // sector of a purple lap is very often purple too, and re-arming the
  // shorter animation here would cut the bigger moment short.
  if (npxPurpleActive && npxPurpleIsLap) {
    return;
  }
  npxPurpleActive = true;
  npxPurpleIsLap = false;
  npxPurpleStartMs = millis();
  npxPurpleSeed = micros();
}

void neopixelNotifyPurpleLap() {
  if (!npxReady) {
    return;
  }
  npxPurpleActive = true;
  npxPurpleIsLap = true;  // takes over an in-flight sector animation
  npxPurpleStartMs = millis();
  npxPurpleSeed = micros();
}

/**
 * @brief The cap in force right now: the day value, or the night one
 * once the LOCAL wall clock is inside the night window (plan 0010).
 *
 * Local time is UTC plus a fixed offset — no DST, no tzdata. Deliberate:
 * this gate exists so the strip stops blinding the driver after dark,
 * and an hour of seasonal drift on that boundary is beneath its
 * resolution. Nothing that gets LOGGED goes through here.
 *
 * Two rules that matter:
 *  - No clock, no swap. gpsData.timeValid is the same gate log-file
 *    creation waits on and can be ~12.5 min out from a cold start, so
 *    until it lands we render at DAY brightness. Guessing night would
 *    mean a strip that comes up dark and looks broken.
 *  - A night cap of 0 blanks the frame; it does NOT cut the 5 V rail.
 *    Only settingLedBrightness 0 does that (NEOPIXEL_SETUP/WAKE), and
 *    it must stay that way — power-cycling the boost converter at 19:00
 *    mid-session is not a brightness change.
 */
static uint8_t npxEffectiveBrightness() {
  if (settingLedBrightness == 0) return 0;  // master disable
  if (!gpsData.timeValid) return settingLedBrightness;

  const local_time::DateTime utc = {
      (uint16_t)(2000 + gpsData.year),  // GpsData carries a 2-digit year
      gpsData.month, gpsData.day, gpsData.hour, gpsData.minute};
  const local_time::DateTime lt =
      local_time::applyOffset(utc, settingUtcOffsetMin);
  const bool night =
      local_time::isNight(local_time::minuteOfDay(lt),
                          (uint16_t)settingLedDayStartHour * 60,
                          (uint16_t)settingLedNightStartHour * 60);
  return night ? settingLedBrightnessNight : settingLedBrightness;
}

/**
 * @brief Push a composed frame: apply THE brightness cap (the single
 * choke point — after this no channel exceeds the cap in force), map
 * logical left-to-right onto the physical wire (the chain is wired
 * data-in on the RIGHT — led_frame::kChainReversed), and show.
 */
static void npxPushFrame(led_frame::Frame& frame) {
  led_frame::applyCap(frame, npxEffectiveBrightness());
  for (int i = 0; i < led_frame::kPixelCount; i++) {
    int const phys = led_frame::physicalIndex(i);
    npxStrip.setPixelColor((uint16_t)phys, frame.px[i].r, frame.px[i].g,
                           frame.px[i].b);
  }
  npxStrip.show();
}

/**
 * @brief Snapshot every source the status modes can read, once per
 * frame. Built once and evaluated twice (one call per pixel) so the two
 * LEDs can never disagree about the same instant.
 *
 * All of these are trivial flag/staleness reads at 30 Hz. Deliberately
 * NOT gated on "does either LED actually want this" — that optimisation
 * costs more in branches than it saves, and loop_profile can say if it
 * ever stops being true.
 *
 * cameraConsumeAutoStop() must NEVER appear here: it clears a latch the
 * main sketch owns, and a renderer that consumes state is a renderer
 * that steals a session end.
 */
static void npxFillStatusInputs(led_status::Inputs& in) {
  in.rpm = (float)tachLastReported;
  in.targetRpm = (float)settingTargetRpm;
  in.speedMph = gps_speed_mph;
  in.speedValid = gpsInitialized && gpsData.fix;
  in.targetSpeedMph = (float)settingTargetSpeedMph;

#if BIRDSEYE_ENABLE_SENSOREGG
  // isNanF, never isnan: -Ofast folds isnan() to false (nan_bits.h).
  const float egtC = sensoreggEgtC();
  in.egtC = egtC;
  in.egtValid = !isNanF(egtC);
  in.egtAlertC = (float)settingTemp1AlertC;
  in.eggSupported = true;
#else
  // The accessor is not even called on a stock build. Before plan 0013
  // it was, unconditionally, and its permanent NaN drove the right
  // status pixel to a solid blue "no probe signal" for the whole of
  // every race on every shipped logger. With eggSupported false the
  // `egt` mode renders dark instead — there is no probe here to lose.
  in.eggSupported = false;
#endif

  in.gpsSats = gpsData.satellites;
  in.gpsFix = gpsData.fix;
  in.gpsTimeValid = gpsData.timeValid;

  in.cameraPaired = cameraIsPaired();
  in.sessionActive = raceActive;
  in.cameraLinkUp = cameraRemoteLinkUp();
  in.cameraSubscribed = cameraCe82Subscribed();
  // Our own belief OR the camera's own report: the FSM knows it asked
  // for a recording, the 0x10 display-string observation knows one is
  // actually running. Either is reason enough to show red.
  in.cameraRecording = cameraActivelyRecording() || cameraObservedRecording();

  in.lapVerdict = npxLapVerdict;
  in.sectorVerdict = npxSectorVerdict;
}

void NEOPIXEL_LOOP() {
  if (!npxReady) {
    return;
  }
  const uint32_t now = millis();
  if (now - npxLastFrameMs < NPX_FRAME_INTERVAL_MS) {
    return;
  }
  npxLastFrameMs = now;

  // Lap/sector close-edge monitor: poll-and-diff against the active
  // timer. One state machine feeds both the purple celebrations and the
  // held verdicts the lap/sector status modes render.
  {
    sector_purple::Sample smp;
    smp.sectorsConfigured = activeTimerSectorsConfigured();
    smp.raceStarted = activeTimerRaceStarted();
    smp.currentSector = activeTimerCurrentSector();
    smp.laps = activeTimerLaps();
    smp.lastLapTime = activeTimerLastLapTime();
    smp.bestLapTime = activeTimerBestLapTime();
    for (int s = 0; s < 3; s++) {
      smp.lapSectorTime[s] = activeTimerLapSectorTime(s + 1);
      smp.bestSectorTime[s] = activeTimerBestSectorTime(s + 1);
    }
    const sector_purple::Event ev = sector_purple::update(npxPurpleMon, smp);
    if (ev.closedSector != 0) {
      npxSectorVerdict = ev.sectorVerdict;
    }
    if (ev.lapClosed) {
      npxLapVerdict = ev.lapVerdict;
    }
    // Lap first: notifyPurpleLap() takes over an in-flight sector
    // animation, and notifyPurpleSector() declines to stomp a lap one,
    // so this order is what makes the bigger event win when both fire.
    if (ev.purpleLap) {
      neopixelNotifyPurpleLap();
    }
    if (ev.purpleSector != 0) {
      neopixelNotifyPurpleSector();
    }
  }

  led_frame::Frame frame;
  bool rendered = false;

  // Priority 1: boot animation.
  if (npxBootAnimActive) {
    rendered = led_animations::renderBoot(now - npxBootAnimStartMs,
                                          npxBootAnimSeed, frame);
    npxBootAnimActive = rendered;
  }
  // Priority 2: overrev — the engine is BROKEN. Whole chain flashes
  // red; outranks the purple celebration, evaluated only in race.
  if (raceActive) {
    const led_frame::Rgb overrevColor =
        led_modes::evalStatus(npxOverrevAction, npxOverrevState,
                              (float)tachLastReported, true, now);
    if (!rendered && npxOverrevState.active) {
      for (int i = 0; i < led_frame::kPixelCount; i++) {
        frame.px[i] = overrevColor;  // flash off-phase = whole chain dark
      }
      rendered = true;
    }
  } else {
    npxOverrevState.active = false;
  }
  // Priority 3: purple celebration — the longer two-wave version for a
  // session-best LAP, the original for a session-best SECTOR.
  if (!rendered && npxPurpleActive) {
    const uint32_t tMs = now - npxPurpleStartMs;
    rendered = npxPurpleIsLap
                   ? led_animations::renderPurpleLap(tMs, npxPurpleSeed, frame)
                   : led_animations::renderPurple(tMs, npxPurpleSeed, frame);
    npxPurpleActive = rendered;
  }
  // Priority 4/5: parked -> off; otherwise the bar in race, and the two
  // assignable status LEDs.
  if (!rendered) {
    led_frame::clear(frame);
    const bool parked = bleActive || usbMscActive;
    if (!parked && raceActive) {
      // Strip selection (plan 0007 order, plus the 0013 speed arm):
      //   engine died (proven tach session, RPM 0) -> bar OFF — a pace
      //     pip counting next to a dead engine reads as a glitch;
      //   no GPS lock -> green search pip (same fix+timeValid gate that
      //     allows log-file creation);
      //   pace valid -> pace pip;
      //   tach session -> RPM scale;
      //   else -> SPEED scale.
      led_frame::Rgb stripPx[led_frame::kStripCount];
      bool stripOff = raceEngineStopped();
      if (!stripOff) {
        if (!gpsData.fix || !gpsData.timeValid) {
          led_modes::renderSearchPip(now, stripPx);
        } else {
          const bool paceValid =
              activeTimerRaceStarted() && activeTimerLaps() >= 1 &&
              !((sprintModeIsActive() || dragModeIsActive()) &&
                !activeTimerRunActive());
          if (paceValid) {
            led_modes::renderPace(activeTimerPaceDifference(), stripPx);
          } else if (raceEntryCause == RACE_ENTRY_TACH) {
            const led_modes::ScaleSpec rpmSpec = {
                0.0f, (float)settingTargetRpm, led_modes::kRpmRedFrac,
                led_frame::kGreen, led_frame::kRed};
            led_modes::renderScale((float)tachLastReported, rpmSpec, stripPx);
          } else {
            // No tachometer has proven itself this session, so an RPM
            // scale would be nine dark pixels until the first lap lands
            // (plan 0013). Scale against the target speed instead —
            // RACE_ENTRY_TACH is the right gate because MANUAL and SPEED
            // sessions promote to it the moment the engine clears
            // 500 rpm (idle_policy::tachProven), so this is exactly "has
            // never seen an engine" and it never flickers per-frame the
            // way a live `tachLastReported > 0` test would.
            //
            // gps_speed_mph, NOT gpsData.speed — that one is knots. The
            // search-pip branch above already caught !fix, so the speed
            // is live by the time we get here.
            const led_modes::ScaleSpec spdSpec = {
                0.0f, (float)settingTargetSpeedMph, led_modes::kSpeedRedFrac,
                led_frame::kGreen, led_frame::kRed};
            led_modes::renderScale(gps_speed_mph, spdSpec, stripPx);
          }
        }
        for (int i = 0; i < led_frame::kStripCount; i++) {
          frame.px[led_frame::kStripFirst + i] = stripPx[i];
        }
      }
    }

    // The two status LEDs. In a session both render whatever the user
    // assigned; parked on the menu only the modes that mean something
    // with no session running stay lit (GPS lock and camera readiness —
    // exactly what you want to see from the paddock). Status LEDs stay
    // live even when the bar is off: a hot engine cooling after a stall
    // is precisely when a temp alert matters.
    if (!parked) {
      led_status::Inputs in;
      npxFillStatusInputs(in);
      const bool showLeft =
          raceActive || led_status::activeOutsideRace(settingLedStatusLeft);
      const bool showRight =
          raceActive || led_status::activeOutsideRace(settingLedStatusRight);
      // A pixel we are not rendering gets its latch released, not just a
      // dark colour: leaving one set would let a threshold that tripped
      // in the last seconds of a session flash the instant the next one
      // starts, before its own source has said anything.
      if (showLeft) {
        frame.px[led_frame::kStatusLeft] =
            led_status::evalMode(settingLedStatusLeft, in, npxLeftState, now);
      } else {
        led_status::reset(npxLeftState);
      }
      if (showRight) {
        frame.px[led_frame::kStatusRight] =
            led_status::evalMode(settingLedStatusRight, in, npxRightState, now);
      } else {
        led_status::reset(npxRightState);
      }
    } else {
      led_status::reset(npxLeftState);
      led_status::reset(npxRightState);
    }

    if (!raceActive) {
      // Out of race: drop the held verdicts so the next session starts
      // blank rather than showing the last one's last lap.
      npxLapVerdict = sector_purple::Verdict::kNone;
      npxSectorVerdict = sector_purple::Verdict::kNone;
    }
  }

  npxPushFrame(frame);
}

void NEOPIXEL_SLEEP() {
  // Unconditional, npxReady or not: driven-LOW GPIO is retained through
  // System OFF, so this is what actually turns the 5 V rail off for the
  // night (the "blue LED stays on after sleep" precedent).
  if (npxReady) {
    npxStrip.clear();
    npxStrip.show();
    delay(1);  // let the last frame latch before the rail drops
    npxReady = false;
  }
  pinMode(NEOPIXEL_PIN_DATA, OUTPUT);
  digitalWrite(NEOPIXEL_PIN_DATA, LOW);
  npxBoostDisable();
}

void NEOPIXEL_WAKE() {
  if (settingLedBrightness == 0) {
    return;  // disabled: rail stays off
  }
  npxBoostEnable();
  delay(NPX_BOOST_SETTLE_MS);
  npxStrip.begin();
  npxStrip.clear();
  npxStrip.show();
  npxReady = true;
}

#else  // !BIRDSEYE_ENABLE_NEOPIXEL

///////////////////////////////////////////
// SUBSYSTEM COMPILED OUT
//
// Since 4.1.0 no shipped channel takes this branch — the flag defaults
// to 1 (see project.h). It is reached only by a build that forces
// -DBIRDSEYE_ENABLE_NEOPIXEL=0, which is why the already-converted case
// below matters more than it looks: the boards most likely to run such a
// build are ones that already ran a flag-on one.
//
// No UICR write, no Adafruit_NeoPixel dependency in the image. On a
// board that has never run a flag-on build the pads stay exactly as the
// chip shipped, and this file drives nothing at all.
//
// ONE exception, and it is a power bug if you remove it: a board that
// HAS run a flag-on build carries the one-way UICR NFC->GPIO conversion
// forever, and a later flag-off image (a beta unit updating to a prod
// release) inherits it. With stubs that truly do nothing, P0.09 — the
// boost converter's EN — is left in its reset state (input, disconnected)
// for the whole session AND through System OFF, where a driven-LOW level
// is the only thing that holds the rail down (the same retention that
// caused the "blue conn LED stays on after sleep" report, subsystem 10).
// EN floating on the Adafruit boost module reads as enabled, so "off"
// keeps the 5 V rail and 11 idle WS2812s alive on a device with no power
// switch — a flat pack in a day or two.
//
// So: drive EN low, but ONLY when the conversion has already happened.
// PROTECT clear (0) means the pads are already GPIO. On an unconverted
// board the bit is set, this is skipped, and the promise above holds
// exactly — we never touch a pad the user didn't opt into.
///////////////////////////////////////////

static void npxHoldConvertedBoostOff() {
#if BIRDSEYE_ENABLE_PROFILING
  // The profiler owns this pin (plan 0011) — leave it alone entirely,
  // including the retained-LOW trick below. A build with the strip
  // compiled out AND profiling on drives no LEDs, so the rail it leaves
  // up is powering nothing but the boost converter's own quiescent
  // draw; the pin's scope signal is worth more than that.
  return;
#else
  if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) != 0) {
    return;  // pads still NFC — never touched by a flag-off build
  }
  pinMode(NEOPIXEL_PIN_BOOST_EN, OUTPUT);
  digitalWrite(NEOPIXEL_PIN_BOOST_EN, LOW);
#endif
}

void NEOPIXEL_SETUP() { npxHoldConvertedBoostOff(); }
void NEOPIXEL_LOOP() {}
// Re-assert before System OFF: the level is retained there, and that is
// the case that costs a battery rather than a few mA of run current.
void NEOPIXEL_SLEEP() { npxHoldConvertedBoostOff(); }
void NEOPIXEL_WAKE() {}
void neopixelNotifyPurpleSector() {}
void neopixelNotifyPurpleLap() {}

#endif  // BIRDSEYE_ENABLE_NEOPIXEL
