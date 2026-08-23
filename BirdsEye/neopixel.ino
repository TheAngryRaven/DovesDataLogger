#include "neopixel.h"
#include "project.h"

#if BIRDSEYE_ENABLE_NEOPIXEL

#include <Adafruit_NeoPixel.h>

#include "led_animations.h"
#include "led_frame.h"
#include "led_modes.h"
#include "local_time.h"
#include "nan_bits.h"
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
static uint32_t npxPurpleStartMs = 0;
static uint32_t npxPurpleSeed = 0;

// Purple-sector monitor + status LED latches.
static sector_purple::State npxPurpleMon;
static led_modes::StatusState npxRevState;
static led_modes::StatusState npxEgtState;
static led_modes::StatusState npxOverrevState;

// Status LED assignments. Hardcoded defaults for now — phase 2 parses
// user settings into these same PODs (led_modes.h). Thresholds that
// depend on settings are filled in at NEOPIXEL_SETUP().
static led_modes::StatusAction npxRevAction = {
    led_modes::Source::kRpm, 15000.0f, 14550.0f, led_frame::kRed,
    led_modes::kRevFlashHalfPeriodMs, led_frame::kOff};
// Temp1 tri-state (plan 0007): red flash when hot, OFF when good, solid
// BLUE when there is no probe signal (NaN/stale — a dropout is
// information, not silence).
static led_modes::StatusAction npxEgtAction = {
    led_modes::Source::kEgtC, led_modes::kEgtAlertC,
    led_modes::kEgtAlertC - led_modes::kEgtClearDeltaC, led_frame::kRed,
    led_modes::kEgtFlashHalfPeriodMs, led_frame::kBlue};
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

  // Rev flasher tracks the configured limit; clear just below it so
  // Kalman jitter at the limiter can't strobe the latch.
  npxRevAction.threshold = (float)settingRevLimit;
  npxRevAction.clearBelow = (float)settingRevLimit * led_modes::kRevClearFrac;
  // Overrev: fire at the problem limit, release only once back under
  // the NORMAL limit's clear point — the whole band stays latched.
  if (settingOverrevLimit > 0) {
    npxOverrevAction.source = led_modes::Source::kRpm;
    npxOverrevAction.threshold = (float)settingOverrevLimit;
    npxOverrevAction.clearBelow =
        (float)settingRevLimit * led_modes::kRevClearFrac;
  }
  // Temp1 alert threshold from the setting (Celsius), fixed clear delta.
  npxEgtAction.threshold = (float)settingTemp1AlertC;
  npxEgtAction.clearBelow =
      (float)settingTemp1AlertC - led_modes::kEgtClearDeltaC;

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
  npxPurpleActive = true;
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

void NEOPIXEL_LOOP() {
  if (!npxReady) {
    return;
  }
  const uint32_t now = millis();
  if (now - npxLastFrameMs < NPX_FRAME_INTERVAL_MS) {
    return;
  }
  npxLastFrameMs = now;

  // Purple-sector monitor: poll-and-diff against the active timer.
  {
    sector_purple::Sample smp;
    smp.sectorsConfigured = activeTimerSectorsConfigured();
    smp.raceStarted = activeTimerRaceStarted();
    smp.currentSector = activeTimerCurrentSector();
    smp.laps = activeTimerLaps();
    smp.lastLapTime = activeTimerLastLapTime();
    for (int s = 0; s < 3; s++) {
      smp.lapSectorTime[s] = activeTimerLapSectorTime(s + 1);
      smp.bestSectorTime[s] = activeTimerBestSectorTime(s + 1);
    }
    if (sector_purple::update(npxPurpleMon, smp) != 0) {
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
  // Priority 3: purple celebration.
  if (!rendered && npxPurpleActive) {
    rendered = led_animations::renderPurple(now - npxPurpleStartMs,
                                            npxPurpleSeed, frame);
    npxPurpleActive = rendered;
  }
  // Priority 4/5: parked or menu -> off; racing -> mode + status.
  if (!rendered) {
    led_frame::clear(frame);
    const bool parked = bleActive || usbMscActive;
    if (!parked && raceActive) {
      // Strip selection (plan 0007 order):
      //   engine died (proven tach session, RPM 0) -> bar OFF — a pace
      //     pip counting next to a dead engine reads as a glitch;
      //   no GPS lock -> green search pip (same fix+timeValid gate that
      //     allows log-file creation);
      //   pace valid -> pace pip;
      //   else -> RPM scale.
      led_frame::Rgb stripPx[led_frame::kStripCount];
      bool stripOff = raceEngineStopped();
      if (!stripOff) {
        if (!gpsData.fix || !gpsData.timeValid) {
          led_modes::renderSearchPip(now, stripPx);
        } else {
          const bool paceValid =
              activeTimerRaceStarted() && activeTimerLaps() >= 1 &&
              !(sprintModeIsActive() && !activeTimerRunActive());
          if (paceValid) {
            led_modes::renderPace(activeTimerPaceDifference(), stripPx);
          } else {
            const led_modes::ScaleSpec rpmSpec = {
                0.0f, (float)settingRevLimit, led_modes::kRpmRedFrac,
                led_frame::kGreen, led_frame::kRed};
            led_modes::renderScale((float)tachLastReported, rpmSpec, stripPx);
          }
        }
        for (int i = 0; i < led_frame::kStripCount; i++) {
          frame.px[led_frame::kStripFirst + i] = stripPx[i];
        }
      }
      // Status LEDs stay live even with the bar off — a hot engine
      // cooling after a stall is exactly when the temp alert matters.
      // RPM is always a live value (0 when stopped); EGT validity is
      // the NaN gate — isNanF, never isnan (-Ofast folds isnan to
      // false, see nan_bits.h). NaN -> the action's invalidColor
      // (solid blue = no probe signal).
      frame.px[led_frame::kStatusLeft] = led_modes::evalStatus(
          npxRevAction, npxRevState, (float)tachLastReported, true, now);
      const float egtC = sensoreggEgtC();
      frame.px[led_frame::kStatusRight] = led_modes::evalStatus(
          npxEgtAction, npxEgtState, egtC, !isNanF(egtC), now);
    } else {
      // Out of race: release the latches so a stale alert can't flash
      // the instant the next session starts.
      npxRevState.active = false;
      npxEgtState.active = false;
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

#endif  // BIRDSEYE_ENABLE_NEOPIXEL
