#include "neopixel.h"
#include "project.h"

#if BIRDSEYE_ENABLE_NEOPIXEL

#include <Adafruit_NeoPixel.h>

#include "led_animations.h"
#include "led_frame.h"
#include "led_modes.h"
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

// Status LED assignments. Hardcoded defaults for now — phase 2 parses
// user settings into these same PODs (led_modes.h). Thresholds that
// depend on settings are filled in at NEOPIXEL_SETUP().
static led_modes::StatusAction npxRevAction = {
    led_modes::Source::kRpm, 15000.0f, 14550.0f, led_frame::kRed,
    led_modes::kRevFlashHalfPeriodMs};
static led_modes::StatusAction npxEgtAction = {
    led_modes::Source::kEgtC, led_modes::kEgtAlertC, led_modes::kEgtClearC,
    led_frame::kOrange, led_modes::kEgtFlashHalfPeriodMs};

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
  // rail off, data low can't back-power an unpowered strip.
  pinMode(NEOPIXEL_PIN_BOOST_EN, OUTPUT);
  digitalWrite(NEOPIXEL_PIN_BOOST_EN, LOW);
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

  digitalWrite(NEOPIXEL_PIN_BOOST_EN, HIGH);
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
 * @brief Push a composed frame: apply THE brightness cap (the single
 * choke point — after this no channel exceeds settingLedBrightness),
 * map logical left-to-right onto the physical wire (the chain is wired
 * data-in on the RIGHT — led_frame::kChainReversed), and show.
 */
static void npxPushFrame(led_frame::Frame& frame) {
  led_frame::applyCap(frame, settingLedBrightness);
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
  // Priority 2: purple celebration.
  if (!rendered && npxPurpleActive) {
    rendered = led_animations::renderPurple(now - npxPurpleStartMs,
                                            npxPurpleSeed, frame);
    npxPurpleActive = rendered;
  }
  // Priority 3/4: parked or menu -> off; racing -> mode + status.
  if (!rendered) {
    led_frame::clear(frame);
    const bool parked = bleActive || usbMscActive;
    if (!parked && raceActive) {
      // Strip: pace pip once pace means something, RPM scale until then
      // (and between sprint runs, where pace shows *waiting* on the
      // OLED too).
      const bool paceValid =
          activeTimerRaceStarted() && activeTimerLaps() >= 1 &&
          !(sprintModeIsActive() && !activeTimerRunActive());
      led_frame::Rgb stripPx[led_frame::kStripCount];
      if (paceValid) {
        led_modes::renderPace(activeTimerPaceDifference(), stripPx);
      } else {
        const led_modes::ScaleSpec rpmSpec = {
            0.0f, (float)settingRevLimit, led_modes::kRpmRedFrac,
            led_frame::kGreen, led_frame::kRed};
        led_modes::renderScale((float)tachLastReported, rpmSpec, stripPx);
      }
      for (int i = 0; i < led_frame::kStripCount; i++) {
        frame.px[led_frame::kStripFirst + i] = stripPx[i];
      }
      // Status LEDs: RPM is always a live value (0 when stopped); EGT
      // validity is the NaN gate — isNanF, never isnan (-Ofast folds
      // isnan to false, see nan_bits.h).
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
  pinMode(NEOPIXEL_PIN_BOOST_EN, OUTPUT);
  digitalWrite(NEOPIXEL_PIN_BOOST_EN, LOW);
}

void NEOPIXEL_WAKE() {
  if (settingLedBrightness == 0) {
    return;  // disabled: rail stays off
  }
  digitalWrite(NEOPIXEL_PIN_BOOST_EN, HIGH);
  delay(NPX_BOOST_SETTLE_MS);
  npxStrip.begin();
  npxStrip.clear();
  npxStrip.show();
  npxReady = true;
}

#else  // !BIRDSEYE_ENABLE_NEOPIXEL

///////////////////////////////////////////
// SUBSYSTEM COMPILED OUT (the master/release default — see project.h)
//
// No UICR write, no pin driving, no Adafruit_NeoPixel dependency in
// the image. The pads stay exactly as the chip shipped.
///////////////////////////////////////////

void NEOPIXEL_SETUP() {}
void NEOPIXEL_LOOP() {}
void NEOPIXEL_SLEEP() {}
void NEOPIXEL_WAKE() {}
void neopixelNotifyPurpleSector() {}

#endif  // BIRDSEYE_ENABLE_NEOPIXEL
