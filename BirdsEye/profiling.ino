#include "profiling.h"

#include "project.h"

#if BIRDSEYE_ENABLE_PROFILING

#include "loop_profile.h"

// The nRF52840 core clock. Arduino's platform.txt defines F_CPU; the
// fallback keeps this file honest if that ever stops being true.
#ifndef F_CPU
  #define F_CPU 64000000UL
#endif

static loop_profile::State profState;
static loop_profile::Report profLastReport;
static uint32_t profSectionStart[loop_profile::kSectionCount];

// Timebase. DWT counts CPU cycles (64 per microsecond) and is the only
// way to see a section that costs a few hundred nanoseconds; micros()
// is the fallback when a debugger owns DWT or TRCENA will not stick.
static bool profUseDwt = false;
static uint32_t profTicksPerUs = 1;

// Cached port + mask for the profiling pin so an edge is a single
// register store rather than a digitalWrite() pin-map lookup — the pin
// exists to be a ground truth the software timing can be checked
// against, so it must not carry meaningful cost of its own.
static NRF_GPIO_Type* profPinPort = nullptr;
static uint32_t profPinMask = 0;

static inline uint32_t profNow() {
  return profUseDwt ? DWT->CYCCNT : (uint32_t)micros();
}

static inline void profPinSet(bool high) {
  if (profPinPort == nullptr) return;
  if (high) {
    profPinPort->OUTSET = profPinMask;
  } else {
    profPinPort->OUTCLR = profPinMask;
  }
}

/**
 * @brief Turn the DWT cycle counter on and prove it is counting.
 *
 * TRCENA can be held off by a debug probe, and CYCCNTENA is optional in
 * the architecture, so "we set the bit" is not evidence. Read the
 * counter across a short spin and require it to have moved; if it has
 * not, the caller falls back to micros() rather than reporting a window
 * full of zeros.
 */
static bool profEnableDwt() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  const uint32_t before = DWT->CYCCNT;
  for (volatile int i = 0; i < 64; i++) {
  }
  return DWT->CYCCNT != before;
}

/**
 * @brief Claim the profiling pin, unless it is an NFC pad that has not
 * been converted to GPIO.
 *
 * The conversion is NEOPIXEL_SETUP's one-way UICR write (subsystem 16)
 * and this module deliberately does not perform it: spending a board's
 * NFC pads is a decision that belongs to the LED feature, not to a
 * bench tool. On an unconverted board the pin is refused and the
 * software rollup runs alone — profilingPinLive() reports that, and the
 * page shows it, so a silent scope is never a mystery.
 */
static void profClaimPin() {
#if (PROFILING_PIN == 30) || (PROFILING_PIN == 31)
  if ((NRF_UICR->NFCPINS & UICR_NFCPINS_PROTECT_Msk) != 0) {
    debugln(F("Profiling: NFC pads not converted — pin unavailable"));
    return;
  }
#endif
  pinMode(PROFILING_PIN, OUTPUT);
  digitalWrite(PROFILING_PIN, LOW);

  const uint32_t psel = g_ADigitalPinMap[PROFILING_PIN];
  profPinPort = (psel >= 32) ? NRF_P1 : NRF_P0;
  profPinMask = 1UL << (psel & 31);
  profPinSet(false);
}

void PROFILING_SETUP() {
  profUseDwt = profEnableDwt();
  profTicksPerUs = profUseDwt ? (uint32_t)(F_CPU / 1000000UL) : 1UL;
  for (uint8_t i = 0; i < loop_profile::kSectionCount; i++) {
    profSectionStart[i] = 0;
  }
  profLastReport.valid = false;

  profClaimPin();
  loop_profile::reset(profState, profNow(), millis());

  debug(F("Profiling: timebase "));
  debug(profUseDwt ? F("DWT") : F("micros"));
  debug(F(", pin "));
  debugln(profPinPort != nullptr ? F("live") : F("unavailable"));
}

void PROFILING_SLEEP() {
  profPinSet(false);
}

// The pin edges are #if'd rather than compared at runtime: with the
// default whole-loop setting the comparison is provably false for every
// valid section, and a dead branch in the two hottest functions in the
// firmware is exactly the kind of cost a profiler must not add to what
// it measures.
void profSectionBegin(uint8_t section) {
  if (section >= loop_profile::kSectionCount) return;
#if PROFILING_PIN_SECTION != PROFILING_PIN_WHOLE_LOOP
  if (section == (uint8_t)(PROFILING_PIN_SECTION)) profPinSet(true);
#endif
  profSectionStart[section] = profNow();
}

void profSectionEnd(uint8_t section) {
  if (section >= loop_profile::kSectionCount) return;
  const uint32_t elapsed =
      loop_profile::ticksSince(profSectionStart[section], profNow());
#if PROFILING_PIN_SECTION != PROFILING_PIN_WHOLE_LOOP
  if (section == (uint8_t)(PROFILING_PIN_SECTION)) profPinSet(false);
#endif
  loop_profile::addSection(profState, section, elapsed);
}

const loop_profile::Report& profilingReport() { return profLastReport; }

const char* profilingTimebaseTag() { return profUseDwt ? "DWT" : "us"; }

bool profilingPinLive() { return profPinPort != nullptr; }

ProfLoopScope::ProfLoopScope() : startTicks_(profNow()) {
#if PROFILING_PIN_SECTION == PROFILING_PIN_WHOLE_LOOP
  profPinSet(true);
#endif
}

ProfLoopScope::~ProfLoopScope() {
  const uint32_t now = profNow();
#if PROFILING_PIN_SECTION == PROFILING_PIN_WHOLE_LOOP
  profPinSet(false);
#endif
  loop_profile::addLoop(profState,
                        loop_profile::ticksSince(startTicks_, now));

  // The rollup lives here rather than in a PROFILING_LOOP() call at the
  // bottom of loop() for the same reason the loop span does: the BLE
  // and USB parked branches return early, and a profiler that stops
  // reporting exactly when the firmware parks is worse than none.
  // millis(), not the tick counter, decides when the window is over —
  // see the loop_profile header. The tick counter measures durations;
  // it is not a clock.
  if (loop_profile::rollup(profState, now, millis(), PROFILING_WINDOW_MS,
                           profTicksPerUs, profLastReport)) {
#ifdef HAS_DEBUG
    debug(F("PROFILE "));
    debug(profLastReport.loopRateHz);
    debug(F("Hz mean "));
    debug(profLastReport.loopMeanUs);
    debug(F("us max "));
    debug(profLastReport.loopMaxUs);
    debug(F("us busy "));
    debug(profLastReport.busyPermille);
    debug(F("/1000 awake "));
    debug(profLastReport.awakeUs);
    debug(F("/"));
    debug(profLastReport.windowUs);
    debug(F("us | "));
    for (uint8_t i = 0; i < loop_profile::kReportSlots; i++) {
      debug(loop_profile::sectionTag(i));
      debug(F(":"));
      debug(profLastReport.permille[i] / 10);
      debug(F("."));
      debug(profLastReport.permille[i] % 10);
      debug(F(" "));
    }
    debugln(F(""));
#endif
  }
}

#else  // !BIRDSEYE_ENABLE_PROFILING

// Nothing to do and nothing to hold: unlike the NeoPixel stubs, this
// module never owns a pin in a flag-off build, so there is no retained
// GPIO state to re-assert before System OFF.
void PROFILING_SETUP() {}
void PROFILING_SLEEP() {}

#endif  // BIRDSEYE_ENABLE_PROFILING
