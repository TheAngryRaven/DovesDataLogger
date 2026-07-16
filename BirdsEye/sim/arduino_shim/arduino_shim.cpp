#include "Arduino.h"

///////////////////////////////////////////
// Arduino shim implementation (sim build)
///////////////////////////////////////////

// ------------------------------------------------------------------ time
unsigned long millis(void) {
  return (unsigned long)(sim_clock::nowUs() / 1000ULL);
}

unsigned long micros(void) {
  return (unsigned long)sim_clock::nowUs();
}

void delay(unsigned long ms) { sim_clock::advanceUs((uint64_t)ms * 1000ULL); }

void delayMicroseconds(unsigned int us) { sim_clock::advanceUs(us); }

// ------------------------------------------------------------------ pins
namespace {
struct PinState {
  uint8_t mode = INPUT;
  uint8_t level = LOW;
  voidFuncPtr isr = nullptr;
};
PinState g_pins[SIM_NUM_PINS];

inline bool validPin(uint32_t pin) { return pin < SIM_NUM_PINS; }
}  // namespace

void pinMode(uint32_t pin, uint32_t mode) {
  if (!validPin(pin)) return;
  g_pins[pin].mode = (uint8_t)mode;
  // Pull-up idles the line HIGH (buttons/tach are active-LOW).
  if (mode == INPUT_PULLUP) g_pins[pin].level = HIGH;
  if (mode == INPUT_PULLDOWN) g_pins[pin].level = LOW;
}

void digitalWrite(uint32_t pin, uint32_t val) {
  if (!validPin(pin)) return;
  g_pins[pin].level = val ? HIGH : LOW;
}

int digitalRead(uint32_t pin) {
  if (!validPin(pin)) return LOW;
  return g_pins[pin].level;
}

int analogRead(uint32_t pin) {
  (void)pin;
  // Battery divider path is #ifndef SIM'd out of the sketch, but keep a
  // plausible mid-charge reading for anything else that asks: ~4.0 V
  // through the calibrated 3.024 divider math at VREF 3.6 / 12-bit.
  return 1505;
}

void analogReadResolution(int) {}

void attachInterrupt(uint32_t pin, voidFuncPtr handler, int) {
  if (!validPin(pin)) return;
  g_pins[pin].isr = handler;
}

void detachInterrupt(uint32_t pin) {
  if (!validPin(pin)) return;
  g_pins[pin].isr = nullptr;
}

void simPinSetLevel(uint32_t pin, int level) {
  if (!validPin(pin)) return;
  g_pins[pin].level = (uint8_t)(level ? HIGH : LOW);
}

int simPinGetLevel(uint32_t pin) {
  if (!validPin(pin)) return LOW;
  return g_pins[pin].level;
}

voidFuncPtr simPinGetIsr(uint32_t pin) {
  if (!validPin(pin)) return nullptr;
  return g_pins[pin].isr;
}

// ---------------------------------------------------------------- serial
HardwareSerial Serial(true);    // debug console -> stdout
HardwareSerial Serial1(false);  // GPS UART -> discarded

// ---------------------------------------------------------------- dtostrf
char* dtostrf(double val, signed char width, unsigned char prec, char* out) {
  // AVR semantics: fixed-point, right-aligned to `width` (never truncates).
  char fmt[16];
  snprintf(fmt, sizeof(fmt), "%%%d.%df", width, prec);
  sprintf(out, fmt, val);
  return out;
}

// ------------------------------------------------------------- randomness
namespace {
// Tiny deterministic LCG — NOT for anything that matters; just keeps the
// (unused in the sim) first-boot settings generator reproducible.
uint32_t g_rngState = 0x12345678u;
}  // namespace

void randomSeed(unsigned long seed) {
  if (seed != 0) g_rngState = (uint32_t)seed;
}

long random(long maxExclusive) {
  if (maxExclusive <= 0) return 0;
  g_rngState = g_rngState * 1664525u + 1013904223u;
  return (long)((g_rngState >> 8) % (uint32_t)maxExclusive);
}

long random(long minInclusive, long maxExclusive) {
  if (maxExclusive <= minInclusive) return minInclusive;
  return minInclusive + random(maxExclusive - minInclusive);
}

// ------------------------------------------------------------------ nRF52
namespace {
SIM_NRF_POWER_Type g_power = {};
SIM_NRF_GPIO_Type g_p0 = {};
SIM_NRF_GPIO_Type g_p1 = {};
SIM_NRF_WDT_Type g_wdt = {};
SIM_NRF_TIMER_Type g_timer3 = {};
SIM_NRF_TIMER_Type g_timer4 = {};
}  // namespace

SIM_NRF_POWER_Type* NRF_POWER = &g_power;
SIM_NRF_GPIO_Type* NRF_P0 = &g_p0;
SIM_NRF_GPIO_Type* NRF_P1 = &g_p1;
SIM_NRF_WDT_Type* NRF_WDT = &g_wdt;
SIM_NRF_TIMER_Type* NRF_TIMER3 = &g_timer3;
SIM_NRF_TIMER_Type* NRF_TIMER4 = &g_timer4;

void NVIC_SystemReset(void) { throw SimResetRequest{}; }

// Identity map: sim "P-numbers" == Arduino pin numbers. wake_cause and
// the SENSE-wake config only need stable consistent values.
const uint32_t g_ADigitalPinMap[SIM_NUM_PINS] = {
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
    16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
    32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47};
