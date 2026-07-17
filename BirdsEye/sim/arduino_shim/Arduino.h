#pragma once

///////////////////////////////////////////
// Arduino.h — host shim (sim build only)
//
// Just enough of the Arduino core (plus the nRF52 core's transitive
// surface: CMSIS registers, SoftDevice calls, board pin macros) for the
// BirdsEye sketch and the real DovesLapTimer / SparkFun-header code to
// compile on a desktop toolchain. Time resolves against the virtual
// clock; pins resolve against a simple pin-state map the host drives.
//
// Behavioral rules:
//  - delay()/delayMicroseconds() ADVANCE virtual time (firmware blocking
//    waits consume time instead of hanging the host).
//  - digitalRead() returns the pin-state map; INPUT_PULLUP defaults HIGH.
//  - Registers are plain RAM structs: writes land, reads see them, and
//    nothing else happens. NVIC_SystemReset() throws SimResetRequest so
//    the host driver can observe a firmware-initiated reboot.
///////////////////////////////////////////

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "../virtual_clock.h"

// ---------------------------------------------------------------- types
typedef bool boolean;
typedef uint8_t byte;
typedef uint16_t word;

// ------------------------------------------------------------ constants
#define HIGH 1
#define LOW 0

#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define INPUT_PULLDOWN 3

#define LSBFIRST 0
#define MSBFIRST 1

#define RISING 1
#define FALLING 2
#define CHANGE 3

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// --------------------------------------------------------------- PROGMEM
#define PROGMEM
#define PGM_P const char*
#define PSTR(s) (s)
#define pgm_read_byte(addr) (*(const unsigned char*)(addr))
#define pgm_read_word(addr) (*(const unsigned short*)(addr))
#define pgm_read_dword(addr) (*(const unsigned long*)(addr))
#define memcpy_P memcpy
#define strcpy_P strcpy
#define strcmp_P strcmp
#define strlen_P strlen
#define snprintf_P snprintf

// F() — flash-string helper collapses to a plain C string on host.
class __FlashStringHelper;
#define F(str) (reinterpret_cast<const __FlashStringHelper*>(str))

// ------------------------------------------------------------ min / max
// The Arduino core defines these as macros; templates avoid the classic
// macro/std collisions while behaving the same for firmware code.
template <typename A, typename B>
static inline auto min(A a, B b) -> decltype(a < b ? a : b) {
  return a < b ? a : b;
}
template <typename A, typename B>
static inline auto max(A a, B b) -> decltype(a > b ? a : b) {
  return a > b ? a : b;
}
template <typename T, typename L, typename H>
static inline T constrain(T v, L lo, H hi) {
  return v < (T)lo ? (T)lo : (v > (T)hi ? (T)hi : v);
}
static inline long map(long x, long inMin, long inMax, long outMin, long outMax) {
  return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}
#define sq(x) ((x) * (x))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define lowByte(w) ((uint8_t)((w) & 0xff))
#define highByte(w) ((uint8_t)((w) >> 8))

// ------------------------------------------------------------------ time
unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
static inline void yield(void) {}

// ------------------------------------------------------------------ pins
// XIAO nRF52840 pin names the sketch uses. Values are arbitrary but
// stable indices into the sim pin-state map (matching the variant's
// Arduino numbering where it matters — D0..D10 are 0..10).
#define D0 0
#define D1 1
#define D2 2
#define D3 3
#define D4 4
#define D5 5
#define D6 6
#define D7 7
#define D8 8
#define D9 9
#define D10 10

// NOTE: deliberately NOT the variant's D4/D5 — the sim build's buttons
// live on pins 4/5/6 (the .ino's #ifdef SIM branch), and i2cBusRecover()
// bit-bangs SDA/SCL with digitalWrite. Distinct numbers keep a recovery
// from ghost-pressing buttons in the pin-state map.
#define PIN_WIRE_SDA 20
#define PIN_WIRE_SCL 21
#define PIN_VBAT 32
#define VBAT_ENABLE 14
#define PIN_CHARGING_CURRENT 22
#define PIN_LSM6DS3TR_C_POWER 40
#define LED_BUILTIN 13

#define ADC_RESOLUTION 12

#define SIM_NUM_PINS 48

void pinMode(uint32_t pin, uint32_t mode);
void digitalWrite(uint32_t pin, uint32_t val);
int digitalRead(uint32_t pin);
int analogRead(uint32_t pin);
void analogReadResolution(int bits);

// Interrupts: record the handler so the host can synthesize pulses
// (tach ISR injection — Phase 3 of the sim plan).
typedef void (*voidFuncPtr)(void);
#define digitalPinToInterrupt(p) (p)
void attachInterrupt(uint32_t pin, voidFuncPtr handler, int mode);
void detachInterrupt(uint32_t pin);
static inline void interrupts(void) {}
static inline void noInterrupts(void) {}

// Host-side access to the pin map + attached ISRs (sim glue only).
void simPinSetLevel(uint32_t pin, int level);
int simPinGetLevel(uint32_t pin);
voidFuncPtr simPinGetIsr(uint32_t pin);

// ----------------------------------------------------------------- Print
// Minimal Print/Stream hierarchy: enough for the debug macros, the
// SparkFun header, DovesLapTimer's Stream* debug hook, and the display
// shim. write(uint8_t) is the single funnel, like the real core.
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  virtual size_t write(const uint8_t* buffer, size_t size) {
    size_t n = 0;
    while (size--) n += write(*buffer++);
    return n;
  }
  size_t write(const char* s) {
    if (!s) return 0;
    return write(reinterpret_cast<const uint8_t*>(s), strlen(s));
  }

  size_t print(const char* s) { return write(s); }
  size_t print(const __FlashStringHelper* s) {
    return write(reinterpret_cast<const char*>(s));
  }
  size_t print(char c) { return write((uint8_t)c); }
  size_t print(unsigned char v, int base = DEC) { return printULong(v, base); }
  size_t print(int v, int base = DEC) { return printLong(v, base); }
  size_t print(unsigned int v, int base = DEC) { return printULong(v, base); }
  size_t print(long v, int base = DEC) { return printLong(v, base); }
  size_t print(unsigned long v, int base = DEC) { return printULong(v, base); }
  size_t print(long long v, int base = DEC) { return printLong((long)v, base); }
  size_t print(unsigned long long v, int base = DEC) {
    return printULong((unsigned long)v, base);
  }
  size_t print(double v, int digits = 2) {
    char buf[40];
    snprintf(buf, sizeof(buf), "%.*f", digits, v);
    return write(buf);
  }

  template <typename T>
  size_t println(T v) {
    size_t n = print(v);
    return n + write((uint8_t)'\n');
  }
  template <typename T>
  size_t println(T v, int base) {
    size_t n = print(v, base);
    return n + write((uint8_t)'\n');
  }
  size_t println(void) { return write((uint8_t)'\n'); }

 private:
  size_t printLong(long v, int base) {
    char buf[24];
    if (base == DEC) {
      snprintf(buf, sizeof(buf), "%ld", v);
    } else {
      snprintf(buf, sizeof(buf), base == HEX ? "%lx" : "%lo", (unsigned long)v);
    }
    return write(buf);
  }
  size_t printULong(unsigned long v, int base) {
    char buf[24];
    snprintf(buf, sizeof(buf), base == HEX ? "%lx" : (base == OCT ? "%lo" : "%lu"),
             v);
    return write(buf);
  }
};

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() = 0;
  virtual void flush() {}
  void setTimeout(unsigned long) {}

  size_t readBytes(uint8_t* buffer, size_t length) {
    size_t n = 0;
    while (n < length) {
      int c = read();
      if (c < 0) break;  // no timeout wait: virtual time, empty = done
      buffer[n++] = (uint8_t)c;
    }
    return n;
  }
  size_t readBytes(char* buffer, size_t length) {
    return readBytes(reinterpret_cast<uint8_t*>(buffer), length);
  }
};

// ------------------------------------------------------------ HardwareSerial
// Serial  — debug console; writes go to stdout (native) / console (wasm).
// Serial1 — GPS UART; a dead end in the sim (PVT is injected directly
//           into onPVTReceived, never parsed from serial bytes).
class HardwareSerial : public Stream {
 public:
  explicit HardwareSerial(bool echoToStdout) : echo_(echoToStdout) {}
  void begin(unsigned long) {}
  void begin(unsigned long, uint16_t) {}
  void end() {}
  operator bool() { return true; }

  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  void flush() override {}

  size_t write(uint8_t c) override {
    if (echo_) fputc(c, stdout);
    return 1;
  }
  using Print::write;

 private:
  bool echo_;
};

extern HardwareSerial Serial;
extern HardwareSerial Serial1;

// ----------------------------------------------------------------- String
// Minimal Arduino String (std::string-backed). The firmware avoids String
// in hot paths (project convention); this exists for library headers
// (SparkFun) that name the type.
#include <string>

class String {
 public:
  String() {}
  String(const char* s) : s_(s ? s : "") {}
  String(const std::string& s) : s_(s) {}
  explicit String(int v) : s_(std::to_string(v)) {}
  explicit String(unsigned int v) : s_(std::to_string(v)) {}
  explicit String(long v) : s_(std::to_string(v)) {}
  explicit String(unsigned long v) : s_(std::to_string(v)) {}
  explicit String(double v) : s_(std::to_string(v)) {}

  const char* c_str() const { return s_.c_str(); }
  unsigned int length() const { return (unsigned int)s_.size(); }
  char charAt(unsigned int i) const { return i < s_.size() ? s_[i] : 0; }
  char operator[](unsigned int i) const { return charAt(i); }

  String& operator+=(const String& o) {
    s_ += o.s_;
    return *this;
  }
  String& operator+=(const char* o) {
    s_ += (o ? o : "");
    return *this;
  }
  String& operator+=(char c) {
    s_ += c;
    return *this;
  }
  friend String operator+(String a, const String& b) { return a += b; }
  bool operator==(const String& o) const { return s_ == o.s_; }
  bool operator==(const char* o) const { return s_ == (o ? o : ""); }

  bool startsWith(const String& p) const {
    return s_.rfind(p.s_, 0) == 0;
  }
  int indexOf(char c) const {
    auto pos = s_.find(c);
    return pos == std::string::npos ? -1 : (int)pos;
  }
  String substring(unsigned int from) const {
    return from <= s_.size() ? String(s_.substr(from)) : String();
  }
  String substring(unsigned int from, unsigned int to) const {
    if (from > s_.size()) return String();
    if (to > s_.size()) to = (unsigned int)s_.size();
    return String(s_.substr(from, to - from));
  }
  void toCharArray(char* buf, unsigned int size) const {
    if (!buf || size == 0) return;
    strncpy(buf, s_.c_str(), size - 1);
    buf[size - 1] = '\0';
  }
  long toInt() const { return atol(s_.c_str()); }
  float toFloat() const { return (float)atof(s_.c_str()); }

 private:
  std::string s_;
};

// ---------------------------------------------------------------- dtostrf
char* dtostrf(double val, signed char width, unsigned char prec, char* out);

// ------------------------------------------------------------- randomness
// Deterministic: the sim must produce identical runs. Firmware only uses
// random() for first-boot default settings, and the sim ships a fixed
// SETTINGS.json, so this path shouldn't fire — keep it deterministic anyway.
long random(long maxExclusive);
long random(long minInclusive, long maxExclusive);
void randomSeed(unsigned long seed);

// ================================================================== nRF52
// CMSIS/Nordic surface the sketch touches directly. Plain RAM structs:
// reads/writes land in memory and nothing else happens.

typedef struct {
  volatile uint32_t RESETREAS;
  volatile uint32_t USBREGSTATUS;
  volatile uint32_t SYSTEMOFF;
  volatile uint32_t GPREGRET;
  volatile uint32_t GPREGRET2;
} SIM_NRF_POWER_Type;

typedef struct {
  volatile uint32_t LATCH;
} SIM_NRF_GPIO_Type;

typedef struct {
  volatile uint32_t CONFIG;
  volatile uint32_t CRV;
  volatile uint32_t RREN;
  volatile uint32_t TASKS_START;
  volatile uint32_t RR[8];
} SIM_NRF_WDT_Type;

typedef struct {
  volatile uint32_t TASKS_START;
  volatile uint32_t TASKS_STOP;
  volatile uint32_t TASKS_CLEAR;
  volatile uint32_t MODE;
  volatile uint32_t BITMODE;
  volatile uint32_t PRESCALER;
  volatile uint32_t CC[6];
  volatile uint32_t SHORTS;
  volatile uint32_t INTENSET;
  volatile uint32_t INTENCLR;
  volatile uint32_t EVENTS_COMPARE[6];
} SIM_NRF_TIMER_Type;

extern SIM_NRF_POWER_Type* NRF_POWER;
extern SIM_NRF_GPIO_Type* NRF_P0;
extern SIM_NRF_GPIO_Type* NRF_P1;
extern SIM_NRF_WDT_Type* NRF_WDT;
extern SIM_NRF_TIMER_Type* NRF_TIMER3;
extern SIM_NRF_TIMER_Type* NRF_TIMER4;

#define POWER_USBREGSTATUS_VBUSDETECT_Msk (1UL << 0)

#define WDT_CONFIG_SLEEP_Run 1
#define WDT_CONFIG_SLEEP_Pos 0
#define WDT_RREN_RR0_Enabled 1
#define WDT_RREN_RR0_Pos 0
#define WDT_RR_RR_Reload 0x6E524635UL

#define TIMER_MODE_MODE_Timer 0
#define TIMER_BITMODE_BITMODE_32Bit 3
#define TIMER_SHORTS_COMPARE0_CLEAR_Msk (1UL << 0)
#define TIMER_INTENSET_COMPARE0_Msk (1UL << 16)
#define TIMER_INTENCLR_COMPARE0_Msk (1UL << 16)

// NVIC / core intrinsics
typedef int IRQn_Type;
#define TIMER3_IRQn 26
#define TIMER4_IRQn 27
#define FPU_IRQn 44

static inline void NVIC_SetPriority(IRQn_Type, uint32_t) {}
static inline void NVIC_EnableIRQ(IRQn_Type) {}
static inline void NVIC_DisableIRQ(IRQn_Type) {}
static inline void NVIC_ClearPendingIRQ(IRQn_Type) {}
static inline void __DSB(void) {}
static inline void __WFE(void) {}
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline uint32_t __get_FPSCR(void) { return 0; }
static inline void __set_FPSCR(uint32_t) {}

// Firmware-initiated reboot. The sim can't re-run static initializers in
// place, so this throws; the host driver catches it, reports it, and (for
// the wasm build) the page re-instantiates the module for a true reset.
struct SimResetRequest {};
[[noreturn]] void NVIC_SystemReset(void);

// Arduino pin -> P-number map (identity is fine for the sim: wake_cause
// masks and SENSE config only need stable, consistent numbers).
extern const uint32_t g_ADigitalPinMap[SIM_NUM_PINS];

// -------------------------------------------------------------- SoftDevice
static inline uint32_t sd_softdevice_is_enabled(uint8_t* enabled) {
  *enabled = 0;
  return 0;
}
static inline uint32_t sd_app_evt_wait(void) { return 0; }
static inline uint32_t sd_power_system_off(void) { return 0; }
static inline uint32_t sd_power_reset_reason_clr(uint32_t) { return 0; }

// ------------------------------------------------------- FreeRTOS critical
// Single-threaded sim: the Bluefruit callback task doesn't exist, so the
// critical sections protect nothing — but they must compile.
static inline void taskENTER_CRITICAL(void) {}
static inline void taskEXIT_CRITICAL(void) {}
