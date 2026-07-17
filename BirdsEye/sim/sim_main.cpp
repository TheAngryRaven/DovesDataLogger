///////////////////////////////////////////
// sim_main.cpp — the firmware translation unit (sim build).
//
// Replicates the Arduino IDE build: the .ino files are concatenated (in
// Arduino's order: main sketch first, then alphabetical) into one C++
// translation unit, with sim_prototypes.h standing in for the IDE's
// auto-generated function prototypes.
//
// bluetooth.ino, camera_ble.ino, firmware_ota.ino and usb_msc.ino are
// DELIBERATELY ABSENT — no BLE / camera / USB-host in the demo scope.
// Their public surfaces are satisfied by stubs/module_stubs.cpp.
//
// The firmware sources compile UNMODIFIED (ground rule): everything
// sim-specific lives behind the SIM flag inside them, or out here.
///////////////////////////////////////////

#define SIM 1

#include "sim_prototypes.h"

#include "../BirdsEye.ino"

#include "../accelerometer.ino"
#include "../display_pages.ino"
#include "../display_ui.ino"
#include "../gps_functions.ino"
#include "../replay.ino"
#include "../sd_functions.ino"
#include "../settings.ino"
#include "../tachometer.ino"

///////////////////////////////////////////
// Host glue — lives in this TU so it can touch the sketch's globals.
///////////////////////////////////////////

#include "frame_hash.h"
#include "sim_host.h"

namespace {

// Main-loop pacing: the real loop free-runs at roughly this period, and
// every virtual-time consumer in the sketch (debounce, display refresh,
// idle timers) just needs time to move between iterations.
constexpr uint64_t kLoopQuantumUs = 4000;  // ~250 Hz

bool g_resetRequested = false;

// Buttons idx 0/1/2 -> the SIM build's button pins (see the #ifdef SIM
// branch of setupButtons() in display_ui.ino).
constexpr uint32_t kButtonPins[3] = {4, 5, 6};

// Tach pulse synthesis (sim_set_rpm). One pulse per rev, matching the
// firmware's tachRevsPerPulse = 1.
int g_simRpm = 0;
uint64_t g_pulsePeriodUs = 0;
uint64_t g_nextPulseUs = 0;

// Unix days -> civil date (Howard Hinnant's civil_from_days; public
// domain algorithm). Valid far beyond any GPS date the sim will see.
void civilFromDays(int64_t z, int* y, unsigned* m, unsigned* d) {
  z += 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = (unsigned)(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const int64_t yr = (int64_t)yoe + era * 400;
  const unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const unsigned mp = (5 * doy + 2) / 153;
  *d = doy - (153 * mp + 2) / 5 + 1;
  *m = mp < 10 ? mp + 3 : mp - 9;
  *y = (int)(yr + (*m <= 2));
}

}  // namespace

extern "C" {

void sim_init(void) {
  sim_clock::reset();
  sim_vfs::reset();
  g_resetRequested = false;
  try {
    setup();
  } catch (const SimResetRequest&) {
    g_resetRequested = true;
  }
}

int sim_step_millis(uint32_t deltaMs) {
  if (g_resetRequested) return -1;
  const uint64_t target = sim_clock::nowUs() + (uint64_t)deltaMs * 1000ULL;
  int iterations = 0;
  try {
    while (sim_clock::nowUs() < target) {
      const uint64_t quantumEnd = sim_clock::nowUs() + kLoopQuantumUs;
      // Fire tach pulses due inside this quantum at their exact virtual
      // microsecond, so micros() inside the real ISR reads pulse time.
      if (g_simRpm > 0) {
        while (g_nextPulseUs <= quantumEnd) {
          if (g_nextPulseUs > sim_clock::nowUs()) {
            sim_clock::advanceUs(g_nextPulseUs - sim_clock::nowUs());
          }
          TACH_COUNT_PULSE();
          g_nextPulseUs += g_pulsePeriodUs;
        }
      }
      if (quantumEnd > sim_clock::nowUs()) {
        sim_clock::advanceUs(quantumEnd - sim_clock::nowUs());
      }
      loop();
      iterations++;
    }
  } catch (const SimResetRequest&) {
    g_resetRequested = true;
    return -1;
  }
  return iterations;
}

void sim_inject_pvt(const SimPvt* p) {
  if (!p) return;

  // Field map per the handoff spec — everything onPVTReceived() reads.
  UBX_NAV_PVT_data_t pvt;
  memset(&pvt, 0, sizeof(pvt));

  pvt.lat = (int32_t)llround(p->lat * 1e7);
  pvt.lon = (int32_t)llround(p->lng * 1e7);
  pvt.hMSL = (int32_t)llround(p->altitude_m * 1000.0);        // m -> mm
  pvt.gSpeed = (int32_t)llround(p->speed_mph * 447.04);       // mph -> mm/s
  pvt.headMot = (int32_t)llround(p->heading_deg * 1e5);
  pvt.hAcc = (uint32_t)llround(p->h_acc_m * 1000.0);          // m -> mm
  pvt.pDOP = (uint16_t)llround(p->hdop * 100.0);
  pvt.numSV = (uint8_t)p->sats;
  // Only iTOW % 1000 is consumed (sub-second milliseconds).
  pvt.iTOW = (uint32_t)(p->timestamp_ms % 1000ULL);

  // Civil UTC from the Unix-ms timestamp (fw subtracts 2000 from year).
  const uint64_t totalSec = p->timestamp_ms / 1000ULL;
  int y;
  unsigned mo, dd;
  civilFromDays((int64_t)(totalSec / 86400ULL), &y, &mo, &dd);
  const uint32_t sod = (uint32_t)(totalSec % 86400ULL);
  pvt.year = (uint16_t)y;
  pvt.month = (uint8_t)mo;
  pvt.day = (uint8_t)dd;
  pvt.hour = (uint8_t)(sod / 3600);
  pvt.min = (uint8_t)((sod / 60) % 60);
  pvt.sec = (uint8_t)(sod % 60);

  if (p->fix) {
    pvt.fixType = 3;
    pvt.flags.bits.gnssFixOK = 1;
    pvt.valid.bits.validDate = 1;
    pvt.valid.bits.validTime = 1;
    pvt.valid.bits.fullyResolved = 1;
  }
  // fix == 0: fixType/flags/valid stay zero, numSV as given — exercises
  // the real "waiting for lock" boot screens.

  // Accel trace rides along with every PVT row (dovex columns).
  accelIMU.simX = p->accel_x;
  accelIMU.simY = p->accel_y;
  accelIMU.simZ = p->accel_z;

  onPVTReceived(&pvt);
}

int sim_inject_pvt_json(const char* json) {
  if (!json) return 0;
  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, json) != DeserializationError::Ok) return 0;
  SimPvt p = {};
  // timestamp as double: 2^53 ns of mantissa covers Unix-ms until long
  // after this firmware matters, and it sidesteps 32-bit long JSON ints.
  p.timestamp_ms = (unsigned long long)(double)(doc["timestamp"] | 0.0);
  p.lat = doc["lat"] | 0.0;
  p.lng = doc["lng"] | 0.0;
  p.altitude_m = doc["altitude_m"] | 0.0;
  p.speed_mph = doc["speed_mph"] | 0.0;
  p.heading_deg = doc["heading_deg"] | 0.0;
  p.h_acc_m = doc["h_acc_m"] | 0.0;
  p.hdop = doc["hdop"] | 0.0;
  p.sats = doc["sats"] | 0;
  p.fix = (doc["fix"] | false) ? 1 : 0;
  p.accel_x = doc["accelX"] | 0.0f;
  p.accel_y = doc["accelY"] | 0.0f;
  p.accel_z = doc["accelZ"] | 0.0f;
  sim_inject_pvt(&p);
  return 1;
}

void sim_set_rpm(int rpm) {
  if (rpm <= 0) {
    g_simRpm = 0;
    return;
  }
  const bool wasOff = (g_simRpm <= 0);
  g_simRpm = rpm;
  g_pulsePeriodUs = (uint64_t)(60000000.0 / (double)rpm);
  if (g_pulsePeriodUs == 0) g_pulsePeriodUs = 1;
  if (wasOff) {
    g_nextPulseUs = sim_clock::nowUs() + g_pulsePeriodUs;
  }
}

void sim_set_accel(float x, float y, float z) {
  accelIMU.simX = x;
  accelIMU.simY = y;
  accelIMU.simZ = z;
}

void sim_button_down(int idx) {
  if (idx < 0 || idx > 2) return;
  simPinSetLevel(kButtonPins[idx], LOW);  // active low
}

void sim_button_up(int idx) {
  if (idx < 0 || idx > 2) return;
  simPinSetLevel(kButtonPins[idx], HIGH);
}

int sim_reset_requested(void) { return g_resetRequested ? 1 : 0; }

const uint8_t* sim_framebuffer(void) {
  // The real Adafruit_GrayOLED buffer — every pixel in it came from the
  // real drawPixel(). 128x64 mono = 1024 bytes, page layout.
  return display.getBuffer();
}

uint32_t sim_frame_hash(void) {
  return frame_hash::fnv1a(display.getBuffer(), 1024);
}

uint32_t sim_millis(void) { return (uint32_t)millis(); }

int sim_current_page(void) { return currentPage; }

int sim_race_active(void) { return raceActive ? 1 : 0; }

int sim_lap_count(void) { return lapHistoryCount; }

int sim_rpm(void) { return tachLastReported; }

int sim_gps_fix(void) { return gpsData.fix ? 1 : 0; }

int sim_logging_active(void) { return sdDataLogInitComplete ? 1 : 0; }

int sim_track_detected(void) { return trackDetected ? 1 : 0; }

int sim_sats(void) { return gpsData.satellites; }

uint32_t sim_best_lap_ms(void) { return (uint32_t)activeTimerBestLapTime(); }

uint32_t sim_last_lap_ms(void) { return (uint32_t)activeTimerLastLapTime(); }

uint32_t sim_current_lap_ms(void) {
  return (uint32_t)activeTimerCurrentLapTime();
}

int sim_lap_anything_active(void) {
  return (courseManager != nullptr && courseManager->isLapAnythingActive())
             ? 1
             : 0;
}

const char* sim_course_name(void) {
  if (courseManager == nullptr) return "";
  const char* n = courseManager->getActiveCourseName();
  return n ? n : "";
}

uint32_t sim_lap_time_ms(int idx) {
  if (idx < 0 || idx >= lapHistoryCount) return 0;
  return (uint32_t)lapHistory[idx];
}

}  // extern "C"
