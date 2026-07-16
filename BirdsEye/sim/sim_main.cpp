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
      sim_clock::advanceUs(kLoopQuantumUs);
      loop();
      iterations++;
    }
  } catch (const SimResetRequest&) {
    g_resetRequested = true;
    return -1;
  }
  return iterations;
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

uint32_t sim_millis(void) { return (uint32_t)millis(); }

int sim_current_page(void) { return currentPage; }

int sim_race_active(void) { return raceActive ? 1 : 0; }

int sim_lap_count(void) { return lapHistoryCount; }

int sim_rpm(void) { return tachLastReported; }

int sim_gps_fix(void) { return gpsData.fix ? 1 : 0; }

int sim_logging_active(void) { return sdDataLogInitComplete ? 1 : 0; }

}  // extern "C"
