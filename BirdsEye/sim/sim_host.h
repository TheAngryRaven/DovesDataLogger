#pragma once

#include <stdint.h>

///////////////////////////////////////////
// Host-facing sim interface (Phase 1 subset of the API contract).
//
// The full WASM binding surface (framebuffer, frame hash, PVT injection,
// RPM synthesis, state JSON) lands in later phases; this is what the
// native driver and tests need to boot the firmware and drive it.
///////////////////////////////////////////

extern "C" {

// Fresh boot: reset virtual clock + VFS, run the sketch's setup().
void sim_init(void);

// Advance virtual time by ~deltaMs, running loop() in main-loop-rate
// quanta. Returns the number of loop() iterations executed, or -1 if the
// firmware requested a reset (NVIC_SystemReset) during the step.
int sim_step_millis(uint32_t deltaMs);

// Buttons: idx 0=Left, 1=Select, 2=Right (active-LOW pins in the map).
void sim_button_down(int idx);
void sim_button_up(int idx);

// ---- GPS injection (Phase 3) ----
// Direct callback injection: builds a real UBX_NAV_PVT_data_t from these
// fields (exact map in the handoff spec) and calls the firmware's
// onPVTReceived(). Everything downstream — validation, DovesLapTimer,
// logging, display — is real firmware. The SparkFun parser is never
// involved. Inject at most once per sim_step_millis batch.
typedef struct SimPvt {
  unsigned long long timestamp_ms;  // Unix ms, UTC (dovex `timestamp`)
  double lat;                       // degrees (dovex `lat`)
  double lng;                       // degrees (dovex `lng`)
  double altitude_m;                // dovex `altitude_m`
  double speed_mph;                 // dovex `speed_mph`
  double heading_deg;               // dovex `heading_deg`
  double h_acc_m;                   // dovex `h_acc_m`
  double hdop;                      // dovex `hdop` (fw treats pDOP~HDOP)
  int sats;                         // dovex `sats`
  int fix;                          // 1: fixType=3/gnssFixOK/valid bits set
                                    // 0: no-fix frame (boot pre-roll UX)
  float accel_x, accel_y, accel_z;  // dovex accel columns -> IMU shim
} SimPvt;

void sim_inject_pvt(const SimPvt* p);

// JSON wrapper over sim_inject_pvt (the wasm-facing spelling). Keys:
//   timestamp, lat, lng, sats, hdop, speed_mph, altitude_m, heading_deg,
//   h_acc_m, fix (bool), accelX, accelY, accelZ
// Missing numeric keys default to 0. Returns 1 on success, 0 on a parse
// error (frame dropped).
int sim_inject_pvt_json(const char* json);

// ---- Tachometer (Phase 3) ----
// Synthesize tach pulses through the REAL falling-edge ISR: period is
// 60e6/rpm µs (1 pulse/rev, matching tachRevsPerPulse), fired at exact
// virtual-µs boundaries during sim_step_millis so micros() inside the
// ISR is correct. The debounce / ring buffer / Kalman path is all real
// firmware. rpm <= 0 stops pulses (the firmware's 500 ms timeout then
// reports 0).
void sim_set_rpm(int rpm);

// ---- Accelerometer (Phase 3) ----
// Plumb g-force values into the LSM6DS3 shim (also set by every
// sim_inject_pvt call — this is for driving accel independently).
void sim_set_accel(float x, float y, float z);

// True once the firmware has requested a reset (sticky until sim_init).
int sim_reset_requested(void);

// ---- display output (Phase 2) ----
// Zero-copy view of the REAL Adafruit_GrayOLED framebuffer: 1024 bytes,
// page layout — bit = buf[x + (y>>3)*128] >> (y&7) & 1.
const uint8_t* sim_framebuffer(void);
// FNV-1a 32-bit hash of those 1024 bytes (golden fixtures + dirty-check).
uint32_t sim_frame_hash(void);

// ---- state peeks (Phase 4 wraps these into getStateJson) ----
uint32_t sim_millis(void);
int sim_current_page(void);
int sim_race_active(void);
int sim_lap_count(void);
int sim_rpm(void);
int sim_gps_fix(void);
int sim_logging_active(void);
int sim_track_detected(void);
int sim_lap_anything_active(void);
// Active course name ("" when no CourseManager / no course yet).
const char* sim_course_name(void);
// lapHistory[idx] in ms (0 when out of range).
uint32_t sim_lap_time_ms(int idx);

}  // extern "C"
