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

}  // extern "C"
