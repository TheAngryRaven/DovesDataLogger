///////////////////////////////////////////
// bindings.cpp — Emscripten exports (sim Phase 4, WASM build only).
//
// Thin EMSCRIPTEN_KEEPALIVE wrappers over the extern-C host surface in
// sim_host.h, plus the three contract pieces that don't exist natively:
// getStateJson, getVersion, readFile. The JS-facing object API lives in
// birdseye-sim.mjs (the hand-written ESM wrapper vendored next to the
// emitted core module) — see API.md for the full contract.
///////////////////////////////////////////

#include <emscripten/emscripten.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

#include "../sdfat_shim/SdFat.h"
#include "../sim_host.h"

#ifndef FIRMWARE_SHA
#define FIRMWARE_SHA "unknown"
#endif
#ifndef SIM_LAPTIMER_SHA
#define SIM_LAPTIMER_SHA "unknown"
#endif
#ifndef SIM_GFX_VERSION
#define SIM_GFX_VERSION "unknown"
#endif
#ifndef SIM_SH110X_VERSION
#define SIM_SH110X_VERSION "unknown"
#endif

#define SIM_API_VERSION 1

extern "C" {

EMSCRIPTEN_KEEPALIVE void simw_init(void) { sim_init(); }

EMSCRIPTEN_KEEPALIVE int simw_step_millis(double ms) {
  if (ms <= 0) return 0;
  return sim_step_millis((uint32_t)ms);
}

EMSCRIPTEN_KEEPALIVE void simw_button_down(int idx) { sim_button_down(idx); }
EMSCRIPTEN_KEEPALIVE void simw_button_up(int idx) { sim_button_up(idx); }

EMSCRIPTEN_KEEPALIVE int simw_inject_pvt(const char* json) {
  return sim_inject_pvt_json(json);
}

EMSCRIPTEN_KEEPALIVE void simw_set_rpm(int rpm) { sim_set_rpm(rpm); }

EMSCRIPTEN_KEEPALIVE const uint8_t* simw_framebuffer(void) {
  return sim_framebuffer();
}

EMSCRIPTEN_KEEPALIVE int simw_boot_frame_count(void) {
  return sim_boot_frame_count();
}

EMSCRIPTEN_KEEPALIVE uint32_t simw_boot_frame_time_ms(int i) {
  return sim_boot_frame_time_ms(i);
}

EMSCRIPTEN_KEEPALIVE const uint8_t* simw_boot_frame_pixels(int i) {
  return sim_boot_frame_pixels(i);
}

EMSCRIPTEN_KEEPALIVE uint32_t simw_frame_hash(void) {
  return sim_frame_hash();
}

EMSCRIPTEN_KEEPALIVE int simw_reset_requested(void) {
  return sim_reset_requested();
}

// State snapshot per the API contract. Returned pointer is valid until
// the next call (static buffer).
EMSCRIPTEN_KEEPALIVE const char* simw_state_json(void) {
  static char buf[512];
  snprintf(buf, sizeof(buf),
           "{\"page\":%d,\"raceActive\":%s,\"lapCount\":%d,"
           "\"bestLapMs\":%u,\"lastLapMs\":%u,\"currentLapMs\":%u,"
           "\"gpsFix\":%s,\"sats\":%d,\"rpm\":%d,\"loggingActive\":%s,"
           "\"trackDetected\":%s,\"courseName\":\"%s\",\"millis\":%u}",
           sim_current_page(), sim_race_active() ? "true" : "false",
           sim_lap_count(), sim_best_lap_ms(), sim_last_lap_ms(),
           sim_current_lap_ms(), sim_gps_fix() ? "true" : "false",
           sim_sats(), sim_rpm(), sim_logging_active() ? "true" : "false",
           sim_track_detected() ? "true" : "false", sim_course_name(),
           sim_millis());
  return buf;
}

EMSCRIPTEN_KEEPALIVE const char* simw_version_json(void) {
  static char buf[384];
  snprintf(buf, sizeof(buf),
           "{\"firmwareSha\":\"%s\",\"buildDate\":\"%s\","
           "\"simApiVersion\":%d,\"dovesLapTimerSha\":\"%s\","
           "\"adafruitGfx\":\"%s\",\"adafruitSh110x\":\"%s\"}",
           FIRMWARE_SHA, __DATE__, SIM_API_VERSION, SIM_LAPTIMER_SHA,
           SIM_GFX_VERSION, SIM_SH110X_VERSION);
  return buf;
}

// readFile: copy a VFS file into a malloc'd buffer the JS side reads
// out of the heap and then frees via simw_free. Returns null when the
// path doesn't exist; length lands in *outLen.
EMSCRIPTEN_KEEPALIVE uint8_t* simw_read_file(const char* path,
                                             int* outLen) {
  if (outLen) *outLen = 0;
  std::vector<uint8_t> bytes;
  if (!sim_vfs::readFileBytes(path ? path : "", bytes)) return nullptr;
  uint8_t* out = (uint8_t*)malloc(bytes.size() ? bytes.size() : 1);
  if (!out) return nullptr;
  memcpy(out, bytes.data(), bytes.size());
  if (outLen) *outLen = (int)bytes.size();
  return out;
}

// List the VFS as a JSON array of paths (companion to readFile — lets
// the viewer offer "download this run's log" without guessing names).
EMSCRIPTEN_KEEPALIVE const char* simw_list_files(void) {
  static std::string out;
  out = "[";
  bool first = true;
  for (const std::string& p : sim_vfs::listFiles()) {
    if (!first) out += ",";
    out += "\"" + p + "\"";
    first = false;
  }
  out += "]";
  return out.c_str();
}

EMSCRIPTEN_KEEPALIVE void simw_free(void* p) { free(p); }

}  // extern "C"
