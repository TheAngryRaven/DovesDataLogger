cmake_minimum_required(VERSION 3.16)
project(birdseye_sim LANGUAGES CXX)

# ---------------------------------------------------------------------------
# BirdsEye WASM/native simulator — Phase 1: native host build.
#
# Builds the REAL firmware sources (one concatenated TU, sim_main.cpp) on a
# desktop toolchain, against:
#   - arduino_shim/  : Arduino core + nRF52 register surface (virtual clock)
#   - sdfat_shim/    : in-memory VFS standing in for SdFat
#   - stubs/         : excluded modules (BLE/camera/USB/OTA) + GNSS driver
#   - REAL libraries : DovesLapTimer/CourseManager (the point of the sim),
#                      SparkFun u-blox header (types only), ArduinoJson.
#
# The WASM (emcmake) target is Phase 4; this file deliberately builds native
# first so the whole firmware TU is exercised in CI from day one.
# ---------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SIM_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(BIRDSEYE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..)

option(SIM_DEBUG_OUTPUT "Enable the firmware's HAS_DEBUG serial prints" ON)

# ---------------------------------------------------------------- libraries
include(FetchContent)

# Lap timing — the REAL library; its math being real firmware code is the
# sim's second-highest fidelity priority. Ref policy matches CI: BETA-channel
# builds track the library's BETA branch (master/release builds pin a tag).
FetchContent_Declare(doveslaptimer
  GIT_REPOSITORY https://github.com/TheAngryRaven/DovesLapTimer.git
  GIT_TAG BETA
  GIT_SHALLOW TRUE
)

# u-blox GNSS — header-only use (UBX struct types + the class the sketch
# instantiates); the driver .cpp is NOT compiled (stubs/gnss_stubs.cpp).
FetchContent_Declare(sparkfun_gnss
  GIT_REPOSITORY https://github.com/sparkfun/SparkFun_u-blox_GNSS_v3.git
  GIT_TAG v3.1.9
  GIT_SHALLOW TRUE
)

FetchContent_Declare(arduinojson
  GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git
  GIT_TAG v6.21.5
  GIT_SHALLOW TRUE
)

# DovesLapTimer.h includes ArxTypeTraits.h
FetchContent_Declare(arxtypetraits
  GIT_REPOSITORY https://github.com/hideakitai/ArxTypeTraits.git
  GIT_TAG v0.3.2
  GIT_SHALLOW TRUE
)

# REAL display stack (pixel-perfect is fidelity priority #1). The class
# chain Adafruit_SH1106G : Adafruit_SH110X : Adafruit_GrayOLED :
# Adafruit_GFX is portable C++; the hardware boundary is the two BusIO
# device pointers, shimmed in busio_shim/. Versions are recorded below
# for the sim's getVersion() surface.
set(SIM_GFX_VERSION "1.12.6")
set(SIM_SH110X_VERSION "2.1.14")
FetchContent_Declare(adafruit_gfx
  GIT_REPOSITORY https://github.com/adafruit/Adafruit-GFX-Library.git
  GIT_TAG ${SIM_GFX_VERSION}
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR cmake-subdir-does-not-exist  # repo root CMakeLists is ESP-IDF-only
)
FetchContent_Declare(adafruit_sh110x
  GIT_REPOSITORY https://github.com/adafruit/Adafruit_SH110x.git
  GIT_TAG ${SIM_SH110X_VERSION}
  GIT_SHALLOW TRUE
  SOURCE_SUBDIR cmake-subdir-does-not-exist  # repo root CMakeLists is ESP-IDF-only
)

FetchContent_MakeAvailable(doveslaptimer sparkfun_gnss arduinojson
                           arxtypetraits adafruit_gfx adafruit_sh110x)

# ------------------------------------------------------------ embed assets
set(SIM_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
file(MAKE_DIRECTORY ${SIM_GEN_DIR})

function(sim_embed_asset input output symbol)
  add_custom_command(
    OUTPUT ${output}
    COMMAND ${CMAKE_COMMAND}
            -DINPUT=${input} -DOUTPUT=${output} -DSYMBOL=${symbol}
            -P ${SIM_DIR}/cmake/bin2h.cmake
    DEPENDS ${input} ${SIM_DIR}/cmake/bin2h.cmake
    COMMENT "Embedding ${input} as ${symbol}"
  )
endfunction()

sim_embed_asset(${SIM_DIR}/assets/SETTINGS.json
                ${SIM_GEN_DIR}/sim_asset_settings.h kAssetSettingsJson)
sim_embed_asset(${SIM_DIR}/assets/TRACKS/OKC.json
                ${SIM_GEN_DIR}/sim_asset_track_okc.h kAssetTrackOkcJson)

add_custom_target(sim_assets DEPENDS
  ${SIM_GEN_DIR}/sim_asset_settings.h
  ${SIM_GEN_DIR}/sim_asset_track_okc.h
)

# ------------------------------------------------------------------- sources
set(SIM_CORE_SOURCES
  ${SIM_DIR}/sim_main.cpp
  ${SIM_DIR}/virtual_clock.cpp
  ${SIM_DIR}/arduino_shim/arduino_shim.cpp
  ${SIM_DIR}/arduino_shim/shim_objects.cpp
  ${SIM_DIR}/sdfat_shim/sim_vfs.cpp
  ${SIM_DIR}/stubs/module_stubs.cpp
  ${SIM_DIR}/stubs/gnss_stubs.cpp
  # Real SparkFun bus classes (SfeSerial is a member of the class the
  # sketch instantiates) — small, portable, compiles against the shim.
  ${sparkfun_gnss_SOURCE_DIR}/src/sfe_bus.cpp
  # Pure-logic units the compiled firmware modules call (same files the
  # host test harness in tests/ builds).
  ${BIRDSEYE_DIR}/camera_fsm.cpp
  ${BIRDSEYE_DIR}/dovex_header.cpp
  ${BIRDSEYE_DIR}/gps_status_page.cpp
  ${BIRDSEYE_DIR}/gps_time.cpp
  ${BIRDSEYE_DIR}/gps_validation.cpp
  ${BIRDSEYE_DIR}/haversine.cpp
  ${BIRDSEYE_DIR}/lap_format.cpp
  ${BIRDSEYE_DIR}/sat_bars.cpp
  ${BIRDSEYE_DIR}/sd_access_policy.cpp
  ${BIRDSEYE_DIR}/sd_format_page.cpp
  ${BIRDSEYE_DIR}/sensoregg_protocol.cpp
  ${BIRDSEYE_DIR}/tach_filter.cpp
  ${BIRDSEYE_DIR}/wake_cause.cpp
  # Real DovesLapTimer sources — the lap/sector timing IS the demo.
  ${doveslaptimer_SOURCE_DIR}/src/DovesLapTimer.cpp
  ${doveslaptimer_SOURCE_DIR}/src/WaypointLapTimer.cpp
  ${doveslaptimer_SOURCE_DIR}/src/CourseDetector.cpp
  ${doveslaptimer_SOURCE_DIR}/src/CourseManager.cpp
  # Real display stack — every pixel comes from the real drawPixel().
  ${adafruit_gfx_SOURCE_DIR}/Adafruit_GFX.cpp
  ${adafruit_gfx_SOURCE_DIR}/Adafruit_GrayOLED.cpp
  ${adafruit_sh110x_SOURCE_DIR}/Adafruit_SH110X.cpp
  ${adafruit_sh110x_SOURCE_DIR}/Adafruit_SH1106G.cpp
  # Display output plumbing (frame hash + PNG dumps).
  ${SIM_DIR}/frame_hash.cpp
  ${SIM_DIR}/png_dump.cpp
  ${BIRDSEYE_DIR}/crc32.cpp
)

# Everything except the driver main()s — shared by the soak binary and
# the golden-fixture binary.
add_library(birdseye_sim_core STATIC ${SIM_CORE_SOURCES})
add_dependencies(birdseye_sim_core sim_assets)

target_include_directories(birdseye_sim_core PUBLIC
  ${SIM_DIR}
  ${SIM_DIR}/arduino_shim
  ${SIM_DIR}/busio_shim
  ${SIM_DIR}/sdfat_shim
  ${SIM_GEN_DIR}
  ${BIRDSEYE_DIR}
  ${doveslaptimer_SOURCE_DIR}/src
  ${sparkfun_gnss_SOURCE_DIR}/src
  ${arduinojson_SOURCE_DIR}/src
  ${arxtypetraits_SOURCE_DIR}
  ${adafruit_gfx_SOURCE_DIR}
  ${adafruit_sh110x_SOURCE_DIR}
)

target_compile_definitions(birdseye_sim_core PUBLIC
  $<$<BOOL:${SIM_DEBUG_OUTPUT}>:HAS_DEBUG>
  SIM_GFX_VERSION="${SIM_GFX_VERSION}"
  SIM_SH110X_VERSION="${SIM_SH110X_VERSION}"
  # Adafruit_GFX keys its Print-override signature (size_t write(uint8_t))
  # off the Arduino version macro; pretend to be a modern core.
  ARDUINO=10819
  # ...but keep ArduinoJson exactly as it behaved before ARDUINO was
  # defined (std mode): the firmware only ever hands it char buffers.
  ARDUINOJSON_ENABLE_ARDUINO_STRING=0
  ARDUINOJSON_ENABLE_ARDUINO_STREAM=0
  ARDUINOJSON_ENABLE_ARDUINO_PRINT=0
  ARDUINOJSON_ENABLE_PROGMEM=0
)

# The firmware TU is Arduino-style code compiled far from its home
# toolchain — keep warnings on but non-fatal for it; the sim's own shims
# should stay clean.
target_compile_options(birdseye_sim_core PUBLIC
  -Wall -Wextra -Wno-unused-parameter
  # The Arduino build force-includes Arduino.h into every source; the
  # DovesLapTimer and Adafruit sources rely on that. Replicate.
  -include ${SIM_DIR}/arduino_shim/Arduino.h
)

# Provenance for getVersion()/version.json: the firmware repo sha and the
# resolved DovesLapTimer sha (its pin is the moving BETA branch).
execute_process(COMMAND git rev-parse --short HEAD
                WORKING_DIRECTORY ${BIRDSEYE_DIR}
                OUTPUT_VARIABLE FIRMWARE_SHA OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET)
execute_process(COMMAND git rev-parse --short HEAD
                WORKING_DIRECTORY ${doveslaptimer_SOURCE_DIR}
                OUTPUT_VARIABLE SIM_LAPTIMER_SHA OUTPUT_STRIP_TRAILING_WHITESPACE
                ERROR_QUIET)
if(NOT FIRMWARE_SHA)
  set(FIRMWARE_SHA "unknown")
endif()
if(NOT SIM_LAPTIMER_SHA)
  set(SIM_LAPTIMER_SHA "unknown")
endif()

if(EMSCRIPTEN)
  # ---- Phase 4: the browser artifact -------------------------------------
  # Emits dist/birdseye-sim-core.mjs + .wasm; the hand-written public
  # wrapper (wasm/birdseye-sim.mjs) and a generated version.json are
  # copied/written next to them. Those four files are what gets vendored
  # into DovesDataViewer/public/sim/.

  # The firmware TU signals NVIC_SystemReset with a C++ throw that
  # sim_step_millis catches — Emscripten disables exception CATCHING by
  # default, so force it on for every TU in the core too.
  target_compile_options(birdseye_sim_core PUBLIC -fexceptions)

  add_executable(birdseye-sim-core
    ${SIM_DIR}/wasm/bindings.cpp
  )
  target_link_libraries(birdseye-sim-core PRIVATE birdseye_sim_core)
  target_compile_definitions(birdseye-sim-core PRIVATE
    FIRMWARE_SHA="${FIRMWARE_SHA}"
    SIM_LAPTIMER_SHA="${SIM_LAPTIMER_SHA}"
  )
  # The firmware TU signals NVIC_SystemReset with a C++ throw — the wasm
  # build must keep exceptions on (core lib compiles with the same flag
  # via the PUBLIC options below).
  target_compile_options(birdseye-sim-core PRIVATE -fexceptions)
  set_target_properties(birdseye-sim-core PROPERTIES
    OUTPUT_NAME "birdseye-sim-core"
    SUFFIX ".mjs"
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/dist
  )
  target_link_options(birdseye-sim-core PRIVATE
    -fexceptions
    -sMODULARIZE=1
    -sEXPORT_ES6=1
    -sEXPORT_NAME=createBirdsEyeSim
    -sALLOW_MEMORY_GROWTH=1
    -sENVIRONMENT=web,worker,node
    -sEXPORTED_RUNTIME_METHODS=cwrap,getValue,HEAPU8
    -sEXPORTED_FUNCTIONS=_malloc,_free
  )

  # version.json — session-capture provenance (see the handoff spec).
  string(TIMESTAMP SIM_BUILD_DATE "%Y-%m-%d" UTC)
  file(WRITE ${CMAKE_BINARY_DIR}/dist/version.json
"{
  \"firmwareSha\": \"${FIRMWARE_SHA}\",
  \"buildDate\": \"${SIM_BUILD_DATE}\",
  \"simApiVersion\": 1,
  \"dovesLapTimerSha\": \"${SIM_LAPTIMER_SHA}\",
  \"adafruitGfx\": \"${SIM_GFX_VERSION}\",
  \"adafruitSh110x\": \"${SIM_SH110X_VERSION}\"
}
")
  add_custom_command(TARGET birdseye-sim-core POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${SIM_DIR}/wasm/birdseye-sim.mjs
            ${SIM_DIR}/wasm/test.html
            ${SIM_DIR}/wasm/smoke.mjs
            ${CMAKE_BINARY_DIR}/dist/
    COMMENT "Staging wrapper + harness into dist/"
  )
  return()  # native executables/tests below are host-only
endif()

add_executable(birdseye_sim_native ${SIM_DIR}/native_main.cpp)
target_link_libraries(birdseye_sim_native PRIVATE birdseye_sim_core)

add_executable(birdseye_sim_golden ${SIM_DIR}/golden_main.cpp)
target_link_libraries(birdseye_sim_golden PRIVATE birdseye_sim_core)

add_executable(birdseye_sim_oracle ${SIM_DIR}/oracle_main.cpp)
target_link_libraries(birdseye_sim_oracle PRIVATE birdseye_sim_core)

enable_testing()

# Phase-1 done-criteria: boots to menu + survives 60 s of virtual time,
# and two runs produce byte-identical output (determinism).
add_test(NAME sim_boot_soak COMMAND birdseye_sim_native)
add_test(NAME sim_determinism
  COMMAND ${CMAKE_COMMAND}
          -DSIM_BIN=$<TARGET_FILE:birdseye_sim_native>
          -P ${SIM_DIR}/cmake/determinism_test.cmake
)

# Phase-2 done-criteria: the real display stack renders the committed
# golden page hashes (pixels AND the navigation that produced them).
add_test(NAME sim_golden_fixtures
  COMMAND birdseye_sim_golden --golden ${SIM_DIR}/golden/golden_hashes.txt
)

# Phase-3 done-criteria: injected GPS/tach drive the REAL pipeline end to
# end — boot page exit, OKC proximity detection, CourseDetector picking
# "Normal", and lap times matching the constructed ground truth within
# one GPS frame (40 ms). (--dovex mode replays a hardware log against its
# own header once a fixture is committed under sim/fixtures/.)
add_test(NAME sim_lap_oracle COMMAND birdseye_sim_oracle)

# The definitive same-code-same-answer check: replay a hardware-recorded
# OKC session (13 laps, real ignition/GPS noise and all) and require the
# sim's DovesLapTimer to reproduce the lap list from the file's own
# header. Currently reproduces it to the exact millisecond.
add_test(NAME sim_lap_oracle_hardware
  COMMAND birdseye_sim_oracle --dovex ${SIM_DIR}/fixtures/okc_tillotson_1.dovex
)

# Track-day state carryover: synthetic session 1 → auto-idle session end →
# a 6-minute parked break in the menu (GPS drift, periodic button presses)
# → the hardware fixture replayed as session 2. The carried-over
# CourseManager must detect the same course and reproduce the same lap
# list as a fresh boot. Also a regression test for the menu idle timer:
# before the lastPressed-anchored reset, the 6-minute break shut the
# device down mid-break despite the button presses (2026-07-19 field
# incident) and session 2 never entered race mode.
add_test(NAME sim_two_session_carryover
  COMMAND birdseye_sim_oracle --two-session
          ${SIM_DIR}/fixtures/okc_tillotson_1.dovex 6
)
