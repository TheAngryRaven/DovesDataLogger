# BirdsEye - Project Guide

> **MAINTAINERS: Keep this file updated when adding/removing files, changing pin
> assignments, modifying subsystem interfaces, or altering the build configuration.
> This file is loaded into Claude's context window on every session and must
> accurately reflect the current state of the project.**

## Maintaining the Quality Bar

This project went through a deliberate hardening pass (tests, CI, static
analysis, security, release pipeline, docs). **Keep it there.** When making
any change — whether you're Claude or a human contributor — hold the line:

- **Add tests when possible.** New pure logic (math, parsing, validation,
  formatting, anything Arduino-free) belongs in a `BirdsEye/*.{h,cpp}` unit
  with a matching `tests/<unit>_test.cpp`. If you're touching existing logic
  that *could* be a pure unit but isn't yet, prefer extracting it so it can
  be tested rather than leaving it tangled in an `.ino`. Don't add untested
  pure logic when a test is feasible.
- **Keep the CHANGELOG updated.** Any user-visible change gets an entry under
  `[Unreleased]` in `CHANGELOG.md` (Added / Changed / Removed / Fixed /
  Security). Flag breaking changes explicitly — they drive the next version
  number per the semver policy in that file.
- **Keep CI green and meaningful.** The checks (compile-sketch + flash-size
  gate, arduino-lint, unit-tests, clang-tidy, coverage) must pass. Fix the
  root cause rather than loosening a check; if a clang-tidy finding is a
  genuine false positive, suppress that one line with `// NOLINT(check)` and
  a reason, never by disabling the check globally. The coverage floor
  (`COVERAGE_MIN` in `coverage.yml`) is intentionally low — raise it as
  coverage grows; don't lower it to pass.
- **Keep the docs in sync.** Update this file's File Map and the relevant
  subsystem section, plus `ARCHITECTURE.md`, when you add/remove a module or
  change a subsystem interface. Stale docs are worse than none.
- **Hold the conventions.** No Arduino `String` in hot paths, all SD access
  through the mutex, never `analogRead()`, `TIMER3` reserved, ISRs trivially
  short. See *Development Conventions* at the bottom for the full list.
- **One concern per PR.** Keep refactors, behavior changes, and new tests in
  separate PRs so each is reviewable and revertable on its own.
- **Once CI is green on a PR, STOP.** Report the green status once and end.
  Do NOT schedule recurring re-checks, polling wake-ups, or "babysit"
  timers on a passing PR — they burn the owner's session usage confirming
  nothing changed. Watch a PR only when explicitly asked, and even then a
  green CI run ends the loop.

The goal: every change should leave the codebase at least as professional as
it found it. If a shortcut would lower the bar, flag it instead of taking it.

## What Is BirdsEye?

A high-precision GPS lap timer and data logger for motorsports / track days.
Built on the **Seeed XIAO nRF52840 Sense** (ARM Cortex-M4, 256 KB RAM, BLE 5.0, onboard LSM6DS3 IMU).

Core capabilities:
- 25 Hz GPS lap timing with sector support (DovesLapTimer library)
- **"Just Drive" auto-detection** via CourseManager: automatic track
  proximity matching, course detection, and Lap Anything fallback
- RPM monitoring via inductive tachometer pickup
- Accelerometer logging (g-force X/Y/Z) via onboard LSM6DS3 IMU
- DOVEX data logging with reserved 1 KB header (crash-safe GPS data)
- 8+ display pages on a 128x64 OLED (3 Hz refresh)
- Bluetooth LE file download to companion apps / LapWingData.com
- On-device session replay: instant DOVEX header replay
- **Insta360 X4 camera auto-record**: emulates the Insta360 GPS Remote as a
  pure BLE peripheral — wakes the camera on engine start, records via a ce82
  shutter toggle, stops and powers off automatically (see subsystem 13)
- **SensorEgg wireless EGT (POC)**: passive BLE observer receives the
  DovesSensorEgg thermocouple pod's advertising broadcasts (`PW-ADV` v1
  and v2), logs `Temp1`/`Junction1`/`Temp2` DOVEX columns + Temp1/Temp2
  race pages (subsystem 14)
- **Loop CPU profiling (beta only)**: `BIRDSEYE_ENABLE_PROFILING` times
  every subsystem call in `loop()`, shows the breakdown on a LOOP PROFILE
  race page, and drives pin 30 as a scope output — the measurement behind
  the nRF52840-vs-nRF5340 board decision. **Takes pin 30 from the boost
  EN line, so a beta image can never switch the 5 V rail** (subsystem 18)
- **NeoPixel strip**: 11 WS2812 pixels on the NFC pads converted
  to GPIO — 2 **user-assignable** status LEDs (eight modes: target RPM,
  target speed, GPS lock, camera sync, last lap, last sector, EGT, off)
  flanking a 9-px pace-pip / RPM- or speed-scale strip, with a global
  brightness cap, boot animation, and a two-stage purple celebration for
  a session-best sector or lap (subsystem 16)

---

## File Map

All sketch sources live in `BirdsEye/` so the folder name matches the
`.ino` filename Arduino IDE expects. Each module has both a `.ino`
(implementation) and a `.h` (public interface, documentation).

### Sketch Sources (`BirdsEye/`)

| File | Purpose |
|---|---|
| `BirdsEye.ino` | Entry point: globals, `setup()`, `loop()`, state machine, course/timer helpers |
| `project.h` | Shared types (`ButtonState`, `TrackLayout`, `TrackManifestEntry`, `TrackMetadata`), debug macros, `MAX_*` constants |
| `display_config.h` | Display driver abstraction (SH110X vs SSD1306 toggle) |
| `gps_config.h` | GPS configuration constants (baud rate, nav rate, serial port) |
| `images.h` | PROGMEM bitmap data — the bird splash only; the crossing animation is generated (`crossing_pattern`) |
| `accelerometer.{h,ino}` | LSM6DS3 IMU init and g-force reads (onboard XIAO Sense) |
| `bluetooth.{h,ino}` | BLE service (file listing, transfer, settings, track sync), reboot on leaving transfer mode (peer disconnect or manual Exit); shared peripheral BLE core init (+ Just-Works bonding) + `bleOwner` radio-ownership routing |
| `camera_ble.{h,ino}` | Insta360 X4 auto-record BLE glue: peripheral remote GATT (0xCE80), all control via ce82 button notifies, executes `camera_fsm` actions, deferred callback→loop pattern (see subsystem 13) |
| `firmware_ota.{h,ino}` | SD-staged firmware OTA: `FW*` BLE protocol, SD staging, CRC verify, self-flash apply (see subsystem 11) |
| `display_pages.{h,ino}` | All page rendering functions (`displayPage_*()`) |
| `display_ui.{h,ino}` | Display init, button reading (multi-sample debounce), menu navigation, I2C bus recovery |
| `gps_functions.{h,ino}` | GPS init (SparkFun UBX PVT), time conversion, DOVEX logging pipeline, TIMER3 serial buffer ISR, V_BCKP recovery |
| `neopixel.{h,ino}` | NeoPixel strip glue (subsystem 16): one-time UICR NFC→GPIO ensure, boost-EN power control, 30 Hz compose→cap→show frame loop, sleep/wake hooks; all decision math in the led_* / sector_purple pure units |
| `profiling.{h,ino}` | Main-loop CPU profiling glue (subsystem 18, beta only): DWT/micros timebase probe, section brackets, the pin-30 scope output, once-a-second rollup. Also the reason a beta build never drives the 5 V boost EN |
| `replay.{h,ino}` | Instant DOVEX header replay |
| `sd_functions.{h,ino}` | SD init, track list/JSON parsing (dual format), track manifest, SD access arbitration |
| `sensoregg.{h,ino}` | SensorEgg wireless EGT: passive BLE scan (observer), scan-callback→loop double buffer, `SENSOREGG_MAC` pairing, Temp1/Junction1 data surface (see subsystem 14) |
| `settings.{h,ino}` | Persistent JSON settings on SD (`/SETTINGS.json`), `getSetting()`/`setSetting()` |
| `tachometer.{h,ino}` | Falling-edge ISR on D0, Kalman-filtered RPM calculation |
| `usb_msc.{h,ino}` | USB Mass Storage (TinyUSB MSC): SD card as a drag-and-drop drive (see subsystem 12) |

### Pure-Logic Units (`BirdsEye/*.{h,cpp}`)

Arduino-free `.cpp` files — compiled into the firmware AND into the
host test harness (`tests/`). No Arduino headers, so they build on a
desktop toolchain. This is where logic worth unit-testing lives.

| File | Purpose |
|---|---|
| `haversine.{h,cpp}` | Great-circle distance in miles (track proximity) |
| `idle_policy.{h,cpp}` | Auto-idle session-end decision table (tach 60 s/2 mph vs manual/speed 5 min/5 mph, camera-yield + GPS-lock-hold exception, sprint engine-aware reset) + the promotion of SPEED/MANUAL sessions to TACH rules once the engine fires |
| `gps_stats.{h,cpp}` | GPS pipeline drop accounting: expected-vs-received PVT window math (exact fractional carry, 1-frame jitter slack, capped credit, rate-switch suppression) feeding the debug-page `Drops` counter |
| `gps_time.{h,cpp}` | Leap-year/Unix-epoch math, `u64ToDecimalString` |
| `gps_validation.{h,cpp}` | PVT sample sanity gate + dtostrf-output check |
| `dovex_header.{h,cpp}` | DOVEX 1 KB header `format()` / `parse()` |
| `filename_validator.{h,cpp}` | FAT-safe / traversal-proof check for BLE filenames |
| `crc32.{h,cpp}` | CRC-32/IEEE-802.3 (zlib) incremental + hex; pins firmware-OTA CRC to the web client |
| `ble_stream.{h,cpp}` | BLE file-transfer read-ahead bookkeeping: chunk size from the negotiated MTU, the compacting refill (`Move`), slice/consume, transfer rate. An off-by-one here corrupts a downloaded session, so the index math is host-tested away from the radio |
| `sd_access_policy.{h,cpp}` | SD access arbitration decision table (mode values + grant/deny rules) |
| `lap_format.{h,cpp}` | ms → `M:SS.mmm` lap-time rendering (three zero-minutes styles), used by all display pages |
| `setting_parse.{h,cpp}` | Strict integer parsing for `/SETTINGS.json` values. Exists because `atoi()` answers 0 for `""` and `"garbage"`, and 0 is an in-range, **destructive** value for the LED keys (`led_brightness` 0 = strip off + boost rail never raised) — so a blank setting read as a deliberate "off". Rejects anything that is not a complete integer, so the caller's range check keeps the compiled-in default |
| `loop_profile.{h,cpp}` | Main-loop CPU accounting: per-section tick accumulation, saturating (never wrapping — a uint32 of DWT ticks is only ~67 s), and the once-a-second rollup into shares of wall time, loop rate, mean and worst iteration, plus measured idle (`SLP`). **Two clocks on purpose**: durations in TICKS with `ticksPerUs` supplied at rollup (so a sub-microsecond section is not quantised to zero), but the WINDOW closed on `millis()` — DWT counts cycles and stops when the core halts, and using it as a wall clock inflated the very first hardware reading. Board-portable by construction — the nRF5340 comparison needs the same instrument |
| `local_time.{h,cpp}` | UTC + a fixed signed minute offset → local wall clock (4-digit year, correct month/year/leap rollover both ways) + the `isNight()` window test. **No DST, and NOTHING logged goes through it** — saved data stays UTC (subsystem 17) |
| `led_frame.{h,cpp}` | NeoPixel pixel layout (11 px: 2 status + 9-px strip), `Rgb`/`Frame` PODs, and **`applyCap()` — the single global-brightness choke point** (post-condition: no channel exceeds the cap) |
| `led_modes.{h,cpp}` | Strip modes + status actions: pace pip math (ms/m, slower = left/red), generic `ScaleSpec` left-fill (RPM red past halfway, speed with no red band at all), the `StatusAction` threshold/hysteresis/flash table, and `flashOn()` — the ONE definition of flash phase, shared with `led_status` |
| `led_status.{h,cpp}` | The eight assignable status-LED modes (subsystem 16): the mode enum + strict name parser that the `led_status_left`/`led_status_right` settings store, the GPS and camera readiness ladders, and `evalMode()` — which delegates every threshold mode to `led_modes::evalStatus` rather than re-implementing hysteresis. `Inputs.eggSupported` is why an `egt` LED is dark on a stock build instead of a permanent solid blue |
| `led_animations.{h,cpp}` | Boot + purple-sector animations as pure functions of `(tMs, seed)` — hash-based sparkles, no rand()/millis(), golden-testable |
| `sector_purple.{h,cpp}` | The lap/sector CLOSE-EDGE monitor (the name predates half its job): open-time best snapshots + a derived S3 defeat the library's lap-line `updateBestSectors()` race, and the same trick one level up defeats it for `getBestLapTime()`. Emits which sector or lap just closed, its verdict **against the last recorded one** (not the best — that only ever answers purple or red), and the two purple flags. No purple on lap 1 |
| `tach_filter.{h,cpp}` | Tachometer 1-D Kalman filter — predict/update math, the RPM-aware noise models, the **outlier gate** that keeps one bad ignition edge out of the trace (plan 0009), the `tach_filter` mode parser — **and the engine geometry**: `revsPerPulse` / `minPulseGapUs` from `spark_mode` + `cylinder_count` |
| `camera_fsm.{h,cpp}` | Insta360 auto-record lifecycle FSM (8 states, all debounce/retry/timeout timing + tunables); board-portable core shared with the nRF54 "Falcon" target |
| `insta360_protocol.{h,cpp}` | Insta360 X4 BLE frame builders/parsers (wake advert, remote scan response, ce82 buttons, ce82 GPS/RMC frame, ce81 serial parsing, ce81 `0x10` record-timer state parse) with golden-byte tests |
| `sensoregg_protocol.{h,cpp}` | SensorEgg `PW-ADV` v1+v2 advertising payload parser (magic filter, int16 deci-°C decode with `0x8000`→NaN sentinel, flags, sequence, v2 aux thermistor + battery) + wrap-safe 1 s staleness rule + passive-scan tuning constants |
| `crossing_pattern.{h,cpp}` | The two-frame crossing animation as geometry (eight 16x16 cells, odd row bands, alternating phase) instead of 2 KB of stored bitmap; golden-tested byte-identical to the images it replaced |
| `sprint_select.{h,cpp}` | Sprint mode selection: newest-course-by-`date_created` ordering (sortable ISO strings) + the circuit-vs-sprint tiebreak decision table (`race_mode` pref; circuit yields to a sprint course created today) |
| `course_prune.{h,cpp}` | Which sprint courses to drop when a track file is full (subsystem 15): `N{YYMMDD}_{HHMM}` matcher, and the drop order — **renamed-in-the-app before device-named** (a rename proves the app has a copy), then oldest by `date_created` |
| `course_creator.{h,cpp}` | On-device course creator model (subsystem 15): screen/row table, required-vs-optional lines, the two webapp-compat save rules, the point-averaging hold (3 s, ≥8 fixes, ≤10 m h_acc), and `N{YYMMDD}_{HHMM}` name generation |
| `track_json.{h,cpp}` | The firmware's only track-JSON **writer** — course/track object emitters + a fixed-point coordinate formatter (integer math: no working `%f` on this core, and `dtostrf` doesn't exist on the host) |
| `wake_cause.{h,cpp}` | Boot wake-cause decode: RESETREAS + GPIO LATCH register snapshots → tach / button / USB / watchdog / soft-reset / cold boot (System OFF shutdown, subsystem 10) |
| `gps_status_page.{h,cpp}` | GPS status boot page state machine: hold, 3 s auto-close after fix+timeValid, button skip, exit destination (menu vs race), idle → shutdown; `timeSyncState()` names which time milestone is outstanding (date/time vs the slow `fullyResolved`) |
| `sd_format_page.{h,cpp}` | SD format-confirm boot page state machine: Select held 3 s continuously → format (release restarts the full window; other buttons never confirm), 5 min idle → shutdown |
| `sat_bars.{h,cpp}` | Status-page satellite signal bars: NAV-SAT CNO selection (used-in-nav first, strongest first) + bar x/w/h layout math for the 128×~30 px bottom half |

### Simulator (`BirdsEye/sim/`)

Host build of the REAL firmware TU under the `SIM` flag (browser/WASM
target in a later phase; native + CI today). The `.ino` sources compile
unmodified — all sim behavior lives in `BirdsEye/sim/` or behind `SIM`.
See `ARCHITECTURE.md` → *Simulator* and the phased plan in the simulator
handoff spec.

| Path | Purpose |
|---|---|
| `sim_main.cpp` | Single TU replicating Arduino's .ino concatenation (bluetooth/camera_ble/usb_msc/firmware_ota deliberately absent) + host glue (`sim_init`/`sim_step_millis`/buttons/state peeks) |
| `sim_prototypes.h` | Hand-written stand-in for Arduino's auto-generated prototypes |
| `virtual_clock.{h,cpp}` | Host-advanced virtual time; `delay()` consumes it; no wall clock (determinism) |
| `arduino_shim/` | Arduino core + nRF52 registers/SoftDevice/FreeRTOS surface, Wire/SPI, LSM6DS3 (settable), Bluefruit types |
| `busio_shim/` | The display stack's entire hardware boundary: `Adafruit_I2CDevice` whose `begin()` is true and whose writes discard — everything above it (GFX/GrayOLED/SH110X) is the REAL pinned library, so the framebuffer is pixel-perfect |
| `sdfat_shim/` | In-memory VFS implementing the exact SdFat subset the firmware calls; preloads `assets/` (fixed SETTINGS.json + OKC track) via cmake-embedded byte arrays |
| `stubs/` | No-op surfaces of the excluded modules + SparkFun GNSS driver (real header, stubbed methods — PVT is injected directly into `onPVTReceived()`) |
| `frame_hash.{h,cpp}` | FNV-1a 32 over the 1024-byte framebuffer (golden fixtures, viewer dirty-check, future HIL tap) |
| `png_dump.{h,cpp}` | Dependency-free PNG writer (stored-deflate + repo crc32) for eyeballing frames |
| `native_main.cpp` | Phase-1 driver: boot → skip GPS status page → 60 s soak, state prints |
| `golden_main.cpp` | Phase-2 driver: scripted real-menu walk capturing 8 golden page hashes (`golden/golden_hashes.txt`; regenerate with `--print`, eyeball with `--dump`) |
| `oracle_main.cpp` | Phase-3 driver: lap-timing oracle. Default = synthetic constant-speed OKC circle (period exact by construction) through the whole real pipeline (boot page → race entry → proximity detect → CourseDetector "Normal" → laps ±40 ms); `--dovex <file>` replays a hardware log against its own header laps; diagnostic modes: `--dovex-noheader <file>` replays a header-less (crashed-session) log and prints live detection/lap state instead of asserting, `--two-session <file> [break-min]` reproduces a full track day (synthetic session 1 → auto-idle end → parked break with GPS drift → real-log session 2) to test CourseManager state carryover |
| `fixtures/okc_tillotson_1.dovex` | Hardware-recorded OKC session (13 laps) — the `--dovex` oracle's CI fixture; the sim reproduces its header lap list to the exact millisecond (also the `--two-session` carryover test's session 2) |
| `API.md` | Canonical WASM API contract (v1): artifact set, method surface, injectPvt schema, deltas from the handoff-spec draft (async `reset()` via module re-instantiation) |
| `wasm/bindings.cpp` | EMSCRIPTEN_KEEPALIVE exports over sim_host.h + getStateJson/getVersion/readFile/listFiles |
| `wasm/birdseye-sim.mjs` | Hand-written public ESM wrapper (stable import; async `reset()` re-instantiates the core module) |
| `wasm/test.html` | Standalone browser harness: canvas blit (hash dirty-check), buttons, dovex file playback (≥13 columns — 4.0.0 logs carry 16), synthetic GPS-fix toggle + mph field (parked on the OKC asset track, 40 ms inject/step interleave) so fix-gated flows like the course creator are reachable |
| `wasm/smoke.mjs` | Node smoke test the wasm CI job runs (boot→menu, state/version/VFS, determinism across instances, reset) |
| `CMakeLists.txt` | Native build; FetchContent pins: DovesLapTimer `BETA` (matches CI channel), SparkFun GNSS v3.1.9 (header-only use), ArduinoJson v6.21.5, ArxTypeTraits v0.3.2, Adafruit GFX 1.12.6 + SH110X 2.1.14 (real display stack) |

### Non-Source

| Path | Contents |
|---|---|
| `.github/workflows/` | CI: compile-sketch (+ flash-size gate), arduino-lint, unit-tests, clang-tidy, coverage, sim-build (native sim TU + 60 s boot soak + determinism + goldens + lap oracles + two-session carryover, plus a wasm job: emsdk 3.1.61 build + node smoke + `birdseye-sim-wasm` artifact), release (dual-board build + GitHub Release + prod OTA manifest to `gh-pages`), beta (dual-board build on `BETA`-branch push → latest-only `beta/` OTA channel on `gh-pages`, no Release). Per-channel build config: `BETA` builds track DovesLapTimer's `BETA` branch and pass `-DBIRDSEYE_ENABLE_SENSOREGG=1`; master/release pin `v4.3.0` and build the `project.h` defaults — which since 4.1.0 means NeoPixel ON, SensorEgg off |
| `tests/` | Host doctest harness (CMake) for the pure-logic units |
| `docs/plans/` | Numbered design records (`NNNN-slug.md`, see its README) — the rationale behind each chunk of work; plan-executing commits cite the number. Same convention as DovesDataViewer |
| `CHANGELOG.md` | Keep-a-Changelog history; release workflow ties to version tags |
| `ARCHITECTURE.md` | Human-facing architecture narrative (subsystems, design decisions) |
| `CONTRIBUTING.md` | Build/test/PR workflow and code conventions |
| `SECURITY.md` | Private vulnerability reporting + known posture |
| `.github/ISSUE_TEMPLATE/` | Bug report + feature request templates |
| `.github/PULL_REQUEST_TEMPLATE.md` | PR checklist |
| `SDCARD/TRACKS/` | Example track JSON files |
| `CASE/` | 3D-printable enclosure STLs |
| `TACHOMETER/` | Tachometer circuit documentation |
| `README.md` | User-facing project documentation |
| `LICENSE` | GPL v3 |

---

## Hardware & Pin Map

| Pin | Function | Detail |
|---|---|---|
| Serial1 RX/TX | GPS UART | u-blox SAM-M10Q, 57600 baud |
| I2C SDA/SCL | OLED display | 128x64, address 0x3C, 400 kHz |
| I2C SDA/SCL | LSM6DS3 IMU | Onboard accelerometer/gyro (Sense variant), address 0x6A |
| SPI MOSI/SCK/MISO | SD card | 2 MHz SPI clock (EMI hardened), CS grounded on PCB |
| D1 | Button 1 (Left) | INPUT_PULLUP, RC filter recommended |
| D2 | Button 2 (Select) | INPUT_PULLUP, RC filter recommended |
| D3 | Button 3 (Right) | INPUT_PULLUP, RC filter recommended |
| D0 | Tachometer input | INPUT_PULLUP, falling-edge ISR |
| PIN_VBAT / VBAT_ENABLE | Battery ADC | 1510/510 ohm divider, 3.6 V ref |
| Pin 30 (P0.09, NFC1 pad) | 5 V boost converter EN | HIGH = rail on; LOW retained through System OFF. Needs the one-way UICR NFC→GPIO conversion (subsystem 16). **On a profiling build this pin is the profiling output instead and EN is never driven** (subsystem 18) |
| Pin 31 (P0.10, NFC2 pad) | NeoPixel data | 11 px WS2812, GRB, 800 kHz. Same UICR requirement; swap with pin 30 in `neopixel.h` if wired the other way |

---

## Subsystem Architecture

### Main Loop Flow (`loop()`)

```
loop()  ~250 Hz
 ├─ GPS_LOOP()              checkUblox, feed CourseManager, log DOVEX
 ├─ TACH_LOOP()             re-enable ISR after debounce, apply EMA filter
 ├─ ACCEL_LOOP()            read LSM6DS3 accelerometer X/Y/Z (g-force)
 ├─ BLUETOOTH_LOOP()        stream file chunks if transfer active
 ├─ SENSOREGG_LOOP()        drain SensorEgg scan buffer → Temp1/Junction1
 ├─ trackDetectionLoop()    haversine scan → create CourseManager on match
 ├─ checkForNewLapData()    reads from active timer (CourseManager or lapTimer)
 ├─ checkAutoIdle()         tach: 60s <2mph; manual/speed: 5min <5mph (+ camera stop)
 ├─ updateGpsLockHold()     pin user to tach page until GPS time lock
 ├─ CAMERA_LOOP()           step Insta360 auto-record FSM (GPS/tach fresh)
 ├─ NEOPIXEL_LOOP()         LED strip frame at 30 Hz (also called in parked branches)
 │   (every call above is PROFILE_SECTION-bracketed; the macro expands to
 │    the bare call unless BIRDSEYE_ENABLE_PROFILING — subsystem 18)
 ├─ cameraConsumeAutoStop() camera 30s-engine-off stop → endRaceSession + menu
 ├─ calculateGPSFrameRate() 1-second PVT counter
 ├─ readButtons()           multi-sample debounce + edge detection
 ├─ gpsStatusPageLoop()     boot GPS status page: GPS re-detect + hold/auto-close/exit
 ├─ sdFormatPageLoop()      boot SD format page: hold-Select confirm → format + reboot
 ├─ displayLoop()           pages read from active timer helpers
 ├─ autoRaceModeCheck()     RPM>500 or speed>=10 → enter race from menu
 └─ resetButtons()          clear pressed flags
```

### 1. GPS & Lap Timing (`gps_functions.ino`, `gps_config.h`)

- Uses SparkFun u-blox GNSS v3 library with UBX binary protocol.
- `myGNSS` (SFE_UBLOX_GNSS_SERIAL) is stack-allocated in `BirdsEye.ino`.
- **Boot probe ladder** (`GPS_SETUP()` → `gpsSetupProbe()`): the SAM-M10Q
  has no flash and every boot is a cold start (sleep = System OFF), so the
  module can be in backup mode, configured-57600, or factory-9600. Setup
  sends the `0xFF` backup-wake byte FIRST (harmless if awake), probes 57600
  (warm case ≈ instant), falls back to 9600 + `setSerialRate`, and only
  pays a 1.5 s cold-boot delay + one retry when nothing answers. Config is
  applied via the VALSET API (GPS-only constellation, automotive dynamic
  model) at the current rate target, PVT (+ NAV-SAT when wanted) callbacks
  registered, and the **5 s PVT-arrival watchdog is armed at boot too** —
  a module that answers the ping but streams nothing gets
  `GPS_BAUD_RECOVERY()`. A GPS missing entirely is re-probed by
  `GPS_STATUS_RETRY_LOOP()` (3×, 10 s apart) while the status page shows
  `NOT DETECTED` / `CHECK WIRING`.
- **Two rate modes** (`gpsEnterStatusMode()` / `gpsEnterRaceMode()`): boot
  starts in status mode — `GPS_NAV_RATE_STATUS_HZ` (5 Hz) with UBX-NAV-SAT
  at ~1 Hz feeding per-satellite CNO into `gpsSatCnos[]` for the status
  page's bars. Leaving the page switches to race mode: `GPS_NAV_RATE_HZ`
  (25 Hz) PVT-only. `GPS_RECONFIGURE()` and every wake/recovery path
  re-assert the *current* targets (`gpsNavRateTarget` / `gpsNavSatWanted`)
  — never hardcode a rate.
- **GPS serial buffer — two buffers, two failure modes**: A 4 KB RAM ring
  buffer (`gpsRxBuf`) sits between Serial1 and the SparkFun library. A
  TIMER3 ISR drains Serial1 into this buffer every 5 ms
  (`GPS_DRAIN_INTERVAL_US`), independent of the main loop — this covers
  *downstream* stalls (SD GC pauses can block the loop 100 ms–2 s; the
  ring holds ~1.6 s). *Upstream*, only the core's Serial1 RX ring absorbs
  bytes while SoftDevice radio ISRs (prio 0–2, unmaskable) defer the
  prio-3 TIMER3 handler — that ring is grown 64→256 B via the
  **required `-DSERIAL_BUFFER_SIZE=256` build flag** (~44 ms of slack at
  57600 baud; `project.h` static_asserts it, CI passes it in all three
  workflows, local setup in CONTRIBUTING.md "Local build flags"). Both
  overflow points and the worst TIMER3 deferral are counted — see the
  `gpsStats*()` accessors, the `gps_stats` pure unit, and the GPS debug
  page (first page of the race rotation). The SparkFun library reads from
  the buffer via `GpsBufferedStream` (a `Stream` wrapper). During
  `GPS_SETUP()` (before timer starts), reads pass through to Serial1
  directly. Timer stopped on shutdown/charging entry, restarted on the
  charging-loop resume (`GPS_WAKE()`).
- `GPS_LOOP()` calls `checkUblox()` + `checkCallbacks()`. The registered
  `onPVTReceived()` callback fires with the full `UBX_NAV_PVT_data_t` struct,
  populates `gpsData`, and sets `gpsDataFresh` flag for downstream processing.
- PVT data is cached in `gpsData` struct (GpsData) for access by display
  pages and other subsystems.
- Feeds lat/lng/alt/speed into `CourseManager.loop()` which handles
  course detection, Lap Anything fallback, and sector timing internally.
- Logs validated data rows to SD as DOVEX: reserved 1 KB header,
  CSV data after byte 1024 (9-check validation pipeline).
- **Log file creation requires a valid time lock**: `onPVTReceived()` sets
  `gpsData.timeValid` only when the module asserts `validDate + validTime +
  fullyResolved` (and folds `gnssFixOK` into `gpsData.fix`). The log file is
  not created from the module's placeholder date — this prevents garbage-named
  files (e.g. `20210307_0000.dovex`) that collided every boot and corrupted on
  reboot. Until the lock arrives, `updateGpsLockHold()` pins the user to the
  tachometer page (engine running; the page shows `WAITING GPS LOCK..` so the
  pin never reads as a crash) and logging waits; a failed open is retried
  at 1 Hz and a write failure stops logging — **none fault out of race mode**.
  The pin **releases after 10 s of engine-off** (session stays active; a
  restart re-latches), and while it is active `checkAutoIdle()` may end the
  fileless session even if the camera is recording — the recording yield
  otherwise left no session-ender and the device looked bricked until a
  power cycle (2026-07-19 field incident).
- Time helpers: `getGpsTimeInMilliseconds()`, `getGpsUnixTimestampMillis()`.
- 64-bit timestamps are manually converted to strings (Arduino lacks `%llu`).
- **Wake hardening**: `GPS_WAKE()` (charging-loop resume) clears stale
  `gpsDataFresh`/`gpsData.fix`, re-applies VALSET config via
  `GPS_RECONFIGURE()`, and arms the same 5 s PVT watchdog as boot.
  If no PVT arrives, `GPS_BAUD_RECOVERY()` re-negotiates baud (9600→57600) and
  reconfigures. The SAM-M10Q has no flash; all config is volatile RAM only.

### 2. Tachometer (`tachometer.ino`)

- ISR `TACH_COUNT_PULSE()` fires on falling edge of D0.
- Minimum pulse gap comes from the spark mode only: `tach_filter::minPulseGapUs()`
  returns 3 ms for a plug firing every rev and 6 ms for a 4-stroke single-fire
  plug, so the ~20 000 RPM ceiling is the same in both modes. Cylinder count is
  not a term (plan 0014) — one clamped wire never delivers pulses faster than
  the cylinder it is wrapped around fires.
- **Ring buffer architecture**: ISR timestamps every valid pulse into a
  16-entry ring buffer (`tachRingBuf`). The ISR checks full before
  publishing (SPSC, one slot sacrificed) and drops + sets
  `tachRingOverflow` instead of lapping the consumer during SD GC stalls;
  `TACH_LOOP()` then discards the one period spanning the gap. `TACH_LOOP()` drains the buffer
  each main-loop iteration, computes mean inter-pulse period from ALL
  accumulated pulses, and feeds the result through a 1D Kalman filter.
- **Kalman filter** replaces the old median-of-3 + EMA. Estimate +
  uncertainty in `tach_filter::Kalman`; all math and tuning lives in the
  host-tested `tach_filter` pure unit. Three properties matter, and all
  three exist because the pre-0009 filter put visible spikes in the
  logged trace (plan 0009 has the full derivation):
  - **An outlier gate, not more smoothing.** The estimator sees ONE
    number per pulse and its steady-state gain is ~0.4, so a single bad
    edge is a spike, not a wobble: a ring clearing the 3 ms debounce at
    3000 RPM reads as 15 000 RPM. A measurement more than `kGateSigmas`
    (5) from the estimate is **not folded in — the estimate coasts**.
    Three *consecutive* rejections is a real step change (clutch dump,
    spin), so the third is **adopted outright** rather than filtered
    toward. 5 sigma is loose on purpose: the gate catches the
    half-/double-period signatures of a spurious/missed spark, which are
    tens of sigma out, and real acceleration (~25 RPM between pulses at
    5500 RPM/s) can never trip it. `k.rejected` counts rejections for the
    tach page's debug line and **survives the engine-stop reset**.
  - **Measurement noise knows the RPM** (`measurementNoise()`).
    RPM = K/period, so a fixed timing error costs RPM²/K of RPM error —
    QUADRATIC. A flat R was right at ~4000 RPM and far too confident
    above 8000. It also takes `revsPerPulse`: a twin's shorter periods
    are genuinely noisier at the same RPM.
  - **Process noise is a RATE** (RPM²/s, `processNoise()`), multiplied by
    the **engine time the batch spans** — `TACH_LOOP()` passes the sum of
    the periods it just consumed, never wall clock. Charging a fixed Q
    per update made the filter 4x looser at 12 000 RPM than at 3000, and
    looser again whenever an SD stall batched pulses together.
  The **first measurement after a reset is adopted whole**: with no prior
  there is nothing to filter against, and a slow climb out of 0 RPM would
  arm the gate partway up and get the engine's own speed rejected.
- **`tach_filter` setting — the track-side A/B switch** (default
  `smooth`). `smooth` is everything above; `legacy` reproduces the
  pre-0009 filter bit for bit (fixed Q/R, no gate) so new sessions
  compare against logs already collected; `raw` bypasses the estimator
  entirely, publishing what the periods say, spikes included — which is
  how you tell a dirty pickup from a bad filter. Read once at boot into
  `tachFilterMode` and applied inside `TACH_LOOP()`, so there is still
  exactly ONE RPM number in the firmware. Anything unrecognised means
  `smooth`.
- Time-based debounce only. Old volatile flag gate removed — ISR
  body is trivially fast (<1 µs) and cannot cause interrupt storms.
- **One correction, one place, and cylinder count is NOT in it** (plan
  0014). There is one sense wire and one clamp, and it goes around one
  spark plug wire — so the pickup sees ONE cylinder's ignition whatever
  the engine has. The whole geometry is the spark mode:
  `revs_per_pulse = (spark_mode == wasted ? 1.0 : 2.0)`. `tachRevsPerPulse`
  is set once at boot and applied at the period→RPM conversion in
  `TACH_LOOP()` — **before the Kalman filter**, because the filter's tuning
  is in RPM units (Q = 800 RPM² models crank inertia), so correcting
  afterwards would filter each engine type differently.
  **No consumer may re-derive or re-apply this**: display, DOVEX rows, the
  camera FSM and auto-race all read the already-corrected
  `tachLastReported`. Any new consumer must too.
- **Why the cylinder term is gone.** Plan 0003 shipped
  `pulses_per_rev = cylinder_count × sparkFactor`, which is only right for
  a clamp on a shared coil/king lead. On this hardware it divided RPM by
  the cylinder count: a V8 on a traditional magneto, configured honestly
  as 8 cylinders + single fire, read **an eighth** of its crank speed. The
  plan papered over it by redefining the setting as "cylinders the pickup
  *sees*" — a field named Cylinders that must not hold the engine's
  cylinder count. `cylinder_count` is now the engine's actual count, and
  it is descriptive: `tach_filter::rpmIsInferred()` uses it for the
  warning, nothing else reads it.
- **The accepted consequence, stated to the user.** Above one cylinder the
  crank speed is INFERRED from one cylinder's firing rate — between
  firings it is an assumption, and a cylinder that stops firing reads as a
  stopped engine. That is how every clamp-on inductive tach works; the
  settings UI, the README table and the boot debug line say so rather than
  leaving it to be discovered from a trace.
- Thresholds mean what they say on any engine whose clamped plug fires
  every rev — auto-race (>500), camera wake/record/stop (500/1500/300).
- `tachLastReported` updates every main-loop call (~250 Hz). Consumers
  (display at 3 Hz, logging at 25 Hz) rate-limit themselves.
- 500 ms timeout sets RPM to 0 (engine stopped), resets Kalman state.
- The engine-start wake from shutdown is NOT this module's job: System OFF
  wakes on the tach pin's GPIO SENSE and the boot decodes the LATCH bit via
  `wake_cause` (the old `tachHavePeriod` latch + `TACH_SLEEP()` are gone).

### 3. Accelerometer (`accelerometer.ino`)

- Onboard LSM6DS3 6-axis IMU on XIAO nRF52840 Sense (I2C address 0x6A).
- Shares I2C bus with OLED display (0x3C) — different addresses, no conflict.
- `ACCEL_SETUP()` initializes IMU; sets `accelAvailable` flag. Graceful
  degradation if IMU not present (non-Sense board).
- `ACCEL_LOOP()` reads `readFloatAccelX/Y/Z()` into global floats every
  main loop iteration (~250 Hz). Values in g-force (1g = 9.81 m/s²).
- No filtering — raw g-force is the standard unit for motorsports data.

### 4. SD Card & Logging (`sd_functions.ino`)

- SdFat library, FAT16/32, 2 MHz SPI (reduced from default for EMI hardening).
  Raised to 8 MHz (`SD_SPI_SPEED_FAST`) for the duration of a BLE or USB
  transfer via `sdSetTransferSpeed(true)` and reverted afterward — transfers
  happen parked (motor off), so the EMI rationale doesn't apply. Re-`SD.begin()`
  is the runtime clock switch; it falls back to 2 MHz if the fast re-init fails.
- Track files live under `/TRACKS/*.json` (ArduinoJson 6 parsing).
- **Blank-card self-provision**: `buildTrackList()` creates `/TRACKS`
  when missing (SdFat's `open()` never creates parent dirs), and
  `processTrackUpload()` mkdirs it again before every upload — so a
  factory-blank soldered-in module can sync tracks over BLE on first boot.
- **On-device format** (`sdPerformFormat()` + the host-tested
  `sd_format_page` unit): when `SD_SETUP()` finds the card answers at the
  SPI level (`SD.cardBegin` + `sectorCount()`) but `volumeBegin()` fails
  — the volume re-check matters: a transient card-level failure with a
  healthy FAT must remount, not be offered an erase — boot lands on
  `PAGE_SD_FORMAT` (buttons live, unlike FAULT). Hold Select ALONE 3 s
  to format FAT16/32 via SdFat's `SD.format()` (zero new RAM; WDT fed
  through the formatter's Print callbacks; 8 MHz clock only when the
  engine isn't turning — EMI corrupts writes silently — else 2 MHz),
  then `/TRACKS` is provisioned (`sdEnsureTracksFolder()`) and the
  device reboots clean. The confirm can never fire from the wake press
  (Select must be seen released once) nor beat the Select+side reboot
  combo (a held side button disarms it). A paired camera is stopped and
  powered off (`CAMERA_SLEEP()`) before the format blocks the loop, since
  the ending hard reset runs no shutdown teardown. Format failure returns
  to the confirm page (marked `FAILED - retry`; fresh full hold to retry —
  keeps the idle-timeout battery protection the FAULT dead-end lacks);
  a dead/absent card never offers the format. 5 min idle → shutdown
  (deferred while the engine runs), and a charging-loop resume with the
  card still unformatted returns to the format page, not the menu.
- **Dual JSON format**: `parseTrackFile()` auto-detects root type:
  - **Object** (LapWingData format): `longName`, `shortName`,
    `defaultCourse`, `courses[]` with `lengthFt`.
  - **Array** (older format, still accepted): bare array of course
    objects, metadata blank, `lengthFt = 0`. CourseDetector can't
    rank by distance without `lengthFt`, so these tracks fall back
    to Lap Anything mode.
- **Track manifest**: `buildTrackList()` also builds an in-RAM
  `trackManifest[]` (up to 200 entries) with first lat/lon per track
  for haversine proximity matching. ~10 KB RAM.
- **SD access arbitration** prevents concurrent access:
  - `acquireSDAccess(mode)` / `releaseSDAccess(mode)`
  - Modes: `SD_ACCESS_NONE` (0), `LOGGING` (1), `REPLAY` (2),
    `BLE_TRANSFER` (3), `TRACK_PARSE` (4), `USB_MSC` (5), `FORMAT` (6) —
    values and grant/deny rules live in the host-tested `sd_access_policy`
    pure unit.
  - `TRACK_PARSE` **nests under `LOGGING`** without taking ownership
    (`ownerAfterAcquire`): track detection and settings reads are brief,
    same-task, and use their own `File` objects, so they are safe alongside
    the session-long logging hold. Before this rule, any boot where the log
    file was created before the 1 Hz track-detect parse had the parse
    denied and silently fell back to Lap Anything for the whole session.
    `USB_MSC` is a normal exclusive holder (held for the whole USB
    mass-storage session; see subsystem 12).
  - Transitions are **atomic**: the check-then-set runs inside a FreeRTOS
    critical section (`taskENTER_CRITICAL`, BASEPRI-masked so SoftDevice
    radio interrupts are unaffected) because the Bluefruit callback task
    and the main loop share the owner flag.
  - Belt-and-suspenders only: all SD-touching BLE work is deferred to the
    main loop (see subsystem 6), so SdFat itself is single-task.
- Data flushes every 10 seconds during logging.

### 5. Display & UI (`display_ui.ino`, `display_pages.ino`, `display_config.h`)

- Driver selected at compile time (`USE_1306_DISPLAY` define).
- **Colour inversion** (`display_invert` setting): `displaySetInverted()` issues
  the controller's invert command — it swaps lit/unlit pixels on the panel and
  does **not** touch the framebuffer, so nothing that renders needs to know.
  `display.begin()` is called from exactly one place, `displayBeginPanel()`,
  which re-applies the preference: `begin()` resets the controller and clears
  the bit, and the I2C recovery path re-begins mid-session, so a second
  hand-written `begin()` would silently un-invert the screen on the first EMI
  glitch. The preference is applied right after `SETTINGS_SETUP()` rather than
  in the main settings block, because `displaySetup()` runs before the SD card
  exists — see the comment there. **No sim coverage is possible**: the golden
  frame hash is over the framebuffer, which inversion does not alter.
- Button debounce: 3 samples at 500 us intervals, 200 ms refire lockout.
- All lap times render via the host-tested `lap_format::formatLapTime()`
  (ms → `M:SS.mmm`, always 3-digit ms; zero-minutes styles: `kOmit` for
  replay results, `kShow` for the lap list, `kSpace` column-stable for the
  big-font live pages). Never hand-roll the `60000`/`%1000` math inline.
- Pages are integer constants; key pages:
  - Boot/menu: `PAGE_BOOT` (999), `PAGE_GPS_STATUS` (900, satellite status
    page every boot lands on — driven by `gpsStatusPageLoop()`, buttons
    deliberately no-op'd in `displayLoop()`), `PAGE_MAIN_MENU` (-1).
  - Racing: `GPS_PROFILE` (2, LOOP PROFILE — **only exists on a
    profiling build**, subsystem 18; being below `GPS_DEBUG` in a
    contiguous rotation, it drags the two diagnostic pages in too),
    `GPS_DEBUG` (3, GPS pipeline counters + lap debug) and
    `GPS_STATS` (4, battery/sats/SD/track status) through `LOGGING_STOP`.
    The two diagnostic pages are **hidden by default at runtime**: the
    `debug_pages` setting (default `hide`) leaves `runningPageStart` at
    `GPS_SPEED`; an explicit `show` lowers it to `GPS_DEBUG`. The constants
    always exist — hiding is purely the rotation's start bound, so the
    contiguous-block navigation needs no holes. `SENSOR_TEMP` (7, SensorEgg
    Temp1) and `SENSOR_TEMP2` (8, v2 aux intake-air temp) sit after
    `TACHOMETER` (6) — non-endurance only, and only when
    `BIRDSEYE_ENABLE_SENSOREGG` is set (beta). With the POC off (the
    master/release default) the block closes up behind the tach page and
    `LOGGING_STOP` is 12 instead of 14, same reshuffle idea as
    `ENDURANCE_MODE`. Page ids are internal — nothing external sees them.
  - Replay: `PAGE_REPLAY_FILE_SELECT` (-3, sessions + a trailing `Back`
    row — see `replayItemCount()` in `replay.h`, the single source of that
    layout for the renderer, the menu limit and the select handler),
    `PAGE_REPLAY_RESULTS` (-8), `PAGE_REPLAY_EXIT` (-9).
  - Transfer: `PAGE_TRANSFER_MENU` (-4) Bluetooth/USB/Back submenu,
    `PAGE_USB_STORAGE` (-5) USB drive active.
  - **Every menu page carries a Back/Cancel row.** Both transfer modes and
    the replay browser leave by rebooting or by walking forward, and the
    idle-shutdown timer only runs on the main menu (and the fault page), so
    a menu without one is a hard trap: the only escape is the unlabelled
    Select+side 5 s reboot combo. The deliberate exceptions are
    `PAGE_INTERNAL_FAULT` (buttons disabled, own idle shutdown),
    `PAGE_SD_FORMAT` (nothing to go back to) and the race rotation (leaves
    via `LOGGING_STOP`, speed-gated).
  - BLE: `PAGE_BLUETOOTH` (-2).
  - Camera: `PAGE_PAIR_CAMERA` (-6) pairing / paired-status management,
    `PAGE_CAMERA_SERIAL_ENTRY` (-7) manual 6-char serial entry fallback,
    `PAGE_CAMERA_TEST` (-10) bench test menu (paired-only manual controls).
  - Course creator (subsystem 15): `PAGE_COURSE_TRACK` (-11) track prompt,
    `PAGE_COURSE_TYPE` (-12) circuit/sprint, `PAGE_COURSE_LINES` (-13) line
    menu, `PAGE_COURSE_LINE` (-14) per-line points, `PAGE_COURSE_POINT`
    (-15) averaging hold. All five are contiguous so `courseCreatorActive()`
    is a range test. `PAGE_COURSE_PRUNE` (-16) sits deliberately OUTSIDE that
    range: it is a plain two-row confirm, not one of the model's screens, so it
    must not be handed to `course_creator::rowCount()`.
  - Errors: `PAGE_INTERNAL_WARNING` (100), `PAGE_INTERNAL_FAULT` (105),
    `PAGE_SD_FORMAT` (106, card responds but FAT won't mount — driven by
    `sdFormatPageLoop()`, buttons live unlike FAULT).

### 6. Bluetooth (`bluetooth.ino`)

- **Shared BLE core, used by transfer and the camera** (see subsystem 13):
  the one-time `bleCoreEnsureInit()` runs `Bluefruit.begin(1, 0)` — one
  peripheral connection, no central (both the transfer service and the
  camera remote are peripheral roles) — configures Just-Works bonding, and
  registers *every* GATT service (DFU, DIS, file service, camera remote via
  `cameraBleRegisterServices()`) before any advertising starts.
  `BLE_SETUP()` / `BLE_STOP()` are now just the transfer-mode owner
  transitions on top of that core.
- **Radio ownership (`BleOwner`)**: the single advert set + peripheral
  connection slot are shared between the transfer service and the camera
  remote. `bleOwner` (`NONE` / `TRANSFER` / `CAMERA`, enum in `project.h`,
  variable in `BirdsEye.ino`) records the current owner; the shared
  connect/disconnect callbacks route on it, so a camera link can never
  trigger the transfer auto-reboot and file commands are ignored unless
  the transfer service owns the radio. `bleActive` / `bleConnected` keep
  their transfer-only meanings — camera mode never sets them. Owner
  transitions happen only on the main loop, never in a Bluefruit callback.
  - **Reboot gate (`bleTransferEngaged`)**: the radio has one BD_ADDR, so a
    bonded camera can connect to the *transfer* advert and be routed as "the
    phone". The auto-reboot-on-disconnect is therefore gated on the peer
    having actually written the file/settings/OTA service — a camera that
    only vets our GATT and drops never reboots the logger out of a transfer
    session (and held no SD, so there's nothing to tear down).
  - **Advert teardown**: `BLE_STOP()` disarms `restartOnDisconnect(false)`
    *before* its async disconnect, so Bluefruit's core handler can't restart
    a stale ownerless transfer advert after the stop (which would let a phone
    reconnect into a mute session and occupy the slot camera auto-record
    needs).
- BLE service UUID `0x1820`.
- Characteristics: file list (0x2A3D), file request (0x2A3E),
  file data (0x2A3F), file status (0x2A40).
- **OTA + version services** (registered in `bleCoreEnsureInit()`):
  - `BLEDfu bledfu` — buttonless Nordic Secure DFU. A companion
    (DovesDataViewer over Web Bluetooth) writes the "enter bootloader"
    command; the board reboots into the bootloader's Secure DFU mode and
    receives a new firmware image over the air — no reset double-tap. The
    bootloader validates the signed/CRC'd DFU `.zip` before writing, so a
    bad/mismatched image is rejected rather than bricking the device. The
    board has no internet radio (BLE only): the companion downloads the
    GitHub release `.zip` and force-feeds it — the bootloader never
    "chooses" a file.
  - `BLEDis bledis` — Device Information Service (0x180A). Publishes
    `FIRMWARE_VERSION` (from `project.h`) via the Firmware Revision
    characteristic (0x2A26) so the companion can compare against the latest
    GitHub release and decide whether to offer an update. The Model string
    is `"BirdsEye-" FIRMWARE_VARIANT` (`BirdsEye-sense` / `BirdsEye-nonsense`)
    — equal to the release asset prefix, so the companion maps model →
    download directly. `FIRMWARE_VARIANT` is set by the per-FQBN build flag
    `-DBIRDSEYE_BOARD_SENSE` / `-DBIRDSEYE_BOARD_NONSENSE` (defaults to
    `sense`).
- **Download throughput — three levers, all verified after the fact**
  (plan 0008). The connect callback fires the three asks (MTU 247, 2M PHY,
  Data Length Extension) and then **checks them**, because firing is not
  the same as landing: the SoftDevice runs one link-layer control procedure
  at a time, so the DLE ask queued right behind the PHY ask can come back
  `NRF_ERROR_BUSY` into a return value nobody reads. `bleTuneLink()` runs on
  the main loop at +500 ms (read + correct) and +1500 ms (final readback):
  - **DLE.** `getDataLength()` still at the 27-byte default means every
    244-byte notify fragments into ten link-layer packets — the single
    biggest tax on a download. Re-request it with nothing else in flight.
  - **Connection interval.** The advertised preference stays
    `setConnInterval(6, 12)` (7.5–15 ms) because desktop/Android honour it
    and must not be slowed. **iOS is required to reject anything under
    15 ms** (Apple accessory rules), so it silently keeps its own choice —
    commonly 30 ms, i.e. half the connection events. Only when the measured
    interval is slower than the target does the device make a second,
    Apple-compliant `requestConnectionParameter(12)` ask. Never lower an
    already-fast interval.
  - **SD off the radio's critical path.** Chunks stream from a 4 KB
    read-ahead (`ble_stream::ReadAhead` + `bleStreamBuf`), refilled with one
    aligned multi-sector read, instead of a `FatFile::read()` between every
    notify. Refills are **compacting** so every notify but the file's last
    carries a full chunk. A failed notify simply does not `consume()` — with
    the bytes in RAM there is no file position to rewind, so the old
    `seekCur()` hole-punching hazard is structurally gone.
  Burst sending is bounded by **wall clock** (`kBurstBudgetMs`, 20 ms), not
  a packet count: the bound exists to keep the exit button and WDT serviced,
  and a fixed count is a wildly different amount of time on a fast link vs a
  slow one. A blocked `notify()` is the flow control and is *desirable* — it
  means the radio is saturated.
- **The transfer page reports what it got**: live KB/s, the SD clock actually
  in force (`sdActiveSpiHz()` — the 8 MHz parked bump falls back to 2 MHz
  silently), the negotiated link-layer PDU, and the ATT payload. Accessors
  `bleTransferRateBps()` / `bleLinkDataLength()` / `bleLinkChunkSize()`. This
  exists because a 4x download regression was only noticeable as "the
  percentage is creeping".
- **No SdFat in the callback task — ever.** Every SD-touching command
  (`LIST`, `GET:`, `DELETE:`, `TLIST`, `TGET:` via the deferred
  `fileCmdBuffer`; settings, `TPUT:`/`TDEL:`, and `FW*` via their own
  deferred state) is only parsed/validated in the BLE callback and is
  executed by `BLUETOOTH_LOOP()` on the main loop. Listings hold the SD
  lock for the whole directory walk; `DELETE` takes the lock and refuses
  (`BUSY`) while a transfer is streaming. One file command may be queued
  at a time — a second gets the protocol's busy reply (`BUSY` /
  `TERR:BUSY`).
- **Filename validation**: every BLE command carrying a filename
  (`GET:`, `DELETE:`, `TGET:`, `TPUT:`, `TDEL:`) runs the name through
  `filename_validator::isValidFilename()` BEFORE any `SD.open()` /
  `SD.remove()` / `"/TRACKS/%s"` path build. Rejects path traversal
  (`..`, leading `.`), separators (`/`, `\`), and FAT-unsafe bytes.
  `GET`/`DELETE` reject with `ERROR` / `NOT_FOUND`; track commands
  reject with `TERR:BAD_NAME`.
- **Settings commands** (via `fileRequestChar` / `fileStatusChar`):
  - `SLIST` → `SVAL:key=value` per entry, then `SEND`
  - `SGET:key` → `SVAL:key=value` or `SERR:NOT_FOUND`
  - `SSET:key=value` → `SOK:key` or `SERR:reason`
  - `SBUSY` returned if a command is already pending.
  - Uses deferred execution: BLE callback copies command into buffer,
    `BLUETOOTH_LOOP()` processes it in main loop for thread-safe SD access.
- **Track management commands** (via `fileRequestChar` / `fileStatusChar`):
  - `TLIST` → `TFILE:name.json` per file, then `TEND`
  - `TGET:name.json` → reuses existing file transfer (`SIZE:N` → data chunks → `DONE`)
  - `TPUT:name.json` → `TREADY` → app sends data chunks → `TDONE` → `TOK`
  - `TDEL:name.json` → `TOK` or `TERR:NO_FILE`
  - **Sprint variants** (`/TRACKS/SPRINT`, plan 0002) — same four verbs with a
    `TS` prefix, sharing the circuit code paths with a `kind` parameter:
    `TSLIST` → `TSFILE:name.json` per file, then `TSEND` (distinct tokens so a
    client can't confuse the two enumerations); `TSGET:` / `TSPUT:` / `TSDEL:`
    behave exactly like their circuit twins and reuse their replies
    (`SIZE:`/`DONE`, `TREADY`/`TDONE`/`TOK`, `TERR:*`).
  - **The folder is never taken from the wire.** `filename_validator` still
    rejects `/`, `..` and FAT-unsafe bytes on every track command; which of the
    two folders a command targets is decided by the *opcode* alone
    (`trackFolderFor()`), so a client cannot path its way between them.
  - Upload uses a static RAM buffer sized from `JSON_BUFFER_SIZE` (8192), so
    the largest track the device can parse is also the largest it can
    receive; `TERR:TOO_LARGE` if exceeded.
  - Error responses: `TERR:SD_BUSY`, `TERR:BUSY`, `TERR:WRITE_FAIL`, `TERR:NO_FILE`, `TERR:BAD_NAME`.
  - Upload/delete state machines: BLE callback sets flags, `BLUETOOTH_LOOP()`
    calls `processTrackUpload()` / `processTrackDelete()` for thread-safe SD
    access. Both call `buildTrackList()` after success.
- **Firmware OTA commands** (`FW*`, handled by `firmware_ota.ino` — see
  subsystem 11): `FWBEGIN`/`FWPUT`/`FWDONE`/`FWAPPLY`/`FWABORT`/`FWDFU`. The BLE
  callback dispatches them via `fwIsCommand()`/`fwHandleCommand()` and routes
  raw image chunks to `fwReceiveChunk()` while `fwReceiving()`. The request
  characteristic max length was raised from 64 to **244** so ~240-byte image
  chunks fit. `BLUETOOTH_LOOP()` calls `FW_OTA_LOOP()` each iteration.
- **Leaving transfer mode always reboots** — both ways out:
  - *Peer disconnect*: `bleDisconnectCallback()` flags a deferred teardown
    that `BLUETOOTH_LOOP()` runs on the main loop — `NVIC_SystemReset()`
    after a 100 ms delay so new settings take effect without a manual power
    cycle, plus `fwReset()` to abort any in-flight OTA and free the staging
    file + SD access.
  - *Manual Exit* (`bleExitTransferMode()`): the parked-loop Exit button
    runs `BLE_STOP()` then the same 100 ms-delay reboot. Before 4.1.0 a
    manual exit dropped back to the menu without rebooting, so settings
    written over BLE silently didn't apply until the next power cycle. The
    SIM stub returns after stopping (no reboot) so the golden menu walk can
    exit the Bluetooth page; `display_ui.ino`'s `PAGE_BLUETOOTH` handler is
    that sim-only path (on hardware the `bleActive` parked branch owns the
    button). **Exception — OTA apply**: if an
  apply has been requested (`fwApplyRequested()`), the teardown skips *both*
  the abort and the reboot. After `FWAPPLY` the web app disconnects on purpose
  to let the device self-flash; rebooting here would discard the staged image
  and boot the old firmware, so the apply is left to `FW_OTA_LOOP()` (called
  later in the same `BLUETOOTH_LOOP()`), which owns its own reset.

### 7. Replay (`replay.ino`)

- Instant DOVEX header-only replay. `parseDovexHeader()` reads the
  metadata line and the lap-times line from the first 1 KB of the
  file, populates `dovexReplay*` globals + `lapHistory[]`, and the
  results page renders directly from those — no file streaming, no
  re-running the lap timer. Only `.dovex` files shown in the browser.
- `haversineDistanceMiles()` lives here too; the track-detection loop
  in `BirdsEye.ino` uses it for proximity matching.

### 8. Settings (`settings.ino`)

- Persistent JSON key-value store at `/SETTINGS.json` on SD card.
- `SETTINGS_SETUP()` called once from `setup()` after SD init; creates
  default file on first boot (random BLE name + PIN + racing-word device name).
- **Auto-populate**: `ensureDefaultSettings()` checks for missing keys on
  boot and adds them with defaults. Existing values are never overwritten.
- `getSetting(key, buf, bufSize)` reads a value into a caller-provided
  buffer. Returns `true` if found, `false` on any failure (buf set empty).
  Always reads fresh from disk (no cache).
- `setSetting(key, value)` does read-modify-write to update a single key.
- Uses `SD_ACCESS_TRACK_PARSE` mode for brief SD access.
- Separate `StaticJsonDocument<1024>` — does not share the track parser's
  `JSON_BUFFER_SIZE` buffer.
- Total RAM cost: ~2 KB (1024-byte file buffer + 1024-byte JSON document).
- **There are TWO parsers of this file, and both are sized by
  `SETTINGS_JSON_CAPACITY` (`settings.h`).** `settings.ino` owns
  `getSetting()`/`setSetting()`; `bluetooth.ino`'s `SLIST` handler has its
  own buffer + document to enumerate the file for the companion app. They
  drifted once — plan 0010 raised settings.ino's pair 512 → 1024 and
  missed the BLE copy, so `SLIST` read 511 B of a 538 B file and answered
  `SERR:PARSE` on every device while `SGET`/`SSET` still worked. The
  shared constant is what makes that impossible now; if you add a third
  parser, size it from the same macro.
- **The two 1024s must stay equal, and adding keys is not free.** Every
  read path caps at `sizeof(settingsFileBuffer) - 1`, so a file bigger
  than the buffer parses as `IncompleteInput` and *every* key read
  fails — which `SETTINGS_SETUP()` reads as corruption, quarantines, and
  regenerates, whereupon `ensureDefaultSettings()` grows it back over the
  cap and the device loses its settings on a **boot loop**. Both were
  512 until plan 0010: the 18-key file was already 436 B and four new
  keys put it at 543, over the 511-byte read cap *and* past the old
  document's capacity (a `<512>` doc returns `NoMemory` at 22 string
  pairs). `setSettingInner()` now refuses any write whose
  `measureJson()` exceeds the buffer or whose document `overflowed()`,
  so the failure is a loud refused write rather than a silent brick.

### 9. CourseManager Integration

- **CourseManager** (`courseManager` global pointer): created when a track
  is detected via haversine proximity match, or with `courseCount=0` for
  immediate Lap Anything activation.
- **Track detection flow** (`trackDetectionLoop()`):
  1. Valid GPS time lock acquired → DOVEX log file created (see GPS section).
  2. Scans `trackManifest[]` via haversine, throttled to 1 Hz (the scan
     is O(N) software-double math; `gpsData.fix` stays true between PVT
     updates, so an unthrottled scan ran every ~250 Hz loop iteration).
  3. Closest match within 5 miles → parse full JSON, build `TrackConfig`.
  4. Create `CourseManager` with settings-configurable thresholds.
  5. CourseManager handles course detection + Lap Anything fallback.
  6. No tracks / no match → `CourseManager(courseCount=0)` → Lap Anything.
- **Active timer abstraction**: helper functions (`activeTimerLaps()`,
  `activeTimerBestLapTime()`, etc.) provide a unified interface for display
  pages. They check CourseManager's active timer (DovesLapTimer or
  WaypointLapTimer) and return appropriate values.
- **Auto-race** (`autoRaceModeCheck()`): from main menu, if RPM > 500 or
  speed >= 10 mph, jumps directly to race mode — but only once the menu has
  been **settled** for `AUTO_RACE_MENU_GRACE_MS` (3 s), anchored on the newest
  of the menu-arrival stamp (`mainMenuEnteredAtMs`, set by
  `switchToDisplayPage()`) and the three button `lastPressed` values. Without
  it, exiting any page while moving landed on the menu and entered race mode on
  the very next loop iteration (~4 ms), so the menu was never drawn and the
  device looked like it acted on its own.
- **Race-entry cause** (`startRaceSession(RaceEntryCause)` — the single
  session-start entry point; the three triggers all route through it):
  `RACE_ENTRY_MANUAL` (menu Race select), `RACE_ENTRY_SPEED` (auto-race
  speed trip), `RACE_ENTRY_TACH` (auto-race RPM trip / tach-wake boot).
  The cause picks the session-end rule and the camera driver below;
  `endRaceSession()` resets it to `RACE_ENTRY_NONE`. **SPEED and MANUAL
  both promote to TACH** the moment the tach proves itself (>500 rpm,
  `idle_policy::tachProven`) — a push/bump-started tach kart trips the
  speed gate before the engine fires, and a menu press happens engine-off;
  neither must carry no-tach rules all session. No-tach devices never
  read >500 rpm, so their sessions keep the 5 min/5 mph rules.
- **Auto-idle** (`checkAutoIdle()`, cause-aware; the decision table —
  rule selection, camera yield, resets — is the host-tested `idle_policy`
  unit, the sketch keeps only the clock and side effects): **tach
  sessions** — if
  speed < 2 mph for 60 seconds continuously, writes DOVEX header, closes
  file, cleans up CourseManager, and returns to main menu (yields to an
  active camera recording, which owns its own 30 s engine-off end).
  **Manual/speed sessions** — speed < 5 mph for 5 minutes ends the session
  AND stops the camera (`CAMERA_NOTIFY_SESSION_END()` before
  `endRaceSession()`); these sessions have no engine signal, so this timer
  never yields to the camera — it is the only ender. **Sprint mode is
  engine-aware**: idle counts only while the tach reads 0 too (between-run
  queue waits keep the engine running), and every completed run re-arms
  the 3-minute grace period.
- **Sprint mode (plan 0002)**: tracks under `/TRACKS/SPRINT/` make the
  session point-to-point. `trackDetectionLoop()` finds the nearest
  manifest entry PER KIND; with both kinds in range the `race_mode`
  setting breaks the tie via the host-tested `sprint_select` unit (the
  event-day heuristic parses the sprint file once for its newest
  `date_created`). The sprint path skips CourseManager/CourseDetector
  entirely — `createSprintSession()` picks the newest course
  (`sprint_select::newestCourseIndex`) and stands up the library's
  `SprintTimer` (start + separate finish + optional S2/S3 lines; ~11.6 KB
  heap, one instance). `sprintTimer != nullptr` IS sprint mode —
  `courseManager` stays null for the session and every `activeTimer*()`
  helper checks the sprint branch first (runs duck-type as laps; "laps"
  verbiage kept everywhere by design). Run completion is captured on the
  RUN-COUNT EDGE in `checkForNewLapData()` (identical consecutive run
  times are normal; value-change dedupe would drop them). Between runs
  the Current Lap / Pace pages show `*waiting*`
  (`sprintModeIsActive() && !activeTimerRunActive()`); everything else
  stays live. The DOVEX header gets `race_mode=SPRINT` and the sprint
  course name.

### 10. Shutdown (System OFF)

"Sleep" is a full power-down: nRF52 **System OFF** (~µA), designed so the
hardware needs no power switch. Wake = chip reset = fresh `setup()`.

- **Entry** (`enterShutdown()`): long-press left+right (5 s) on main menu,
  5-min menu idle, the GPS status page's idle timeout, the SD format
  page's idle timeout (deferred while the engine runs, so a tach-wake
  with a bad card doesn't power-cycle all session), or — only with
  `BIRDSEYE_ENABLE_ONBOARD_CHARGING` — USB present on the main menu after
  60 s of button inactivity (`USB_MENU_CHARGE_IDLE_MS` — not immediate, so
  a charging-loop button wake doesn't bounce and the device stays usable
  for replay/transfer while plugged in). With onboard charging off (the
  default) that USB trigger is compiled out entirely: the firmware isn't
  managing the charge current, so a cable is no reason to cut the menu
  short — the plain 5-min idle still fires and still parks on VBUS.
- **Teardown order** (wdtPet-bracketed — `CAMERA_SLEEP()`'s 3 s ce82
  power-off hold is the longest step under the armed ~4 s WDT): end race
  session → `CAMERA_SLEEP()` → `BLE_STOP()` if active →
  `SENSOREGG_SLEEP()` + `bleShutdownQuiesce()` (**unconditional** radio
  quiesce: stop the egg scanner and any advertising, drop a surviving
  link with a bounded WDT-fed settle, then `bleConnLedOff()` LAST —
  `BLE_STOP()` is transfer-only, so a camera-owned radio used to reach
  System OFF with the conn LED still driven; GPIO state is retained
  there, hence the "blue light stays on after sleep" field report) →
  `DISPLAY_SLEEP()` → `GPS_SLEEP()` (u-blox software backup, µA, config
  retained while powered; TIMER3 stopped) → IMU power rail off →
  `NEOPIXEL_SLEEP()` (blank while 5 V is up, data LOW, then boost EN
  LOW — before the charging branch, so the strip is dark on the cable
  too). **`NEOPIXEL_SLEEP()` is not a no-op on a flag-OFF build**: it
  still drives boost EN low when `UICR->NFCPINS` shows the pads were
  already converted by an earlier beta build, because the driven level
  is what survives System OFF and a floating EN leaves the rail up (the
  same retention as the conn-LED report above). An unconverted board is
  never touched. The charging-loop soft resume
  (`softResumeFromCharging()`) restarts the egg scanner via
  `SENSOREGG_WAKE()` and re-raises the strip via `NEOPIXEL_WAKE()`;
  BLE/camera stay lazy.
- **System OFF entry** (`shutdownSystemOff()`, no return): wait for the
  entry combo's buttons to release (a held button = SENSE satisfied =
  instant wake-reset), **sample the tach line's parked idle level**
  (15 reads over ~30 ms, majority vote in the host-tested
  `wake_cause::tachIdleIsHigh()`) and configure
  `nrf_gpio_cfg_sense_input(pull-up, SENSE opposite the idle level)` on
  the tach pin — the pickup's Schmitt-inverter + optocoupler output
  stage idles high or low depending on the circuit build, and arming
  toward the idle level was an instant wake-reset loop on battery
  (runtime RPM counting can't tell the polarities apart: one falling
  edge per pulse either way). Buttons are fixed active-low →
  `SENSE-LOW` on all 3 (P-numbers via `g_ADigitalPinMap`, never
  hardcoded). Then clear the GPIO LATCH registers
  (a set latch = pending DETECT = instant re-wake), clear pending FPU
  exceptions, then `sd_power_system_off()` when the SoftDevice is enabled
  (BLE is lazy — check `sd_softdevice_is_enabled()`) else raw
  `NRF_POWER->SYSTEMOFF`. **GPREGRET is untouched** — register 0 belongs
  to the OTA/bootloader handoff (subsystem 11). The WDT halts in System
  OFF (all clocks stop); `wdtSetup()` re-arms on the fresh boot.
- **Wake sources**: tach pulse (D0, engine start — any transition away
  from the sampled idle level), any button, or VBUS (USB plug-in,
  always armed on nRF52840).
- **Wake-cause decode** (`captureBootWakeCause()`, FIRST thing in
  `setup()`): reads then clears `RESETREAS` + `NRF_P0/P1->LATCH` (sticky,
  cumulative) and decodes via the host-tested `wake_cause` unit. A tach
  wake makes the GPS status page exit into race mode with logging; a USB
  wake skips the status page straight into the charging loop — that
  shortcut too is `BIRDSEYE_ENABLE_ONBOARD_CHARGING`-only, since with
  charging off a cable means "host connected" and the device just boots
  normally (its idle timeout parks it on VBUS soon enough).
- **Onboard charging is a build flag** (`BIRDSEYE_ENABLE_ONBOARD_CHARGING`
  in `project.h`, **0 in every shipped build** since 3.0.1): the hardware
  now carries an external charging circuit, so the firmware leaves HICHG
  alone (BQ25100 stays at its ~50 mA default) and drops the charging UX.
  Set it to 1 to restore the pre-3.0.1 behavior. The VBUS park below is
  NOT gated on it.
- **Charging loop — the one soft-sleep survivor** (`runChargingShutdownLoop()`):
  System OFF is never entered while VBUS is present. Two reasons: the
  HICHG fast-charge pin (`PIN_CHARGING_CURRENT`) is software-held when
  onboard charging is compiled in, and — regardless of the flag — VBUS is
  an always-armed System OFF wake source, so entering OFF with the cable
  in risks an immediate wake-reset loop. After the
  same full teardown, the loop shows the charging screen for 10 s then
  turns the display off; **any button is a full wake to the main menu**
  (`softResumeFromCharging()`: IMU re-init, race-mode GPS targets +
  `GPS_WAKE()`, display on); unplugging drops to System OFF. CPU idles
  via `shutdownIdleWait()` — `sd_app_evt_wait()` only when the SoftDevice
  is actually enabled, `__WFE()` otherwise.

### 11. Firmware OTA (`firmware_ota.ino`, `crc32.{h,cpp}`)

- **Why self-flash**: Chrome's Web Bluetooth blocklist bans the Nordic
  *legacy* DFU service `BLEDfu` exposes, and the sealed units have no
  button/SWD pins to install a web-allowed Secure-DFU bootloader. So the app
  updates itself: the web app streams the image to SD over the existing
  `0x1820` service, the firmware CRC-verifies it, stages it to a free flash
  region, and a RAM flasher swaps it into the app region and resets. The
  bootloader is **not** changed for the field flow.
- **Wire protocol** (text on `0x2A3E` in / `0x2A40` out; image bytes are raw
  binary writes to `0x2A3E`):
  - `FWBEGIN:<size>,<crc32hex>,<variant>` → `FWCRC:<crc32hex>` (echo handshake
    to verify the control channel before any upload). `<variant>` (`sense` /
    `nonsense`) is the target board variant the web app derives from the
    device's DIS Model Number; the firmware compares it (case-insensitive) to
    `FIRMWARE_VARIANT` and replies `FWERR:VARIANT` here — the single variant
    gate, before any upload.
  - `FWPUT:<size>` → `FWREADY`, then raw ≤240-byte chunks streamed to SD
    (`/fw/pending.bin`), then `FWDONE` → `FWOK:<crc>` (CRC of the stored
    file) or `FWERR:CRC|SIZE|WRITE`.
  - `FWAPPLY` → `FWSTAGE:<pct>` (0–100, repeatable) → `FWAPPLIED` then reset,
    or `FWERR:<reason>`. `FWABORT` cancels at any point.
  - `FWDFU` → `FWDFU:OK`, then the device reboots into the Adafruit
    bootloader's **UF2 mass-storage DFU** (GPREGRET `0x57` =
    `DFU_MAGIC_UF2_RESET` — a stock bootloader feature, same handoff
    register the OTA recovery flag uses). The device enumerates as a USB
    drive; copying a `.uf2` onto it flashes the app region directly — **no
    image-size cap, no staging region**. This is the "pre-update" escape
    hatch (plan 0004): any unit carrying this command can always be
    updated with just a USB cable regardless of future image growth.
    Exiting UF2 mode without flashing boots the existing app unchanged.
    Accepted from any FW state (an in-flight OTA is cleanly aborted
    first); executed on the main loop like all FW commands.
  - Error tokens: `CRC`, `SIZE`, `WRITE`, `BATTERY`, `VARIANT`, `STATE`,
    `FLASH`.
- **CRC**: CRC-32/IEEE-802.3 (zlib), reflected poly `0xEDB88320`, init/xor
  `0xFFFFFFFF`, lowercase 8-char hex, compared case-insensitively. Shared
  with the web client via the host-tested `crc32` pure unit. Sanity vector
  `crc32("123456789") == 0xcbf43926`.
- **Threading**: like track upload, the BLE callback only parses commands and
  copies chunk bytes into a RAM double-buffer; `FW_OTA_LOOP()` (main loop)
  does all SD writes, CRC verify, and the apply sequence. SD held via
  `SD_ACCESS_BLE_TRANSFER` for the receive.
- **Apply** (`fwDoApply()`): guards first — refuse below `FW_MIN_APPLY_VOLTAGE`
  (3.6 V, uses cached `lastBatteryVoltage`) → `FWERR:BATTERY`. (Variant is
  validated earlier, at `FWBEGIN`; no image-byte scan here. The image still
  embeds `kFwImageDescriptor` for forensics.) Then `fwStageToFlash()` copies
  SD → upper flash
  (`FW_STAGE_BASE`, via the `flash_nrf5x` HAL) and **re-verifies the CRC in
  flash before the app region is ever erased** (`FWERR:FLASH` on mismatch).
  Only then: emit `FWAPPLIED`, arm the GPREGRET recovery flag
  (`FW_GPREGRET_OTA_DFU`), disable the SoftDevice, and call the RAM-resident
  `fwRamFlasher()` to erase the app region, copy the staged image down, and
  reset.
- **Recovery net**: an interrupted swap leaves an invalid app, so the
  bootloader comes up in BLE DFU and the unit is re-flashable over the air via
  the nRF Connect mobile app — no pins. **This is the one Phase 0 spike still
  unproven on hardware.** The apply path itself has shipped and is flashing
  units in the field, which closes the other spikes by demonstration; but a
  *successful* update never walks the recovery path, so it stays untested
  until someone deliberately corrupts an app region and confirms the unit
  comes back. See `docs/plans/0000-firmware-ota-phase0.md` → *The one test
  still outstanding*.
- **Fleet migration**: the first firmware carrying `FW*` is pushed to sealed
  units once via nRF Connect (native app, buttonless trigger works on the
  existing single-bank bootloader); all later updates go through the web app.

### 12. USB Mass Storage (`usb_msc.ino`)

- **Why**: a wired, app-free way to move files. The SD is FAT16/32, so a
  host PC can mount it as a drive and drag-and-drop track JSON / DOVEX logs.
  Complements (does not replace) the BLE transfer service.
- **Stack**: TinyUSB `Adafruit_USBD_MSC` (bundled in the Seeed/Adafruit
  nRF52 core; the core's default USB stack is TinyUSB). Three block
  callbacks wrap SdFat's block device: `msc_read_cb` →
  `SD.card()->readSectors()`, `msc_write_cb` → `writeSectors()`,
  `msc_flush_cb` → `syncDevice()` + `SD.cacheClear()`. These run on the
  USBD task, not the main loop.
- **UI flow**: main-menu **Transfer** → `PAGE_TRANSFER_MENU` (Bluetooth /
  USB / Back). **Bluetooth** keeps the existing `BLE_SETUP()` +
  `PAGE_BLUETOOTH` path untouched. **USB** → `PAGE_USB_STORAGE` +
  `USB_MSC_ENABLE()`. **Back** returns to the main menu — both transfer
  modes exit by rebooting, so without it this page could only be left by
  starting a transfer.
- **Opt-in enumeration**: `USB_MSC_SETUP()` (called from `setup()` after a
  successful `SD_SETUP()`) only registers the callbacks — no drive is
  presented at boot, so charging/plug-in behaves as before.
  `USB_MSC_ENABLE()` first requires **VBUS present** (`isUsbConnected()`) —
  without a cable there is nothing to mount and the parked loop would read
  absent VBUS as a cable-pull and instantly reset, so it bails before taking
  the lock or enumerating (the menu shows "Plug in USB cable first"). It then
  acquires `SD_ACCESS_USB_MSC`, sets the capacity from
  `sectorCount()`, marks the unit ready, `begin()`s the interface (bailing
  out — restoring the clock and releasing the lock — if `begin()` fails),
  and forces a `TinyUSBDevice.detach()` / 50 ms / `attach()` re-enumeration
  so the host mounts the drive. If the SD mutex is busy it bails to a
  warning page and changes nothing.
- **Loop parking**: while `usbMscActive`, `loop()` takes an early-return
  branch (mirroring the `bleActive` branch) that skips all GPS/tach/lap/SD
  processing — the host PC owns the FAT, so the firmware must not touch it
  concurrently. The branch services only the Exit button and the status
  page, and watches VBUS: if the cable is unplugged it calls
  `USB_MSC_DISABLE()` so the SD lock and fast SPI clock can't leak past the
  session.
- **Exit = reboot**: `USB_MSC_DISABLE()` drops media-ready, **detaches USB**
  (`TinyUSBDevice.detach()` — `setUnitReady(false)` only blocks *new* SCSI
  commands, so without the detach the host keeps issuing traffic and an
  in-flight `READ10`/`WRITE10` keeps calling the block callbacks on the USBD
  task), then **drains**: waits (WDT-fed, up to 4 s — SD garbage collection
  can stall one `writeSectors()` 100 ms–2 s) until no block callback is
  executing and none has finished for 100 ms, tracked by
  `mscIoInFlight`/`mscLastIoMs` around all three callbacks — reads and flush
  included. Only then `syncDevice()` + `NVIC_SystemReset()` (mirrors the BLE
  transfer-mode exit reboot). The 4.0.0 exit tracked only write *entry*
  times, so a callback still on the SPI bus raced the main-loop sync and the
  wedge came back via the watchdog instead of the clean reset. The reboot drops the MSC interface and remounts a clean
  filesystem, so host edits are picked up without any SdFat cache-coherency
  dance. Triggered by the on-device Exit button or a cable unplug.
- **Mutex**: the whole session holds `SD_ACCESS_USB_MSC`, so logging,
  replay, and BLE transfer are locked out (and vice-versa) — though loop
  parking, not the mutex, is the primary guarantee the firmware stays off
  the card. The transfer/USB pages are not the main menu, so the
  USB-on-main-menu charge-mode entry never fires while transferring.
- **No pure unit test**: the block-callback glue is TinyUSB/Arduino-bound
  (hardware), so there is no host-testable logic here — only the
  `sd_access_policy` mode addition is unit-tested.

### 13. Camera Auto-Record (`camera_ble.ino`, `camera_fsm.{h,cpp}`, `insta360_protocol.{h,cpp}`)

- **What**: hands-free Insta360 X4 control. The device *is* the Insta360
  "GPS Remote": it emulates the physical remote so a paired X4 is woken
  when the engine starts, records the session, and powers itself off
  afterward — the driver never touches the camera.
- **Single BLE role: peripheral remote** on the one SoftDevice
  (`Bluefruit.begin(1, 0)`). We host the remote's GATT — service `0xCE80`
  (ce81 WRITE camera→us carrying its serial + status frames, ce82 NOTIFY
  us→camera button frames, ce83 READ static) — advertise the wake/identity
  payload, and **the camera connects to us as central** and subscribes to
  ce82. **All control is a ce82 button notification**, byte-for-byte the
  physical remote's frames: recording toggles via the shutter button, and
  power-off streams the 3-second power-hold. We never act as central: no
  scanning, no `be80` client, no `be81` writes. (The old central role held
  a `be80` link to the camera for start/stop-video — removed. It made
  power-off impossible: power-off only exists as a remote `ce82` hold,
  which cannot coexist with being the camera's `be80` client.)
- **GPS overlay** (`cameraServiceGpsStream()`): a Wireshark capture of the
  genuine remote↔camera link showed the remote streams GPS on **`ce82` at
  10 Hz** as a non-standard NMEA-RMC frame (`FC EF FE 83 00 <len>
  ,26.7,\x07,$GNRMC,...` — signed longitude with a constant `E`, an extra
  `V` field; built + golden-tested in `insta360_protocol::buildGpsRmcFrame`).
  The firmware streams it continuously while the camera is connected +
  subscribed — this doubles as the remote's **liveness heartbeat** (never
  go silent or the camera drops us; status `V` with last-known coords when
  there's no fix), paused only during a power-off hold. GPS still logs to
  SD independently.
- **Lifecycle FSM** (`camera_fsm` pure unit, host-tested): the race-mode
  lifecycle is deliberately **RPM-driven and simple** — with one parallel
  driver, `Inputs::sessionDemand`, for **manual/speed-entered sessions**
  (no tachometer, RPM pinned at 0): the glue sets it from `raceActive` +
  `raceEntryCause` (MANUAL/SPEED), and it substitutes for the RPM gates —
  wake immediately from IDLE (no debounce), suppress the WAKING rpm-gone
  abort, arm the record clock (the 5 s `kRecordStartDelayMs` still
  applies), and **suppress the 30 s rpm<300 auto-stop** (rpm=0 would end
  every such recording); these recordings end only via
  `sessionEndRequested` — the sketch's 5 min/<5 mph idle timer
  (`checkAutoIdle()`, which calls `CAMERA_NOTIFY_SESSION_END()`) or the
  manual stop confirm. Tach sessions (`RACE_ENTRY_TACH`) never set it and
  behave exactly as below. 7 states — UNPAIRED /
  IDLE / WAKING / AWAIT_READY / RECORDING / **WATCHING** / PAIRING (the old
  COOLDOWN/POWERING_OFF tail is gone — power-off is now sleep-only).
  RPM > 500 held 2 s enters WAKING, which broadcasts the 31-byte CONNECTABLE
  wake advert — the sniffed GPS-Action-Remote manufacturer payload (serial at
  mfg[14..19], per the primary `pchwalek/insta360_ble_esp32` source) in the
  primary PDU with the "Insta360 GPS Remote" name in the scan response, both
  set as raw bytes so the stack can't reshape them (retry ×3). The woken
  camera connects back and the FSM moves to AWAIT_READY (wait for the ce82
  subscription; bounded re-wake if it never subscribes) → **WATCHING**.
  (Wake only reaches an ARMED camera: on the X4, Bluetooth Wakeup is armed
  when **QuickCapture is OFF** — an armed camera keeps its radio scanning even
  fully powered off. Every advert goes through `bleAdvFinalizePadded()` — see
  bluetooth.ino — to defeat the Bluefruit 0.21.0 frozen-packet-length core
  bug.) **Recording starts** from WATCHING once RPM has held at/above
  `kRecordRpmThreshold` (1500 — deliberately far above the 500 wake
  threshold: pull-start cranking blips clear 500 and once started a
  recording during a failed first start; any dip below 1500 restarts the
  clock) for `kRecordStartDelayMs` (5 s) — **no GPS-lock gate** (GPS still
  streams the whole time) — by sending one shutter-toggle ce82 frame. The shutter is a
  stateful TOGGLE, so the FSM never blind-fires it: it **confirms** record
  state from the camera's own `0x10` ce81 display-string frame (a live
  `.HH:MM:SS` timer while recording — the `0x02` status word is not reliable;
  `insta360_protocol::parseRecordingState`) and reconciles `recordingActive`
  against that observation (`RecordObs` Input). On reconnect it adopts the
  camera's real state instead of toggling; if the camera reports idle while we
  believe we're recording it re-asserts the shutter once; and the belief is
  preserved on any path where the camera is unreachable, so a dropped link
  can't invert on reconnect. **Recording stops** after `kStopRecordDelayMs`
  (30 s) of engine-off (RPM < 300) — **RPM only, no speed**, so a
  stationary-but-running grid idle keeps recording — sending one shutter
  toggle and returning to WATCHING. The 30 s-engine-off auto-stop also **ends
  the race log session** (see the read-only note below). A manual session end
  (`CAMERA_NOTIFY_SESSION_END()` from the logging-stop confirm) stops the
  camera immediately, also to WATCHING. **WATCHING** keeps the camera ON and
  connected: if RPM returns it re-records (stall recovery), and it powers the
  camera off **only when the device shuts down** (`CAMERA_SLEEP()` streams the
  ce82 power-hold synchronously) — there is no post-record cooldown/power-off
  timeout. All timing lives in the FSM so the temporal behavior is
  host-testable; every tunable is a single-point `constexpr` in
  `camera_fsm.h`. The unit is the board-portable core shared with the nRF54
  ("Falcon") target — nothing in it may `#ifdef` on the platform.
- **Telemetry consumer, with one deliberate write-back**: the FSM consumes an
  `Inputs` snapshot (RPM, link state, observed record state, one-shot events)
  built fresh each `CAMERA_LOOP()` and returns at most one `Action`. Camera
  mode never parks the main loop (unlike `bleActive` / `usbMscActive`). The
  ONE exception to the old read-only guarantee: when the camera auto-stops
  (30 s engine-off), the glue latches `cameraConsumeAutoStop()` and the main
  sketch calls `endRaceSession()` + returns to the menu — so with a camera
  paired+recording the log ends on 30 s-no-RPM instead of the speed-based
  auto-idle (which `checkAutoIdle()` suppresses while `cameraActivelyRecording()`).
  Without a camera, logging is unchanged.
- **Pairing / bonding**: entering pairing from `PAGE_PAIR_CAMERA`
  advertises connectably as "Insta360 GPS Remote"; after connecting, the
  camera writes its 6-char ASCII serial to ce81, which is captured and
  persisted in the `camera_serial` setting (empty = unpaired). The manual
  6-char entry page (`PAGE_CAMERA_SERIAL_ENTRY`) is the fallback. Pairing
  times out after 2 min. The genuine remote link is encrypted + bonded, so
  we support Just-Works (NoInputNoOutput) pairing as peripheral — the
  camera may withhold its ce82 subscription until the link is encrypted.
- **Coexistence** (`bleOwner`, see subsystem 6): the camera shares the
  single advert set + peripheral slot with the transfer service. Opening
  the Bluetooth transfer page calls `CAMERA_FORCE_RELEASE()` before
  `BLE_SETUP()` — best-effort stop recording, drop the camera link, stop
  camera-owned advertising, force the FSM to IDLE, release the radio.
  Shutdown entry runs `CAMERA_SLEEP()` (same, plus a power-off that is
  **streamed synchronously** before the disconnect — the chip powers off
  right after, so the non-blocking ce82 hold would otherwise never transmit
  a frame and the camera would run all night). BLE
  comes up **lazily** on the first camera action (first advertising
  `Action`), so unpaired users pay zero RAM/power cost.
- **Threading**: same deferred pattern as `firmware_ota` — Bluefruit
  callbacks (connect/disconnect/ce81 writes/ce82 CCCD writes) only copy
  into RAM and set volatile flags; `CAMERA_LOOP()` on the main loop
  consumes them, steps the FSM, and does all real work (including the one
  `setSetting()` that persists a captured serial and streaming the
  power-off hold). The ce82 CCCD callback latches a cached `ce82NotifyOn`
  flag so the hot paths (per-loop Inputs snapshot + 10 Hz GPS tick) never
  SVC into the SoftDevice for the subscription state; `CAMERA_LOOP()`
  re-reads `notifyEnabled()` only at a low rate while linked-but-not-yet-known-
  subscribed, to catch a bonded peer's silent sys-attr CCCD restore.
- **X4-VERIFY posture**: all frame bytes live in the host-tested
  `insta360_protocol` pure unit with golden-byte tests. The **wake
  advert + scan response are X4-CONFIRMED ground truth** — captured from
  a genuine GPS Remote with nRF Connect (2026-07-10 bench session) and
  the replayed packet woke the sleeping X4; flags are `0x05`, and the
  paired camera-mode advert presents this same remote-identity packet.
  Remaining `// X4-VERIFY(sniff)`: the ce82 button frames (proven capture
  bytes; the record/power-off *effect* still to be confirmed end-to-end on
  an X4) and the ce81/ce83 parsers.
- **Bench test menu** (`PAGE_CAMERA_TEST`): the paired Camera page has a
  **Test** entry opening a manual-control menu (Wake / Record / Power Off /
  Back) plus live remote (**R**) link status (`R:UP+` when the camera is
  connected AND subscribed to ce82, `R:UP` when connected but ignoring our
  buttons), advert (**Adv:**) status, a **G:** GPS-feed indicator (`SYNC`
  with a fix / `V` voided-but-streaming / `--` not streaming) and a
  **rec:yes/no** state driven by the camera's own `0x10` record timer
  (`rec:--` when there's no fresh observation — no link, or the camera hasn't
  pushed a `0x10` yet — so a missing signal isn't misread as "not recording")
  — so both the GPS link and that a Record press actually started the camera
  can be verified without staging RPM/GPS to drive the FSM. **Wake** presents the
  wake / remote-identity advert so a standby camera wakes and an on camera
  connects back to us; **Record** sends the ce82 shutter toggle; **Power
  Off** streams the ce82 hold — both need the camera connected + subscribed.
  `cameraTestEnterMode()` forces the FSM to IDLE and sets `cameraTestActive`,
  which makes `CAMERA_LOOP()` suppress the FSM step (so auto-record can't
  fight the manual actions) while the Bluefruit callbacks keep the link
  serviced; `cameraTestExitMode()` stops any recording (tracked in a
  bench-local belief so the camera is **guaranteed** left stopped on exit)
  and tears the session down. The `cameraTest*()` action helpers reuse the exact
  `cameraExecuteAction()` code paths the FSM would run — no new FSM states,
  so the board-portable pure unit is untouched.
- **X4 field notes** (from live bench testing, informing the above): the
  camera connects to our ce80 remote (R link) only *after* it has been
  paired from the camera's own **Settings → Bluetooth remote** menu —
  capturing the serial (subsystem UI pairing) is not sufficient. The
  wake-burst advert only wakes an *armed* camera (X4: QuickCapture OFF =
  "Bluetooth Wakeup Enabled"; armed cameras keep the radio scanning even
  powered off, un-armed ones are BLE-dead). There is **no** `be80`
  power-off command in any known reference implementation — power-off is
  the `ce82` 3-second-hold button frame over the R link, so both Record and
  Power Off depend on the R link being up and subscribed.

### 14. SensorEgg Wireless EGT (`sensoregg.ino`, `sensoregg_protocol.{h,cpp}`)

- **BUILD FLAG — `BIRDSEYE_ENABLE_SENSOREGG` (`project.h`)**: this whole
  subsystem is a beta-channel feature. `0` (master/release default)
  compiles `sensoregg.ino` down to no-op `SENSOREGG_SETUP/LOOP` and NaN
  accessors, drops the Temp1 and Temp2 race pages from the rotation
  (`display_pages.ino` + the page-constant block in `BirdsEye.ino`), and
  returns BLE to lazy init. `1` (passed by `beta.yml`, and by
  `compile-sketch.yml` for PRs targeting `BETA`) is everything described
  below. **Plan 0013 closed the three leaks**: the DOVEX
  `Temp1`/`Junction1`/`Temp2` columns, the `temp1_alert_c` setting, and
  the status LED that was hardwired to Temp1 are all now behind the flag
  too — so the log format DOES fork by channel (13 stock columns vs 16),
  reversing the earlier uniform-shape rule, and a stock logger no longer
  shows a permanent solid-blue "no probe signal" pixel all race. The
  `egt` status mode still parses and round-trips on any build; only its
  rendering is gated (`led_status::Inputs.eggSupported`).
  Keep any new egg code behind the flag.
- **What (POC)**: a wireless thermocouple pod (DovesSensorEgg repo) reads a
  K-type EGT probe via MCP9600 and broadcasts EGT + cold junction in BLE
  **advertising packets** — protocol `PW-ADV-1`: 14-byte Manufacturer
  Specific Data (`FF FF` company ID + `50 57` magic *inside* the array,
  version, flags, int16 LE deci-°C ×2 with `0x8000` = invalid sentinel,
  raw MCP9600 STATUS, battery stub, uint16 sequence), ~10 Hz.
- **Radio role — do not "improve" this**: the logger is a pure passive
  OBSERVER (`Bluefruit.Scanner`, `useActiveScan(false)`, 90 ms interval /
  40 ms window ≈ 44% duty, RSSI ≥ −90). No SCAN_REQ, no connection, no
  GATT — so it cannot contend with the camera peripheral link for TX
  airtime; S140 time-slices scan windows around connection events. The
  egg accepts no connections. **The camera link wins every tradeoff** —
  and scan duty is capped (test-enforced ≤45%) because SoftDevice
  scan-window ISRs defer the TIMER3 GPS drain (see subsystem 1).
- **Scanner robustness (bench-proven, do not remove)**: (1)
  `Scanner.filterMSD(0xFFFF)` rejects ambient packets INLINE — Bluefruit
  self-resumes filtered reports, while an accepted report pauses scanning
  until the deferred rx callback runs, so without this filter desk BLE
  traffic collapses the scan duty in bursts. (2) Anti-phase-lock lives in
  the **interval**: equal 100 ms adv/scan periods phase-lock and parked
  the egg in the deaf zone for seconds; the 90 ms scan interval (plus the
  egg advertising off-100 ms) sweeps relative phase ~10 ms/cycle so a
  deaf-zone park escapes in ≤~450 ms. (The original fix was a 60 ms
  window — 60% radio duty, which deferred the GPS drain enough to drop
  PVT frames; the interval retune replaced it and returned the window to
  the spec's 40 ms.) (3) `SENSOREGG_LOOP()` kicks stop+start after 30 s
  with no accepted packet — a lost deferred callback otherwise halts the
  scanner silently forever.
- **Pairing (POC)**: `SENSOREGG_MAC` #define in `sensoregg.h`, human byte
  order; all-zeros (default) = accept any advertiser matching the payload
  magic. The scan callback filters length + magic + MAC, copies the raw 14
  bytes into a double buffer (camera ce81 idiom), stamps `millis()`, and
  calls `Scanner.resume()` — **mandatory**, or the scanner halts after one
  report. No Serial/SD/display in the callback (BLE task context).
- **Consumption**: `SENSOREGG_LOOP()` (main loop) drains + parses via the
  host-tested `sensoregg_protocol` unit. Accessors: `sensoreggEgtC()` /
  `sensoreggJunctionC()` (NaN when stale, egg-invalid, or app-hung),
  `sensoreggLinkUp()`, `sensoreggTcFault()`, `sensoreggAppHung()`.
- **Zombie-egg detection**: BLE radios rebroadcast the last-set advert
  buffer autonomously, so an egg whose *application* hangs (suspected
  blocking MCP9600 I2C read under ignition EMI; 2026-07-19 field incident,
  ~3–4 h in) keeps beaconing a frozen payload at 10 Hz — arrival-time
  freshness alone reports a live link with a flat-lined value. The payload's
  uint16 sequence counter is the sign of life: `sensoregg_protocol::SeqMonitor`
  (host-tested; wrap-safe) marks the reading dead when the sequence hasn't
  changed within `kStalenessMs` even though packets arrive. Readings go NaN
  (log `nan`), and the Temp1 page shows `rf:HUNG` (egg needs a power cycle)
  instead of `rf:OK`. **Staleness (1 s) is absolute** — a reading is never
  held across a dropout (a held value draws a flat line indistinguishable
  from real data). Logging writes `Temp1`/`Junction1` (or `nan`) **in
  Celsius**; the `SENSOR_TEMP` race page (after the tach page) shows big
  EGT + junction + `rf:` link subtext **in Fahrenheit** — converted at
  render time only via `sensoregg_protocol::celsiusToFahrenheit()` (a C/F
  display setting comes later).
- **BLE lifetime change**: `SENSOREGG_SETUP()` (called from `setup()` after
  `CAMERA_SETUP()`) runs `bleCoreEnsureInit()` at boot — BLE is no longer
  lazy. Scanner start failure logs the documented `Bluefruit.begin(1, 1)`
  fallback note (spec §7.2.3) rather than touching the shared `begin(1, 0)`.
- **Sim**: `sensoregg.ino` is excluded from the sim TU like the other BLE
  modules; `module_stubs.cpp` returns NaN/false so the page renders `---`
  and rows log `nan`.

### 15. On-device Course Creator (`course_creator.{h,cpp}`, `track_json.{h,cpp}`)

- **What**: main menu → **Create** → walk the cones and capture the timing
  lines. Sprint venues re-lay their course every event, so the device has
  to be able to author one without a laptop (plan 0002 §5). Serves circuit
  courses too — same lines, one fewer of them.
- **HARD RULE — no text entry on-device, ever.** Names are generated from
  the GPS clock and renamed later in the webapp. A track file and its first
  course are both `N{YYMMDD}_{HHMM}` (12 chars, so the 13-char track
  browser shows it whole); a new track's `shortName` is `MMDDHHMM` —
  exactly the webapp's 8-char budget and half of the `(kind, shortName)`
  key its sync merge uses. Sprint courses also get the sortable
  `date_created` stamp `sprint_select` compares.
- **Five screens**, all driven by `course_creator`'s row table (nothing
  renders a local list, so a row can't display in one order and act in
  another): track prompt (`Here` / `New Track`, skipped when nothing is in
  range) → type picker → line menu → per-line Point A/B → the capture hold.
  Input rides the sketch's existing `menuSelectionIndex`/`menuLimit`
  machinery; `rowCount()` supplies the limit, which changes with course
  type (sprint grows a Finish row). **Every screen ends in a Cancel/Back
  row**, the two entry screens included — the type picker's is the only way
  out of the creator for a user with no track in range, since that entry
  path skips the prompt.
- **Point capture averages, it does not snapshot**: a 3 s hold folds every
  fresh PVT into a mean. Under `kCaptureMinFixes` (8) usable fixes the hold
  **fails** rather than averaging noise into a timing line; fixes worse
  than 10 m h_acc are dropped; fixes after the window are ignored so a mean
  already shown to the user can't shift. Feeding it needed a new monotonic
  **`gpsPvtSequence`** — `gpsDataFresh` is consumed by `GPS_LOOP()` earlier
  in the same iteration, and `gpsData` holds its last value between
  updates, so an un-gated feed averaged one fix 250 times a second.
- **Line edits are scratch-then-commit**: opening a line copies it, `Save
  line` commits, `Back` discards. Back is a real undo.
- **Two save rules exist to keep courses editable in the webapp**, and Save
  is refused (with the reason on the row) until they hold: circuit sectors
  are **all-or-nothing** (the app accepts zero or exactly three majors), and
  sprint splits **fill in order** (the app re-exports them positionally, so
  a lone sector 3 returns as a sector 2). A course the device writes and the
  app then can't save is worse than one never written.
- **Writing** (`sdSaveCreatedCourse`, in `sd_functions.ino` with the other
  track I/O): a new track is one emitted object; an append is a
  read-modify-write through the existing `JSON_BUFFER_SIZE` `trackJson`
  document, capped
  at `MAX_LAYOUTS` and rejected on overflow. Appends serialize to
  `<file>.tmp` and **rename over the original only once closed** — in-place
  rewriting would leave a truncated track file after a power loss in a
  field, on a battery, at an event. `buildTrackList()` re-runs on success so
  the new course is in the manifest for the next session.
- **A full SPRINT track can make room** (plan 0005). A sprint venue re-lays
  its cones every event, so the file fills with runs nobody drives again;
  before this, walking a course onto a full track just lost it. On
  `SD_COURSE_WRITE_TOO_BIG`, `sdPlanSprintPrune()` works out what would have
  to go **without touching the card** (it parses into RAM and discards the
  copy) and `sdSaveCreatedCourse(req, dropOldest)` commits through the same
  temp-file + rename swap. The order comes from the host-tested
  `course_prune` unit: **renamed-in-the-app first** (the device has no text
  entry ever, so a course still called `N260803_1432` may exist nowhere but
  this card, while a renamed one is provably in the app and on cloud sync),
  then oldest by `date_created`. Dropping only renamed courses happens
  **silently**; anything device-named raises `PAGE_COURSE_PRUNE` first.
  **Circuit is deliberately still a dead end** — its layouts are all driven,
  so nothing is safe to drop.
  Three traps, all avoided on purpose: prune **before** the append (ArduinoJson's
  `overflowed()` is sticky, so shrinking back under the limit could never clear
  it — `measureJson()` decides when enough has gone); remove **by name**, since
  every removal shifts the indices the drop order was computed against; and hold
  the generated names across the confirmation, or the course gets renamed to
  whenever the user answered.
- **Entry needs a fix and a time lock** (capture + filename), refused at the
  menu rather than at Save.
- **Sim**: fully exercised — five golden fixtures walk the real menus,
  inject real PVT, run a real averaging hold, and lock the rendered pixels.

### 16. NeoPixel Strip (`neopixel.{h,ino}`, `led_frame/led_modes/led_animations/sector_purple.{h,cpp}`)

- **BUILD FLAG — `BIRDSEYE_ENABLE_NEOPIXEL` (`project.h`)**: `1` on
  **every** channel as of 4.1.0 — master, beta and release — so the strip
  is a core feature rather than a special build. Everything below is in
  every image. The cost is charged fleet-wide and cannot be taken back:
  the first boot after updating performs the ONE-WAY UICR NFC→GPIO
  conversion and self-resets once, on every device, LEDs wired or not.
  `0` (no shipped channel sets it) compiles the subsystem out — no
  Adafruit NeoPixel dependency, and no UICR write on a board that has not
  already been converted; on one that HAS, the `#else` stubs still hold
  boost EN low, because a floating EN leaves the 5 V rail up through
  System OFF.
- **Hardware**: 11 WS2812 pixels fed by an Adafruit 5 V boost converter.
  Pixels 0 and 10 are status indicators; pixels 1–9 are the strip with
  pixel 5 the centerline. Pin 30 (P0.09/NFC1) drives the boost EN
  (HIGH = 5 V rail on), pin 31 (P0.10/NFC2) is the data line — both
  `#define`s in `neopixel.h`, swap to match wiring. **The chain is
  wired data-in on the physical RIGHT** (chain px 0 = rightmost LED):
  everything renders in logical left-to-right space and
  `led_frame::physicalIndex()` mirrors the whole chain — status LEDs
  included — once at push time (`kChainReversed`, true for this build).
- **NFC→GPIO is a one-time runtime UICR write** (`NEOPIXEL_SETUP()`):
  if `UICR->NFCPINS` still has the PROTECT bit, unlock NVMC, program
  `0xFFFFFFFE`, relock, `NVIC_SystemReset()` — NFCPINS latches only at
  reset. **ONE-WAY** (undo = full chip erase / bootloader reflash;
  accepted, NFC is never used). Deliberately NOT the
  `-DCONFIG_NFCT_PINS_AS_GPIOS` core flag: that's consumed by the
  core's `system_nrf52840.c` (a `.c` file `compiler.cpp.extra_flags`
  can't reach) and would silently not apply to IDE builds. Direct NVMC
  access is illegal under the SoftDevice, so `NEOPIXEL_SETUP()` runs
  **before `CAMERA_SETUP()`/`SENSOREGG_SETUP()`** (which
  `bleCoreEnsureInit()` on beta) and before `wdtSetup()` arms the
  watchdog. Every later boot skips the branch.
- **The global brightness cap is THE invariant**: modes and animations
  author colors in full 0–255; `led_frame::applyCap()` scales every
  channel by cap/255 exactly once, at push time, in `npxPushFrame()`.
  After it, no channel exceeds the cap — host-tested as a
  post-condition. Never use `strip.setBrightness()` (lossy buffer
  rewrite, spreads the invariant). **Which** cap is
  `npxEffectiveBrightness()`: `led_brightness` by day,
  `led_brightness_night` inside the local night window (subsystem 17).
  `led_brightness` 0 = LEDs disabled: the boost rail is never even
  enabled — **a night cap of 0 is NOT the same thing** (blank frames,
  rail stays up; power-cycling the boost converter at 19:00 mid-session
  is not a brightness change).
- **Frame loop** (`NEOPIXEL_LOOP()`, self-throttled 30 Hz, called after
  `CAMERA_LOOP()` AND inside both parked branches so the strip blanks
  rather than freezes): snapshot inputs → step the `sector_purple`
  monitor → compose by priority — **boot animation > overrev
  whole-chain red flash > purple animation > (parked ‖ !raceActive ‖
  brightness 0 → off) > race rendering** — → `applyCap` → show.
- **Strip policy** (plan 0007 order + the 0013 speed arm): the 9-px bar
  is off outside a race session (driving aid, not menu bling); the two
  status LEDs follow their own rule below. In race, first match wins:
  1. **Engine stopped** (`raceEngineStopped()`: tach-proven session —
     `raceEntryCause == RACE_ENTRY_TACH`, which manual/speed promote
     to — at 0 RPM) → bar OFF; status LEDs stay live (a hot engine
     cooling after a stall is when the temp alert matters). The OLED
     pace page shows `STOPPED` on the same condition. Self-clears on
     restart; no-tach devices can never trip it.
  2. **No GPS lock** (`!(gpsData.fix && gpsData.timeValid)` — the
     log-file-creation gate) → green **search pip** bouncing end-to-end
     (`renderSearchPip`, 1.6 s round-trip triangle wave).
  3. **Pace valid** (`activeTimerRaceStarted() && laps >= 1 && !(sprint
     && between-runs)`, mirroring the OLED pace page) → the **pace
     pip**: `activeTimerPaceDifference()` is **ms per meter**, positive
     = slower; full deflection ±1.0 ms/m (`kPaceFullScaleMsPerM`,
     0.25/pixel), ±0.125 deadband = dim-white centerline only. Slower =
     LEFT of center in red, faster = RIGHT in green.
  4. **Tach session** (`raceEntryCause == RACE_ENTRY_TACH`) → **RPM
     scale** (green filling left→right, red past halfway, ceiling =
     `target_rpm`).
  5. Else → **SPEED scale** (plan 0013), ceiling = `target_speed_mph`,
     **green all the way up** (`kSpeedRedFrac = 1.0`). Without a tach
     the RPM scale was nine dark pixels until the first lap landed.
     `RACE_ENTRY_TACH` is the gate rather than a live `rpm > 0` test
     because manual/speed sessions promote to it at >500 rpm
     (`idle_policy::tachProven`), so it means "has never seen an
     engine" and cannot flicker between two scales at every stall. The
     RPM bar's red band means "approaching the limiter"; there is no
     equivalent hazard in reaching a target speed, so painting it red
     would invert the meaning of the same nine pixels.
- **Status LEDs are USER-ASSIGNED** (plan 0013, `led_status`): pixel 0
  from `led_status_left`, pixel 10 from `led_status_right`, each one of
  eight modes — `off` / `rpm` (red flash ≥ `target_rpm`, clears at 97%,
  100 ms) / `speed` (red flash ≥ `target_speed_mph`, clears 2 mph
  below, dark with no fix) / `gps` (steady red no sats · yellow sats but
  no fix · blue fix but no time lock · green locked) / `camera` (dark
  unpaired · yellow session up but camera not · **flashing** blue linked
  yet not ce82-subscribed, i.e. we cannot command it · steady blue ready
  · red recording) / `lap` / `sector` (steady green–red–purple verdicts,
  below) / `egt` (Temp1 tri-state: red flash ≥ `temp1_alert_c`, clears
  20 °C below, OFF when good, **solid blue** when the probe signal is
  NaN/stale — a dropout is information).
  - `rpm`/`speed`/`egt` build a `StatusAction` on the stack and delegate
    to `led_modes::evalStatus` — hysteresis, latch-release-on-invalid
    and flash phase have exactly one implementation, with the
    regression tests they already had. **Never re-implement them.**
  - A NaN/stale source (checked with `isNanF()`, NEVER `isnan()` —
    `-Ofast`) always releases the latch. So does leaving race mode, and
    so does a pixel whose mode is not currently being rendered —
    a latch left set would flash the instant the next session starts.
  - **`egt` is dark on a build without `BIRDSEYE_ENABLE_SENSOREGG`**
    (`led_status::Inputs.eggSupported`), because there is no probe there
    to lose. Before plan 0013 pixel 10 was hardwired to this mode, so a
    stock logger showed the blue no-signal state for the whole of every
    race. The setting still parses and round-trips either way — only the
    rendering is gated.
  - **`gps` and `camera` stay lit on the main menu**
    (`led_status::activeOutsideRace()`); every other mode, and the bar,
    is race-only.
  - **Overrev** is not a status mode — it reuses `StatusAction` as a
    whole-chain action: ≥ `overrev_limit` (setting, 0 = disabled)
    flashes ALL 11 px red at 100 ms until RPM falls below
    `target_rpm × 0.97`, the engine-is-broken signal, ranked above the
    purple celebration.
- **Lap/sector close-edge monitor** (`sector_purple`, host-tested — the
  filename predates half its job): the library updates best-sector times
  **at the start/finish crossing**, not at sector lines, so the monitor
  snapshots each sector's best when the sector OPENS, closes S1/S2 on
  `getCurrentSector()` transitions, and derives S3 on the lap edge as
  `lastLapTime − s1 − s2` — immune to the lap-line race. `bestLapAtOpen`
  is the same trick one level up, because `getBestLapTime()` is folded at
  the crossing too. Sprint-first wrappers feed it:
  `activeTimerCurrentSector()`, `activeTimerLapSectorTime(n)`,
  `activeTimerBestSectorTime(n)` (WaypointLapTimer → 0, sector half
  stays idle) plus `activeTimerBestLapTime()`.
  - **Two outputs.** Purple flags → `neopixelNotifyPurpleSector()` /
    `neopixelNotifyPurpleLap()`. And a **verdict** per close edge, held
    by the glue until the next one, driving the `lap` / `sector` status
    modes: green faster than the LAST recorded one, red slower, purple a
    session best, **dark when there is nothing to compare against**.
  - **Verdicts compare the last recorded time, NOT the best** — against
    the best you only ever get purple or red, which says nothing about
    whether you are improving. Purple still requires beating a nonzero
    prior best, so lap 1 is dark everywhere.
  - **Lap tracking does not need sector lines.** Only the sector half is
    gated on `sectorsConfigured`; a Lap Anything session still reports
    lap verdicts. Equal times read as "slower" (strict improvement), and
    a sector that never closed emits no event at all rather than a bogus
    verdict.
- **Animations** (`led_animations`): pure functions of `(tMs, seed)` —
  sparkles hash `(seed, timeSlot, pixel)`, no `rand()`/`millis()`
  inside, so frames are golden-testable. Boot: 2.6 s hue comet circling
  the 11 px + fade-out + white sparkles, plays over the GPS status
  page. Seeds come from `micros()` (house entropy rule).
- **Sleep/wake**: `NEOPIXEL_SLEEP()` (in `enterShutdown()` with the IMU
  rail-off, before the charging branch — so the strip is dark while
  charging too) blanks the strip while 5 V is up, then data LOW, then
  boost EN LOW — driven-LOW is retained through System OFF (the "blue
  LED stays on" precedent). `NEOPIXEL_WAKE()` (charging soft-resume)
  re-raises EN, waits the 5 ms settle, re-inits the strip.
- **Radio/timing safety**: Adafruit_NeoPixel's nRF52 `show()` grabs a
  FREE PWM instance (EasyDMA, interrupts ON, ~0.4 ms for 11 px ≈ 1%
  CPU at 30 Hz). This sketch uses no `tone()`/`analogWrite()`, so
  PWM0–2 are always free; TIMER3's GPS drain and the tach ISR are
  unaffected. If all PWMs were ever occupied the library bit-bangs
  **with interrupts off** — never create that path. `show()` also
  mallocs/frees a ~560 B pattern buffer per call: same-size alloc/free
  is fragmentation-benign, it is NOT a leak.
- **Sim**: `neopixel.ino` excluded from the sim TU (BLE-module
  precedent), surface no-op'd in `module_stubs.cpp`; the four pure
  units build into the sim via `SIM_CORE_SOURCES`.
- **Still not built**: per-mode threshold/colour customisation (today a
  mode's colours and hysteresis are the unit's, only the assignment is
  the user's), temp scales through the same `ScaleSpec`, a C/F display
  preference, and a strip-mode (bar) selector to match the status-LED
  one.

### 17. Local Time (`local_time.{h,cpp}`)

- **What**: UTC plus a fixed signed minute offset (`utc_offset_min`),
  giving the device a local wall clock. Its ONE consumer today is the
  NeoPixel day/night brightness swap, which needs "7am" to mean the
  driver's 7am (plan 0010).
- **NOTHING LOGGED GOES THROUGH THIS.** DOVEX row timestamps are Unix
  epoch ms (UTC by definition), and the header `datetime`, the log
  filenames and the generated course names are all still UTC. That is
  deliberate, not an oversight: a log is routinely *viewed* somewhere
  other than where it was recorded, so the conversion belongs to the
  viewing app, which knows the reader's preference. Do not wire
  `local_time` into the logging pipeline.
- **No DST, on purpose.** A fixed offset walks the boundary an hour
  twice a year, which is beneath the resolution of a dim-after-dark
  gate. Rule tables are a standing correctness liability (legislatures
  keep moving the dates) and tzdata is ~100 KB shipped to a sealed
  device. `local_time` is where rules would go if that changes — which
  is why `DateTime` carries a **4-digit year** (the sketch's
  `gpsData.year` is 2-digit; callers add 2000) and why the leap rule is
  `gps_time::isLeapYear` reused rather than re-derived.
- **Minutes, not hours** — India +330, Newfoundland −210, Chatham
  +765. An out-of-band offset (beyond ±840) is **ignored, not clamped**:
  a corrupt setting must not be able to walk the calendar, so it falls
  back to 0 = UTC = the pre-0010 behaviour.
- **`isNight(m, dayStart, nightStart)`** tests the window
  `[nightStart, dayStart)` **modulo the day**, so the ordinary wrapped
  case (19:00 → 07:00) needs no special casing at the call site. Equal
  bounds = empty window = never night, which is how the swap is disabled
  without a separate enable flag.
- **No clock means DAY.** `gpsData.timeValid` needs the module's
  `fullyResolved` (~12.5 min worst case from a cold start);
  `npxEffectiveBrightness()` renders at the day cap until it lands.
  Guessing night would bring the strip up dark and read as dead hardware.

### 18. Loop CPU Profiling (`profiling.{h,ino}`, `loop_profile.{h,cpp}`)

- **BUILD FLAG — `BIRDSEYE_ENABLE_PROFILING` (`project.h`)**: `0` in
  master/release (the whole subsystem vanishes — the section brackets
  are macros that expand to the bare call, so the loop dispatch is
  byte-identical to what it always was), `1` on the beta channel
  (`beta.yml`, and `compile-sketch.yml` for PRs targeting `BETA`).
  Keep any new profiling code behind the flag.
- **What it is for.** Two board questions gate the commercial design —
  nRF52840 or nRF5340 (a dedicated network core would take the BLE
  stack off this CPU), and whether leaving the Arduino core for the
  Nordic SDK is worth the one-way effort. Both are answerable by
  measurement, and nothing in the firmware measured anything finer than
  a `HAS_DEBUG` millisecond-resolution "SLOW LOOP" print. Plan 0011.
- **Idle is MEASURED, not assumed.** The first version of this
  subsystem asserted that `loop()` runs back to back so the duty cycle
  is 100% by construction and a duty cycle should never be added. That
  assumption got baked into the measurement and produced a wrong first
  reading (see the two-clocks note below). The rollup now computes
  `busyPermille` — wall time actually spent executing `loop()` — and
  reports the balance as the `SLP` slot: scheduler dispatch, other
  FreeRTOS tasks, and CPU sleep. Alongside it, what always carried the
  information: the SHAPE of an iteration — mean and **worst-case**
  length (an SD garbage-collection stall of 100 ms–2 s never shows in a
  mean), the per-subsystem split, and how much no subsystem accounts
  for.
- **Two instruments, and the second exists to check the first**:
  - **The pin.** Pin 30 goes HIGH for the span being profiled and LOW
    outside it — loop period off the rising edges, span cost off the
    high time, no software in the measurement path.
    `PROFILING_PIN_SECTION` picks the span: whole loop body by default,
    or `-DPROFILING_PIN_SECTION=PROF_SEC_GPS` (and friends — a
    preprocessor mirror of `loop_profile::Section`, `static_assert`ed
    against it, because `#if` cannot evaluate a scoped C++ name).
  - **The rollup.** The same sections timed in software and rolled up
    once a second onto the `GPS_PROFILE` page.
- **THE PIN COSTS THE 5 V RAIL — this is the whole trade.** Pin 30 is
  the NeoPixel boost converter's EN line and cannot be both. The
  profiler takes it, so a profiling build never drives EN (setup, sleep
  and the charging-loop resume all no-op via `npxBoost*()` in
  `neopixel.ino`) and the regulator sits at its hardware default
  (EN pulled up = rail on). That works because the rail does not need
  firmware control, only switchability — and switching it is a
  requirement of **use**, not of **testing**. Two accepted consequences:
  the rail stays up through System OFF (levels are retained there and
  the driven LOW was the only thing holding it down — the "blue conn LED
  stays on after sleep" precedent), so a beta unit asleep on a battery
  with a strip wired drains it; and if the EN jumper is still physically
  connected on the rig the toggling chops the rail at loop rate — pull
  it or tie EN high.
- **Timebase: DWT, verified, with a self-announcing fallback.** Most
  sections are well under a microsecond, so `micros()` alone quantises
  half of them to zero. `profEnableDwt()` turns on the Cortex-M4 cycle
  counter (64 ticks/µs at 64 MHz) and then **reads it across a spin to
  prove it moved** — a debug probe can hold TRCENA off and CYCCNTENA is
  architecturally optional, so setting the bit is not evidence. On
  fallback the page prefixes its first row with `*`.
- **TWO CLOCKS — do not collapse them back into one.** DWT counts CPU
  CYCLES, so it stops dead when the core halts (WFE/WFI in the FreeRTOS
  idle task, `sd_app_evt_wait`). It measures DURATIONS; it is not a
  clock. The rollup window is closed on `millis()` and every share is
  computed against that wall time. Using DWT to close the window was the
  first real bug here: a "one second" window was one second of
  CPU-awake time, so the loop rate came out multiplied by the sleep
  factor (the first hardware run pegged the display clamp) and every
  share was a fraction of awake time wearing a wall-time label. The
  `micros()` fallback never had the bug — it is a real clock. `Report`
  carries `awakeUs` next to `windowUs` so the discrepancy stays visible.
- **The pure unit works in TICKS**, taking `ticksPerUs` only at rollup,
  so shares come out of raw ticks and lose nothing. Accumulators
  **saturate, never wrap**: a uint32 of DWT ticks is only ~67 s and a
  rollup can be arbitrarily late behind a stall — a pegged window reads
  as pegged, a wrapped one reads as near-idle, which is a lie.
- **`OTH` is reported, not hidden**: loop time no section bracketed. It
  is the honesty check on the instrumentation — if it is large, work is
  happening where `loop()` is not looking. **`SLP`** is its counterpart:
  wall time not inside `loop()` at all. All 14 slots are shares of the
  same wall-clock second, so they sum to ~1000‰ — an invariant a reader
  can check on the page at a glance.
- **A scope guard, not a `*_LOOP()`.** Both parked branches (`bleActive`,
  `usbMscActive`) return early and `enterShutdown()` never returns, so
  the whole-iteration timing AND the rollup live in `~ProfLoopScope()`
  (`PROFILE_LOOP_SCOPE()` at the top of `loop()`). `PROFILING_SLEEP()`
  parks the pin LOW on the shutdown path the destructor never reaches.
  This module deliberately has no `PROFILING_LOOP()`.
- **Overhead is inside the numbers, on purpose — and it is NOT small.**
  A bracket is two counter reads plus a saturating add (~30 cycles,
  ~0.5 µs; ~6–8 µs per iteration across all 13). That was written off
  as "under 0.1%" against the ~4 ms iteration this file assumed for
  years; the first hardware run measured a mean iteration **under
  100 µs**, which puts the instrument at order 10% of what it reports
  and means the un-instrumented loop is faster than the rate shown. A
  bracket still lands in the section it brackets rather than in `OTH`,
  because for a subsystem's SHARE that is the honest accounting — but a
  section reading under ~1% is at its own bracket's noise floor, so do
  not rank those against each other. The pin edges
  are `#if`'d rather than compared at runtime: with the default
  whole-loop setting the comparison is provably false for every section,
  and a dead branch in the two hottest functions in the firmware is
  exactly the cost a profiler must not add.
- **The page** (`GPS_PROFILE` = 2, first of the race rotation on a
  profiling build; the session still LANDS on the speed page, so it is
  three Lefts away). Eight rows, no title: a stats line (`14481Hz 61us
  mx42` — mean in µs below 1 ms and ms above, worst iteration always ms;
  `*` = degraded timebase, `!` = pin refused) then seven rows of two
  slots covering all 12 sections plus `OTH` and `SLP`. The stats row's
  clamps are reciprocal-aware rather than fixed: the first hardware run
  came back `999Hz av0.0` because both fields had been sized from the
  stale ~250 Hz assumption, and a clamp that hides the finding is worse
  than no clamp. **Adding a section means finding it a row** — the grid
  is exactly full, and `SLP`'s slot came from folding the boot-page
  state machines (`PGE`) into `DSP`. Because the rotation is
  a contiguous range and this page sits below `GPS_DEBUG`, a profiling
  build effectively forces `debug_pages=show`.
- **Sim**: `profiling.ino` is excluded from the sim TU (BLE-module
  precedent) and stubbed in `module_stubs.cpp` — there is no pin and
  host timings of a virtual-clock loop would mean nothing. The pure unit
  still builds in via `SIM_CORE_SOURCES`.

---

## Data Formats

### DOVEX Log (`.dovex` files) — New UI default

```
datetime,driver,course,short_name,best_lap_ms,optimal_ms,device_name,race_mode
lap1_ms,lap2_ms,lap3_ms,...
\n padding to byte 1024
timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm,accel_x,accel_y,accel_z,Temp1,Junction1,Temp2
1710512400123,12,0.8,35.12345678,-97.12345678,65.32,234.56,...
```

- **Reserved header** (bytes 0–1023): Line 1 = session metadata, Line 2 =
  all lap times (comma-separated ms values), padded with `\n` to 1024 bytes.
- **`device_name`** and **`race_mode`** are trailing metadata columns
  (after `optimal_ms`, in that order). Appending keeps old logs
  readable (parsed as empty) and lets older readers ignore the extra
  columns — backwards compatible by design. `race_mode` is `CIRCUIT` /
  `SPRINT` (empty = circuit): a webapp loading helper — with `SPRINT`,
  the laps line is a runs line. Nothing on-device reads it back.
- **GPS data** (byte 1024+): CSV column header then streaming GPS rows.
- **`Temp1` / `Junction1` / `Temp2`** (trailing columns): SensorEgg EGT +
  cold junction + v2 aux intake-air temp, all °C. Literal `nan` when the
  egg link is stale (>1 s), the egg reports an invalid probe/divider, or
  (for `Temp2`) the egg is v1 — a dropout must be a visible gap, never a
  held value. These fields never cause a GPS row to be skipped.
  **THE THREE COLUMNS EXIST ONLY ON A `BIRDSEYE_ENABLE_SENSOREGG` BUILD**
  (plan 0013), so a stock log has 13 data columns and a beta log 16.
  Until 0013 they were written on every channel purely so the shape never
  forked; three dead `nan` columns on every row of every stock log paid
  for nothing. Readers must key off the CSV header line (all three
  optional there) rather than assuming a column count — the companion
  app's `doveParser.ts` already builds a name→index map, which is why
  this cost no client change.
- **Crash safety**: file created with pre-filled newlines to 1024 bytes
  before any data. Header written on session end. If header is empty
  (crash), GPS data after 1024 is still valid.
- **Filename**: `20YYMMDD_HHMM.dovex`
- **Everything here is UTC and stays that way.** The row `timestamp` is
  Unix epoch ms, the header `datetime` is the UTC wall clock, and the
  filename is stamped from the same UTC fields. The device's
  `utc_offset_min` setting (subsystem 17) is presentation-only and must
  never be applied on this path — timezone display is the viewing app's
  job, since a log is often read in a different zone than it was
  recorded in.
- 1 KB handles ~100 laps (8 chars per lap time). Extremely unlikely to exceed.

### Track JSON (`/TRACKS/*.json`)

**New format** (LapWingData / web simulator):
```json
{
  "longName": "Orlando Kart Center",
  "shortName": "OKC",
  "defaultCourse": "Normal",
  "courses": [
    {
      "name": "Normal",
      "lengthFt": 3383,
      "start_a_lat": 28.4127081705638,
      ...
    }
  ]
}
```

**Older format** (bare array, still parsed):
```json
[
  {
    "name": "Full Course",
    "start_a_lat": 28.41270817,
    ...
  }
]
```

Auto-detected by JSON root type (object vs array). The older bare-array
form sets `lengthFt = 0` for all courses, which means CourseDetector
cannot rank by distance — CourseManager falls back to Lap Anything
immediately.

Stored in `trackLayouts[MAX_LAYOUTS]` (max 10 per track).

**Sprint track JSON** (`/TRACKS/SPRINT/*.json`) uses the same object
format plus `"type": "sprint"` (track level — redundant with the folder,
which is authoritative) and per-course `finish_a/b_lat/lng` (required for
timing; a course without a finish line can't run) and `date_created` (a
sortable ISO-8601 stamp, `YYYY-MM-DDTHH:MM`; the newest course is always
the one loaded). Sector lines stay optional — zero, one, or two.

### Settings JSON (`/SETTINGS.json`)

```json
{
  "bluetooth_name": "DovesDataLogger-042",
  "bluetooth_pin": "7391",
  "camera_serial": "",
  "device_name": "ApexTurbo",
  "race_mode": "circuit",
  "display_invert": "normal",
  "debug_pages": "hide",
  "spark_mode": "wasted",
  "cylinder_count": "1",
  "tach_filter": "smooth",
  "driver_name": "Driver",
  "lap_detection_distance": "7",
  "waypoint_detection_distance": "30",
  "waypoint_speed": "30",
  "led_brightness": "64",
  "target_rpm": "15000",
  "overrev_limit": "0",
  "target_speed_mph": "60",
  "led_status_left": "rpm",
  "led_status_right": "egt",
  "temp1_alert_c": "650",
  "utc_offset_min": "0",
  "led_brightness_night": "16",
  "led_day_start_hour": "7",
  "led_night_start_hour": "19"
}
```

| Key | Type | Default | Purpose |
|-----|------|---------|---------|
| `bluetooth_name` | string | Random | BLE device name |
| `bluetooth_pin` | string | Random 4-digit | BLE pairing PIN |
| `camera_serial` | string | `""` (empty = unpaired) | Paired Insta360 X4's 6-char serial (auto-captured on pairing, or entered manually) |
| `device_name` | string | Random racing words | Identifies the logging device (DOVEX header) |
| `driver_name` | string | `"Driver"` | Logged in DOVEX header |
| `race_mode` | string | `"circuit"` | Tiebreak pref when BOTH a circuit and a sprint track are in range: `circuit` yields only to a sprint course created today; `sprint` always prefers the sprint track. Never overrides single-kind detection |
| `lap_detection_distance` | int | `7` | DovesLapTimer crossing threshold (meters) |
| `waypoint_detection_distance` | int | `30` | WaypointLapTimer proximity zone (meters) |
| `waypoint_speed` | int | `30` | Speed threshold (mph) for waypoint/detection |
| `spark_mode` | string | `"wasted"` | Ignition rate: `wasted` = 1 spark/rev (2T, or 4T wasted spark); `single` = 1 spark per 2 revs (4T single-fire). Anything other than an explicit `single` is treated as `wasted` |
| `display_invert` | string | `"normal"` | Panel colours: `normal` = lit-on-black as shipped, `inverted` = black-on-lit. Anything other than an explicit `inverted` means normal |
| `debug_pages` | string | `"hide"` | Race-rotation diagnostic pages (`GPS_DEBUG` + `GPS_STATS`): `hide` = rotation starts at the speed page (end-user default), `show` = diagnostics restored at the front. Anything other than an explicit `show` means hide. Also swaps the tachometer page's subtext line for the tach filter diagnostic (`max:NNNNN S rj:NN`, plan 0009). No-op on the rotation under `ENDURANCE_MODE` (already starts at speed) |
| `cylinder_count` | int | `1` | The engine's **actual** cylinder count. **Does not scale RPM** (plan 0014): one clamp on one plug wire sees one cylinder, so `spark_mode` alone sets the geometry. Above 1 it means crank speed is *inferred* from one cylinder's ignition pulses — standard clamp-on-tach behaviour, warned about in the settings UI. Clamp 1–16 |
| `tach_filter` | string | `"smooth"` | RPM estimator (plan 0009). `smooth` = outlier gate + RPM-aware noise models; `legacy` = the pre-0009 filter bit for bit, for A/B against older logs; `raw` = no estimator at all, the `rpm` column is exactly what the pickup delivers. Anything else means `smooth`. Diagnostic knob — the intent is one session each at the track, not a permanent tuning dial |
| `led_brightness` | int | `64` | NeoPixel global brightness cap 0–255 — no LED channel ever exceeds it (`led_frame::applyCap`). `0` disables the LEDs entirely (boost rail never enabled). Written AND parsed on every channel (`ensureDefaultSettings()` + the boot block in `BirdsEye.ino`); only its *use* is compiled out with the flag. A non-numeric value keeps the 64 default — via `setting_parse::parseIntSetting`, never `atoi()`, because `atoi("")` is 0 and 0 here means "LEDs off" |
| `target_rpm` | int | `15000` | True RPM SHIFT/warning point: RPM-scale ceiling and the `rpm` status-LED flasher threshold. Clamp 1000–20000 (tach filter's ceiling). Was `rev_limit` before plan 0013 — a device carrying the old key has its value migrated into this one on first boot and the old key removed |
| `overrev_limit` | int | `0` (disabled) | True RPM PROBLEM limit (plan 0007): past it the whole 11-px chain flashes red (outranks the purple celebration) and the tach page shows `*OVER REV*`; latch clears below `target_rpm × 0.97`. 0 = off (no chain flash, no header); else clamp 1000–20000 |
| `temp1_alert_c` | int | `650` | **SensorEgg builds only** since plan 0013 — a stock image neither writes nor reads it. Temp1 (EGT) alert threshold in **Celsius** for the `egt` status mode: red flash at/above, clears 20 °C below, solid blue when the probe signal is NaN/stale. Clamp 50–1200 |
| `utc_offset_min` | int | `0` | Minutes east of UTC (US Central standard `-360`, India `330`, Newfoundland `-210`). Clamp ±840; out of band keeps 0 (= UTC). **Presentation only** — nothing logged is converted (subsystem 17) |
| `led_brightness_night` | int | `16` | NeoPixel cap 0–255 used inside the night window. `0` blanks the strip but leaves the 5 V rail UP — only `led_brightness` 0 cuts the rail |
| `led_day_start_hour` | int | `7` | **Local** hour the day cap takes over. Clamp 0–23 |
| `led_night_start_hour` | int | `19` | **Local** hour the night cap takes over. Clamp 0–23. Equal to `led_day_start_hour` = swap disabled (one cap around the clock) |
| `target_speed_mph` | int | `60` | Ceiling of the LED speed bar on a session with no tachometer, and the `speed` status mode's threshold. **Stored in mph**; the companion app converts for display. Clamp 5–250 — the floor is 5, not 0, because a zero ceiling trips `renderScale`'s span guard and blanks the bar, which is indistinguishable from dead hardware |
| `led_status_left` | string | `"rpm"` | What the LEFT status pixel shows: `off`/`rpm`/`speed`/`gps`/`camera`/`lap`/`sector`/`egt` (subsystem 16). **Strictly** parsed — anything unrecognised keeps the compiled-in default rather than darkening the LED or picking another mode |
| `led_status_right` | string | `"egt"` (SensorEgg) / `"lap"` (stock) | Same, for the RIGHT status pixel. `egt` renders dark on a build without SensorEgg support, but still parses and still round-trips over `SGET`/`SLIST` |

- Created automatically on first boot with random BLE values.
- Missing keys auto-populated on boot via `ensureDefaultSettings()`.
- **Corrupt-file self-heal**: a non-empty file that fails to parse is
  quarantined to `/SETTINGS.json.bad` (kept for inspection, previous `.bad`
  overwritten) and a fresh default file is generated — checked at
  `SETTINGS_SETUP()` and again on any `setSetting()` that hits a parse
  error (single retry against the regenerated file). An *empty* file is
  not corrupt — the default-population paths rebuild it in place.
- Editable on a computer or via BLE `SSET` command — changes take effect
  on next reboot (BLE disconnect triggers auto-reboot).
- Read on-demand via `getSetting()`, written via `setSetting()`.

---

## Key Constants

| Constant | Value | Location |
|---|---|---|
| GPS baud | 57 600 | `gps_config.h` |
| GPS nav rate (race) | 25 Hz | `gps_config.h` |
| GPS nav rate (boot/status page) | 5 Hz + NAV-SAT ~1 Hz | `gps_config.h` |
| Status page auto-close | 3 s after fix+timeValid | `gps_status_page.h` |
| UTC resolve worst case | ~12.5 min cold start (nav-msg subframe 4 page 18) | `gps_status_page.h` |
| Status page idle shutdown | 5 min (no lock, no engine) | `gps_status_page.h` |
| SD format confirm hold | 3 s continuous Select | `sd_format_page.h` |
| SD format page idle shutdown | 5 min | `sd_format_page.h` |
| GPS boot re-detect | 3 tries, 10 s apart | `gps_functions.ino` |
| Menu idle shutdown | 5 min (`SLEEP_IDLE_TIMEOUT_MS`) | `project.h` |
| USB-on-menu charge idle | 60 s (`USB_MENU_CHARGE_IDLE_MS`) — compiled out unless `BIRDSEYE_ENABLE_ONBOARD_CHARGING` | `project.h` |
| Onboard charging (HICHG hold + USB charge UX) | `BIRDSEYE_ENABLE_ONBOARD_CHARGING`, default 0 (all channels) | `project.h` |
| SensorEgg wireless EGT POC | `BIRDSEYE_ENABLE_SENSOREGG`, default 0; 1 on the beta channel | `project.h` |
| Charging screen timeout | 10 s (`CHARGE_DISPLAY_TIMEOUT_MS`) | `project.h` |
| Sat bars display cap / CNO ceiling | 16 bars / 50 dB-Hz | `sat_bars.h` |
| Crossing threshold | 7.0 m | `BirdsEye.ino` |
| Max laps/session | 1 000 | `BirdsEye.ino` |
| Max locations | 200 | `project.h` |
| Max layouts/track | 10 | `project.h` |
| Max replay files | 20 | `replay.ino` |
| DOVEX header size | 1 024 bytes | `project.h` |
| Auto-idle timeout (tach sessions) | 60 s at <2 mph | `BirdsEye.ino` |
| Auto-idle timeout (manual/speed sessions) | 5 min at <5 mph → ends data + camera | `BirdsEye.ino` |
| Auto-race menu grace | 3 s settled (arrival + buttons) before auto-race can fire | `project.h` |
| Track detect radius | 5 miles | `BirdsEye.ino` |
| Course creator point hold | 3 s, ≥8 usable fixes else FAILED | `course_creator.h` |
| Course creator h_acc gate | drop >10 m, warn >5 m | `course_creator.h` |
| Course creator name format | `N{YYMMDD}_{HHMM}` (+ `MMDDHHMM` short name) | `course_creator.h` |
| Sprint prune order | renamed-in-app first, then oldest `date_created`; confirm only when a device-named course would go | `course_prune.h` |
| Track JSON coordinate precision | 8 decimals (~1.1 mm) | `track_json.h` |
| Tach min pulse gap | 3 ms (`wasted`) / 6 ms (`single`) — same ~20 000 RPM ceiling either way | `tach_filter.h` (`minPulseGapUs`) |
| Tach revs per pulse | `wasted` ? 1.0 : 2.0 — **no cylinder term** (plan 0014) | `tach_filter.h` (`revsPerPulse`) |
| Tach ring buffer | 16 entries | `BirdsEye.ino` |
| Tach Kalman Q (legacy mode) | 800 RPM² per update | `tach_filter.h` |
| Tach Kalman process noise (smooth) | 80 000 RPM²/s × engine time, clamped 0.5 s | `tach_filter.h` |
| Tach Kalman R_BASE | 2500 RPM² floor | `tach_filter.h` |
| Tach measurement noise (smooth) | R_BASE + (80 µs × RPM²/K + 1% × RPM)² | `tach_filter.h` |
| Tach outlier gate | 5 sigma; 3 consecutive rejects → adopt; armed after 4 updates | `tach_filter.h` |
| Tach filter mode | `tach_filter` setting: `smooth` (default) / `legacy` / `raw` | `tach_filter.h` |
| Track manifest scan throttle | 1 Hz | `BirdsEye.ino` |
| Tach stop timeout | 500 ms | `BirdsEye.ino` |
| Display refresh | 3 Hz | `display_ui.ino` |
| Button debounce | 200 ms | `display_ui.ino` |
| SD SPI clock (normal) | 2 MHz | `BirdsEye.ino` |
| SD SPI clock (transfer) | 8 MHz (`SD_SPI_SPEED_FAST`) | `BirdsEye.ino` |
| Battery check interval | 5 s | `BirdsEye.ino` |
| BLE default MTU | 23 | `bluetooth.ino` |
| BLE target conn interval (adaptive 2nd ask) | 12 units = 15 ms, Apple's floor | `bluetooth.ino` |
| BLE link-layer PDU "extended" floor | 100 B (27 = DLE never happened) | `bluetooth.ino` |
| BLE transfer read-ahead | 4096 B (`kReadAheadSize`) | `ble_stream.h` |
| BLE burst budget | 20 ms wall clock (`kBurstBudgetMs`) | `ble_stream.h` |
| BLE max notify payload | 244 B (`kMaxNotifyLen`) | `ble_stream.h` |
| JSON buffer (`JSON_BUFFER_SIZE`) | 8192 (SIM builds too) — read buffer + `trackJson` doc | `BirdsEye.ino` |
| Settings file buffer / JSON doc | 1024 each (keep equal; see subsystem 8) | `settings.ino` |
| Settings file path | `/SETTINGS.json` | `settings.ino` |
| Track upload buffer | `JSON_BUFFER_SIZE` (8192) | `bluetooth.ino` |
| GPS serial buffer | 4096 | `gps_functions.ino` |
| GPS serial timer | TIMER3, 5 ms (`GPS_DRAIN_INTERVAL_US`) | `gps_config.h` |
| Core Serial1 RX/TX rings | 256 B via required `-DSERIAL_BUFFER_SIZE=256` (asserted) | `project.h` + workflows |
| GPS drop-count slack / credit cap | 1 frame / 2 frames | `gps_stats.h` |
| OTA staging path | `/fw/pending.bin` | `firmware_ota.ino` |
| OTA receive buffer | 2 × 4096 (double-buffer) | `firmware_ota.ino` |
| OTA app base | `0x27000` | `firmware_ota.ino` |
| OTA staging flash base | `0x8E000` | `firmware_ota.ino` |
| OTA max image size | 408 KiB (half the 820 KiB app+staging span, page-aligned; `static_assert`ed) | `firmware_ota.ino` |
| OTA min apply voltage | 3.6 V | `firmware_ota.ino` |
| Camera record-start gate | RPM ≥ 1500 (`kRecordRpmThreshold`) held 5 s, strict — dips restart the clock (no GPS gate) | `camera_fsm.h` |
| Camera stop-record delay | 30 s engine-off (RPM only) → also ends log session; suppressed while `sessionDemand` (manual/speed sessions end via the 5 min idle timer) | `camera_fsm.h` |
| Camera power-off | shutdown only (no post-record cooldown/timeout) | `camera_ble.ino` (`CAMERA_SLEEP`) |
| Camera RPM on/off thresholds | 500 / 300 (2 s on-debounce) | `camera_fsm.h` |
| Camera wake attempt window | 20 s ×3 (beacon) | `camera_fsm.h` |
| Camera connect / subscribe timeouts | 20 s connect / 10 s ce82 subscribe, 3 retries each | `camera_fsm.h` |
| Camera record-confirm re-assert | 2.5 s camera-idle before re-shutter | `camera_fsm.h` |
| Camera record-obs freshness | 3 s (stale 0x10 → kUnknown) | `camera_ble.ino` |
| Camera pairing timeout | 120 s | `camera_fsm.h` |
| SensorEgg staleness | 1000 ms (older → NaN/`---`) | `sensoregg_protocol.h` |
| SensorEgg scan interval / window | 90 ms / 40 ms (≈44% duty, test-capped ≤45%), passive | `sensoregg_protocol.h` |
| SensorEgg scanner self-heal | 30 s no packet → stop+start kick | `sensoregg_protocol.h` |
| SensorEgg RSSI floor | −90 dBm | `sensoregg_protocol.h` |
| SensorEgg pairing MAC | `SENSOREGG_MAC` (all-zeros = any egg) | `sensoregg.h` |
| NeoPixel strip flag | `BIRDSEYE_ENABLE_NEOPIXEL`, default **1** on every channel since 4.1.0 | `project.h` |
| Loop profiling flag | `BIRDSEYE_ENABLE_PROFILING`, default 0; 1 on the beta channel | `project.h` |
| Profiling pin / span | 30 (`PROFILING_PIN`, = boost EN) / whole loop (`PROFILING_PIN_SECTION`) | `profiling.h` |
| Profiling rollup window | 1000 ms (`PROFILING_WINDOW_MS`) | `profiling.h` |
| Profiling timebase | DWT cycle counter, 64 ticks/us; `micros()` fallback (page shows `*`) | `profiling.ino` |
| NeoPixel pins | 30 = boost EN, 31 = data (NFC pads, post-UICR) | `neopixel.h` |
| NeoPixel layout | 11 px: status 0 + strip 1–9 (center px 5) + status 10 | `led_frame.h` |
| LED frame rate | 30 Hz (`NPX_FRAME_INTERVAL_MS` 33) | `neopixel.ino` |
| LED brightness default | 64 / 255 (`led_brightness`; 0 = disabled) | `settings.ino` |
| Pace pip full scale / deadband | ±1.0 ms/m (0.25 per pixel) / ±0.125 | `led_modes.h` |
| RPM scale red fraction | 0.5 (red past halfway) | `led_modes.h` |
| Rev flasher clear / EGT clear delta | 97% of `target_rpm` / alert − 20 °C | `led_modes.h` |
| GPS-search pip bounce period | 1600 ms round trip (`kSearchBouncePeriodMs`) | `led_modes.h` |
| Overrev latch clear | `target_rpm × 0.97` (same point as the warning flasher) | `neopixel.ino` |
| Boot / purple animation | 2600 ms / 1600 ms sector, 2600 ms lap (2 waves) | `led_animations.h` |
| Status LED modes | `off`/`rpm`/`speed`/`gps`/`camera`/`lap`/`sector`/`egt`; `gps`+`camera` also lit on the menu | `led_status.h` |
| Status LED defaults | left `rpm`; right `egt` on a SensorEgg build, `lap` otherwise | `settings.ino` |
| Speed bar ceiling / red band | `target_speed_mph` (default 60, clamp 5–250) / none (`kSpeedRedFrac` 1.0) | `settings.ino`, `led_modes.h` |
| Speed alert clear delta / flash | 2 mph (`kSpeedClearDeltaMph`) / 250 ms half-period | `led_modes.h` |
| Camera "cannot command" flash | 400 ms half-period (`kCameraFlashHalfPeriodMs`) | `led_status.h` |
| UTC offset band | ±840 min (±14 h), `kOffsetMinLimit`; out of band = 0 | `local_time.h` |
| LED day / night window | local 07:00 → 19:00 (`led_day_start_hour` / `led_night_start_hour`); equal = disabled | `settings.ino` |
| LED night brightness default | 16 / 255 (`led_brightness_night`; 0 = blank, rail stays UP) | `settings.ino` |

---

## Required Libraries

| Library | Purpose |
|---|---|
| Adafruit GFX | Graphics primitives |
| Adafruit SSD1306 | SSD1306 OLED driver |
| Adafruit SH110X | SH110X OLED driver |
| SparkFun u-blox GNSS v3 | UBX binary PVT GPS interface |
| ArduinoJson 6.x | Track file JSON parsing |
| SdFat | SD card (FAT16/32) |
| DovesLapTimer | Lap/sector timing (external: TheAngryRaven/DovesLapTimer). CI refs: `BETA`-targeted builds track the library's `BETA` branch; master/release builds pin `v4.3.0` (bump deliberately) |
| Seeed Arduino LSM6DS3 | Onboard IMU accelerometer/gyro (Sense variant, ±16g) |
| Adafruit NeoPixel | WS2812 strip driver (subsystem 16). **Only compiled/linked when `BIRDSEYE_ENABLE_NEOPIXEL` is set** — the include sits inside the `#if` in `neopixel.ino`, so a master/release image carries none of it |
| Bluefruit nRF52 | BLE (built into board package) |
| Adafruit TinyUSB | USB Mass Storage (`Adafruit_USBD_MSC`); built into board package |

---

## EMI Mitigation

This device operates in ignition-noise environments. Three layers of defense:

1. **Hardware**: RC low-pass filters on buttons (10 K + 100 nF) and tach
   (1 K + 100 nF + optional TVS diode).
2. **ISR design**: Volatile flag gating (never `noInterrupts()` in ISR);
   3 ms minimum pulse gap in tachometer.
3. **Software**: Multi-sample button reads (3x at 500 us), 200 ms refire
   lockout, Kalman-filtered RPM (absorbs ISR jitter), 2 MHz SPI clock for
   SD stability (raised to 8 MHz only during parked BLE/USB transfers, where
   the motor is off and ignition EMI is absent — see subsystem 4).
4. **GPS serial buffer**: TIMER3 ISR drains Serial1 into a 4 KB RAM ring
   buffer every 5 ms, preventing GPS data loss during SD card GC pauses
   that can block writes for 100 ms–2 s; the core Serial1 ring (256 B via
   the required `-DSERIAL_BUFFER_SIZE=256` flag) covers SoftDevice
   radio-ISR deferral of the drain itself.

---

## Build Notes

- **Board**: Seeed XIAO nRF52840 Sense (Arduino IDE). The firmware also
  builds and runs on the plain (non-Sense) Seeed XIAO nRF52840 — same MCU,
  BLE, bootloader and pin map; it just lacks the onboard LSM6DS3 IMU, so
  accelerometer logging degrades gracefully (`accelAvailable = false`). CI
  (`compile-sketch`) and the `release` workflow build a matrix of both
  variants (FQBNs `xiaonRF52840Sense` and `xiaonRF52840`), publishing
  per-board `BirdsEye-sense.*` / `BirdsEye-nonsense.*` assets. The `.zip`
  in each is the Secure DFU package used for OTA. Each build passes
  `-DBIRDSEYE_BOARD_SENSE` / `-DBIRDSEYE_BOARD_NONSENSE` (via
  `compiler.cpp.extra_flags`) so the image self-reports its variant over
  BLE; a plain IDE build with no flag defaults to `sense`.
- **Firmware version** is a single `#define FIRMWARE_VERSION` in `project.h`.
  Keep it in sync with the release git tag (`v2.0.0` -> `"2.0.0"`); it is
  reported over BLE (DIS) for the OTA update check. `FIRMWARE_VARIANT`
  (also in `project.h`) feeds the DIS model string. The version literal can be
  overridden at build time with `-DFIRMWARE_VERSION_OVERRIDE=<token>` (a bare
  token; `project.h` stringizes it) — the `beta` workflow uses this to stamp
  nightly builds as `<base>-beta.<gitsha>`. Normal builds leave it undefined.
- **Required build flag `-DSERIAL_BUFFER_SIZE=256`** — grows the core's
  Serial1 rings so radio-ISR deferral of the GPS drain can't drop bytes
  (see subsystem 1). `project.h` static_asserts it on non-SIM builds; CI
  passes it in all three workflows (merged into the SAME
  `compiler.cpp.extra_flags` property — a second `--build-property` for
  one key replaces the first). Local setup: CONTRIBUTING.md "Local build
  flags".
- **Feature flags** (`project.h`, tested with `#if` so an explicit
  `-DFLAG=0` wins; `BIRDSEYE_ENABLE_NEOPIXEL` defaults to `1`, the rest
  to `0`):
  - `BIRDSEYE_ENABLE_ONBOARD_CHARGING` — off in **every** channel. See
    subsystem 10: HICHG hold + the USB charging UX. The hardware now has
    an external charging circuit.
  - `BIRDSEYE_ENABLE_SENSOREGG` — off in master/release, **on in beta**
    (`beta.yml`, plus `compile-sketch.yml` for PRs targeting `BETA` so the
    flag-on build is compile-checked before it reaches the publish
    workflow). See subsystem 14.
  - `BIRDSEYE_ENABLE_PROFILING` — off in master/release, **on in beta**
    (`beta.yml`, plus `compile-sketch.yml` for PRs targeting `BETA`).
    See subsystem 18: it takes pin 30 from the NeoPixel boost EN line,
    so a beta image can never switch the 5 V rail.
  - `BIRDSEYE_ENABLE_NEOPIXEL` — **on everywhere since 4.1.0** (the
    `project.h` default; no workflow needs to pass it). The first boot of
    any 4.1.0+ image performs the ONE-WAY UICR NFC→GPIO conversion and
    self-resets once, on every device. This is the one flag whose default
    is 1 — see subsystem 16 and the upgrade note at the top of
    CHANGELOG.md's 4.1.0 section.
  When adding a flag: give it a `#ifndef` default in `project.h`, decide
  its per-channel value in the workflows, and document it here + in
  CONTRIBUTING.md's flag table.
- The sketch lives in `BirdsEye/` so the folder name matches the
  `.ino` file — required by Arduino IDE / arduino-cli.
- `project.h` is included before other `.ino` modules so Arduino's
  auto-prototype generator sees custom types first.
- PROGMEM is used for bitmap images to save RAM.
- Avoid Arduino `String` in hot paths (heap fragmentation risk on 256 KB).
- SD chip-select is hardwired to GND; pass `-1` to SdFat.
- `#define SIM` enables simulator-specific tweaks (no WDT, fixed battery
  voltage, placeholder GPS setup, sim button pins). Never defined in CI
  firmware builds — it's the compile flag for the browser/WASM simulator
  build (sources under `BirdsEye/sim/`; replaces the old Wokwi target).
- `#define ENDURANCE_MODE` hides the tachometer page and reshuffles
  page numbers — for endurance racing where RPM isn't relevant.
- **TIMER3 is reserved** for the GPS serial buffer ISR. Use TIMER4 if another
  hardware timer is needed. TIMER0 is reserved by SoftDevice; TIMER1/2 may
  be used by PWM/tone.
- **CRITICAL: NEVER use `analogRead()` on any GPIO pin.** On the nRF52840,
  `analogRead()` permanently disables the digital input buffer on the target
  pin for the remainder of the session. Every analog-capable pin on the XIAO
  is also a critical digital function: A0=tach ISR, A1-A3=buttons, A4=SDA,
  A5=SCL. Use `micros()` or the hardware RNG for entropy instead.

---

## Development Conventions

- `.ino` files act as modules; Arduino IDE concatenates them alphabetically
  after the main sketch file.
- Each module has a matching `.h` declaring its public surface. The `.ino`
  includes its own header as the first include so any drift between
  declaration and definition is caught at compile time.
- Each subsystem exposes `*_SETUP()` and `*_LOOP()` entry points called
  from `BirdsEye.ino`.
- SD access must go through `acquireSDAccess()` / `releaseSDAccess()`.
- GPS data validation (9 checks) must pass before any CSV row is written.
- Display pages are rendered by `displayPage_*()` functions routed via
  `currentPage` in `displayLoop()`.
- Cross-module globals (e.g. `dovexReplay*`, `trackManifest[]`, `courseManager`)
  are declared and defined in `BirdsEye.ino`. Module headers may `extern`-declare
  them where the module's own API touches that state.
- Library includes that define **parameter or return types** used in
  auto-prototyped functions (`DovesLapTimer.h`, `CourseManager.h`,
  `SparkFun_u-blox_GNSS_v3.h`, `ArduinoJson.h`) must be in the top include
  block of `BirdsEye.ino` (before Arduino generates prototypes). Included any
  later, the generated prototype cannot see the type, silently degrades it to
  `int`, and the function fails with *"redeclared as different kind of
  entity"*. **The simulator cannot catch this** — it hand-writes its
  prototypes in `sim/sim_prototypes.h` — so a green sim build says nothing
  about it; only the Arduino compile does. `ArduinoJson.h` joined this list
  when `sd_functions.ino` grew helpers taking `JsonArray` (plan 0005).
