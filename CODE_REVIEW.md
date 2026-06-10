# BirdsEye — Professional Code Review

**Scope:** Full codebase (~7,500 lines firmware + ~970 lines host tests + CI).
**Method:** Five independent specialist passes — architecture/modularity, embedded
concurrency & ISR safety, security, testing & CI, code quality & performance.
**Bar:** Professional / Linux-kernel-grade standards, judged harshly.
**Constraint:** Review only — no code was modified.

---

## Scores

| Dimension | Score (1–10) |
|---|---|
| Architecture & modularity | **3.5** |
| Embedded correctness (concurrency/ISR/memory) | **5.0** |
| Security (BLE/OTA attack surface) | **3.0** |
| Testing & CI maturity | **4.5** |
| Code quality & performance | **5.5** |
| **Overall professional quality** | **≈ 4.5 / 10** |
| **Ambition / scope ("10 = impossible for a solo dev")** | **≈ 7 / 10** |

**Reading the two numbers.** As a *solo* project the ambition is genuinely high
(7): a 25 Hz GPS logger with auto track-detection, BLE file sync, a self-flash
OTA pipeline, dual-board CI, host unit tests, coverage, clang-tidy, a release
pipeline, and real documentation is far past hobby grade. But measured against
*professional production standards* (4.5), the same project has structural and
safety defects that would block merge on a serious team: a "mutex" that isn't
one and is reachable from radio range, an unauthenticated unsigned firmware
flash over BLE, a god file holding every other module's state, a coverage gate
set to 1%, and a user-visible lap-time rendering bug in the product's core
output. The skeleton is professional; the load-bearing guarantees are not.

**Cross-agent corroboration (the issues you should trust most):** the SD
"mutex" race was independently flagged by **four of five** reviewers; the tach
ring-buffer overflow and the double-precision hot-path scan by two each. These
are not opinion calls.

---

## CRITICAL — fix before any field release

### CR-1. The SD "mutex" is a non-atomic check-then-set, bypassed from the BLE task
`sd_functions.ino:13-27`. `acquireSDAccess()` is a TOCTOU on a `volatile int`
with no critical section, `__LDREX/STREX`, or RTOS semaphore — yet it is called
from **both** the main-loop task and the Bluefruit callback task. Multiple live
paths reach concurrent SdFat access from radio range:
- `GET:`/`TGET:` run `bleStartFileTransfer()` **directly in the BLE callback
  task** (`bluetooth.ino:346,383`), whose first act is to `close()` a file the
  main loop may be mid-`read()` on (`bluetooth.ino:233,844`) → use-after-close in SdFat.
- `DELETE:` → `bleDeleteFile()` (`bluetooth.ino:269-286`) does `SD.exists()`/
  `SD.remove()` with **no `acquireSDAccess()` at all** — can delete the file
  being streamed or collide with OTA staging writes.
- The `mode == currentSDAccess` "idempotent" early-return and the preemptible
  `SD_ACCESS_TRACK_PARSE` mode let two contexts both win the lock.
- `bleSendFileList`/`bleSendTrackList` only *peek* at the flag, then iterate the
  directory for seconds (`delay(10)` per entry) without holding it.

**Impact:** FAT corruption, lost session logs, SPI wedge until WDT reset — all
triggerable by a stranger's BLE client sending overlapping commands.
**Fix:** a real mutex (FreeRTOS semaphore / critical section), and route
*every* SD-touching BLE command through the existing defer-to-`BLUETOOTH_LOOP()`
pattern — no SdFat in the callback task, ever.

### CR-2. Unauthenticated, unsigned firmware self-flash over BLE = remote code execution
`bluetooth.ino:428-430`, `firmware_ota.ino:225-302,561-670`. Any BLE central in
range can drive `FWBEGIN → FWPUT → chunks → FWDONE → FWAPPLY`; the device erases
its app region and boots the supplied image. The only gates are **CRC-32**
(integrity the attacker computes themselves — *not* authentication), a
cosmetic attacker-supplied variant string, and a 3.6 V battery check. This path
deliberately bypasses the signed Nordic Secure DFU bootloader and writes flash
from a RAM flasher. **`SECURITY.md` does not mention OTA at all.**
**Fix:** require an encrypted+bonded link before honoring `FW*`, and add image
signature verification (public-key) — CRC must never be the sole gate on code
execution. Until then, document it honestly and gate OTA behind a physical/paired action.

### CR-3. No BLE pairing/bonding/PIN enforcement on any command — `bluetooth_pin` is decorative
`bluetooth.ino:97-119` (all chars `SECMODE_OPEN`); `settings.ino:36-37`. There
is no call to any Bluefruit security/pairing API anywhere; the stored
`bluetooth_pin` is read/written as a JSON string and **never passed to the BLE
stack**. Any connected central can `LIST/GET/DELETE/SSET/SRESET/FW*` with no
credential — download every GPS trace (where the vehicle has been), delete logs,
rewrite settings, force reboots, or flash firmware (CR-2). `SECURITY.md`
acknowledges the read gap but understates it as eavesdropping-only, omitting all
the **write** capabilities.
**Fix:** wire the existing PIN into LESC pairing + bonding; reject
write/delete/settings/`FW*` on unencrypted links.

### CR-4. Sleep mode is permanently defeated after the first engine pulse
`tachometer.ino:66` sets `tachHavePeriod = true` and **nothing anywhere clears
it** (`enterSleepMode()` does not; `BirdsEye.ino:1129` reads it as the RPM wake
trigger). Once the engine has fired one pulse since boot, every sleep entry
(long-press, 5-min idle, USB) instantly bounces into `exitSleepMode(true)` —
which re-enters **race mode with logging enabled**. The driver parks, requests
sleep, and the device silently starts a new logging session and drains the pack
overnight. Negates the entire sleep + auto-idle subsystem.
**Fix:** clear `tachHavePeriod` (and stale ring state) in `enterSleepMode()`.

---

## HIGH

### H-1. Lap-time rendering bug — trailing zero-pad corrupts the product's core output
`display_pages.ino:380-385, 486-491, 580-585`. Milliseconds are zero-padded
*after* printing the value: 7 ms renders as `.700`, so `1:23.007` shows as
`1:23.700` — a 693 ms error on the lap time the whole product exists to display.
`displayPage_gps_lap_list` (`:644-648`) does no padding at all. The replay page
(`:203-205`) does it correctly, proving copy-paste mutation. **Root cause is H-2.**

### H-2. Six divergent inline copies of ms→M:SS.mmm formatting
`display_pages.ino:196-208, 213-223, 364-385, 469-491, 564-586, 632-648` — 18
occurrences of the `60000`/`%1000` math with four different padding behaviors.
One tested `formatLapTime()` pure unit (per the project's own stated convention)
would have prevented H-1 entirely.

### H-3. Hot-path O(N) double-precision haversine scan runs ~250 Hz, not per GPS fix
`BirdsEye.ino:775-790`. The guard (`trackDetected || !gpsData.fix || count==0`)
never latches when no track is near, and `gpsData.fix` stays true between PVT
updates, so the device runs a full manifest scan **every main-loop iteration**.
Each `haversineDistanceMiles()` is ~7 software-emulated `double` libm calls on
the (single-precision-only-FPU) Cortex-M4F; at the 200-entry manifest ceiling
that's ~5–10 ms *per loop*, collapsing the documented ~250 Hz loop to <100 Hz —
for data that changes at 25 Hz. CLAUDE.md says "every GPS fix"; the code doesn't.
**Fix:** gate on fresh-PVT consumption or throttle to ≤1 Hz.

### H-4. Tach ring buffer ISR has no overflow check — laps the consumer during documented SD stalls
`tachometer.ino:60-63` advances head unconditionally (the sibling GPS ISR at
`gps_functions.ino:85` checks full correctly). The comment claims 48 ms of
margin; the project's own GPS rationale documents SD GC stalls of 100 ms–2 s. At
6,000 RPM a 200 ms stall overruns the 16-entry buffer, the SPSC invariant
breaks, and `TACH_LOOP()` computes a confident-but-wrong RPM that is logged to
CSV **and** feeds the `>500` auto-race trigger. Silent bad data.
**Fix:** detect full-and-drop in the ISR, or size for the documented stall.

### H-5. `GPS_BAUD_RECOVERY()` can out-wait the 4 s watchdog → boot loop with dead GPS
`gps_functions.ino:515-573`, run from `GPS_LOOP()` under the armed WDT
(`BirdsEye.ino:558`, fed only at loop top). Up to three `myGNSS.begin()` calls
(~1.1 s timeouts each, multiple pings) plus ~500 ms of `delay()` can exceed 4 s
when the module is genuinely hung → WDT reset → re-setup → repeat. The one path
built to recover a sick GPS is the one that trips the watchdog. (`fwStageToFlash`
got `wdtPet()` calls for this reason; this path didn't.)

### H-6. `BirdsEye.ino` is a god file: ~110+ mutable globals, 1,308 lines, ≥14 responsibilities
It defines the private state of *every other module* — the tach ring buffer, the
BLE service objects + 12 flags, `gpsData`, all five SD `File` handles, all replay
and display state — so `tachometer.ino` operates on state declared 1,000 lines
away in another file. The inter-module contract rides on Arduino's `.ino`
concatenation: only **11 `extern`s exist across all headers** while dozens of
globals are consumed cross-module with no declaration anywhere (`enableLogging`
touched by 5 modules, `gps_speed_mph` by 4, etc.). The `GpsData` *type* lives in
the `.ino`, so no header can name it. No module can be compiled, tested, or
reused independently.

### H-7. No layering — drivers, domain, storage, and UI are mutually entangled
The GPS driver owns the logging subsystem (`GPS_LOOP()` hardcodes the tach/accel
CSV columns; adding a sensor means editing the GPS module). The UI mutates domain
state directly (`handleMenuPageSelection()` sets `raceActive`/`enableLogging`,
calls `BLE_SETUP()`/`endRaceSession()`). Render functions have side effects
(`displayPage_gps_stats` reads the battery ADC; `displayLoop` mutates
`currentPage` mid-render). The OTA module reaches into six subsystems. The only
clean layer is the ~430 lines of pure `.cpp` units.

### H-8. The riskiest code is untested and left un-extracted, violating the project's own convention
- **OTA protocol state machine** (`firmware_ota.ino`, 727 lines) — parse-on-
  untrusted-BLE-input feeding app-region erase, zero host tests. The 2.2.3
  CHANGELOG shows three field OTA-apply sequencing bugs — exactly what a
  host-tested state machine catches.
- **BLE command dispatcher** (`bluetooth.ino:302-417,546-650`, ~18 branches) —
  pure string parsing on untrusted input, untested.
- **Tach Kalman filter** (`tachometer.ino`) — pure float math (wrap handling,
  Q/R/K update, sanity bounds), untested.
All three are trivially extractable to tested pure units; CLAUDE.md explicitly
forbids leaving testable pure logic in `.ino` files.

### H-9. Supply chain: nothing pinned to an immutable ref
`.github/workflows/*`: every action pinned by mutable tag (none by SHA),
including third-party `peaceiris/actions-gh-pages@v3` holding `contents: write`.
Libraries installed by **bare name, no version** (`ArduinoJson`, `SdFat`,
`DovesLapTimer` from a default-branch git URL with `unsafe_install`), BSP
unpinned, `uf2conv.py` `curl`ed from `microsoft/uf2` **master** at release time
and executed. No reproducible builds, no `dependabot.yml`. Two release builds a
month apart can differ silently.

---

## MEDIUM

- **M-1. Coverage gate is theater.** `coverage.yml` `COVERAGE_MIN: 1` over a
  denominator gerrymandered to `BirdsEye/*.cpp` only — ~7% of the firmware.
  Coverage could fall 100%→2% and stay green. Raise to ≥90 (actuals are near 100%).
- **M-2. Beta OTA channel ships with zero quality gates.** `beta.yml` triggers
  only on push to `BETA`; tests, clang-tidy, coverage, and the flash/OTA-size
  gates all trigger only on `main`/PR. A direct push compiles with
  `--warnings none` and publishes a self-flash image real devices consume.
- **M-3. Release builds mute all warnings** (`--warnings none` in release.yml /
  beta.yml); production binaries get *less* scrutiny than the test binaries
  (`-Werror`). No warnings-as-errors on the firmware anywhere.
- **M-4. No release-tag ↔ `FIRMWARE_VERSION` sync check.** Tag `v2.3.0` against a
  stale `project.h` ships a manifest whose binary self-reports the wrong version
  over BLE DIS → OTA update-check loops. `beta.yml` already greps the version,
  proving a 5-line gate is easy.
- **M-5. The page/mode state machine is integer arithmetic over magic numbers.**
  Load-bearing ordered `const int` page IDs, navigation via `currentPage++/--`
  with `LOGGING_STOP-1` / `GPS_LAP_BEST-1` special cases, a 17–21-branch dispatch
  chain plus a parallel hand-maintained `menuLimit` table, `#ifdef ENDURANCE_MODE`
  renumbering into a second incompatible ID space. The author's own TODOs admit
  it. The *mode* machine is 8+ unowned booleans mutated from four files with no
  enum, transition function, or invariant check.
- **M-6. Function-length monsters:** `displayLoop()` 246 lines, `GPS_LOOP()` 204,
  `loop()` 188 (three divergent inline schedulers for normal/BLE/sleep),
  `parseTrackFile()` 171, `bleFileRequestCallback()` 144.
- **M-7. PRIMASK (`__disable_irq()`) inside the TIMER3 ISR with SoftDevice
  active**, wrapped per-byte around two virtual `Uart` calls
  (`gps_functions.ino:78-82`). Nordic forbids app PRIMASK masking of SoftDevice
  IRQs (use BASEPRI / `sd_nvic_critical_region_enter`). Risk: radio-timing
  violations under exactly the SD-stall load that makes the drain loop longest.
- **M-8. Pervasive `double` on a double-emulated core.** `GpsData` makes
  altitude/speed/HDOP/heading/hAcc `double`; `onPVTReceived()` does 7 soft-double
  divides per PVT at 25 Hz; the log path runs 10 `dtostrf` double conversions per
  row. Only lat/lng need double.
- **M-9. No sanitizers / fuzzing on the host suite.** `dovex_header::parse()` and
  `filename_validator` are buffer-walking parsers over attacker-influenceable
  bytes; a 10-line `-fsanitize=address,undefined` CI variant + a libFuzzer harness
  cost almost nothing and the suite already builds on host.
- **M-10. clang-tidy covers ~7% of code and silently omits `crc32.cpp`**
  (hardcoded 5-file list vs 6 built by CMake). The `.ino` files — ISRs, BLE
  callbacks, OTA flasher — get no static analysis beyond "it compiled." No
  cppcheck/CodeQL.
- **M-11. SD-mutex bypasses in read paths:** `parseDovexHeader()`
  (`replay.ino:182-195`) and the file/track listers don't acquire the lock —
  TOCTOU against main-loop logging.
- **M-12. `SRESET`/`SSET` unauthenticated config tampering & forced reboot**
  (`bluetooth.ino:622-659`); unbounded settings keys can bloat the fixed
  512-byte JSON doc until `/SETTINGS.json` is unparseable (loses BLE name/PIN).
- **M-13. Wokwi sim config exists but is unused in CI; no integration test of any
  kind** (no GPS-PVT→validation→DOVEX-row round-trip, no DOVEX golden file, no
  BLE protocol round-trip). The gap between "428 lines unit-tested" and "in a
  kart" is empty.
- **M-14. Render functions mutate state** (`paceFlashStatus`, `notificationFlash`,
  `calculatingFlip`, battery ADC read inside `displayPage_gps_stats`).
- **M-15. Duplicated logic blocks:** GPS VALSET config twice, settings defaults
  twice, battery-throttle twice, knots→mph literal twice, 11 near-identical
  `activeTimer*()` wrappers, 4× filename-wrap block, the 12 `activeTimer*` dual-
  dispatch helpers.
- **M-16. OTA GPREGRET recovery overstated** — GPREGRET survives `SYSRESETREQ`,
  not battery removal/brownout, so the "power lost mid-erase" comment overclaims;
  real defense is the battery gate + bootloader app-validity check
  (`firmware_ota.ino:57-66,633`).
- **M-17. Thin loop-task stack already hardfaulted once** (`firmware_ota.ino:485`).
  `dovex_header::parse()` stacks ~1.3 KB; `processSettingsCommand` ~1 KB;
  `BLUETOOTH_LOOP` a 524 B buffer. No high-water-mark instrumentation.
- **M-18. Cross-task flags without `volatile`/atomics** (`bleConnected`,
  `bleTransferInProgress`, `fwExpectedSize`, `fwBytesReceived`) — held together
  by incidental opaque-call reloads; LTO could break them.
- **M-19. Headers document but don't abstract** — flat free-function prototype
  dumps, zero opaque handles, zero encapsulation. `display_config.h` *defines* the
  display object in a header (ODR-safe only because it's one TU).
- **M-20. Workflow hygiene:** no `concurrency:` groups (gh-pages push races),
  missing least-privilege `permissions:` blocks, soft-fail size gates that
  degrade to ignored warnings, beta.yml an acknowledged copy-paste of release.yml.

---

## LOW (representative — full set in agent notes)

- **L-1.** Release debug macro `#define debug dummy_debug` (variadic) still
  *evaluates arguments* and materializes hundreds of `F()` strings into flash;
  use `#define debug(...) ((void)0)`. (`project.h:75-83`)
- **L-2.** Ignored write/return values: DOVEX pre-fill + CSV header
  (`gps_functions.ino:458-463`) and `writeDovexHeader()` flush
  (`BirdsEye.ino:1037`) — session lap times can vanish silently at session end.
- **L-3.** `displayPage_bluetooth` `(bytes*100)/fileSize` ÷0 on empty file and
  uint32 overflow >42 MB (`display_pages.ino:59`).
- **L-4.** Dead code: `epsilonPrecision`, `buttonPressIntv/HoldIntv`,
  `readReplayLine()` (no callers), `replayFile` (only ever closed),
  `forceReleaseSDAccess()`, `PAGE_TEST`/`PAGE_RC_ERROR`.
- **L-5.** In-band control tokens in binary streams (`TDONE`/`FWDONE`/`FWABORT`)
  — a fw image whose tail spells the token can't transfer; no framing.
- **L-6.** Naming chaos: `SCREAMING_SNAKE` functions vs camelCase vs snake_case
  variables; `#define` constants vs `const int` page IDs.
- **L-7.** `forceDisplayRefresh()` = `displayLastUpdate += 5000;` with the comment
  *"why does adding work but not subtracting?"* — trial-and-error on the UI timing
  path. (`display_ui.ino:218`)
- **L-8.** `gps_validation` test asserts `isNumericString("---")` is valid — a row
  of `---` passes the numeric gate into the CSV; the test enshrines the hole.
- **L-9.** Mutable globals that should be `constexpr`
  (`batteryUpdateInterval`, `displayUpdateRateHz`, `BUTTON_SAMPLE_*`).

---

## What's genuinely good (don't lose these)

- The **six pure-logic units** (`haversine`, `gps_time`, `gps_validation`,
  `dovex_header`, `filename_validator`, `crc32`) with host doctests — the same
  `.cpp` ships and is tested, no copy-paste. Test *quality* per line is high:
  property-style checks, real boundary tests, negative tests, the truncated-DOVEX-
  header contract test. The problem is scope, not craft.
- **`filename_validator`** is a model defensive control — allowlist-based, applied
  before every SD path build, no traversal/separator/control/high-bit/length
  bypass found. Genuinely closed.
- The **OTA apply sequencing** is the best code in the repo: CRC re-verified *in
  staging flash before* the app region is erased, battery/overlap guards,
  RAM-resident flasher with inline WDT feeds and inline AIRCR reset,
  SoftDevice-disable return code actually checked.
- The **GPS serial ring buffer** (TIMER3 drain, correct SPSC full-check) and the
  deferred-to-main-loop BLE threading discipline (where applied) show real
  embedded literacy.
- **Consistent unsigned-wrap time math**, bounded string handling
  (`strncpy`+manual NUL, `snprintf`, post-`dtostrf` garbage check), DOVEX
  crash-safe header design, dual-board CI matrix with flash/OTA-size gates, and
  honest comments that flag their own unverified assumptions (Phase 0 notes).
- **Documentation (CLAUDE.md / ARCHITECTURE.md) accurately describes the system**,
  including, implicitly, its flaws.

---

## Suggested order of work

1. **CR-1** (SD mutex) and **CR-4** (sleep flag) — small, high-impact correctness fixes.
2. **CR-2 / CR-3** (BLE auth + OTA signing) — the security floor; pair with an honest `SECURITY.md` rewrite.
3. **H-1 / H-2** (lap-time bug + extract `formatLapTime`) — user-visible, cheap, satisfies your own test convention.
4. **H-3 / H-4 / H-5** (hot-path scan, tach overflow, WDT-vs-recovery) — silent data/availability defects.
5. **M-1 / M-2 / M-3 / M-9** (coverage gate, beta gating, release warnings, sanitizers) — make CI mean what it claims.
6. **H-6 / H-7 / M-5** (god file, layering, state machine) — the big refactor; do it incrementally, one concern per PR.
7. **H-8** (extract + test OTA/BLE/Kalman logic) — pairs naturally with the refactor.
