# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project aims to follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html):

- **MAJOR** — a change that breaks existing track files, log file formats,
  the BLE command protocol, or removes a user-visible mode.
- **MINOR** — new features or device behavior that is backwards compatible.
- **PATCH** — bug fixes and internal changes with no user-visible behavior change.

## [Unreleased]

### Security
- **SD card arbitration race fixed — a BLE client in radio range could
  corrupt the card.** `acquireSDAccess()` was a non-atomic check-then-set on
  a shared flag, and several BLE commands (`LIST`, `GET:`, `DELETE:`,
  `TLIST`, `TGET:`) ran SdFat directly in the Bluefruit callback task —
  concurrently with the main loop's 25 Hz session logging. Overlapping
  commands could close a file mid-read, delete the file being streamed
  (`DELETE:` took no lock at all), or interleave two SdFat operations:
  FAT corruption, lost session logs, or an SPI wedge. Now: lock
  transitions are atomic (FreeRTOS critical section; the grant/deny table
  is the new host-tested `sd_access_policy` unit), all five commands are
  deferred to `BLUETOOTH_LOOP()` on the main loop like the
  settings/track/OTA commands already were, directory listings hold the
  lock for the whole walk, and `DELETE` takes the lock and refuses with
  `BUSY` while a transfer is streaming. Protocol impact: commands that
  arrive while another file command is still queued now get the existing
  `BUSY` / `TERR:BUSY` replies instead of executing concurrently.

## [2.2.3] - 2026-06-10

### Added
- **The OLED now shows a full-screen "UPDATING FIRMWARE / Do not power off"
  notice during an OTA apply.** The apply blocks the main loop (UI frozen) and
  ends in a reboot; previously the screen just sat on a stale page with no
  indication anything was happening. The notice stays up through staging and
  the swap until the new firmware boots and repaints; on a failed apply the
  normal 3 Hz display refresh replaces it automatically.
- **Beta OTA manifest now lists each variant's `.uf2`** (`builds[model].uf2`).
  The `.uf2` was already published to `beta/`, just not referenced — exposing
  it makes drag-and-drop bootloader (DFU-mode) recovery easy when an OTA fails.
- **Firmware OTA apply emits `FWDBG:*` breadcrumbs** (`APPLY`, `VBAT=<mv>`,
  `STAGE`, `ERASE=<pages>`, `ERASED`) over the status characteristic so a stall
  before `FWAPPLIED` can be pinpointed from the web app's raw notification log
  — apply entered? battery reading seen? staging/erase reached? The up-front
  staging-region erase is several seconds with no progress notify (and may run
  while BLE is still connected), so it is bracketed explicitly.
- **Beta / nightly firmware channel.** A new `beta` CI workflow builds both
  XIAO nRF52840 variants on every push to the `BETA` branch and publishes them
  to a `beta/` subtree on `gh-pages` — a second OTA channel that lives
  alongside the production one at the Pages root (the deploy's `keep_files`
  keeps the two from clobbering each other). No GitHub Release and no version
  tag are cut; these are throwaway debug builds you flash from your phone via
  DovesDataViewer's beta channel while at the track. `beta/manifest.json`
  mirrors the prod manifest shape (so the web client reuses one parser) plus
  `channel`, `commit`, and `branch` markers, and points at `beta/` asset URLs.
  Retention is **latest-only** — a single flat `beta/` slot, overwritten each
  push (no per-build history, unlike prod's `firmware/<version>/`).

### Fixed
- **Device now boots straight into the new firmware after an OTA apply
  (no more manual power-cycle).** The apply arms the bootloader-recovery
  magic (`GPREGRET = 0xA8`, OTA-DFU) before the destructive swap, but never
  cleared it on success — so after the RAM flasher's reset the bootloader saw
  the magic and parked in BLE DFU mode instead of booting the freshly
  installed app (blank/stale screen, odd USB device on the PC; a manual
  power-cycle cleared the register, which is why it booted fine afterwards).
  The RAM flasher now clears `GPREGRET` after a successful copy, immediately
  before its reset. An interrupted swap never reaches that line, so the
  recovery net is unchanged.
- **Firmware OTA self-flash swap never took effect — `FWAPPLIED` but still the
  old image after reboot.** The apply disabled the SoftDevice (`sd_softdevice_
  disable()`) with the web app *still connected* and ignored the return code.
  The SoftDevice won't cleanly disable while a link is up, so it stayed partly
  active, flash remained SoftDevice-protected, and the RAM flasher's raw NVMC
  erase/copy silently no-opped — but the final `SCB->AIRCR` reset still fired,
  rebooting into the intact old application. The apply now **disconnects the
  central and waits for the link to close** before disabling the SoftDevice,
  and **checks the disable return** — on failure it resets toward the armed
  bootloader-recovery flag instead of running a swap that can't write.
- **Firmware OTA apply was aborted by the BLE disconnect, so the update never
  installed.** After `FWAPPLY`, the web app disconnects to hand the device off
  to self-flash — but `BLUETOOTH_LOOP()` ran its disconnect teardown (which
  calls `fwReset()` to abort the OTA, then `NVIC_SystemReset()`) *before*
  `FW_OTA_LOOP()`, where the apply actually runs. So a disconnect at that point
  discarded the staged image and rebooted into the **old** firmware (reported
  as "applied OK, rebooted, still the old version"). The disconnect teardown
  now detects an in-flight apply (`fwApplyRequested()`) and skips both the abort
  and the reboot, leaving the install to `FW_OTA_LOOP()`, which owns its own
  reset. The SoftDevice is still up at that point, so the SD→flash staging and
  CRC re-verify proceed normally with the radio already gone.

### Changed
- **`FIRMWARE_VERSION` can now be overridden at build time.** `project.h`
  stringizes an optional `-DFIRMWARE_VERSION_OVERRIDE=<token>` build flag, so a
  beta build self-reports its exact commit over BLE as `<base>-beta.<gitsha>`
  (e.g. `2.2.2-beta.abcdef0`) — read the version off a device and you know
  precisely which nightly is on it. A normal build (prod release, plain IDE,
  `compile-sketch` CI) leaves the flag undefined and uses the literal base
  version, so nothing changes for production. The OTA image descriptor's
  `version` field was widened 16 → 32 bytes to hold the longer beta strings.

## [2.2.2] - 2026-06-08

### Fixed
- **`FIRMWARE_VARIANT` now follows the board you select in the Arduino IDE.**
  Previously, only the CI/release flags (`-DBIRDSEYE_BOARD_SENSE` /
  `-DBIRDSEYE_BOARD_NONSENSE`) set the variant; a plain IDE build with neither
  flag always reported `"sense"` regardless of the selected board, so a
  non-Sense unit flashed from the IDE mislabeled itself in its BLE DIS Model
  Number (and thus to the OTA update check). `project.h` now derives the
  variant from the Seeeduino core's `ARDUINO_Seeed_XIAO_nRF52840[_Sense]` board
  macro when no explicit build flag is present. The explicit CI flags still
  take precedence, and an unknown board still defaults to `"sense"`.

## [2.2.1] - 2026-06-08

### Changed
- **Firmware OTA: the target variant is now declared in the `FWBEGIN`
  handshake instead of inferred from the image bytes.** The command gains a
  third field — `FWBEGIN:<size>,<crc32>,<variant>` — where `<variant>` is the
  target board variant (`sense` / `nonsense`) that the web app derives
  authoritatively from the device's own DIS Model Number. The firmware
  compares it (case-insensitively) to its compile-time `FIRMWARE_VARIANT` and
  replies `FWERR:VARIANT` *before any upload* on a mismatch. The old image-byte
  scan at `FWAPPLY` (which misfired `FWERR:VARIANT` on correct sense→sense
  flashes) has been removed; the embedded image descriptor is retained for
  forensics only. **Breaking:** a web client that still sends the two-field
  `FWBEGIN:<size>,<crc32>` is rejected — the web side updates in lockstep.

## [2.2.0] - 2026-06-08

### Fixed
- **Logging no longer starts before the GPS has a real time lock.** File
  creation was gated only on `day > 0`, so before the module resolved UTC it
  would create a log named from its placeholder date (e.g.
  `20210307_0000.dovex`). That name was identical on every boot and, once a
  write was interrupted, the half-written file could no longer be reopened —
  producing a "Error saving log" fault that reproduced on every reboot. File
  creation now requires the module's `validDate + validTime + fullyResolved`
  flags, and `fix` now also requires `gnssFixOK`.

### Changed
- **Logging failures never drop out of race mode.** When a session is running
  with the engine turning but no GPS lock yet, the device now pins the user to
  the tachometer and waits, then begins logging and resumes normal race-mode
  navigation the moment a valid lock arrives. A failed log-file open is
  retried (throttled to 1 Hz) instead of faulting, and a mid-session write
  failure stops logging while the race continues — none of these show the
  full-screen "Please Reboot Device" fault anymore.

### Added
- **SD-staged firmware OTA over the custom BLE file service.** Because
  Chrome's Web Bluetooth blocklist bans the Nordic legacy DFU service that
  `BLEDfu` exposes (and our sealed units have no pins to install a
  web-allowed Secure-DFU bootloader), the firmware can now update *itself*:
  the DovesDataViewer web app streams the new image to the SD card over the
  existing `0x1820` file service, the firmware CRC-32 verifies it, copies it
  into a free internal-flash region, and a RAM-resident flasher swaps it into
  the application region and resets. New `FW*` command/response protocol on
  `0x2A3E`/`0x2A40`: `FWBEGIN:<size>,<crc>` → `FWCRC:<crc>` handshake,
  `FWPUT:<size>` → `FWREADY` + raw chunks → `FWDONE` →
  `FWOK:<crc>`/`FWERR:<reason>`, then `FWAPPLY` → `FWSTAGE:<pct>` →
  `FWAPPLIED`. CRC is CRC-32/IEEE-802.3 (zlib), lowercase 8-char hex; the new
  host-tested `crc32` pure unit pins it to the web client's algorithm. The
  apply path is guarded by a battery-voltage check (`FWERR:BATTERY`), an
  embedded variant/magic check (`FWERR:VARIANT`), an in-flash CRC re-verify
  before the app region is ever erased, and a GPREGRET bootloader-recovery
  flag so an interrupted swap leaves the unit re-flashable over BLE. The
  request characteristic max length was raised to 244 to carry ~240-byte
  image chunks. See `docs/firmware-ota-phase0.md` for the apply-strategy
  decision and the hardware spikes that gate it. (The previously added
  `BLEDfu` buttonless Secure DFU service remains registered for the one-time
  fleet-migration push via the nRF Connect mobile app.)
- **OTA manifest now publishes the raw app image + its CRC-32.** The release
  workflow extracts `BirdsEye.ino.bin` from each variant's DFU `.zip`,
  publishes it next to the `.zip`, and adds `appBin`, `appCrc32`
  (CRC-32/IEEE-802.3, lowercase 8-char hex), and `appSize` to each
  `manifest.json` build entry. This is the authoritative checksum for the
  SD-staged OTA: the web client can download `appBin` directly (no client-side
  unzip), send `FWBEGIN:<appSize>,<appCrc32>`, and the device's `FWOK:<crc>`
  must equal `appCrc32` — build pipeline, web app, and firmware all agree on
  one value. Additive/backwards-compatible with the existing `dfuZip` field.
- **Over-the-air (OTA) firmware updates.** The firmware now registers the
  buttonless Secure DFU service (`BLEDfu`), so a companion (DovesDataViewer
  over Web Bluetooth) can reboot the board into the bootloader's Nordic
  Secure DFU mode and flash a new image without a physical reset
  double-tap. The bootloader validates the signed/CRC'd DFU package before
  writing, so a corrupt or mismatched image is rejected rather than
  bricking the board.
- Firmware version reporting over BLE via the standard Device Information
  Service (`BLEDis`, 0x180A / Firmware Revision 0x2A26). Lets the companion
  read the installed version and compare it against the latest GitHub
  release to decide whether an update is available. Version is defined once
  as `FIRMWARE_VERSION` in `project.h` (starting at `2.0.0`). The DIS model
  string encodes the board variant (`BirdsEye-sense` / `BirdsEye-nonsense`,
  selected by the per-FQBN build flag `-DBIRDSEYE_BOARD_SENSE` /
  `-DBIRDSEYE_BOARD_NONSENSE`) so the companion fetches the matching OTA
  package.
- Release builds now cover **both XIAO nRF52840 variants** (Sense and
  non-Sense). The release workflow builds a matrix and publishes per-board
  `.hex` / `.uf2` / `.zip` assets named `BirdsEye-sense.*` and
  `BirdsEye-nonsense.*`; the `.zip` is the Secure DFU package used for OTA.
  `compile-sketch` CI also builds both variants.
- OTA update manifest published to GitHub Pages. On a version tag the
  release workflow pushes the DFU packages plus a stable `manifest.json`
  (latest version + per-variant download URLs, keyed by the DIS model
  string) to the `gh-pages` branch. The companion reads it from the Pages
  URL — which serves with permissive CORS, unlike raw release-asset URLs —
  to check for updates and fetch the matching image. Older versions are
  retained under `firmware/<version>/` for rollback.
- Host-side unit test harness (doctest + CMake) covering the pure-logic
  units: haversine distance, GPS time/epoch math, GPS sample validation,
  and the DOVEX header format/parse. Runs in CI on every push.
- `clang-tidy` static analysis in CI (bugprone / performance / portability
  / clang-analyzer families, warnings-as-errors) over the host-buildable
  units.
- Flash-size budget gate in CI: the build fails if the firmware uses more
  than 90% of available program flash, protecting OTA-update headroom.
- Self-hosted code coverage: a `coverage` CI job runs `gcovr` over the
  host-testable units, gates on a (currently low) line-coverage floor,
  publishes a live shields.io badge via an orphan `badges` branch, and
  posts a per-PR coverage summary comment. No third-party coverage service
  is used.
- `compile-sketch` and `arduino-lint` CI workflows for the Seeed XIAO
  nRF52840 Sense, plus status badges in the README.
- Per-module `.h` headers documenting each subsystem's public interface.

### Changed
- Sketch sources moved into the `BirdsEye/` subfolder so the folder name
  matches `BirdsEye.ino` (Arduino IDE / arduino-cli requirement).
- DOVEX header read/write now goes through a single tested
  `dovex_header::format()` / `parse()` implementation. On-disk format is
  unchanged (byte-for-byte compatible with existing `.dovex` files).

### Removed
- **(Breaking)** The legacy `ENABLE_NEW_UI` compile path and everything it
  gated: the manual track/location/direction selection menus, the
  `DovesLapTimer`-direct flow, legacy `.dove`/`.nmea` streamed replay, and
  the `use_legacy_csv` setting. The device now always uses the "Just Drive"
  auto-detect flow and DOVEX logging.

### Security
- BLE filename validation: every BLE command that carries a filename
  (`GET`, `DELETE`, `TGET`, `TPUT`, `TDEL`) is now checked for path
  traversal (`..`, leading `.`), path separators, and FAT-unsafe
  characters before any SD-card access. Rejected with `ERROR` /
  `NOT_FOUND` (file commands) or `TERR:BAD_NAME` (track commands).

## [1.0.0] - 2024

Initial tagged release. Core capabilities:

- 25 Hz GPS lap timing with optional 2/3-sector support (DovesLapTimer).
- "Just Drive" auto track/course detection via CourseManager, with a
  Lap Anything waypoint fallback.
- DOVEX crash-safe logging with a reserved 1 KB session header and
  instant on-device replay.
- RPM via inductive tachometer (Kalman-filtered), g-force via the onboard
  LSM6DS3 IMU.
- 8+ OLED display pages, Bluetooth LE file download / settings / track
  sync, and a low-power sleep mode.

[Unreleased]: https://github.com/TheAngryRaven/DovesDataLogger/compare/v2.2.0...HEAD
[2.2.0]: https://github.com/TheAngryRaven/DovesDataLogger/compare/v1.0.0...v2.2.0
[1.0.0]: https://github.com/TheAngryRaven/DovesDataLogger/releases/tag/v1.0.0
