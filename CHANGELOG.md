# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project aims to follow [Semantic Versioning](https://semver.org/spec/v2.0.0.html):

- **MAJOR** — a change that breaks existing track files, log file formats,
  the BLE command protocol, or removes a user-visible mode.
- **MINOR** — new features or device behavior that is backwards compatible.
- **PATCH** — bug fixes and internal changes with no user-visible behavior change.

## [Unreleased]

### Added
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
  as `FIRMWARE_VERSION` in `project.h` (starting at `2.0.0`).
- Release builds now cover **both XIAO nRF52840 variants** (Sense and
  non-Sense). The release workflow builds a matrix and publishes per-board
  `.hex` / `.uf2` / `.zip` assets named `BirdsEye-sense.*` and
  `BirdsEye-nonsense.*`; the `.zip` is the Secure DFU package used for OTA.
  `compile-sketch` CI also builds both variants.
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

[Unreleased]: https://github.com/TheAngryRaven/DovesDataLogger/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/TheAngryRaven/DovesDataLogger/releases/tag/v1.0.0
