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
- **GPS status boot page (MyChron-style).** Every boot now lands on a
  satellite status page after the splash instead of the main menu: the
  top half shows used/seen satellite count, HDOP, lock state,
  constellation mode and the configured + live nav rate plus battery;
  the bottom half draws one vertical signal bar per satellite (height =
  carrier-to-noise, from UBX-NAV-SAT). During the page the GPS runs a
  5 Hz status config; leaving it switches to the 25 Hz PVT-only race
  config. The page **holds but never locks**: any button skips it
  immediately, and once a stable lock (position fix + fully-resolved
  time, held 3 s) is acquired it auto-advances. It exits to the main
  menu — or straight into race mode with logging when the engine-start
  (tach) wake booted the device or the engine is running. Five minutes
  with no lock and no engine activity powers the device back down. A GPS
  that fails to initialize is re-probed up to 3 times in the background
  and surfaced as `NOT DETECTED` / `CHECK WIRING` instead of failing
  silently.

### Changed
- **Sleep is now a full shutdown (nRF52 System OFF).** Replacing the
  software sleep loop, the device tears everything down and powers off
  to µA-level System OFF; a tachometer pulse (engine start), any button,
  or plugging in USB wakes it with a fresh boot, making a power switch
  unnecessary. The engine-start wake boots through the GPS status page
  and auto-enters race mode; wake cause is decoded from the reset-reason
  and GPIO latch registers. **Charging is the one exception**: while USB
  power is present the device stays in a live charging loop (software
  holds the fast-charge pin) — screen on for 10 s then dark, any button
  fully wakes to the main menu, and unplugging powers off. USB connected
  on the main menu now enters the charging loop after 60 s of button
  inactivity (previously immediate sleep), so the device remains usable
  for replay/transfer while plugged in.
- **GPS boot connection hardening.** Boot now sends the u-blox
  backup-mode wake byte before probing (a module left in backup mode by
  the shutdown wakes correctly), probes the configured 57600 baud first
  (warm boots connect near-instantly; the old fixed 2.25 s boot delay is
  only paid when nothing answers), and validates that PVT data actually
  flows within 5 s of boot — recovering via the existing baud-recovery
  ladder if the module answered the probe but was silently misconfigured.

### Removed
- **24-hour periodic GPS fix during sleep** — nothing runs during System
  OFF to schedule it; the GPS warm-starts on wake instead.
- **RPM sleep-wake latch (`tachHavePeriod` / `TACH_SLEEP()`)** —
  superseded by the tach pin's GPIO SENSE wake from System OFF.

### Fixed
- **Boot hang on "Initializing..."** introduced by the GPS boot probe
  ladder: it called `Serial1.end()` before the port had ever been opened,
  and the nRF52 core's `Uart::end()` spin-waits on stop events a
  never-enabled UART can't produce — hanging `setup()` forever (before
  the watchdog is armed). Baud switches now go through a guard that only
  closes a port we actually opened.
- **Insta360 X4 camera auto-record (remote emulation).** The device
  emulates the physical Insta360 GPS Remote as a pure BLE **peripheral**
  to drive an X4 hands-free. Engine start (RPM > 500 held 2 s) wakes a
  paired, powered-off camera with the remote's manufacturer-data
  advertisement carrying the camera's serial — byte-for-byte the genuine
  remote's payload, **X4-confirmed** (captured with nRF Connect and
  replayed to wake a sleeping X4; flags `0x05`, serial at mfg[14..19]).
  The camera connects back to *us* and subscribes to our `ce82` button
  characteristic, and **all** control rides `ce82` notifications exactly
  like the real remote: recording toggles via the shutter button (the FSM
  tracks record state so a mid-session reconnect never blind-toggles), and
  power-off streams the remote's 3-second power hold. Recording starts once
  the camera is connected + subscribed and GPS locks (or after 30 s),
  stops after 60 s of *stationary AND engine-off* (grid idling and coasting
  stalls keep recording; a manual session end stops it immediately), and
  the camera powers off after a 3-minute cooldown. Pairing captures the
  camera's 6-character serial (new `camera_serial` setting; manual-entry
  fallback) from the new main-menu **Camera** page, which also has a bench
  **Test** menu (*Wake / Record / Power Off*) with live `R:UP+` / `Adv`
  status. The single BLE peripheral slot is shared with the file-transfer
  service via an explicit `bleOwner`, so a camera link can never trigger
  the transfer auto-reboot; the link is Just-Works bonded; the FSM is a
  read-only telemetry consumer (logs are byte-identical with the feature
  on or off) and BLE comes up lazily, so unpaired users pay nothing. All
  lifecycle timing lives in a host-tested pure FSM; the frame bytes in a
  host-tested protocol unit with golden tests. *Note:* an earlier design
  ran the BLE **central** role too (writing the camera's `be80`
  start/stop-video plus a 1 Hz GPS-overlay frame) — but one BLE link holds
  one role, and power-off exists only as a remote `ce82` hold, so central
  and remote could never coexist and power-off was impossible. Committing
  to the remote role fixes that. The in-camera GPS overlay rides the same
  remote channel after all: a Wireshark capture of the genuine
  remote↔camera link revealed the remote streams GPS on `ce82` at **10 Hz**
  as a (non-standard, signed-longitude) NMEA-RMC frame — so the firmware
  now streams that continuously while the camera is connected (it doubles
  as the remote's liveness heartbeat; status `V` with no fix, never
  silent), golden-tested byte-for-byte against the capture. **GPS still
  logs to SD** exactly as before, independently.
- **USB mass-storage file transfer.** The Transfer screen now opens a
  submenu offering **Bluetooth** (the existing BLE file-transfer flow,
  unchanged) or **USB**. Choosing USB presents the SD card to a connected
  computer as a standard drive (TinyUSB MSC) for drag-and-drop of track
  files and logs — no companion app required. The drive is opt-in: it only
  enumerates while on the USB page, so normal plug-in/charging is unchanged.
  Exiting USB mode reboots the device so the firmware remounts a clean
  filesystem after host edits. USB transfer holds the SD card exclusively
  via the access mutex (new `SD_ACCESS_USB_MSC` mode), so it cannot collide
  with logging, replay, or BLE.
- **`device_name` setting to label logs per device.** A new persistent
  setting identifies which unit produced a log — handy when dumping logs
  from a fleet. On first boot (or upgrade) a friendly default is generated
  by picking two of twelve racing-adjacent words at random (e.g.
  `ApexTurbo`); editable on a computer or over BLE like any other setting.
- **`device_name` recorded in the DOVEX header.** The session metadata line
  now carries the logging device's name as a new trailing column. The column
  is appended after `optimal_ms`, so older readers ignore it and the firmware
  reads pre-existing logs (no column) as an empty device name — fully
  backwards compatible with the reserved 1 KB header.

### Changed
- **Camera auto-record is now a simple, RPM-driven lifecycle.** Recording is
  no longer gated on a GPS lock and no longer stops on a speed+engine combo
  with a cooldown/power-off tail. Instead: the engine starting wakes+connects
  the camera; ~5 s of sustained RPM starts recording (one shutter); 30 s of
  engine-off (RPM only — a stationary but running grid idle keeps rolling)
  stops recording (one shutter) **and** ends+saves the race log session,
  returning to the menu. The camera then enters a **watching** state — it
  stays on and connected, so a brief on-track stall recovers straight back
  into recording when RPM returns — and powers off only when the device
  sleeps. GPS still streams to the camera the whole time (voided RMC until a
  lock, real data after). While the camera is recording, the speed-based log
  auto-idle yields to it; with no camera paired, logging is unchanged.
- **CI now controls which DovesLapTimer the firmware is built against.**
  Builds targeting (or running on) the `BETA` branch track the library's own
  `BETA` branch, so the two beta channels move together; `master` CI and the
  release/tag builds pin the known-good `v4.1.0` tag instead of floating on
  the library's default-branch tip. Bump the pin deliberately when a new
  library release is validated.

- **SD SPI clock raised during file transfers.** While a BLE or USB
  mass-storage transfer is active the SD card is clocked at 8 MHz (4x the
  normal 2 MHz), reverting to 2 MHz afterward. Transfers only happen parked
  with the motor off, so the ignition-EMI rationale for the slow clock doesn't
  apply — this lifts the USB drag-and-drop ceiling from ~250 KB/s. Falls back
  to 2 MHz automatically if the fast re-init fails.

### Fixed
- **Camera record state is now confirmed from the camera, not just believed.**
  The shutter is a stateful toggle, so a lost or mistimed frame used to
  *invert* our belief and stop a live recording (or start one in the paddock).
  A Wireshark capture of the genuine remote link showed the camera reports its
  state implicitly through its `0x10` display-string frame — a live
  `.HH:MM:SS` timer while recording, the mode/battery string when idle (the
  `0x02` status word is not reliable). The firmware now parses that timer and
  **reconciles** its record belief against it: on reconnect it adopts the
  camera's real state instead of blind-toggling, and if the camera reports
  idle while we think we're recording it re-asserts the shutter once. The
  belief is preserved (not cleared) whenever the camera is unreachable, so a
  dropped link mid-session can never invert on reconnect. The bench **Test**
  page gains a `rec:yes/no` state (the camera's own timer; `rec:--` when there
  is no fresh observation) and a `G:SYNC` / `G:V`
  GPS indicator so the link can be verified end-to-end. (Parser is golden-
  tested in the `insta360_protocol` unit; the reconcile in the `camera_fsm`
  unit.)
- **Camera now powers off on sleep instead of running all night.** Entering
  sleep armed the streamed power-off hold and then immediately disconnected —
  but the loop parks on sleep, so the hold was never serviced and *zero*
  power-off frames were sent. Sleep entry now streams the hold synchronously
  (feeding the watchdog) before dropping the link.
- **A nearby paired camera no longer reboots the logger out of a transfer
  session.** The radio has one address, so a bonded X4 can connect to the
  transfer advert and get routed as "the phone"; its sub-second drop hit the
  transfer auto-reboot, kicking the user out of Bluetooth/USB transfer
  repeatedly. The reboot is now gated on the peer actually having used the
  file/settings/OTA service — a camera that only vets our GATT and leaves is
  ignored. Relatedly, exiting Bluetooth no longer leaves a stale transfer
  advert on air (`restartOnDisconnect` is disarmed before the teardown
  disconnect), which had let a phone reconnect into a mute session and
  blocked camera auto-record until a power cycle.
- **A camera that connects but never subscribes no longer loops forever.**
  The ce82 subscribe-timeout re-advertised without counting attempts, so a
  stale bond (encryption never comes up) span connect→timeout→re-advertise
  indefinitely. It is now bounded — after a few cycles the FSM gives up back
  to IDLE.
- **Camera Pair/Test menus scroll the right way.** The new pages render a
  static top-to-bottom list like the main menu, but weren't in the
  direction-reversal set, so the first "down" press wrapped to the last item
  (once landing on Unpair). They now move with the buttons.
- **ce82 subscription is cached off the hot loop.** The subscription check was
  a SoftDevice round-trip called ~500×/s (per loop + per GPS tick); it is now
  latched from the CCCD callback (with a low-rate re-read to catch a bonded
  peer's silent sys-attr restore), keeping the 25 Hz logging loop clear.
- **USB mass-storage exit no longer risks truncating a host write.** Exiting
  USB mode syncs the card and reboots, but `setUnitReady(false)` only stops
  *new* SCSI commands — a `WRITE10` already in flight keeps calling the write
  block callback on the USB task. The exit now waits (bounded to 1 s) for the
  write callback to go quiet before syncing and resetting, so a reset can't
  cut an in-progress `writeSectors()` and leave a truncated file or an
  inconsistent FAT.
- **Selecting USB storage with no cable plugged in no longer reboots the
  device.** `USB_MSC_ENABLE()` had no VBUS precondition and the parked USB
  loop reads "VBUS absent" as "cable pulled" on its first iteration, so
  choosing USB before plugging in instantly reset the unit. USB mode now
  requires VBUS: the menu shows "Plug in USB cable first!" instead of
  entering (and the enable path bails defensively too, taking no SD lock and
  not enumerating).
- **Camera Power Off now streams the held-button frame — decoded from a
  live remote capture.** A genuine remote's ce82 traffic (nRF Connect,
  2026-07-10) showed two things we had wrong: the button frame's byte[4]
  is a running sequence counter the remote steps by 2 with every frame
  (we hardcoded `0x00`), and **power-off is a held button** — the remote
  streams the 3-second-hold frame continuously (fresh seq each) until the
  camera powers off, rather than sending one frame. We now stream the
  hold non-blockingly for ~3.5 s (or until the camera drops the link,
  which is it powering off), with a per-frame sequence counter. ce82 is
  also now NOTIFY-only, matching the captured remote's GATT. (The
  captured shutter/mode/power-tap frames confirm our other button byte
  patterns exactly.)
- **Wake advert matched byte-for-byte to the genuine GPS Remote —
  X4-confirmed ground truth.** A real remote was captured with nRF
  Connect against the camera, and replaying its advertisement from a
  phone woke the sleeping X4 — making the packet authoritative rather
  than reference-derived. Our transmission differed by exactly one byte:
  the Flags AD is `0x05` (LE Limited Discoverable + BR/EDR Not
  Supported), not the `0x1A` inherited from the ESP32 replication. The
  scan response now also matches the real remote (Appearance 0x0180
  "remote control" + name — previously name only), and the paired
  camera-mode connectable advert is unified onto this same
  remote-identity packet (it is the only advert a genuine remote ever
  transmits, and what the camera's reconnect scan expects) — the
  invented name+service advert is now used only for the unpaired pairing
  flow, where no serial exists yet for the manufacturer block.
- **Camera R-link diagnosis + bench advert re-arm.** The camera has been
  observed connecting to the remote service intermittently — sometimes
  dropping faster than the display refresh, invisibly. R-link connects
  and disconnects (with the HCI reason code, which says who ended the
  link and why) are now traced to serial, and in the bench-test menu the
  connectable remote advert automatically re-arms whenever the R-link is
  down — the camera retries on its own schedule and could previously
  only reconnect if the tester pressed Connect at the right moment.
- **Camera Power Off no longer fails silently — ce82 sends honor the
  camera's subscription and report failures.** The power-off button frame
  was a fire-and-forget notify behind two invisible gates: it returned
  silently with no remote link, and a notification to a camera that never
  wrote the ce82 CCCD (or subscribed for *indications* rather than
  notifications) was discarded by the stack while the UI showed nothing.
  ce82 sends now pick notify vs indicate from the actual subscription,
  return a real result, and the bench-test menu shows *why* a Power Off
  failed ("run Connect first" / "camera not subscribed" / "rejected by
  stack"). The test page's R status gains a `+` when the camera has
  subscribed to buttons (`R:UP+` = deliverable; `R:UP` = connected but
  ignoring us), and ce82 CCCD writes are traced to serial for bench
  diagnosis.
- **Camera wake advert was garbled on air by a BLE core bug — all adverts
  now transmit as fixed 31-byte packets.** Bluefruit 0.21.0 (the version
  in the Seeed board package; fixed upstream in Adafruit 1.7.0) freezes
  the advertising packet lengths at whatever the FIRST advert of the
  boot carried (`_start()` uses a function-local static), so any later
  advert of a different length went out truncated or with a stale tail —
  a malformed PDU every receiver silently discards while the API reports
  success. In practice: after the 28-byte connect advert ran once, the
  31-byte Insta360 wake advert lost its last 3 bytes on air and could
  never wake the camera. Every advert (transfer, camera wake, camera
  connectable) is now zero-padded to exactly 31+31 bytes (spec-legal)
  via a shared `bleAdvFinalizePadded()` before start, making the frozen
  length always correct. The wake path also skips (with a debug note)
  when the camera already holds the peripheral slot, instead of failing
  the connectable start with `NRF_ERROR_CONN_COUNT`. *Camera-side note
  from research:* on the X4, Bluetooth Wakeup is armed only when
  **QuickCapture is OFF** — with QuickCapture on, the powered-down
  camera's radio never scans and no beacon can wake it.
- **Main menu scrolls — the Camera entry is reachable again.** The 4-item
  main menu was drawn as four full-height rows filling the panel's nominal
  64 px exactly, and the last row (Camera) was cut off on real hardware.
  The menu now shows a 3-row window that follows the selection (same
  pattern as the replay file list) with a `v more` scroll hint on the
  spare bottom line.
- **I2C bus recovery no longer risks a reboot loop.** `i2cBusRecover()`
  re-inits Wire and the OLED over I2C, which can block if ignition EMI is
  still glitching the bus — and it never fed the watchdog, so the recovery
  routine itself could trip the 4 s WDT and reset, then re-trigger on the
  next boot. It now pets the watchdog before each blocking re-init step
  (matching the GPS baud-recovery hardening), and `safeDisplayUpdate()`
  recovers a hung bus immediately instead of waiting one more (also-slow)
  frame.
- **Logging init aborts cleanly if the SD card drops mid-header.** The DOVEX
  1 KB header pre-fill ignored each `write()` return, so a card dropping
  sectors during log creation could leave a truncated header region while
  the firmware still marked logging ready and streamed rows into it. The
  pre-fill now verifies every write and aborts the open (retrying next
  second) on a short write.
- **A mid-session SD write failure now tries to save the lap times.** When a
  data-row write fails (typical of a failing card/tray), logging stops
  cleanly as before — but the reserved header is now written first, so the
  session's lap list and best/optimal times survive instead of being lost
  with the session.
- **`buildTrackList()` now arbitrates SD access.** The track-list/manifest
  rebuild walked the `/TRACKS` directory with raw SdFat calls and no access
  mutex, the lone SD consumer that didn't — leaving a window where a BLE
  track upload/delete completing during a logging teardown could touch SdFat
  from two tasks. It now holds `SD_ACCESS_TRACK_PARSE` for the walk and
  checks the directory open.
- **USB mass-storage no longer races the host for the SD card.** While the
  drive was mounted the main loop kept running `GPS_LOOP()`,
  `trackDetectionLoop()`, auto-idle, etc., so the firmware and the host PC
  could both drive the FAT — corruption was prevented only as a side effect
  of the access mutex denying those acquires, not by design. The loop now
  parks while `usbMscActive` (mirroring the BLE branch): GPS/tach/lap/SD
  processing is skipped entirely, only the Exit button and the status page
  are serviced, so the host owns the card uncontested.
- **Unplugging the USB cable now exits mass-storage mode.** Previously the
  only way out was the on-device Exit button; pulling the cable (the natural
  way to "finish") left the device stuck `usbMscActive` — holding the
  `SD_ACCESS_USB_MSC` lock and the EMI-unsafe 8 MHz SD clock indefinitely,
  so a subsequent drive would silently fail to log. The parked loop now
  watches VBUS and reboots out of USB mode when the cable is removed.
- **Track detection no longer throttles the whole main loop.** The manifest
  scan (O(N) software-double haversine, several ms at the 200-entry ceiling)
  was gated only on `gpsData.fix`, which stays true between PVT updates — so
  it ran every ~250 Hz loop iteration instead of "every GPS fix" as
  documented, dragging the loop rate down while hunting for a track. The
  scan is now throttled to 1 Hz, which is still instant at driving pace.
- **Tach ring buffer can no longer be lapped during SD stalls.** The pulse
  ISR advanced the ring head unconditionally; an SD GC stall (the documented
  100 ms–2 s, the same reason the GPS serial ring exists) at racing RPM
  overruns the 16-entry buffer, breaking the ring invariant and producing a
  confident-but-wrong RPM that was logged to CSV and fed the >500 RPM
  auto-race trigger. The ISR now checks full and drops the pulse instead,
  flagging the gap so `TACH_LOOP()` discards the one period spanning it —
  the Kalman estimate coasts briefly rather than going silently wrong. The
  filter math itself moved to the host-tested `tach_filter` pure unit.
- **GPS baud recovery no longer risks tripping the 4 s hardware watchdog.**
  `GPS_BAUD_RECOVERY()` can block for up to three ~1.1 s module probes plus
  ~500 ms of delays — against a genuinely hung GPS that out-waited the WDT,
  causing a reset → re-setup → recovery → reset boot loop. It now pets the
  watchdog before each blocking probe (same treatment `fwStageToFlash()`
  already had).
- **Sleep mode actually sleeps now.** The tach ISR latched `tachHavePeriod`
  on the first engine pulse since boot and nothing ever cleared it, so every
  sleep entry (long-press, 5-min menu idle, USB) instantly bounced through
  the RPM-wake path back into race mode **with logging enabled** — the
  device would silently start a new session and drain the pack overnight.
  `enterSleepMode()` now calls the new `TACH_SLEEP()`, which re-arms the
  wake trigger and drops stale ring-buffer/Kalman state; the next *real*
  pulse still RPM-wakes straight into race mode as designed.
- **Lap times under 100 ms-fraction rendered wrong on the live pages.** The
  current-lap, best-lap, and optimal-lap pages appended the millisecond
  zero-padding *after* the value, so `1:23.007` displayed as `1:23.700` (a
  693 ms error), and the lap-history list did no padding at all (`1:5.7`).
  All six divergent inline copies of the ms → `M:SS.mmm` math are replaced
  by one host-tested `lap_format` unit; the replay page's already-correct
  rendering is unchanged, and the lap list now shows zero-padded
  `M:SS.mmm`. (Cosmetic side effect: once a lap passes one minute the
  big-font live pages now zero-pad the seconds — `1:05.007` rather than the
  old `1: 5.007` — same field width, so the column stays stable.)

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
