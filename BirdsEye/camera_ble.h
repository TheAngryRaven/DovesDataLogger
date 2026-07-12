#pragma once

#include <stdint.h>
#include <stddef.h>

#include "camera_fsm.h"

///////////////////////////////////////////
// CAMERA BLE MODULE  (Insta360 X4 auto-record — remote emulation)
//
// This device IS the Insta360 GPS Remote. We are a BLE PERIPHERAL only:
// we host the remote's GATT (service 0xCE80: ce81 WRITE camera->us state,
// ce82 NOTIFY us->camera button frames, ce83 READ static), advertise the
// wake/identity payload, and the camera connects to US as central and
// subscribes to ce82. ALL control — record (shutter toggle) and power-off
// — is a ce82 notification, byte-for-byte the physical remote's frames.
//
// We never act as central: no scanning, no connecting to the camera's own
// be80 service, no be81 writes. (The in-camera GPS overlay used the be81
// central path; its true transport is unidentified, so it is dropped here.
// GPS still logs to SD independently.)
//
// All camera behavior is decided by the host-tested camera_fsm pure unit;
// this module builds the Inputs snapshot, executes the returned Action,
// and owns the Bluefruit plumbing. Frame bytes come from the host-tested
// insta360_protocol pure unit.
//
// THREADING: Bluefruit callbacks (connect/disconnect/ce81 writes/ce82
// CCCD writes) only copy into RAM and set volatile flags — never SD,
// never blocking Bluefruit calls. CAMERA_LOOP() on the main loop consumes
// the flags, steps the FSM, and does all real work (including the one
// setSetting() that persists a captured serial, and streaming the
// power-off hold).
//
// OWNERSHIP: shares the single peripheral slot + advert set with the
// file-transfer service via bleOwner (see bluetooth.h). Camera mode never
// sets bleActive — the main loop keeps running race processing while the
// camera is linked. The transfer page takes the radio over via
// CAMERA_FORCE_RELEASE().
//
// BONDING: the genuine remote link is encrypted + bonded. We support
// Just-Works pairing (NoInputNoOutput) as peripheral so the camera can
// bring up encryption; the camera may withhold its ce82 subscription
// until the link is encrypted.
///////////////////////////////////////////

// ---- lifecycle (called from BirdsEye.ino) ----

// Load the persisted camera serial ("camera_serial" setting, 6 ASCII
// chars, empty = unpaired) and init the FSM. No BLE init here — the
// SoftDevice comes up lazily on the first advertising action, so unpaired
// users pay zero RAM/power cost. Call after SETTINGS_SETUP().
void CAMERA_SETUP();

// Step the FSM and execute its action; stream an in-flight power-off hold.
// Call from the normal main-loop path (after GPS_LOOP/TACH_LOOP so
// telemetry is fresh). Not called in the bleActive / usbMscActive parking
// branches or sleep — intentional.
void CAMERA_LOOP();

// Session-end hook: called from the MANUAL logging-stop confirm (the
// user's explicit "I'm done") — deliberately NOT from auto-idle, which
// ends the log on speed alone and must not cut camera footage during a
// grid idle. Queues the one-shot sessionEndRequested event for the next
// CAMERA_LOOP() step. Sleep entry uses CAMERA_SLEEP() instead.
void CAMERA_NOTIFY_SESSION_END();

// Transfer takeover: best-effort stop recording if active, drop the camera
// link, stop camera-owned advertising, force the FSM to IDLE, and release
// bleOwner. Called before BLE_SETUP() when the user opens the Bluetooth
// transfer page (and the USB page).
void CAMERA_FORCE_RELEASE();

// Sleep hook: best-effort power-off if the camera is connected, then the
// same teardown as CAMERA_FORCE_RELEASE(). Called from enterSleepMode()
// before BLE_STOP().
void CAMERA_SLEEP();

// ---- BLE core integration (called from bluetooth.ino) ----

// Register the peripheral camera GATT (ce80 service). Called exactly once
// from bleCoreEnsureInit(), before advertising ever starts.
void cameraBleRegisterServices();

// Peripheral connect/disconnect routing for the camera link.
// Bluefruit-task context: set volatile flags only.
void cameraBleOnConnect(uint16_t connHandle);
void cameraBleOnDisconnect(uint16_t connHandle, uint8_t reason);

// True while connHandle is the camera's remote-service link. The shared
// disconnect callback checks this BEFORE the bleOwner test: a
// teardown-initiated camera disconnect completes asynchronously, so its
// event can arrive after ownership has already moved to NONE/TRANSFER —
// routing that event by owner would misdeliver it to the transfer
// teardown (whose auto-reboot must never fire for camera links).
bool cameraBleOwnsConnHandle(uint16_t connHandle);

// ---- UI surface (called from display_ui.ino / display_pages.ino) ----

// True when a camera serial is stored (FSM armed).
bool cameraIsPaired();

// Current FSM state, for the pairing/status pages (render with
// camera_fsm::stateName()).
camera_fsm::State cameraFsmState();

// True while the camera is connected to our remote service.
bool cameraRemoteLinkUp();

// True while OUR advertising is actually on air — for the bench-test
// status page. An advert whose SoftDevice config was rejected fails
// silently otherwise (the "no blue LED" symptom).
bool cameraAdvertisingUp();

// True once the camera has subscribed (CCCD write) to our ce82 button
// characteristic — button frames (record, power-off) are only deliverable
// when this is true. Shown as the "+" after R:UP on the bench-test page:
// R:UP without "+" = camera connected but ignoring our buttons.
bool cameraCe82Subscribed();

// True while we are actively streaming the 10 Hz GPS/RMC feed to the camera
// (connected + subscribed + not paused by a power-off hold). The bench-test
// page shows this as G:SYNC / G:V (see the fix status) so the GPS link can
// be 100% verified. GPS is streamed regardless of fix (voided RMC when no
// lock) — this reflects that frames are actually going out.
bool cameraGpsStreaming();

// True when the camera's own 0x10 display string reports it is RECORDING
// (a fresh ".HH:MM:SS" timer). Lets the bench page confirm the shutter took.
bool cameraObservedRecording();

// True when we have ANY fresh 0x10 observation (recording OR idle). Lets the
// bench page show `rec:--` (no data yet) distinctly from `rec:no` (the camera
// actively reports idle), so a missing signal isn't misread as "not recording".
bool cameraRecordObservationFresh();

// True while the camera FSM is in RECORDING. `checkAutoIdle()` reads this so the
// speed-based log-idle yields to an active recording (the camera owns the end).
bool cameraActivelyRecording();

// Returns true ONCE after the camera auto-stops recording on the 30 s engine-off
// condition (clears the latch). The main sketch calls `endRaceSession()` +
// returns to the menu in response. A manual logging-stop does NOT set this (that
// path already ends the session).
bool cameraConsumeAutoStop();

// Copy the stored 6-char serial into buf (NUL-terminated; bufSize >= 7).
// Returns false (buf = "") when unpaired.
bool cameraPairedSerial(char* buf, size_t bufSize);

// Enter pairing (honored only from UNPAIRED/IDLE — returns false and does
// nothing otherwise, e.g. mid-cooldown).
bool cameraRequestPair();

// Leave the pairing screen without a capture.
void cameraCancelPair();

// Clear the stored serial and disarm (honored only from UNPAIRED/IDLE;
// returns false when the FSM is busy with a session).
bool cameraRequestUnpair();

// Manual pairing fallback: validate a user-entered 6-char serial
// (A-Z, 0-9), persist it, and re-arm the FSM. Returns false on an invalid
// serial or a failed settings write.
bool cameraSetManualSerial(const char* serial6);

// ---- Bench test menu (PAGE_CAMERA_TEST, paired only) ----
// Manual camera controls for bench testing, where staging RPM/GPS to drive
// the auto-record FSM is impractical. cameraTestEnterMode() forces the FSM
// to IDLE and suppresses it (so auto-record can't fight the manual
// actions); cameraTestExitMode() stops any recording, drops the link, and
// releases the radio. The action helpers reuse the exact FSM action code
// paths, so wire behavior matches a real session.

void cameraTestEnterMode();
void cameraTestExitMode();

// Present the wake / remote-identity advert so a standby camera wakes and
// an on camera connects back to us. Returns true (advert started).
bool cameraTestWake();
// ce82 shutter button — TOGGLES recording. Returns true if the frame could
// reach the camera (connected + subscribed to ce82).
bool cameraTestRecord();
// ce82 power-off (streamed 3s hold). Returns true if it could reach the
// camera (connected + subscribed).
bool cameraTestPowerOff();
