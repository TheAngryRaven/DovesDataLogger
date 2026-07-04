#pragma once

#include <stdint.h>
#include <stddef.h>

#include "camera_fsm.h"

///////////////////////////////////////////
// CAMERA BLE MODULE  (Insta360 X4 auto-record)
//
// Impersonates the Insta360 "GPS Action Remote" so a paired X4 can be
// woken, recorded, GPS-overlaid, and powered off fully hands-free.
// Dual BLE role on the one SoftDevice:
//
//   PERIPHERAL — we host the remote's GATT (service 0xCE80: ce81 WRITE
//   camera->us carrying its serial + status frames, ce82 NOTIFY us->camera
//   button frames, ce83 static; plus the D0FF secondary service the real
//   remote exposes). The camera pairs against this and connects to us
//   after a wake advert (31-byte mfg-data burst carrying its 6-char
//   ASCII serial).
//
//   CENTRAL — we connect to the camera's own 0xBE80 service for
//   deterministic start/stop-video, keep-alive, and the 1 Hz GPS
//   telemetry frame (be81 write / be82 notify).
//
// All camera behavior is decided by the host-tested camera_fsm pure
// unit; this module only builds the Inputs snapshot, executes the
// returned Action, and owns the Bluefruit plumbing. Frame bytes come
// from the host-tested insta360_protocol pure unit.
//
// THREADING: Bluefruit callbacks (connect/disconnect/scan/ce81 writes/
// be82 notifies) only copy into RAM and set volatile flags — never SD,
// never Bluefruit API calls that block. CAMERA_LOOP() on the main loop
// consumes the flags, steps the FSM, and does all real work (including
// the one setSetting() that persists a captured serial).
//
// OWNERSHIP: shares the single peripheral slot + advert set with the
// file-transfer service via bleOwner (see bluetooth.h). Camera mode
// never sets bleActive — the main loop keeps running race processing
// while the camera is linked. The transfer page takes the radio over
// via CAMERA_FORCE_RELEASE().
///////////////////////////////////////////

// ---- lifecycle (called from BirdsEye.ino) ----

// Load the persisted camera serial ("camera_serial" setting, 6 ASCII
// chars, empty = unpaired) and init the FSM. No BLE init here — the
// SoftDevice comes up lazily on the first advertising action, so
// unpaired users pay zero RAM/power cost. Call after SETTINGS_SETUP().
void CAMERA_SETUP();

// Step the FSM and execute its action. Call from the normal main-loop
// path (after GPS_LOOP/TACH_LOOP so telemetry is fresh). Not called in
// the bleActive / usbMscActive parking branches or sleep — intentional.
void CAMERA_LOOP();

// Session-end hook: called at the top of endRaceSession(). Queues the
// one-shot sessionEndRequested event for the next CAMERA_LOOP() step
// (stops recording immediately / powers off from cooldown).
void CAMERA_NOTIFY_SESSION_END();

// Transfer takeover: best-effort stop-video if recording, drop both
// camera links, stop camera-owned advertising, force the FSM to IDLE,
// and release bleOwner. Called before BLE_SETUP() when the user opens
// the Bluetooth transfer page.
void CAMERA_FORCE_RELEASE();

// Sleep hook: best-effort power-off if a camera is connected, then the
// same teardown as CAMERA_FORCE_RELEASE(). Called from enterSleepMode()
// before BLE_STOP().
void CAMERA_SLEEP();

// ---- BLE core integration (called from bluetooth.ino) ----

// Register the peripheral camera GATT (ce80 + D0FF services) and the
// central client objects (be80/be81/be82). Called exactly once from
// bleCoreEnsureInit(), before advertising ever starts.
void cameraBleRegisterServices();

// Peripheral connect/disconnect routing for camera-owned links.
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

// True while the camera is connected to our remote service — lets the
// pairing page show "Connected — reading serial...".
bool cameraRemoteLinkUp();

// Copy the stored 6-char serial into buf (NUL-terminated; bufSize >= 7).
// Returns false (buf = "") when unpaired.
bool cameraPairedSerial(char* buf, size_t bufSize);

// Enter pairing (honored only from UNPAIRED/IDLE — returns false and
// does nothing otherwise, e.g. mid-cooldown).
bool cameraRequestPair();

// Leave the pairing screen without a capture.
void cameraCancelPair();

// Clear the stored serial and disarm (honored only from UNPAIRED/IDLE;
// returns false when the FSM is busy with a session).
bool cameraRequestUnpair();

// Manual pairing fallback: validate a user-entered 6-char serial
// (A-Z, 0-9), persist it, and re-arm the FSM. Returns false on an
// invalid serial or a failed settings write.
bool cameraSetManualSerial(const char* serial6);
