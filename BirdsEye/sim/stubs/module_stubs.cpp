///////////////////////////////////////////
// Stubs for the firmware modules deliberately excluded from the sim TU:
// bluetooth.ino, camera_ble.ino, usb_msc.ino, firmware_ota.ino,
// sensoregg.ino.
//
// The sim has no BLE, no camera, no USB host (demo scope — see the
// handoff spec). These implement the excluded modules' public headers
// so the compiled modules (menus, main loop) link and behave sanely:
// every query reports "not present / not active", every action succeeds
// silently, and nothing prints warnings into the UX.
///////////////////////////////////////////

#include <Arduino.h>

#include "bluetooth.h"
#include "camera_ble.h"
#include "sensoregg.h"
#include "usb_msc.h"

// ---- bluetooth.ino surface ----

// Transfer-mode flags live in BirdsEye.ino (compiled); the stubs keep
// them false so the main loop never parks in the BLE branch.
extern bool bleActive;
extern bool bleConnected;

void bleCoreEnsureInit() {}
void bleApplyTransferAdvertising() {}
void bleAdvFinalizePadded() {}

void BLE_SETUP() {
  // Radio never comes up in the sim: the Bluetooth page renders its
  // "waiting for connection" state and the Exit button works, but no
  // peer can ever appear.
}

void BLE_STOP() {
  bleActive = false;
  bleConnected = false;
}

void BLUETOOTH_LOOP() {}

// ---- camera_ble.ino surface ----

void CAMERA_SETUP() {}
void CAMERA_LOOP() {}
void CAMERA_NOTIFY_SESSION_END() {}
void CAMERA_FORCE_RELEASE() {}
void CAMERA_SLEEP() {}

void cameraBleRegisterServices() {}
void cameraBleOnConnect(uint16_t) {}
void cameraBleOnDisconnect(uint16_t, uint8_t) {}
bool cameraBleOwnsConnHandle(uint16_t) { return false; }

bool cameraIsPaired() { return false; }
camera_fsm::State cameraFsmState() { return camera_fsm::State::kUnpaired; }
bool cameraRemoteLinkUp() { return false; }
bool cameraAdvertisingUp() { return false; }
bool cameraCe82Subscribed() { return false; }
bool cameraGpsStreaming() { return false; }
bool cameraObservedRecording() { return false; }
bool cameraRecordObservationFresh() { return false; }
bool cameraActivelyRecording() { return false; }
bool cameraConsumeAutoStop() { return false; }

bool cameraPairedSerial(char* buf, size_t bufSize) {
  if (buf && bufSize > 0) buf[0] = '\0';
  return false;
}

bool cameraRequestPair() { return false; }
void cameraCancelPair() {}
bool cameraRequestUnpair() { return true; }
bool cameraSetManualSerial(const char*) { return false; }

void cameraTestEnterMode() {}
void cameraTestExitMode() {}
bool cameraTestWake() { return false; }
bool cameraTestRecord() { return false; }
bool cameraTestPowerOff() { return false; }

// ---- sensoregg.ino surface ----

// No egg ever appears in the sim: the Temp1 page renders its '---'
// stale state and DOVEX rows log "nan" for both egg columns.

void SENSOREGG_SETUP() {}
void SENSOREGG_LOOP() {}

bool sensoreggLinkUp() { return false; }
float sensoreggEgtC() { return NAN; }
float sensoreggJunctionC() { return NAN; }
bool sensoreggTcFault() { return false; }
uint16_t sensoreggSequence() { return 0; }

// ---- usb_msc.ino surface ----

bool usbMscActive = false;

void USB_MSC_SETUP() {}

bool USB_MSC_ENABLE() {
  // Unreachable in practice: the menu checks isUsbConnected() first and
  // the sim's VBUS register always reads "no cable".
  return false;
}

void USB_MSC_DISABLE() {}
