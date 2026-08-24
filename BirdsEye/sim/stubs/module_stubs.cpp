///////////////////////////////////////////
// Stubs for the firmware modules deliberately excluded from the sim TU:
// bluetooth.ino, camera_ble.ino, usb_msc.ino, firmware_ota.ino,
// sensoregg.ino, neopixel.ino, profiling.ino.
//
// The sim has no BLE, no camera, no USB host, no LED strip (demo scope
// — see the handoff spec). These implement the excluded modules' public headers
// so the compiled modules (menus, main loop) link and behave sanely:
// every query reports "not present / not active", every action succeeds
// silently, and nothing prints warnings into the UX.
///////////////////////////////////////////

#define SIM 1  // project.h (via bluetooth.h) gates firmware-only build asserts on this

#include <Arduino.h>

#include "bluetooth.h"
#include "camera_ble.h"
#include "neopixel.h"
#include "profiling.h"
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

// On hardware this reboots and never returns; the sim just stops the stub
// radio and returns so the Bluetooth page's Exit continues to the menu
// (the golden walk exits this page mid-script).
void bleExitTransferMode() {
  BLE_STOP();
}

void bleConnLedOff() {}
void bleShutdownQuiesce() {}

void BLUETOOTH_LOOP() {}

// Transfer diagnostics the Bluetooth page prints. No radio in the sim, so
// the page renders its idle "waiting" state and these are never meaningful —
// they exist so display_pages.ino compiles unchanged.
uint32_t bleTransferRateBps() { return 0; }
uint16_t bleLinkDataLength() { return 27; }
uint16_t bleLinkChunkSize() { return 20; }
uint16_t bleLinkIntervalUnits() { return 0; }
uint8_t bleLinkPhy() { return 0; }

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
void SENSOREGG_SLEEP() {}
void SENSOREGG_WAKE() {}

bool sensoreggLinkUp() { return false; }
bool sensoreggAppHung() { return false; }
float sensoreggEgtC() { return NAN; }
float sensoreggJunctionC() { return NAN; }
float sensoreggAuxC() { return NAN; }
uint8_t sensoreggBatteryPct() { return 0xFF; }
bool sensoreggTcFault() { return false; }
uint16_t sensoreggSequence() { return 0; }

// ---- neopixel.ino surface ----

// No LED strip in the sim (and no UICR to program). The pure units
// (led_frame/led_modes/led_animations/sector_purple) still build into
// the sim via SIM_CORE_SOURCES for any future host harness use.

void NEOPIXEL_SETUP() {}
void NEOPIXEL_LOOP() {}
void NEOPIXEL_SLEEP() {}
void NEOPIXEL_WAKE() {}
void neopixelNotifyPurpleSector() {}

// ---- profiling.ino surface ----

// BIRDSEYE_ENABLE_PROFILING is a beta-firmware flag and is never set for
// the sim: there is no pin to toggle and host wall-clock timings of a
// virtual-clock loop would mean nothing. These are the flag-off stubs
// the real profiling.ino would provide; the pure loop_profile unit still
// builds into the sim via SIM_CORE_SOURCES.

void PROFILING_SETUP() {}
void PROFILING_SLEEP() {}

// ---- usb_msc.ino surface ----

bool usbMscActive = false;

void USB_MSC_SETUP() {}

bool USB_MSC_ENABLE() {
  // Unreachable in practice: the menu checks isUsbConnected() first and
  // the sim's VBUS register always reads "no cable".
  return false;
}

void USB_MSC_DISABLE() {}
