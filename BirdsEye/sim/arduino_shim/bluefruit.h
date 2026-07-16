#pragma once

#include "Arduino.h"

///////////////////////////////////////////
// Bluefruit shim — sim build.
//
// bluetooth.ino / camera_ble.ino / firmware_ota.ino are NOT compiled
// into the sim (no BLE in the demo scope). BirdsEye.ino still declares
// the service/characteristic globals at file scope, so the types must
// construct; nothing ever calls into them.
///////////////////////////////////////////

class BLEService {
 public:
  explicit BLEService(uint16_t uuid) { (void)uuid; }
};

class BLECharacteristic {
 public:
  explicit BLECharacteristic(uint16_t uuid) { (void)uuid; }
};

class BLEDfu {};
class BLEDis {};
