#pragma once

#include "Arduino.h"

///////////////////////////////////////////
// LSM6DS3 IMU shim — sim build.
//
// begin() succeeds so the firmware takes the "Sense" path and logs
// accelerometer columns. Values are host-settable: the playback engine
// feeds the dovex accel_x/y/z trace in via simAccelSet() (wired up in
// Phase 3 of the sim plan); until then reads return a boring 0/0/1g.
///////////////////////////////////////////

#define I2C_MODE 0

class LSM6DS3 {
 public:
  struct Settings {
    int accelRange = 4;
  };

  LSM6DS3(int mode, uint8_t address) {
    (void)mode;
    (void)address;
  }

  int begin() { return 0; }  // 0 = success (matches SparkFun/Seeed core)

  float readFloatAccelX() { return simX; }
  float readFloatAccelY() { return simY; }
  float readFloatAccelZ() { return simZ; }

  Settings settings;

  // Host-side injection point (sim glue only).
  float simX = 0.0f;
  float simY = 0.0f;
  float simZ = 1.0f;  // resting flat: 1 g on Z
};
