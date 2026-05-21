#pragma once

///////////////////////////////////////////
// ACCELEROMETER MODULE
// Onboard LSM6DS3 6-axis IMU (XIAO nRF52840 Sense).
// Reads g-force X/Y/Z into the global accelX/Y/Z floats at 50 Hz.
// Graceful degradation when the IMU isn't present (non-Sense board):
// accelAvailable stays false and values stay at 0.0.
///////////////////////////////////////////

#include <LSM6DS3.h>

extern LSM6DS3 accelIMU;
extern bool   accelAvailable;
extern float  accelX;
extern float  accelY;
extern float  accelZ;

void ACCEL_SETUP();
void ACCEL_LOOP();
