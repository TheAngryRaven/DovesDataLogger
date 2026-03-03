///////////////////////////////////////////
// ACCELEROMETER MODULE
// LSM6DS3 6-axis IMU on XIAO nRF52840 Sense (onboard, I2C address 0x6A)
// Reads accelerometer X/Y/Z in g-force for CSV logging
// Gyroscope data is available but not used in this version
///////////////////////////////////////////

static unsigned long accelLastReadMs = 0;
static const unsigned long ACCEL_READ_INTERVAL_MS = 20;  // 50 Hz — plenty for motorsports logging

/**
 * Accelerometer setup - initialize the onboard LSM6DS3 IMU
 *
 * Configured for ±16g range to capture hard braking and crash events
 * without clipping. Resolution at 16g is 0.488 mg/LSB — still far
 * finer than needed for motorsports data.
 *
 * Graceful degradation: if the IMU is not present (non-Sense board
 * or hardware fault), accelAvailable stays false and accel values
 * remain at 0.0 — logged as "0.000" in CSV with no other side effects.
 */
void ACCEL_SETUP() {
  accelIMU.settings.accelRange = 16;  // ±16g (captures crashes without clipping)

  if (accelIMU.begin() != 0) {
    debugln(F("Accelerometer not detected (non-Sense board?)"));
    accelAvailable = false;
    return;
  }

  accelAvailable = true;
  debugln(F("Accelerometer ready (LSM6DS3, 16g range)"));
}

/**
 * Accelerometer main loop - read raw g-force values
 *
 * Rate-limited to 50 Hz to avoid hammering the I2C bus every loop
 * iteration (~250 Hz). 50 Hz is 2x the GPS logging rate and more
 * than sufficient for motorsports g-force analysis.
 *
 * No filtering applied — raw g-force is the standard unit for
 * motorsports data analysis (lateral G, braking G, etc.).
 */
void ACCEL_LOOP() {
  if (!accelAvailable) return;

  unsigned long now = millis();
  if (now - accelLastReadMs < ACCEL_READ_INTERVAL_MS) return;
  accelLastReadMs = now;

  accelX = accelIMU.readFloatAccelX();
  accelY = accelIMU.readFloatAccelY();
  accelZ = accelIMU.readFloatAccelZ();
}
