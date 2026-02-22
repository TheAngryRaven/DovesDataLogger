///////////////////////////////////////////
// TACHOMETER MODULE
// ISR and main loop processing for tachometer input
///////////////////////////////////////////

/**
 * Tachometer ISR - called on rising edge of tach signal
 *
 * CRITICAL: This ISR must be fast and must NOT call noInterrupts().
 * Calling noInterrupts() here would disable ALL system interrupts,
 * and if the main loop is delayed (e.g., SD write), interrupts would
 * stay disabled forever, freezing button input and causing system lockup.
 *
 * Instead, we use a volatile flag gate (tachInterruptShouldProcess) that
 * the main loop re-enables after the debounce period.
 */
void TACH_COUNT_PULSE() {
  // Exit immediately if we're in the debounce window
  if (!tachInterruptShouldProcess) return;

  uint32_t now = micros();
  uint32_t dt = now - tachLastPulseUs;

  // Secondary debounce check (belt and suspenders with the flag)
  if (dt < tachMinPulseGapUs) return;

  // Record this pulse
  tachLastPulseUs = now;
  tachLastPeriodUs = dt;
  tachHavePeriod = true;

  // Gate off further interrupts until main loop re-enables
  // DO NOT call noInterrupts() here - that causes system-wide deadlock!
  tachInterruptShouldProcess = false;
}

/**
 * Tachometer main loop processing
 *
 * Re-enables the ISR gate after debounce period, reads pulse data,
 * applies exponential moving average filter, and handles timeout.
 */
void TACH_LOOP() {
  // Re-enable interrupt processing after debounce window expires
  if (!tachInterruptShouldProcess) {
    uint32_t elapsed = micros() - tachLastPulseUs;
    if (elapsed >= tachMinPulseGapUs) {
      tachInterruptShouldProcess = true;
      // Note: We don't call interrupts() here anymore - they were never disabled!
    }
  }

  // CRITICAL SECTION: Read-modify-clear of ISR shared data
  // This DOES need noInterrupts() because:
  // 1. Read tachHavePeriod
  // 2. Read tachLastPeriodUs
  // 3. Clear tachHavePeriod
  // Without protection, ISR could fire between 2 and 3, writing a new
  // period value that we'd then lose when we clear the flag.
  uint32_t periodUs = 0;
  bool havePeriod = false;

  noInterrupts();
  havePeriod = tachHavePeriod;
  if (havePeriod) {
    periodUs = tachLastPeriodUs;
    tachHavePeriod = false;
  }
  interrupts();

  // Apply exponential moving average filter to smooth RPM
  if (havePeriod && periodUs > 0) {
    float rpmInst = (60.0e6f * tachRevsPerPulse) / (float)periodUs;
    tachRpmFiltered += tachFilterAlpha * (rpmInst - tachRpmFiltered);
  }

  // Timeout: if no pulses for tachStopTimeoutUs, engine is stopped
  // Note: 32-bit reads are atomic on ARM Cortex-M4, no noInterrupts() needed
  uint32_t lastPulseUs = tachLastPulseUs;

  if ((uint32_t)(micros() - lastPulseUs) > tachStopTimeoutUs) {
    tachRpmFiltered = 0.0f;
  }

  // Update display/log value at configured rate
  if (millis() - tachLastUpdate > (1000 / tachUpdateRateHz)) {
    tachLastUpdate = millis();
    tachLastReported = (int)(tachRpmFiltered + 0.5f);
  }
}
