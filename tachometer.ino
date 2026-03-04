///////////////////////////////////////////
// TACHOMETER MODULE
// ISR and main loop processing for tachometer input
///////////////////////////////////////////

// After engine-stopped timeout, the first pulse period is garbage (it's the
// time since the last pulse before stopping, not a real RPM measurement).
// This flag tells TACH_LOOP to discard that first period.
static volatile bool tachNeedFirstPulseDiscard = true;

// Median-of-3 filter: rejects single-pulse noise spikes from ignition
// ringing that slip past the debounce window. Two out of three readings
// must agree for a value to pass through to the EMA.
static uint32_t tachPeriodBuf[3];
static uint8_t tachPeriodIdx = 0;
static uint8_t tachPeriodCount = 0;

/**
 * Tachometer ISR - called on falling edge of tach signal
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

  // Apply median-of-3 filter then EMA to smooth RPM
  if (havePeriod && periodUs > 0) {
    // Discard first period after engine-stopped state. That period is
    // the gap since the last pulse before stopping — not a real RPM.
    if (tachNeedFirstPulseDiscard) {
      tachNeedFirstPulseDiscard = false;
      // Still update tachLastPulseUs (already done in ISR) so the NEXT
      // period is measured from this pulse. Just don't feed filters.
    } else {
      // Feed period into median-of-3 ring buffer
      tachPeriodBuf[tachPeriodIdx] = periodUs;
      tachPeriodIdx = (tachPeriodIdx + 1) % 3;
      if (tachPeriodCount < 3) tachPeriodCount++;

      // Need at least 3 samples for median
      if (tachPeriodCount >= 3) {
        // Median of 3 values (branchless-ish sort)
        uint32_t a = tachPeriodBuf[0], b = tachPeriodBuf[1], c = tachPeriodBuf[2];
        uint32_t median;
        if (a <= b) {
          median = (b <= c) ? b : ((a <= c) ? c : a);
        } else {
          median = (a <= c) ? a : ((b <= c) ? c : b);
        }

        float rpmInst = (60.0e6f * tachRevsPerPulse) / (float)median;
        tachRpmFiltered += tachFilterAlpha * (rpmInst - tachRpmFiltered);
      }
    }
  }

  // Timeout: if no pulses for tachStopTimeoutUs, engine is stopped
  // Note: 32-bit reads are atomic on ARM Cortex-M4, no noInterrupts() needed
  uint32_t lastPulseUs = tachLastPulseUs;

  if ((uint32_t)(micros() - lastPulseUs) > tachStopTimeoutUs) {
    tachRpmFiltered = 0.0f;
    tachNeedFirstPulseDiscard = true;
    tachPeriodCount = 0;  // Reset median buffer for next startup
  }

  // Update display/log value at configured rate
  if (millis() - tachLastUpdate > (1000 / tachUpdateRateHz)) {
    tachLastUpdate = millis();
    tachLastReported = (int)(tachRpmFiltered + 0.5f);
  }
}
