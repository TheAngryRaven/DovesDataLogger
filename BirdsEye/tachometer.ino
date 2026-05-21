///////////////////////////////////////////
// TACHOMETER MODULE
// Ring buffer ISR + Kalman-filtered RPM from mean inter-pulse period
//
// Architecture: The ISR timestamps every valid falling edge into a 16-entry
// ring buffer. TACH_LOOP() drains the buffer, computes mean inter-pulse
// period from all accumulated pulses, and feeds the result through a 1D
// Kalman filter. This gives microsecond-resolution RPM from ALL pulses
// between reads, not just the most recent pair.
//
// At 5000 RPM (12ms period), 1us timestamp resolution gives ~0.008% RPM
// error. Mean of ~3 periods per 25Hz window further reduces noise. The
// Kalman filter smooths mechanical variation while tracking real RPM changes
// bounded by crankshaft inertia.
///////////////////////////////////////////

// ---- Kalman filter state ----
static float kalmanX = 0.0f;       // RPM estimate
static float kalmanP = 10000.0f;   // Estimate uncertainty (RPM^2)

// Process noise Q: how much RPM^2 can change between Kalman updates.
// A kart engine with light flywheel can shift ~200 RPM per pulse at 5k RPM.
// Q=800 is conservative-smooth. Increase to 1500-2000 if tracking feels sluggish.
static const float KALMAN_Q = 800.0f;

// Measurement noise R_BASE: uncertainty of a single-period RPM measurement.
// ~50 RPM std dev from combustion variation + ISR latency jitter = variance 2500.
// Scales inversely with number of periods: more pulses → lower noise.
static const float KALMAN_R_BASE = 2500.0f;

// After engine-stopped timeout, the first period is garbage (it spans the
// entire stopped duration). This flag discards it.
static volatile bool tachNeedFirstPulseDiscard = true;

// Previous timestamp carried across TACH_LOOP calls for period calculation.
// When timestamp T_n is read in one call and T_{n+1} in the next, we need
// T_n to compute the period.
static uint32_t tachPrevTimestamp = 0;
static bool tachHavePrevTimestamp = false;

/**
 * Tachometer ISR - called on falling edge of tach signal (D0)
 *
 * Timestamps every valid pulse into a ring buffer. The 3ms time-based
 * debounce is the sole protection against ignition ringing — the old
 * volatile flag gate is removed because this ISR body is trivially fast
 * (<1us, ~10 ARM instructions) and cannot cause interrupt-storm CPU issues.
 */
void TACH_COUNT_PULSE() {
  uint32_t now = micros();
  uint32_t dt = now - tachLastPulseUs;

  // Time-based debounce: reject ignition ringing within 3ms of last valid pulse
  if (dt < tachMinPulseGapUs) return;

  // Record timestamp
  tachLastPulseUs = now;
  tachRingBuf[tachRingHead] = now;
  // ARM Cortex-M4: single-byte write is atomic. Data is visible before head
  // advances because stores are observed in program order on same processor.
  tachRingHead = (tachRingHead + 1) % TACH_RING_SIZE;

  // Wake trigger for sleep mode (BirdsEye.ino reads this)
  tachHavePeriod = true;
}

/**
 * Tachometer main loop processing
 *
 * Drains the ring buffer, computes mean inter-pulse period from all
 * accumulated timestamps, and updates the Kalman filter. Called every
 * main loop iteration (~250Hz). No rate limiter — consumers (display
 * at 3Hz, logging at 25Hz) rate-limit themselves.
 */
void TACH_LOOP() {
  // ---- Step 1: Read new timestamps from ring buffer ----
  uint8_t head = tachRingHead;  // Atomic byte read

  uint8_t available;
  if (head >= tachRingTail) {
    available = head - tachRingTail;
  } else {
    available = TACH_RING_SIZE - tachRingTail + head;
  }

  if (available > 0) {
    // Copy timestamps to local array
    uint32_t ts[TACH_RING_SIZE];
    for (uint8_t i = 0; i < available; i++) {
      ts[i] = tachRingBuf[(tachRingTail + i) % TACH_RING_SIZE];
    }
    tachRingTail = head;  // Consume all entries

    // ---- Step 2: Compute periods from consecutive timestamps ----
    uint32_t periods[TACH_RING_SIZE];
    uint8_t periodCount = 0;

    for (uint8_t i = 0; i < available; i++) {
      if (tachHavePrevTimestamp) {
        uint32_t dt = ts[i] - tachPrevTimestamp;  // unsigned handles micros() wrap

        // First-pulse discard: after engine stop, the period from the last
        // pre-stop pulse to the first new pulse spans the entire stopped
        // duration — not a real RPM measurement. Discard it.
        if (tachNeedFirstPulseDiscard) {
          tachNeedFirstPulseDiscard = false;
          tachPrevTimestamp = ts[i];
          continue;
        }

        // Sanity bounds: 3ms (20k RPM) to 2s (30 RPM)
        if (dt >= tachMinPulseGapUs && dt <= 2000000) {
          periods[periodCount++] = dt;
        }
      } else {
        // First timestamp ever — no period to compute yet
        tachHavePrevTimestamp = true;
        if (tachNeedFirstPulseDiscard) {
          tachNeedFirstPulseDiscard = false;
        }
      }
      tachPrevTimestamp = ts[i];
    }

    // ---- Step 3: Kalman filter update with all new periods ----
    if (periodCount > 0) {
      // Mean period
      uint32_t periodSum = 0;
      for (uint8_t i = 0; i < periodCount; i++) {
        periodSum += periods[i];
      }
      float meanPeriodUs = (float)periodSum / (float)periodCount;
      float rpmMeasured = (60.0e6f * tachRevsPerPulse) / meanPeriodUs;

      // Predict step: constant-RPM model, uncertainty grows
      kalmanP += KALMAN_Q;

      // Measurement noise scales inversely with number of periods
      float R = KALMAN_R_BASE / (float)periodCount;

      // Update step
      float K = kalmanP / (kalmanP + R);
      kalmanX += K * (rpmMeasured - kalmanX);
      kalmanP *= (1.0f - K);

      // Uncertainty floor to prevent numerical collapse
      if (kalmanP < 1.0f) kalmanP = 1.0f;
    }
  }

  // ---- Step 4: Engine-stopped timeout ----
  // 32-bit reads are atomic on ARM Cortex-M4, no noInterrupts() needed
  uint32_t lastPulseUs = tachLastPulseUs;
  if ((uint32_t)(micros() - lastPulseUs) > tachStopTimeoutUs) {
    kalmanX = 0.0f;
    kalmanP = 10000.0f;           // High uncertainty for next startup
    tachNeedFirstPulseDiscard = true;
    tachHavePrevTimestamp = false;
    tachRingTail = tachRingHead;  // Flush ring buffer
  }

  // ---- Step 5: Update reported value ----
  tachLastReported = (int)(kalmanX + 0.5f);
}
