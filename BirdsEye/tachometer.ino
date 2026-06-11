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

#include "tachometer.h"
#include "tach_filter.h"

// ---- Kalman filter state ----
// The predict/update math and the tuning constants (Q, R_BASE, the
// uncertainty floor) live in the host-tested tach_filter pure unit.
static tach_filter::Kalman tachKalman;

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

  tachLastPulseUs = now;

  // Wake trigger for sleep mode (BirdsEye.ino reads this). Set even when
  // the ring is full below — a dropped timestamp is still a real pulse.
  tachHavePeriod = true;

  // SPSC full check (one slot is sacrificed so head==tail is unambiguously
  // "empty"). TACH_LOOP can be blocked for 100 ms–2 s by the same SD GC
  // stalls the GPS serial ring exists for; at 6000 RPM a 200 ms stall
  // overruns 16 entries, and advancing head unconditionally would lap the
  // consumer and corrupt every period it computes. Dropping the pulse and
  // flagging the gap is safe — the consumer discards the one period that
  // spans the gap and the Kalman estimate coasts until fresh data arrives.
  uint8_t nextHead = (tachRingHead + 1) % TACH_RING_SIZE;
  if (nextHead == tachRingTail) {
    tachRingOverflow = true;
    return;
  }

  // Data is visible before head advances because stores are observed in
  // program order on the same processor (single-byte head write is atomic).
  tachRingBuf[tachRingHead] = now;
  tachRingHead = nextHead;
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
      float rpmMeasured =
          tach_filter::rpmFromMeanPeriodUs(meanPeriodUs, tachRevsPerPulse);
      tach_filter::update(tachKalman, rpmMeasured, periodCount);
    }
  }

  // ---- Step 3.5: Ring overflow recovery ----
  // The ISR dropped pulses while the ring was full (main loop stalled).
  // The period between the last retained timestamp and the next retained
  // pulse spans the whole gap and is NOT a real measurement — drop the
  // carried prev-timestamp so that period is never computed. The estimate
  // simply coasts until consecutive post-gap pulses arrive.
  if (tachRingOverflow) {
    tachRingOverflow = false;
    tachHavePrevTimestamp = false;
  }

  // ---- Step 4: Engine-stopped timeout ----
  // 32-bit reads are atomic on ARM Cortex-M4, no noInterrupts() needed
  uint32_t lastPulseUs = tachLastPulseUs;
  if ((uint32_t)(micros() - lastPulseUs) > tachStopTimeoutUs) {
    tach_filter::reset(tachKalman);  // High uncertainty for next startup
    tachNeedFirstPulseDiscard = true;
    tachHavePrevTimestamp = false;
    tachRingTail = tachRingHead;  // Flush ring buffer
  }

  // ---- Step 5: Update reported value ----
  tachLastReported = (int)(tachKalman.x + 0.5f);
}

/**
 * Prepare the tachometer for sleep mode — call from enterSleepMode().
 *
 * Re-arms the RPM wake trigger and discards pre-sleep pulse state. Without
 * this, tachHavePeriod stays true forever after the first engine pulse
 * since boot, and every sleep entry (long-press, menu idle, USB) instantly
 * bounces through the RPM-wake path back into race mode with logging
 * enabled — silently starting a session and draining the pack overnight.
 *
 * The ISR stays attached during sleep: the next valid pulse sets
 * tachHavePeriod again, and THAT is the legitimate RPM wake.
 */
void TACH_SLEEP() {
  tachHavePeriod = false;

  // Drop any buffered pre-sleep pulses and filter state so the wake's RPM
  // computation starts clean instead of chewing on stale timestamps (the
  // first post-wake period would otherwise span the whole sleep).
  tachRingTail = tachRingHead;
  tachRingOverflow = false;
  tachHavePrevTimestamp = false;
  tachNeedFirstPulseDiscard = true;
  tach_filter::reset(tachKalman);
  tachLastReported = 0;
}
