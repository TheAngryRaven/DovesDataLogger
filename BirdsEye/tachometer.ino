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
//
// The filter sees ONE number per pulse, so a single bad edge is a spike
// rather than a wobble — see tach_filter.h for the outlier gate that keeps
// ignition ringing and missed sparks out of the logged trace (plan 0009),
// and for the `tach_filter` setting that switches the estimator off for a
// track-side A/B against the raw pickup.
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
      // The predict step advances by the ENGINE time this batch spans —
      // the sum of the periods themselves — not by wall clock since the
      // last TACH_LOOP call. At 1500 RPM those differ by 40x (a 40 ms
      // period vs a 4 ms loop), and charging process noise for time the
      // crank did not turn is exactly the per-update-Q mistake this
      // replaces. It is also free: periodSum is already computed, needs
      // no clock read, and makes the filter's behaviour a pure function
      // of the pulse train — which is what the host tests exercise.
      const float dtSeconds = (float)periodSum / 1.0e6f;
      tach_filter::update(tachKalman, tachFilterMode, rpmMeasured, periodCount,
                          dtSeconds, tachRevsPerPulse);
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
 * Periods the outlier gate has thrown away this power-cycle.
 *
 * Diagnostic only — nothing behaves differently on the count. It is the
 * one number that says whether the pickup is clean: a session that ends
 * with single digits saw a clean signal, and a count climbing with RPM
 * is ignition ringing or missed sparks reaching the ISR. Deliberately
 * NOT cleared by the engine-stop reset, so it accumulates across a whole
 * session rather than resetting at every corner.
 */
uint16_t tachRejectedPeriods() {
  return tachKalman.rejected;
}
