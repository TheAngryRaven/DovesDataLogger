///////////////////////////////////////////
// GPS MODULE
// GPS time functions, configuration, setup, main loop, and frame rate
// Uses SparkFun u-blox GNSS v3 library with UBX PVT binary protocol
///////////////////////////////////////////

#include "gps_functions.h"
#include "gps_time.h"
#include "gps_validation.h"

///////////////////////////////////////////
// GPS SERIAL BUFFER
// Timer ISR drains Serial1 into a 4KB RAM buffer every 10ms,
// preventing data loss during SD card write stalls (GC pauses).
// SparkFun library reads from this buffer via GpsBufferedStream.
//
// At 25Hz PVT (~2500 bytes/sec), the hardware serial buffer
// (64-256 bytes) overflows in 25-100ms of blocking. SD card
// garbage collection can block writes for 100ms-2s.
// This 4KB buffer survives up to 1.6 seconds of stalls.
///////////////////////////////////////////

// Forward-declare ISR with C linkage BEFORE Arduino's preprocessor
// auto-generates a C++ prototype (which would conflict with extern "C").
extern "C" void TIMER3_IRQHandler(void);

#define GPS_RX_BUF_SIZE 4096
static uint8_t gpsRxBuf[GPS_RX_BUF_SIZE];
static volatile uint16_t gpsRxHead = 0;  // Written by ISR only
static volatile uint16_t gpsRxTail = 0;  // Read by main loop only (via gpsStream)
static volatile bool gpsTimerActive = false;

// Stream wrapper: SparkFun library reads from our 4KB buffer instead of Serial1.
// Before the timer ISR is started (during GPS_SETUP), reads pass through to
// GPS_SERIAL directly so that myGNSS.begin() can communicate with the module.
class GpsBufferedStream : public Stream {
public:
  int available() override {
    if (!gpsTimerActive) return GPS_SERIAL.available();
    return (GPS_RX_BUF_SIZE + gpsRxHead - gpsRxTail) % GPS_RX_BUF_SIZE;
  }
  int read() override {
    if (!gpsTimerActive) return GPS_SERIAL.read();
    if (gpsRxHead == gpsRxTail) return -1;
    uint8_t c = gpsRxBuf[gpsRxTail];
    gpsRxTail = (gpsRxTail + 1) % GPS_RX_BUF_SIZE;
    return c;
  }
  int peek() override {
    if (!gpsTimerActive) return GPS_SERIAL.peek();
    if (gpsRxHead == gpsRxTail) return -1;
    return gpsRxBuf[gpsRxTail];
  }
  size_t write(uint8_t c) override {
    return GPS_SERIAL.write(c);
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    return GPS_SERIAL.write(buffer, size);
  }
  void flush() override {
    GPS_SERIAL.flush();
  }
};

static GpsBufferedStream gpsStream;

// Timer3 ISR: drains Serial1 into our 4KB buffer every ~10ms.
// Single-producer (ISR writes gpsRxHead), single-consumer (main loop
// reads gpsRxTail via gpsStream) — lock-free ring buffer.
//
// Brief __disable_irq() around each GPS_SERIAL read protects the UART
// driver's internal FIFO from concurrent access by the UART DMA ISR
// (which can preempt us at higher priority). Each critical section is
// ~0.3µs — well within SoftDevice's 6µs safe window.
void TIMER3_IRQHandler(void) {
  if (NRF_TIMER3->EVENTS_COMPARE[0]) {
    NRF_TIMER3->EVENTS_COMPARE[0] = 0;
    while (true) {
      __disable_irq();
      int c = GPS_SERIAL.available() ? GPS_SERIAL.read() : -1;
      __enable_irq();
      if (c < 0) break;

      uint16_t nextHead = (gpsRxHead + 1) % GPS_RX_BUF_SIZE;
      if (nextHead == gpsRxTail) break;  // Buffer full, drop bytes
      gpsRxBuf[gpsRxHead] = (uint8_t)c;
      gpsRxHead = nextHead;
    }
  }
}

void startGpsSerialTimer() {
  NRF_TIMER3->TASKS_STOP = 1;
  NRF_TIMER3->TASKS_CLEAR = 1;
  NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer;
  NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
  NRF_TIMER3->PRESCALER = 4;              // 16MHz / 2^4 = 1MHz tick
  NRF_TIMER3->CC[0] = 10000;              // 10ms interval
  NRF_TIMER3->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
  NRF_TIMER3->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
  NVIC_SetPriority(TIMER3_IRQn, 3);       // Below SoftDevice (0-2), above main loop
  NVIC_ClearPendingIRQ(TIMER3_IRQn);      // Clear stale pending interrupt from prior session
  NVIC_EnableIRQ(TIMER3_IRQn);
  gpsTimerActive = true;
  NRF_TIMER3->TASKS_START = 1;
  debugln(F("GPS serial buffer timer started"));
}

void stopGpsSerialTimer() {
  NRF_TIMER3->TASKS_STOP = 1;
  NRF_TIMER3->INTENCLR = TIMER_INTENCLR_COMPARE0_Msk;
  NVIC_DisableIRQ(TIMER3_IRQn);
  NVIC_ClearPendingIRQ(TIMER3_IRQn);      // Ensure no stale ISR fires after disable
  __DSB();                                 // ARM barrier: NVIC ops complete before flag update
  gpsTimerActive = false;
}

/**
  * @brief Returns the GPS time since midnight in milliseconds, or 0 if GPS unavailable.
  *        The pure math lives in gps_time::timeOfDayMs — this just plumbs through gpsData.
  */
unsigned long getGpsTimeInMilliseconds() {
  if (!gpsInitialized) return 0;
  return gps_time::timeOfDayMs(gpsData.hour, gpsData.minute,
                               gpsData.seconds, gpsData.milliseconds);
}

/**
  * @brief Converts GPS date/time to Unix timestamp in seconds. 0 if GPS unavailable.
  *        gpsData.year is the 2-digit year offset from 2000.
  */
unsigned long getGpsUnixTimestamp() {
  if (!gpsInitialized) return 0;
  return static_cast<unsigned long>(gps_time::unixTimestampSeconds(
      2000 + gpsData.year, gpsData.month, gpsData.day,
      gpsData.hour, gpsData.minute, gpsData.seconds));
}

/**
  * @brief Converts GPS date/time to Unix timestamp with millisecond precision.
  *        0 if GPS unavailable.
  */
unsigned long long getGpsUnixTimestampMillis() {
  if (!gpsInitialized) return 0;
  return gps_time::unixTimestampMillis(
      2000 + gpsData.year, gpsData.month, gpsData.day,
      gpsData.hour, gpsData.minute, gpsData.seconds, gpsData.milliseconds);
}

// PVT callback — called synchronously from checkCallbacks() when a new
// NAV-PVT message arrives.  Populates the shared gpsData struct and sets
// the gpsDataFresh flag so GPS_LOOP() knows to run lap-timer / logging.
void onPVTReceived(UBX_NAV_PVT_data_t *pvt) {
  gpsData.latitudeDegrees = pvt->lat / 1e7;
  gpsData.longitudeDegrees = pvt->lon / 1e7;
  gpsData.altitude = pvt->hMSL / 1000.0;          // mm → meters
  gpsData.speed = pvt->gSpeed / 514.444;           // mm/s → knots
  gpsData.HDOP = pvt->pDOP / 100.0;               // pDOP ≈ HDOP for track use
  gpsData.heading = pvt->headMot / 1e5;            // deg * 1e-5 → degrees
  gpsData.horizontalAccuracy = pvt->hAcc / 1000.0; // mm → meters
  gpsData.satellites = pvt->numSV;
  // A fix is only trustworthy when the module also asserts gnssFixOK — a bare
  // fixType >= 2 can appear during convergence with garbage coordinates.
  gpsData.fix = (pvt->fixType >= 2) && (pvt->flags.bits.gnssFixOK != 0);
  // Time is only usable for naming/saving the log once the module reports the
  // date AND time AND a fully-resolved UTC. Before this, the module emits a
  // placeholder date (e.g. 2021-03-07) that must NOT drive file creation.
  gpsData.timeValid = (pvt->valid.bits.validDate != 0) &&
                      (pvt->valid.bits.validTime != 0) &&
                      (pvt->valid.bits.fullyResolved != 0);
  gpsData.year = pvt->year - 2000;
  gpsData.month = pvt->month;
  gpsData.day = pvt->day;
  gpsData.hour = pvt->hour;
  gpsData.minute = pvt->min;
  gpsData.seconds = pvt->sec;
  gpsData.milliseconds = (pvt->iTOW % 1000);       // ms from GPS time-of-week

  gpsDataFresh = true;
  gpsFrameCounter++;
}

void GPS_SETUP() {
  gpsInitialized = false;  // Reset flag at start of setup

  #ifndef WOKWI
    debugln(F("ACTUAL GPS SETUP"));

    // Start at u-blox default baud rate (9600 for fresh modules)
    GPS_SERIAL.begin(9600);
    // Wait for the GPS to boot
    delay(2250);

    if (GPS_SERIAL) {
      // Connect to GPS at default baud
      // gpsStream wraps GPS_SERIAL: before timer starts, reads pass
      // through directly so myGNSS.begin() can communicate with module.
      if (!myGNSS.begin(gpsStream)) {
        debugln(F("GPS not detected at 9600 baud, trying 57600..."));
        GPS_SERIAL.end();
        delay(100);

        // Module may already be configured to 57600 from a previous session
        GPS_SERIAL.begin(GPS_BAUD_RATE);
        delay(100);
        if (!myGNSS.begin(gpsStream)) {
          debugln(F("ERROR: GPS not detected at any baud rate!"));
          return;  // Leave gpsInitialized = false
        }
        debugln(F("GPS found at 57600 baud (already configured)"));
      } else {
        // Connected at 9600, switch module to 57600
        debugln(F("GPS found at 9600, switching to 57600..."));
        myGNSS.setSerialRate(GPS_BAUD_RATE);
        delay(100);
        GPS_SERIAL.end();
        delay(100);
        GPS_SERIAL.begin(GPS_BAUD_RATE);
        delay(100);

        if (!myGNSS.begin(gpsStream)) {
          debugln(F("ERROR: GPS lost after baud switch!"));
          return;
        }
        debugln(F("GPS reconnected at 57600"));
      }

      // Configure via VALSET API
      myGNSS.setUART1Output(COM_TYPE_UBX);          // UBX only, disable NMEA output
      myGNSS.setNavigationFrequency(GPS_NAV_RATE_HZ); // 25 Hz
      myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE);

      // Enable GPS only (disable GLONASS/Galileo/BeiDou/SBAS/QZSS for max nav rate)
      myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
      myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS);
      myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GALILEO);
      myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU);
      myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);

      // Register PVT callback (called from checkCallbacks())
      myGNSS.setAutoPVTcallbackPtr(&onPVTReceived);

      gpsInitialized = true;
      debugln(F("GPS initialized successfully (SparkFun UBX PVT)"));

      // Drain any remaining bytes from Serial1 into our buffer, then
      // start the timer ISR for continuous background serial drain.
      while (GPS_SERIAL.available()) {
        uint16_t nextHead = (gpsRxHead + 1) % GPS_RX_BUF_SIZE;
        if (nextHead == gpsRxTail) break;
        gpsRxBuf[gpsRxHead] = GPS_SERIAL.read();
        gpsRxHead = nextHead;
      }
      startGpsSerialTimer();
    } else {
      debugln(F("ERROR: GPS Serial not available!"));
    }
  #else
    debugln(F("WOKWI GPS SETUP"));
    GPS_SERIAL.begin(19200);
    if (myGNSS.begin(gpsStream)) {
      myGNSS.setAutoPVTcallbackPtr(&onPVTReceived);
      gpsInitialized = true;
      startGpsSerialTimer();
    } else {
      debugln(F("ERROR: WOKWI GPS not detected!"));
    }
  #endif
}

void GPS_LOOP() {
  // Safety check: skip if GPS not initialized
  if (!gpsInitialized) {
    return;
  }

  // Process incoming UBX bytes and fire onPVTReceived() callback if a
  // complete PVT message has arrived.  The callback populates gpsData
  // and sets gpsDataFresh = true.
  myGNSS.checkUblox();
  myGNSS.checkCallbacks();

  // PVT arrival watchdog: after GPS_WAKE(), if no PVT data arrives within
  // 5 seconds, the module likely lost its config during backup (V_BCKP
  // dropped → reverted to 9600 baud NMEA). Attempt baud recovery.
  if (!gpsWakeValidated) {
    if (gpsDataFresh) {
      // PVT arrived — module is alive and configured correctly
      gpsWakeValidated = true;
      debugln(F("GPS wake validated: PVT received"));
    } else if (millis() - gpsWakeTime >= 5000) {
      debugln(F("GPS wake FAILED: no PVT after 5s, attempting recovery"));
      if (GPS_BAUD_RECOVERY()) {
        // Recovery succeeded — restart the watchdog for validation
        gpsWakeTime = millis();
        debugln(F("GPS recovery succeeded, waiting for PVT..."));
      } else {
        // Recovery failed — give up, mark validated to stop retrying
        gpsWakeValidated = true;
        debugln(F("GPS recovery FAILED — GPS unavailable this session"));
      }
    }
  }

  if (gpsDataFresh) {
    gpsDataFresh = false;

    // Feed fresh GPS data into the active course/timer
    if (gpsData.fix && courseManager != nullptr) {
      double ltLat = gpsData.latitudeDegrees;
      double ltLng = gpsData.longitudeDegrees;
      double ltAlt = gpsData.altitude;
      double ltSpeed = gpsData.speed;

      courseManager->updateCurrentTime(getGpsTimeInMilliseconds());
      courseManager->loop(ltLat, ltLng, ltAlt, ltSpeed);
    }

  #ifdef SD_CARD_LOGGING_ENABLED
    // Determine if logging conditions are met.
    // File creation requires a VALID GPS time lock (validDate+validTime+
    // fullyResolved), not merely a non-zero day. Before the module resolves
    // real time it emits a placeholder date; creating a file from it produced
    // garbage-named logs (e.g. 20210307_0000.dovex) that collided every boot
    // and corrupted on reboot. With a real lock the module keeps time across
    // the V_BCKP backup, so this still fires within ~1 s of a warm wake.
    // While we wait, updateGpsLockHold() pins the user to the tachometer.
    // Data writing still requires gpsData.fix for valid coordinates.
    bool canWriteData = gpsData.fix && sdSetupSuccess && enableLogging && sdDataLogInitComplete;
    bool canCreateFile = sdSetupSuccess && enableLogging && !sdDataLogInitComplete && gpsData.timeValid;

    if (canWriteData) {

      /////////////////////////////////////////////////////////////////
      // Log every PVT update (~25Hz)

      // Snapshot GPS values for logging
      const double snapLat   = gpsData.latitudeDegrees;
      const double snapLng   = gpsData.longitudeDegrees;
      const double snapAlt   = gpsData.altitude;
      const double snapHdop  = gpsData.HDOP;
      const double snapSpeed = gpsData.speed;            // knots, for sanity
      const int    snapSats  = gpsData.satellites;
      const double snapSpeedMph = snapSpeed * 1.15078;

      // Reject obviously-corrupt samples (sats=0, Null Island, NaN/Inf,
      // out-of-range lat/lng/alt/hdop/speed). The rule set lives in
      // gps_validation::isSampleValid and is exercised by host tests.
      const gps_validation::GpsSample snapForCheck = {
          snapLat, snapLng, snapAlt, snapHdop, snapSpeedMph, snapSats};
      if (!gps_validation::isSampleValid(snapForCheck)) {
        // Skip this sample - data is not trustworthy
      } else {
        char csvLine[256];
        char latStr[24], lngStr[24], hdopStr[12], speedStr[16], altStr[16];
        char headingStr[12], hAccStr[12];

        dtostrf(snapLat, 1, 8, latStr);
        dtostrf(snapLng, 1, 8, lngStr);
        dtostrf(snapHdop, 1, 1, hdopStr);
        dtostrf(snapSpeedMph, 1, 2, speedStr);
        dtostrf(snapAlt, 1, 2, altStr);
        dtostrf(gpsData.heading, 1, 2, headingStr);
        dtostrf(gpsData.horizontalAccuracy, 1, 2, hAccStr);

        char accelXStr[12], accelYStr[12], accelZStr[12];
        dtostrf(accelX, 1, 3, accelXStr);
        dtostrf(accelY, 1, 3, accelYStr);
        dtostrf(accelZ, 1, 3, accelZStr);

        // dtostrf() can produce garbage (empty, too-long, non-numeric) on
        // some BSPs when given NaN/Inf even though the sample passed
        // validation. Walk every formatted string and reject the row if
        // any look wrong.
        bool stringsValid = true;
        const char* strs[] = {latStr, lngStr, hdopStr, speedStr, altStr,
                              headingStr, hAccStr, accelXStr, accelYStr, accelZStr};
        for (size_t i = 0; i < sizeof(strs)/sizeof(strs[0]) && stringsValid; i++) {
          if (!gps_validation::isNumericString(strs[i], 20)) {
            stringsValid = false;
          }
        }

        if (!stringsValid) {
          // dtostrf produced garbage - skip this entry silently
        } else {
          // Arduino lacks %llu in printf; format the 64-bit timestamp
          // ourselves into a stack buffer, then splice via %s.
          char timestampStr[24];
          gps_time::u64ToDecimalString(getGpsUnixTimestampMillis(),
                                        timestampStr, sizeof(timestampStr));

          snprintf(csvLine, sizeof(csvLine), "%s,%d,%s,%s,%s,%s,%s,%s,%s,%d,%s,%s,%s",
                   timestampStr, snapSats, hdopStr, latStr, lngStr,
                   speedStr, altStr, headingStr, hAccStr,
                   tachLastReported, accelXStr, accelYStr, accelZStr);

          size_t written = dataFile.println(csvLine);
          if (written == 0) {
            // A write failed mid-session. Don't fault — keep racing so lap
            // timing / RPM / display stay live; just stop logging for this
            // session (closing cleanly rather than truncating-and-restarting
            // the same-minute filename).
            debugln(F("SD write failed - stopping logging, race continues"));
            enableLogging = false;
            sdDataLogInitComplete = false;
            dataFile.close();
            releaseSDAccess(SD_ACCESS_LOGGING);
          }
        }
      }
      /////////////////////////////////////////////////////////////////

      // Flush periodically to avoid data loss - every 10 seconds
      if (millis() - lastCardFlush > 10000) {
        lastCardFlush = millis();
        dataFile.flush();
      }
    } else if (canCreateFile && millis() - lastLogCreateAttempt >= 1000) {
      // Throttle open attempts to once per second so a problem card can't
      // churn SPI at 25 Hz. We never fault out of race mode here — if the
      // file can't be created we simply keep waiting (the user stays on the
      // tachometer via the GPS-lock hold) and retry next second.
      lastLogCreateAttempt = millis();
      debugln(F("Attempt to initialize logfile"));

      if (!acquireSDAccess(SD_ACCESS_LOGGING)) {
        // Another subsystem holds the card (BLE/replay). Don't fault — just
        // retry on the next throttled pass once it releases.
        debugln(F("Cannot start logging - SD card busy, will retry"));
      } else {
        char dataFileName[80];

        // DOVEX filename: 20YYMMDD_HHMM.dovex
        snprintf(dataFileName, sizeof(dataFileName),
                 "20%02d%02d%02d_%02d%02d.dovex",
                 gpsData.year, gpsData.month, gpsData.day,
                 gpsData.hour, gpsData.minute);

        debug(F("dataFileName: ["));
        debug(dataFileName);
        debugln(F("]"));

        dataFile.open(dataFileName, O_CREAT | O_WRITE | O_TRUNC);

        if (!dataFile) {
          // Open failed — do NOT fault out of race mode. Release the card and
          // leave logging pending; updateGpsLockHold() keeps the user on the
          // tachometer and we retry on the next throttled pass.
          debugln(F("Error opening log file - will retry"));
          releaseSDAccess(SD_ACCESS_LOGGING);
        } else {
          // DOVEX: pre-fill header region with newlines so the FAT clusters
          // for it are allocated up front; the metadata header is written
          // back into this region at session end.
          char padBuf[64];
          memset(padBuf, '\n', sizeof(padBuf));
          for (uint32_t i = 0; i < DOVEX_HEADER_SIZE; i += sizeof(padBuf)) {
            uint32_t toWrite = min((uint32_t)sizeof(padBuf), DOVEX_HEADER_SIZE - i);
            dataFile.write(padBuf, toWrite);
          }
          // Cursor is now at exactly DOVEX_HEADER_SIZE
          dataFile.println(F("timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,heading_deg,h_acc_m,rpm,accel_x,accel_y,accel_z"));
          debugln(F("CSV header written"));
          sdDataLogInitComplete = true;
        }
      }
    }
  #endif

    // Update display speed with fresh PVT data
    gps_speed_mph = gpsData.speed * 1.15078;
  } // end if (gpsDataFresh)
}

void GPS_SLEEP() {
  if (!gpsInitialized) return;
  stopGpsSerialTimer();  // Stop serial drain ISR during sleep (saves power)
  myGNSS.powerOff(0);   // 0 = indefinite sleep until woken
}

/**
 * @brief Re-apply all GPS module configuration via VALSET.
 *
 * The SAM-M10Q has no flash — all config lives in volatile RAM. If V_BCKP
 * drops during backup mode (battery sag, loose connector, cranking brownout),
 * the module reverts to factory defaults (9600 baud, NMEA, 1Hz). This
 * function re-applies the full configuration. It's fast (~50ms total) and
 * idempotent — safe to call even if config was retained.
 */
void GPS_RECONFIGURE() {
  myGNSS.setUART1Output(COM_TYPE_UBX);
  myGNSS.setNavigationFrequency(GPS_NAV_RATE_HZ);
  myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE);
  myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS);
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GALILEO);
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU);
  myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);
  debugln(F("GPS config re-applied (VALSET)"));
}

/**
 * @brief Attempt to recover GPS communication when module has reverted to 9600 baud.
 *
 * If V_BCKP was lost during backup, the module reverts to 9600 baud.
 * Our UART is at 57600 so communication is broken. This function:
 * 1. Stops the serial timer (switch to direct Serial1 mode)
 * 2. Tries to talk at 57600 first (maybe module is fine)
 * 3. If that fails, switches to 9600, sends baud change command, switches back
 * 4. Re-applies full config and restarts the serial timer
 *
 * @return true if communication was recovered, false if unrecoverable
 */
bool GPS_BAUD_RECOVERY() {
  debugln(F("GPS baud recovery: attempting..."));

  // This path runs from GPS_LOOP() under the armed ~4 s hardware watchdog,
  // and it is the slowest thing in the firmware short of the OTA apply:
  // up to three myGNSS.begin() probes (~1.1 s of ping timeouts each) plus
  // ~500 ms of explicit delay(). Against a genuinely hung module that
  // out-waits the WDT → reset → re-setup → recovery → reset boot loop —
  // the one path built to revive a sick GPS must not trip the watchdog.
  // Pet before every blocking probe (same rationale as fwStageToFlash()).
  wdtPet();

  // Stop timer so GpsBufferedStream passes through to Serial1 directly.
  // The SparkFun library needs direct serial access for begin()/setSerialRate().
  stopGpsSerialTimer();

  // First: try at current baud — module might be fine, just slow to start
  if (myGNSS.begin(gpsStream)) {
    debugln(F("GPS baud recovery: module responding at 57600"));
    GPS_RECONFIGURE();
    myGNSS.setAutoPVTcallbackPtr(&onPVTReceived);  // begin() resets library state
    gpsRxHead = 0;
    gpsRxTail = 0;
    startGpsSerialTimer();
    return true;
  }

  // Module not responding at 57600 — try 9600 (factory default)
  debugln(F("GPS baud recovery: trying 9600..."));
  wdtPet();  // the first begin() just burned ~1 s of the 4 s budget
  GPS_SERIAL.end();
  delay(50);
  GPS_SERIAL.begin(9600);
  delay(100);

  if (myGNSS.begin(gpsStream)) {
    // Found at 9600 — switch it back to 57600
    debugln(F("GPS baud recovery: found at 9600, switching to 57600"));
    wdtPet();  // second begin() done; baud switch + final probe still ahead
    myGNSS.setSerialRate(GPS_BAUD_RATE);
    delay(100);
    GPS_SERIAL.end();
    delay(50);
    GPS_SERIAL.begin(GPS_BAUD_RATE);
    delay(100);
    wdtPet();

    if (myGNSS.begin(gpsStream)) {
      debugln(F("GPS baud recovery: reconnected at 57600"));
      GPS_RECONFIGURE();
      myGNSS.setAutoPVTcallbackPtr(&onPVTReceived);
      gpsRxHead = 0;
      gpsRxTail = 0;
      startGpsSerialTimer();
      return true;
    }
    debugln(F("GPS baud recovery: lost after baud switch!"));
  } else {
    debugln(F("GPS baud recovery: not found at 9600 either"));
  }

  // Restore 57600 even on failure so other code doesn't break
  wdtPet();
  GPS_SERIAL.end();
  delay(50);
  GPS_SERIAL.begin(GPS_BAUD_RATE);
  delay(50);
  gpsRxHead = 0;
  gpsRxTail = 0;
  startGpsSerialTimer();
  return false;
}

void GPS_WAKE() {
  if (!gpsInitialized) return;

  // Clear stale state from before sleep / periodic checks
  gpsDataFresh = false;
  gpsData.fix = false;

  // Any UART activity on RX wakes u-blox from powerOff backup mode
  GPS_SERIAL.write(0xFF);
  delay(100);

  // Reset buffer pointers (stale data from before sleep is useless)
  gpsRxHead = 0;
  gpsRxTail = 0;

  startGpsSerialTimer();  // Resume serial drain ISR

  // Re-apply configuration in case module lost RAM during backup.
  // This is idempotent — if config was retained, these are no-ops.
  GPS_RECONFIGURE();

  myGNSS.checkUblox();

  // Start PVT arrival watchdog. GPS_LOOP() will validate within 5 seconds,
  // and trigger baud recovery if no PVT data arrives.
  gpsWakeTime = millis();
  gpsWakeValidated = false;
}

void GPS_SLEEP_PERIODIC_CHECK() {
  GPS_WAKE();
  sleepGpsWakeActive = true;
  sleepGpsWakeStartedAt = millis();
}

void calculateGPSFrameRate() {
  // calculate actual GPS fix frequency
  gpsFrameEndTime = millis();
  // Check if the update interval has passed
  if (gpsFrameEndTime - gpsFrameStartTime >= 1000) {
    // Calculate the frame rate (loops per second)
    gpsFrameRate = (float)gpsFrameCounter / ((gpsFrameEndTime - gpsFrameStartTime) / 1000.0);
    // Reset the loop counter and start time for the next interval
    gpsFrameCounter = 0;
    gpsFrameStartTime = millis();
  }
}
