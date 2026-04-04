///////////////////////////////////////////
// GPS MODULE
// GPS time functions, configuration, setup, main loop, and frame rate
// Uses SparkFun u-blox GNSS v3 library with UBX PVT binary protocol
///////////////////////////////////////////

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
  * @brief Returns the GPS time since midnight in milliseconds
  *
  * @return unsigned long The time since midnight in milliseconds, or 0 if GPS unavailable
  */
unsigned long getGpsTimeInMilliseconds() {
  if (!gpsInitialized) return 0;

  unsigned long timeInMillis = 0;
  timeInMillis += gpsData.hour * 3600000ULL;   // Convert hours to milliseconds
  timeInMillis += gpsData.minute * 60000ULL;   // Convert minutes to milliseconds
  timeInMillis += gpsData.seconds * 1000ULL;   // Convert seconds to milliseconds
  timeInMillis += gpsData.milliseconds;        // Add the milliseconds part

  return timeInMillis;
}

/**
  * @brief Converts GPS date/time to Unix timestamp (seconds since Jan 1, 1970)
  *
  * @return unsigned long Unix timestamp in seconds, or 0 if GPS unavailable
  */
unsigned long getGpsUnixTimestamp() {
  if (!gpsInitialized) return 0;

  // Days in each month (non-leap year)
  const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

  uint16_t year = 2000 + gpsData.year;
  uint8_t month = gpsData.month;
  uint8_t day = gpsData.day;

  // Calculate days since Unix epoch (Jan 1, 1970)
  unsigned long days = 0;

  // Add days for complete years since 1970
  for (uint16_t y = 1970; y < year; y++) {
    if ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0)) {
      days += 366; // Leap year
    } else {
      days += 365;
    }
  }

  // Add days for complete months this year
  for (uint8_t m = 1; m < month; m++) {
    days += daysInMonth[m - 1];
    // Add leap day if February and leap year
    if (m == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
      days++;
    }
  }

  // Add remaining days
  days += day - 1;

  // Convert to seconds and add time of day
  unsigned long timestamp = days * 86400UL;
  timestamp += gpsData.hour * 3600UL;
  timestamp += gpsData.minute * 60UL;
  timestamp += gpsData.seconds;

  return timestamp;
}

/**
  * @brief Converts GPS date/time to Unix timestamp with millisecond precision
  *
  * @return unsigned long long Unix timestamp in milliseconds since Jan 1, 1970
  */
unsigned long long getGpsUnixTimestampMillis() {
  if (!gpsInitialized) return 0;

  // Get the Unix timestamp in seconds
  unsigned long long timestampMillis = (unsigned long long)getGpsUnixTimestamp() * 1000ULL;

  // Add the milliseconds from GPS
  timestampMillis += gpsData.milliseconds;

  return timestampMillis;
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
  gpsData.fix = (pvt->fixType >= 2);               // 2D or 3D
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

  if (gpsDataFresh) {
    gpsDataFresh = false;

    // Update the lap timer with fresh GPS data
    #ifdef ENABLE_NEW_UI
    if (gpsData.fix && courseManager != nullptr) {
      double ltLat = gpsData.latitudeDegrees;
      double ltLng = gpsData.longitudeDegrees;
      double ltAlt = gpsData.altitude;
      double ltSpeed = gpsData.speed;

      courseManager->updateCurrentTime(getGpsTimeInMilliseconds());
      courseManager->loop(ltLat, ltLng, ltAlt, ltSpeed);
    }
    #else
    if (trackSelected && gpsData.fix) {
      double ltLat = gpsData.latitudeDegrees;
      double ltLng = gpsData.longitudeDegrees;
      double ltAlt = gpsData.altitude;
      double ltSpeed = gpsData.speed;

      lapTimer.updateCurrentTime(getGpsTimeInMilliseconds());
      lapTimer.loop(ltLat, ltLng, ltAlt, ltSpeed);
    }
    #endif

  #ifdef SD_CARD_LOGGING_ENABLED
    // Determine if logging conditions are met.
    // ENABLE_NEW_UI: File creation does NOT require GPS fix — the file opens
    // as soon as date is available (from RTC even before fix). This eliminates
    // the delay between GPS reacquisition and first logged row after sleep wake.
    // Data writing still requires gpsData.fix for valid coordinates.
    #ifdef ENABLE_NEW_UI
    bool canWriteData = gpsData.fix && sdSetupSuccess && enableLogging && sdDataLogInitComplete;
    bool canCreateFile = sdSetupSuccess && enableLogging && !sdDataLogInitComplete && gpsData.day > 0;
    #else
    bool loggingCondition = trackSelected && gpsData.fix && sdSetupSuccess && enableLogging;
    bool canWriteData = loggingCondition && sdDataLogInitComplete;
    bool canCreateFile = loggingCondition && !sdDataLogInitComplete && gpsData.day > 0;
    #endif

    if (canWriteData) {

      /////////////////////////////////////////////////////////////////
      // Log every PVT update (~25Hz)

      // Snapshot GPS values for logging
      double snapLat = gpsData.latitudeDegrees;
      double snapLng = gpsData.longitudeDegrees;
      double snapAlt = gpsData.altitude;
      double snapHdop = gpsData.HDOP;
      double snapSpeed = gpsData.speed;
      int snapSats = gpsData.satellites;

      // Convert speed to mph
      double snapSpeedMph = snapSpeed * 1.15078;

      // VALIDATE: Minimal checks to prevent genuinely corrupt data from being logged
      bool hasActualFix = (snapSats > 0);
      bool validCoords = !(snapLat == 0.0 && snapLng == 0.0);
      bool validLat = (snapLat >= -90.0 && snapLat <= 90.0);
      bool validLng = (snapLng >= -180.0 && snapLng <= 180.0);
      bool validAlt = (snapAlt >= -1000.0 && snapAlt <= 50000.0);
      bool validHdop = (snapHdop > 0.0);
      bool validSpeed = (snapSpeedMph >= 0.0 && snapSpeedMph <= 500.0);

      bool noNaN = !isnan(snapLat) && !isnan(snapLng) && !isnan(snapAlt) &&
                   !isnan(snapHdop) && !isnan(snapSpeed);
      bool noInf = !isinf(snapLat) && !isinf(snapLng) && !isinf(snapAlt) &&
                   !isinf(snapHdop) && !isinf(snapSpeed);

      if (!hasActualFix || !validCoords || !validLat || !validLng || !validAlt ||
          !validHdop || !validSpeed || !noNaN || !noInf) {
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

        // String validation
        bool stringsValid = true;
        const char* strs[] = {latStr, lngStr, hdopStr, speedStr, altStr, headingStr, hAccStr, accelXStr, accelYStr, accelZStr};
        for (int i = 0; i < 10 && stringsValid; i++) {
          int len = strlen(strs[i]);
          if (len == 0 || len > 20) {
            stringsValid = false;
          }
          for (int j = 0; j < len && stringsValid; j++) {
            char c = strs[i][j];
            if (!((c >= '0' && c <= '9') || c == '.' || c == '-')) {
              stringsValid = false;
            }
          }
        }

        if (!stringsValid) {
          // dtostrf produced garbage - skip this entry silently
        } else {
          unsigned long long timestamp = getGpsUnixTimestampMillis();
          char timestampStr[24];
          if (timestamp == 0) {
            strcpy(timestampStr, "0");
          } else {
            char temp[24];
            int i = 0;
            unsigned long long t = timestamp;
            while (t > 0) {
              temp[i++] = '0' + (t % 10);
              t /= 10;
            }
            for (int j = 0; j < i; j++) {
              timestampStr[j] = temp[i - 1 - j];
            }
            timestampStr[i] = '\0';
          }

          snprintf(csvLine, sizeof(csvLine), "%s,%d,%s,%s,%s,%s,%s,%s,%s,%d,%s,%s,%s",
                   timestampStr, snapSats, hdopStr, latStr, lngStr,
                   speedStr, altStr, headingStr, hAccStr,
                   tachLastReported, accelXStr, accelYStr, accelZStr);

          size_t written = dataFile.println(csvLine);
          if (written == 0) {
            debugln(F("SD write failed - disabling logging"));
            enableLogging = false;
            sdDataLogInitComplete = false;
            dataFile.close();
            releaseSDAccess(SD_ACCESS_LOGGING);
            strncpy(internalNotification, "SD Write Failed!\nCheck card/connections", sizeof(internalNotification) - 1);
            internalNotification[sizeof(internalNotification) - 1] = '\0';
            switchToDisplayPage(PAGE_INTERNAL_FAULT);
          }
        }
      }
      /////////////////////////////////////////////////////////////////

      // Flush periodically to avoid data loss - every 10 seconds
      if (millis() - lastCardFlush > 10000) {
        lastCardFlush = millis();
        dataFile.flush();
      }
    } else if (canCreateFile) {
      debugln(F("Attempt to initialize logfile"));

      if (!acquireSDAccess(SD_ACCESS_LOGGING)) {
        debugln(F("Cannot start logging - SD card busy"));
        enableLogging = false;
        strncpy(internalNotification, "SD card busy!\nCannot start logging", sizeof(internalNotification) - 1);
        internalNotification[sizeof(internalNotification) - 1] = '\0';
        switchToDisplayPage(PAGE_INTERNAL_FAULT);
      } else {
        char dataFileName[80];

        #ifdef ENABLE_NEW_UI
        if (settingUseLegacyCsv && selectedLocation >= 0 && selectedTrack >= 0) {
          // Legacy .dove filename
          snprintf(dataFileName, sizeof(dataFileName),
                   "%s_%s_%s_20%02d_%02d%02d_%02d%02d%02d.dove",
                   locations[selectedLocation],
                   tracks[selectedTrack],
                   selectedDirection == RACE_DIRECTION_FORWARD ? "fwd" : "rev",
                   gpsData.year, gpsData.month, gpsData.day,
                   gpsData.hour, gpsData.minute, gpsData.seconds);
        } else {
          // DOVEX filename: 20YYMMDD_HHMM.dovex
          snprintf(dataFileName, sizeof(dataFileName),
                   "20%02d%02d%02d_%02d%02d.dovex",
                   gpsData.year, gpsData.month, gpsData.day,
                   gpsData.hour, gpsData.minute);
        }
        #else
        snprintf(dataFileName, sizeof(dataFileName),
                 "%s_%s_%s_20%02d_%02d%02d_%02d%02d%02d.dove",
                 locations[selectedLocation],
                 tracks[selectedTrack],
                 selectedDirection == RACE_DIRECTION_FORWARD ? "fwd" : "rev",
                 gpsData.year, gpsData.month, gpsData.day,
                 gpsData.hour, gpsData.minute, gpsData.seconds);
        #endif

        debug(F("dataFileName: ["));
        debug(dataFileName);
        debugln(F("]"));

        dataFile.open(dataFileName, O_CREAT | O_WRITE | O_TRUNC);

        if (!dataFile) {
          debugln(F("Error opening log file"));
          releaseSDAccess(SD_ACCESS_LOGGING);
          snprintf(internalNotification, sizeof(internalNotification),
                   "Error saving log:\n%s", dataFileName);
          switchToDisplayPage(PAGE_INTERNAL_FAULT);
          enableLogging = false;
        } else {
          #ifdef ENABLE_NEW_UI
          if (!settingUseLegacyCsv) {
            // DOVEX: pre-fill header region with newlines (ensures FAT clusters are allocated)
            char padBuf[64];
            memset(padBuf, '\n', sizeof(padBuf));
            for (uint32_t i = 0; i < DOVEX_HEADER_SIZE; i += sizeof(padBuf)) {
              uint32_t toWrite = min((uint32_t)sizeof(padBuf), DOVEX_HEADER_SIZE - i);
              dataFile.write(padBuf, toWrite);
            }
            // Cursor is now at exactly DOVEX_HEADER_SIZE
          }
          #endif
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

#ifdef ENABLE_NEW_UI
void GPS_SLEEP() {
  if (!gpsInitialized) return;
  stopGpsSerialTimer();  // Stop serial drain ISR during sleep (saves power)
  myGNSS.powerOff(0);   // 0 = indefinite sleep until woken
}

void GPS_WAKE() {
  if (!gpsInitialized) return;
  // Any UART activity on RX wakes u-blox from powerOff backup mode
  GPS_SERIAL.write(0xFF);
  delay(100);
  // Reset buffer pointers (stale data from before sleep is useless)
  gpsRxHead = 0;
  gpsRxTail = 0;

  startGpsSerialTimer();  // Resume serial drain ISR
  myGNSS.checkUblox();
}

void GPS_SLEEP_PERIODIC_CHECK() {
  GPS_WAKE();
  sleepGpsWakeActive = true;
  sleepGpsWakeStartedAt = millis();
}
#endif

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
