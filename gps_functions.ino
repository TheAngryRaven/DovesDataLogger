///////////////////////////////////////////
// GPS MODULE
// GPS time functions, configuration, setup, main loop, and frame rate
///////////////////////////////////////////

#ifndef GPS_CONFIGURATION
  /**
   * @brief Returns the GPS time since midnight in milliseconds
   *
   * @return unsigned long The time since midnight in milliseconds, or 0 if GPS unavailable
   */
  unsigned long getGpsTimeInMilliseconds() {
    if (!gpsInitialized || gps == NULL) return 0;

    unsigned long timeInMillis = 0;
    timeInMillis += gps->hour * 3600000ULL;   // Convert hours to milliseconds
    timeInMillis += gps->minute * 60000ULL;   // Convert minutes to milliseconds
    timeInMillis += gps->seconds * 1000ULL;   // Convert seconds to milliseconds
    timeInMillis += gps->milliseconds;        // Add the milliseconds part

    return timeInMillis;
  }

  /**
   * @brief Converts GPS date/time to Unix timestamp (seconds since Jan 1, 1970)
   *
   * @return unsigned long Unix timestamp in seconds, or 0 if GPS unavailable
   */
  unsigned long getGpsUnixTimestamp() {
    if (!gpsInitialized || gps == NULL) return 0;

    // Days in each month (non-leap year)
    const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    uint16_t year = 2000 + gps->year;
    uint8_t month = gps->month;
    uint8_t day = gps->day;

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
    timestamp += gps->hour * 3600UL;
    timestamp += gps->minute * 60UL;
    timestamp += gps->seconds;

    return timestamp;
  }

  /**
   * @brief Converts GPS date/time to Unix timestamp with millisecond precision
   *
   * @return unsigned long long Unix timestamp in milliseconds since Jan 1, 1970
   */
  unsigned long long getGpsUnixTimestampMillis() {
    if (!gpsInitialized || gps == NULL) return 0;

    // Get the Unix timestamp in seconds
    unsigned long long timestampMillis = (unsigned long long)getGpsUnixTimestamp() * 1000ULL;

    // Add the milliseconds from GPS
    timestampMillis += gps->milliseconds;

    return timestampMillis;
  }

  /**
   * @brief Sends a GPS configuration command stored in program memory to the GPS module via [GPS_SERIAL].
   *
   * This function reads a configuration command from PROGMEM (program memory) and sends it byte by byte to the GPS module using the [GPS_SERIAL] interface.
   * The function also prints the configuration command in hexadecimal format for debugging purposes.
   *
   * @note This function contains blocking code and should be used during setup only.
   *
   * @param Progmem_ptr Pointer to the PROGMEM (program memory) containing the GPS configuration command.
   * @param arraysize Size of the configuration command stored in PROGMEM.
   */
  void GPS_SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize) {
    uint8_t byteread, index;

    debug(F("GPSSend  "));

    for (index = 0; index < arraysize; index++)
    {
      byteread = pgm_read_byte_near(Progmem_ptr++);
      if (byteread < 0x10)
      {
        debug(F("0"));
      }
      debug(byteread, HEX);
      debug(F(" "));
    }

    debugln();
    //set Progmem_ptr back to start
    Progmem_ptr = Progmem_ptr - arraysize;

    for (index = 0; index < arraysize; index++)
    {
      byteread = pgm_read_byte_near(Progmem_ptr++);
      GPS_SERIAL.write(byteread);
    }
    delay(200);
  }

  void GPS_SETUP() {
    gpsInitialized = false;  // Reset flag at start of setup

    #ifndef WOKWI
      debugln(F("ACTUAL GPS SETUP"));
      // first try serial at 9600 baud
      GPS_SERIAL.begin(9600);
      // wait for the GPS to boot
      delay(2250);
      if (GPS_SERIAL) {
        GPS_SendConfig(uart115200NmeaOnly, 28);
        GPS_SERIAL.end();
      }

      // reconnect at proper baud
      gps = new Adafruit_GPS(&GPS_SERIAL);
      if (gps == NULL) {
        debugln(F("ERROR: GPS allocation failed!"));
        return;  // Leave gpsInitialized = false
      }

      GPS_SERIAL.begin(115200);
      // wait for the GPS to boot
      delay(2250);
      // Send GPS Configurations
      if (GPS_SERIAL) {
        GPS_SendConfig(NMEAVersion23, 28);
        GPS_SendConfig(FullPower, 16);

        GPS_SendConfig(GPGLLOff, 16);
        GPS_SendConfig(GPVTGOff, 16);
        GPS_SendConfig(GPGSVOff, 16);
        GPS_SendConfig(GPGSAOff, 16);
        // GPS_SendConfig(GPGGAOn5, 16); // for 10hz
        // GPS_SendConfig(GPGGAOn10, 16); // for 18hz
        GPS_SendConfig(GPGGAOn1, 16); // every packet - full 25Hz logging
        GPS_SendConfig(NavTypeAutomobile, 44);
        // GPS_SendConfig(ENABLE_GPS_ONLY, 68);
        GPS_SendConfig(ENABLE_GPS_ONLY_M10, 60);
        // GPS_SendConfig(Navrate10hz, 14);
        // GPS_SendConfig(Navrate18hz, 14);
        // GPS_SendConfig(Navrate20hz, 14);
        GPS_SendConfig(Navrate25hz, 14);

        gpsInitialized = true;  // Success!
        debugln(F("GPS initialized successfully"));
      } else {
        debugln(F("ERROR: GPS Serial not available!"));
      }
    #else
      debugln(F("WOKWI GPS SETUP"));
      // reconnect at proper baud
      gps = new Adafruit_GPS(&GPS_SERIAL);
      if (gps == NULL) {
        debugln(F("ERROR: GPS allocation failed!"));
        return;
      }
      GPS_SERIAL.begin(19200);
      gpsInitialized = true;
    #endif
  }
#endif

void GPS_LOOP() {
  // Safety check: skip if GPS not initialized (prevents null pointer crash)
  if (!gpsInitialized || gps == NULL) {
    return;
  }

  // CRITICAL: Drain ALL available characters from GPS serial buffer each call.
  // At 25Hz nav rate the GPS sends GPRMC+GPGGA = ~50 sentences/sec (~4000 chars/sec).
  // Reading only 1 char per loop() caused serial buffer overflow during display
  // updates and SD writes, dropping NMEA sentences and producing jagged/stale data.
  // Safety cap prevents runaway loop in case of hardware malfunction.
  int charsProcessed = 0;
  const int maxCharsPerCall = 512;

  while (GPS_SERIAL.available() && charsProcessed < maxCharsPerCall) {
    gps->read();
    charsProcessed++;

    if (gps->newNMEAreceived() && gps->parse(gps->lastNMEA())) {
      // Only count GPGGA/GNGGA sentences for frame rate — GPRMC is also
      // sent at the nav rate but isn't disabled, so counting all sentences
      // would double the reported Hz.
      const char* nmeaSentence = gps->lastNMEA();
      if (strstr(nmeaSentence, "$GPGGA") != NULL || strstr(nmeaSentence, "$GNGGA") != NULL) {
        gpsFrameCounter++;
      }

      // Update the lap timer with fixed GPS data
      if (trackSelected && gps->fix) {
        double ltLat = gps->latitudeDegrees;
        double ltLng = gps->longitudeDegrees;
        double ltAlt = gps->altitude;
        double ltSpeed = gps->speed;

        lapTimer.updateCurrentTime(getGpsTimeInMilliseconds());
        lapTimer.loop(ltLat, ltLng, ltAlt, ltSpeed);
      }

    #ifdef SD_CARD_LOGGING_ENABLED
      if (trackSelected && gps->fix && sdSetupSuccess && sdDataLogInitComplete && enableLogging) {

        /////////////////////////////////////////////////////////////////
        // Only log GPGGA packets (contains all the data we need, ~25Hz at our GPS settings)
        const char* nmea = gps->lastNMEA();

        // Check if this is a GPGGA or GNGGA packet
        if (strstr(nmea, "$GPGGA") != NULL || strstr(nmea, "$GNGGA") != NULL) {

          // Snapshot GPS values for logging
          // Note: No race condition here - gps->read()/parse() runs in this same
          // main-loop context, not in an ISR, so values can't change mid-read.
          double snapLat = gps->latitudeDegrees;
          double snapLng = gps->longitudeDegrees;
          double snapAlt = gps->altitude;
          double snapHdop = gps->HDOP;
          double snapSpeed = gps->speed;
          int snapSats = gps->satellites;

          // Convert speed to mph
          double snapSpeedMph = snapSpeed * 1.15078;

          // VALIDATE: Minimal checks to prevent genuinely corrupt data from being logged
          // Philosophy: log everything possible, HDOP/sats are in the CSV for post-filtering
          // Only reject data that would be file-corrupting garbage or clearly unfixed
          bool hasActualFix = (snapSats > 0);  // Must have satellites, not just fix flag
          bool validCoords = !(snapLat == 0.0 && snapLng == 0.0);  // Both zero = no position computed
          bool validLat = (snapLat >= -90.0 && snapLat <= 90.0);
          bool validLng = (snapLng >= -180.0 && snapLng <= 180.0);
          bool validAlt = (snapAlt >= -1000.0 && snapAlt <= 50000.0);
          bool validHdop = (snapHdop > 0.0);  // HDOP 0 = no fix, any positive value gets logged
          bool validSpeed = (snapSpeedMph >= 0.0 && snapSpeedMph <= 500.0);

          // Check for NaN/Inf which can slip through range checks
          bool noNaN = !isnan(snapLat) && !isnan(snapLng) && !isnan(snapAlt) &&
                       !isnan(snapHdop) && !isnan(snapSpeed);
          bool noInf = !isinf(snapLat) && !isinf(snapLng) && !isinf(snapAlt) &&
                       !isinf(snapHdop) && !isinf(snapSpeed);

          if (!hasActualFix || !validCoords || !validLat || !validLng || !validAlt ||
              !validHdop || !validSpeed || !noNaN || !noInf) {
            // Skip this sample - data is not trustworthy
            // Don't spam debug output - this can happen frequently during GPS acquisition
          } else {
            // Build CSV line: timestamp,sats,hdop,lat,lng,speed_mph,alt_m,rpm,...
            char csvLine[256];

            // Convert floats to strings with proper precision
            char latStr[24], lngStr[24], hdopStr[12], speedStr[16], altStr[16];

            // 8 decimals for lat/lng (racing precision)
            dtostrf(snapLat, 1, 8, latStr);
            dtostrf(snapLng, 1, 8, lngStr);

            // 1 decimal for HDOP
            dtostrf(snapHdop, 1, 1, hdopStr);

            // 2 decimals for speed and altitude
            dtostrf(snapSpeedMph, 1, 2, speedStr);
            dtostrf(snapAlt, 1, 2, altStr);

            // FINAL SAFETY CHECK: Verify strings are valid ASCII numbers
            // Catches any remaining garbage that slipped through numeric validation
            bool stringsValid = true;
            const char* strs[] = {latStr, lngStr, hdopStr, speedStr, altStr};
            for (int i = 0; i < 5 && stringsValid; i++) {
              int len = strlen(strs[i]);
              if (len == 0 || len > 20) {
                stringsValid = false;  // Empty or suspiciously long
              }
              for (int j = 0; j < len && stringsValid; j++) {
                char c = strs[i][j];
                // Valid: digits, decimal point, minus sign
                if (!((c >= '0' && c <= '9') || c == '.' || c == '-')) {
                  stringsValid = false;
                }
              }
            }

            if (!stringsValid) {
              // dtostrf produced garbage - skip this entry silently
            } else {
              // Convert timestamp to string manually - Arduino's snprintf doesn't support %llu
              // This was causing "lu,0,," garbage in the CSV output
              unsigned long long timestamp = getGpsUnixTimestampMillis();
              char timestampStr[24];
              // Convert 64-bit integer to string (Arduino lacks %llu support)
              if (timestamp == 0) {
                strcpy(timestampStr, "0");
              } else {
                // Build string from right to left
                char temp[24];
                int i = 0;
                unsigned long long t = timestamp;
                while (t > 0) {
                  temp[i++] = '0' + (t % 10);
                  t /= 10;
                }
                // Reverse into timestampStr
                for (int j = 0; j < i; j++) {
                  timestampStr[j] = temp[i - 1 - j];
                }
                timestampStr[i] = '\0';
              }

              // Build the complete CSV line (using %s for timestamp now)
              snprintf(csvLine, sizeof(csvLine), "%s,%d,%s,%s,%s,%s,%s,%d,0,0",
                       timestampStr,
                       snapSats,
                       hdopStr,
                       latStr,
                       lngStr,
                       speedStr,
                       altStr,
                       tachLastReported
              );

              // Write with error checking
              size_t written = dataFile.println(csvLine);
              if (written == 0) {
                debugln(F("SD write failed - disabling logging"));
                enableLogging = false;
                sdDataLogInitComplete = false;
                dataFile.close();
                releaseSDAccess(SD_ACCESS_LOGGING);
                internalNotification = "SD Write Failed!\nCheck card/connections";
                switchToDisplayPage(PAGE_INTERNAL_FAULT);
              }
            }
          }
        }
        /////////////////////////////////////////////////////////////////

        // Flush periodically to avoid data loss - every 10 seconds
        if (millis() - lastCardFlush > 10000) {
          lastCardFlush = millis();
          dataFile.flush();
        }
      } else if (trackSelected && gps->fix && sdSetupSuccess && !sdDataLogInitComplete && enableLogging && gps->day > 0) {
        debugln(F("Attempt to initialize logfile"));

        // Check if we can acquire SD access for logging
        if (!acquireSDAccess(SD_ACCESS_LOGGING)) {
          debugln(F("Cannot start logging - SD card busy"));
          enableLogging = false;
          internalNotification = "SD card busy!\nCannot start logging";
          switchToDisplayPage(PAGE_INTERNAL_FAULT);
        } else {
          // Build filename from GPS date/time
          String gpsYear = "20" + String(gps->year);
          String gpsMonth = gps->month < 10 ? "0" + String(gps->month) : String(gps->month);
          String gpsDay = gps->day < 10 ? "0" + String(gps->day) : String(gps->day);
          String gpsHour = gps->hour < 10 ? "0" + String(gps->hour) : String(gps->hour);
          String gpsMinute = gps->minute < 10 ? "0" + String(gps->minute) : String(gps->minute);
          String gpsSecond = gps->seconds < 10 ? "0" + String(gps->seconds) : String(gps->seconds);

          String trackLocation = locations[selectedLocation];
          String trackLayout= tracks[selectedTrack];
          String layoutDirection = selectedDirection == RACE_DIRECTION_FORWARD ? "fwd" : "rev";

          String dataFileNameS = trackLocation + "_" + trackLayout + "_" + layoutDirection + "_" + gpsYear + "_" + gpsMonth + gpsDay + "_" + gpsHour + gpsMinute + gpsSecond + ".dove";

          debug(F("dataFileNameS: ["));
          debug(dataFileNameS.c_str());
          debugln(F("]"));

          // Open for writing - O_TRUNC ensures clean file if name collision occurs
          dataFile.open(dataFileNameS.c_str(), O_CREAT | O_WRITE | O_TRUNC);

          if (!dataFile) {
            debugln(F("Error opening log file"));
            releaseSDAccess(SD_ACCESS_LOGGING);  // Release on failure

            String errorMessage = String("Error saving log:\n") + dataFileNameS;
            internalNotification = errorMessage;
            switchToDisplayPage(PAGE_INTERNAL_FAULT);
            enableLogging = false;
          } else {
            // Write CSV header as first line
            dataFile.println(F("timestamp,sats,hdop,lat,lng,speed_mph,altitude_m,rpm,exhaust_temp_c,water_temp_c"));
            debugln(F("CSV header written"));
            sdDataLogInitComplete = true;
          }
        }
      }
      if (gps->fix) {
        // debug(lastNMEA);
      }
    #endif

      // Update display speed - do this inside NMEA received block
      // so it only updates when we have fresh data
      gps_speed_mph = gps->speed * 1.15078;
    }
  } // end while (GPS_SERIAL.available())
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
