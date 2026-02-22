///////////////////////////////////////////
// GPS MODULE
// GPS time functions, configuration, setup, main loop, and frame rate
// Uses SparkFun u-blox GNSS v3 library with UBX PVT binary protocol
///////////////////////////////////////////

#ifndef GPS_CONFIGURATION
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
        if (!myGNSS.begin(GPS_SERIAL)) {
          debugln(F("GPS not detected at 9600 baud, trying 115200..."));
          GPS_SERIAL.end();
          delay(100);

          // Module may already be configured to 115200 from a previous session
          GPS_SERIAL.begin(GPS_BAUD_RATE);
          delay(100);
          if (!myGNSS.begin(GPS_SERIAL)) {
            debugln(F("ERROR: GPS not detected at any baud rate!"));
            return;  // Leave gpsInitialized = false
          }
          debugln(F("GPS found at 115200 baud (already configured)"));
        } else {
          // Connected at 9600, switch module to 115200
          debugln(F("GPS found at 9600, switching to 115200..."));
          myGNSS.setSerialRate(GPS_BAUD_RATE);
          delay(100);
          GPS_SERIAL.end();
          delay(100);
          GPS_SERIAL.begin(GPS_BAUD_RATE);
          delay(100);

          if (!myGNSS.begin(GPS_SERIAL)) {
            debugln(F("ERROR: GPS lost after baud switch!"));
            return;
          }
          debugln(F("GPS reconnected at 115200"));
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

        // Enable automatic PVT messages
        myGNSS.setAutoPVT(true);

        gpsInitialized = true;
        debugln(F("GPS initialized successfully (SparkFun UBX PVT)"));
      } else {
        debugln(F("ERROR: GPS Serial not available!"));
      }
    #else
      debugln(F("WOKWI GPS SETUP"));
      GPS_SERIAL.begin(19200);
      if (myGNSS.begin(GPS_SERIAL)) {
        myGNSS.setAutoPVT(true);
        gpsInitialized = true;
      } else {
        debugln(F("ERROR: WOKWI GPS not detected!"));
      }
    #endif
  }
#endif

void GPS_LOOP() {
  // Safety check: skip if GPS not initialized
  if (!gpsInitialized) {
    return;
  }

  // Process incoming UBX bytes from GPS serial buffer
  myGNSS.checkUblox();

  // Check if new PVT data is available
  if (myGNSS.getPVT()) {
    // Populate gpsData cache from PVT message
    gpsData.latitudeDegrees = myGNSS.getLatitude() / 1e7;
    gpsData.longitudeDegrees = myGNSS.getLongitude() / 1e7;
    gpsData.altitude = myGNSS.getAltitudeMSL() / 1000.0;     // mm -> meters
    gpsData.speed = myGNSS.getGroundSpeed() / 514.444;        // mm/s -> knots
    gpsData.HDOP = myGNSS.getHorizontalDOP() / 100.0;        // 0.01 units -> actual
    gpsData.satellites = myGNSS.getSIV();
    gpsData.fix = (myGNSS.getFixType() >= 2);                 // 2D or 3D fix
    gpsData.year = myGNSS.getYear() - 2000;                   // Keep 2-digit for compat
    gpsData.month = myGNSS.getMonth();
    gpsData.day = myGNSS.getDay();
    gpsData.hour = myGNSS.getHour();
    gpsData.minute = myGNSS.getMinute();
    gpsData.seconds = myGNSS.getSecond();
    gpsData.milliseconds = myGNSS.getMillisecond();

    // Count frames for frame rate calculation
    gpsFrameCounter++;

    // Update the lap timer with fresh GPS data
    if (trackSelected && gpsData.fix) {
      double ltLat = gpsData.latitudeDegrees;
      double ltLng = gpsData.longitudeDegrees;
      double ltAlt = gpsData.altitude;
      double ltSpeed = gpsData.speed;

      lapTimer.updateCurrentTime(getGpsTimeInMilliseconds());
      lapTimer.loop(ltLat, ltLng, ltAlt, ltSpeed);
    }

  #ifdef SD_CARD_LOGGING_ENABLED
    if (trackSelected && gpsData.fix && sdSetupSuccess && sdDataLogInitComplete && enableLogging) {

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
      /////////////////////////////////////////////////////////////////

      // Flush periodically to avoid data loss - every 10 seconds
      if (millis() - lastCardFlush > 10000) {
        lastCardFlush = millis();
        dataFile.flush();
      }
    } else if (trackSelected && gpsData.fix && sdSetupSuccess && !sdDataLogInitComplete && enableLogging && gpsData.day > 0) {
      debugln(F("Attempt to initialize logfile"));

      // Check if we can acquire SD access for logging
      if (!acquireSDAccess(SD_ACCESS_LOGGING)) {
        debugln(F("Cannot start logging - SD card busy"));
        enableLogging = false;
        internalNotification = "SD card busy!\nCannot start logging";
        switchToDisplayPage(PAGE_INTERNAL_FAULT);
      } else {
        // Build filename from GPS date/time
        String gpsYear = "20" + String(gpsData.year);
        String gpsMonth = gpsData.month < 10 ? "0" + String(gpsData.month) : String(gpsData.month);
        String gpsDay = gpsData.day < 10 ? "0" + String(gpsData.day) : String(gpsData.day);
        String gpsHour = gpsData.hour < 10 ? "0" + String(gpsData.hour) : String(gpsData.hour);
        String gpsMinute = gpsData.minute < 10 ? "0" + String(gpsData.minute) : String(gpsData.minute);
        String gpsSecond = gpsData.seconds < 10 ? "0" + String(gpsData.seconds) : String(gpsData.seconds);

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
    if (gpsData.fix) {
      // debug(lastNMEA);
    }
  #endif

    // Update display speed with fresh PVT data
    gps_speed_mph = gpsData.speed * 1.15078;
  } // end if (myGNSS.getPVT())
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
