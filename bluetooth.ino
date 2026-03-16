///////////////////////////////////////////
// BLUETOOTH (BLE) MODULE
// All BLE-related functions: callbacks, setup, file transfer, loop
///////////////////////////////////////////

// Deferred settings command buffer (BLE callback -> main loop)
static volatile bool settingsCmdPending = false;
static char settingsCmdBuffer[65];  // 64 chars + null

// Track upload state (BLE callback -> main loop)
static volatile bool trackUploadActive = false;
static volatile bool trackUploadReady = false;      // signals main loop to send TREADY
static volatile bool trackUploadComplete = false;    // signals main loop to write file
static volatile bool trackUploadError = false;
static char trackUploadFilename[25];                 // just the filename (e.g. "OKC.json")
static char trackUploadBuffer[4096];
static volatile uint16_t trackUploadOffset = 0;

// Track delete state (BLE callback -> main loop)
static volatile bool trackDeletePending = false;
static char trackDeleteFilename[25];

void bleConnectCallback(uint16_t conn_handle) {
  debugln(F("BLE: Device connected!"));
  bleConnected = true;

  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  debug(F("BLE: Initial MTU: "));
  debugln(connection->getMtu());

  // Request MTU exchange - result will be read in BLUETOOTH_LOOP() after 500ms
  debugln(F("BLE: Requesting MTU exchange to 247..."));
  if (connection->requestMtuExchange(247)) {
    debugln(F("BLE: MTU exchange requested successfully"));
  } else {
    debugln(F("BLE: MTU exchange request failed!"));
  }

  // Request 2M PHY for double raw throughput (BLE 5.0, both sides must support)
  connection->requestPHY(BLE_GAP_PHY_2MBPS);
  // Request Data Length Extension (max PDU size, reduces L2CAP overhead)
  connection->requestDataLengthUpdate();

  // Defer MTU read to main loop instead of blocking here with delay(500)
  bleWaitingForMTU = true;
  bleMTURequestTime = millis();
  bleMTUConnHandle = conn_handle;

  debug(F("BLE: Connection interval: "));
  debug(connection->getConnectionInterval() * 1.25);
  debugln(F("ms"));
}

void bleDisconnectCallback(uint16_t conn_handle, uint8_t reason) {
  debugln(F("BLE: Disconnected!"));
  bleConnected = false;
  bleNegotiatedMtu = 23; // Reset to default
  if (bleCurrentFile) {
    bleCurrentFile.close();
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);  // Release SD access on disconnect
  }
  bleTransferInProgress = false;
  // Reset track upload/delete state on disconnect
  trackUploadActive = false;
  trackUploadReady = false;
  trackUploadComplete = false;
  trackUploadError = false;
  trackDeletePending = false;

  // Auto-reboot after BLE disconnect to apply any changed settings —
  // but only if the phone disconnected (not us calling BLE_STOP()).
  // BLE_STOP() sets bleActive=false before triggering async disconnect.
  if (!bleActive) {
    debugln(F("BLE: Local disconnect (BLE_STOP), skipping reboot"));
  } else if (enableLogging) {
    debugln(F("BLE: Skipping reboot (logging active)"));
  } else {
    debugln(F("BLE: Rebooting to apply settings..."));
    delay(100);  // Brief delay for debug output to flush
    NVIC_SystemReset();
  }
}

// Forward declaration for callback
void bleFileRequestCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);

void bleSetupFileService() {
  fileService.begin();

  // File List Characteristic
  fileListChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  fileListChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  fileListChar.setMaxLen(244);
  fileListChar.begin();

  // File Request Characteristic
  fileRequestChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  fileRequestChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  fileRequestChar.setMaxLen(64);
  fileRequestChar.setWriteCallback(bleFileRequestCallback);
  fileRequestChar.begin();

  // File Data Characteristic
  fileDataChar.setProperties(CHR_PROPS_NOTIFY);
  fileDataChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  fileDataChar.setMaxLen(244);
  fileDataChar.begin();

  // File Status Characteristic
  fileStatusChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  fileStatusChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  fileStatusChar.setMaxLen(64);
  fileStatusChar.begin();
}

void bleStartAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(fileService);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

void bleSendFileList() {
  // BLE callbacks run in a separate FreeRTOS task from loop().
  // SdFat is NOT thread-safe — concurrent access from BLE task and main
  // loop (e.g. CSV logging) can corrupt internal state.
  // Check if SD is in use and return BUSY if so.
  if (currentSDAccess != SD_ACCESS_NONE) {
    debugln(F("BLE: SD busy, cannot list files"));
    fileListChar.notify((uint8_t*)"BUSY", 4);
    return;
  }

  File32 root = SD.open("/");
  if (!root) {
    debugln(F("BLE: Failed to open root directory"));
    fileListChar.notify((uint8_t*)"BUSY", 4);
    return;
  }

  // Stream entries directly over BLE using a fixed buffer per entry
  // instead of building one giant String (avoids heap fragmentation
  // that was silently truncating the file list)
  char entryBuf[300];
  int fileCount = 0;
  bool firstEntry = true;

  while (true) {
    File32 entry = root.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory()) {
      char name[256];
      entry.getName(name, sizeof(name));

      // Build single entry: "|name:size" (skip | for first entry)
      int len = snprintf(entryBuf, sizeof(entryBuf), "%s%s:%lu",
                         firstEntry ? "" : "|",
                         name,
                         (unsigned long)entry.size());
      firstEntry = false;

      if (len > 0 && len < (int)sizeof(entryBuf)) {
        fileListChar.notify((uint8_t*)entryBuf, len);
        delay(10);
        fileCount++;
      }
    }
    entry.close();
  }
  root.close();

  fileListChar.notify((uint8_t*)"END", 3);
  debug(F("BLE: File list sent, "));
  debug(fileCount);
  debugln(F(" files"));
}

void bleSendTrackList() {
  if (currentSDAccess != SD_ACCESS_NONE) {
    debugln(F("BLE: SD busy, cannot list tracks"));
    fileStatusChar.notify((uint8_t*)"TERR:SD_BUSY", 12);
    return;
  }

  File32 trackDir2 = SD.open("/TRACKS/");
  if (!trackDir2) {
    debugln(F("BLE: Failed to open TRACKS directory"));
    fileStatusChar.notify((uint8_t*)"TEND", 4);
    return;
  }

  int fileCount = 0;
  while (true) {
    File32 entry = trackDir2.openNextFile();
    if (!entry) break;

    if (!entry.isDirectory()) {
      char name[64];
      entry.getName(name, sizeof(name));

      char msg[70];
      int len = snprintf(msg, sizeof(msg), "TFILE:%s", name);
      if (len > 0 && len < (int)sizeof(msg)) {
        fileStatusChar.notify((uint8_t*)msg, len);
        delay(10);
        fileCount++;
      }
    }
    entry.close();
  }
  trackDir2.close();

  fileStatusChar.notify((uint8_t*)"TEND", 4);
  debug(F("BLE: Track list sent, "));
  debug(fileCount);
  debugln(F(" files"));
}

void bleStartFileTransfer(const char* filename) {
  if (bleCurrentFile) bleCurrentFile.close();

  // Check if we can acquire SD access for BLE transfer
  if (!acquireSDAccess(SD_ACCESS_BLE_TRANSFER)) {
    debugln(F("BLE: SD card busy - cannot start transfer"));
    fileStatusChar.notify((uint8_t*)"BUSY", 4);
    return;
  }

  debug(F("BLE: Opening file: ["));
  debug(filename);
  debugln(F("]"));

  bleCurrentFile = SD.open(filename, FILE_READ);

  if (!bleCurrentFile) {
    debugln(F("BLE: Failed to open file!"));
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);  // Release on failure
    fileStatusChar.notify((uint8_t*)"ERROR", 5);
    return;
  }

  bleFileSize = bleCurrentFile.size();
  bleBytesTransferred = 0;
  bleTransferInProgress = true;

  debug(F("BLE: File size: "));
  debug(bleFileSize);
  debug(F(" bytes, MTU: "));
  debugln(bleNegotiatedMtu);

  char sizeMsg[32];
  snprintf(sizeMsg, sizeof(sizeMsg), "SIZE:%lu", bleFileSize);
  fileStatusChar.notify((uint8_t*)sizeMsg, strlen(sizeMsg));
}

void bleDeleteFile(const char* filename) {
  debug(F("BLE: Deleting file: ["));
  debug(filename);
  debugln(F("]"));

  if (SD.exists(filename)) {
    if (SD.remove(filename)) {
      debugln(F("BLE: File deleted successfully"));
      fileStatusChar.notify((uint8_t*)"DELETED", 7);
    } else {
      debugln(F("BLE: Failed to delete file"));
      fileStatusChar.notify((uint8_t*)"DEL_ERR", 7);
    }
  } else {
    debugln(F("BLE: File not found"));
    fileStatusChar.notify((uint8_t*)"NOT_FOUND", 9);
  }
}

void bleFileRequestCallback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  char buffer[65];
  memset(buffer, 0, sizeof(buffer));
  uint16_t copyLen = len < 64 ? len : 64;
  memcpy(buffer, data, copyLen);

  // Trim trailing whitespace/newlines in-place
  int end = strlen(buffer) - 1;
  while (end >= 0 && (buffer[end] == ' ' || buffer[end] == '\r' || buffer[end] == '\n')) {
    buffer[end--] = '\0';
  }

  // Handle upload data mode — all writes are raw data until TDONE
  if (trackUploadActive) {
    if (strncmp(buffer, "TDONE", 5) == 0 && len <= 6) {
      debugln(F("BLE: TDONE received"));
      trackUploadComplete = true;
      return;
    }
    // Append raw data to buffer
    if (trackUploadOffset + len <= sizeof(trackUploadBuffer)) {
      memcpy(trackUploadBuffer + trackUploadOffset, data, len);
      trackUploadOffset += len;
    } else {
      trackUploadError = true;
    }
    return;
  }

  debug(F("BLE: Received command: ["));
  debug(buffer);
  debugln(F("]"));

  if (strncmp(buffer, "LIST", 4) == 0) {
    bleSendFileList();
  } else if (strncmp(buffer, "GET:", 4) == 0) {
    // Skip "GET:" prefix and trim leading whitespace
    char* filename = buffer + 4;
    while (*filename == ' ') filename++;
    bleStartFileTransfer(filename);
  } else if (strncmp(buffer, "DELETE:", 7) == 0) {
    // Skip "DELETE:" prefix and trim leading whitespace
    char* filename = buffer + 7;
    while (*filename == ' ') filename++;
    bleDeleteFile(filename);
  } else if (strcmp(buffer, "SLIST") == 0 ||
             strncmp(buffer, "SGET:", 5) == 0 ||
             strncmp(buffer, "SSET:", 5) == 0 ||
             strcmp(buffer, "SRESET") == 0) {
    // Settings commands — defer to main loop for thread-safe SD access
    if (settingsCmdPending) {
      fileStatusChar.notify((uint8_t*)"SBUSY", 5);
      return;
    }
    strncpy(settingsCmdBuffer, buffer, sizeof(settingsCmdBuffer) - 1);
    settingsCmdBuffer[sizeof(settingsCmdBuffer) - 1] = '\0';
    settingsCmdPending = true;

  // Track management commands
  } else if (strcmp(buffer, "TLIST") == 0) {
    bleSendTrackList();
  } else if (strncmp(buffer, "TGET:", 5) == 0) {
    char filepath[FILEPATH_MAX];
    snprintf(filepath, sizeof(filepath), "/TRACKS/%s", buffer + 5);
    bleStartFileTransfer(filepath);
  } else if (strncmp(buffer, "TPUT:", 5) == 0) {
    if (trackUploadActive || bleTransferInProgress) {
      fileStatusChar.notify((uint8_t*)"TERR:BUSY", 9);
      return;
    }
    strncpy(trackUploadFilename, buffer + 5, sizeof(trackUploadFilename) - 1);
    trackUploadFilename[sizeof(trackUploadFilename) - 1] = '\0';
    trackUploadOffset = 0;
    trackUploadError = false;
    trackUploadComplete = false;
    trackUploadReady = true;
    trackUploadActive = true;
    debugln(F("BLE: Track upload started"));
  } else if (strncmp(buffer, "TDEL:", 5) == 0) {
    if (trackDeletePending) {
      fileStatusChar.notify((uint8_t*)"TERR:BUSY", 9);
      return;
    }
    strncpy(trackDeleteFilename, buffer + 5, sizeof(trackDeleteFilename) - 1);
    trackDeleteFilename[sizeof(trackDeleteFilename) - 1] = '\0';
    trackDeletePending = true;

  // Battery query — no SD access needed, uses cached voltage
  } else if (strcmp(buffer, "BATT") == 0) {
    int pct = getBatteryPercent(lastBatteryVoltage);
    char vbuf[8];
    dtostrf(lastBatteryVoltage, 4, 2, vbuf);
    char response[24];
    snprintf(response, sizeof(response), "BATT:%d,%s", pct, vbuf);
    fileStatusChar.notify((uint8_t*)response, strlen(response));
  }
}

void BLE_SETUP() {
  if (bleInitialized) {
    // Already initialized, just start advertising
    debugln(F("BLE: Restarting advertising..."));
    Bluefruit.autoConnLed(true);
    Bluefruit.setConnLedInterval(250); // Blink every 250ms when connected
    Bluefruit.Advertising.start(0);
    bleActive = true;
    return;
  }

  debugln(F("BLE: Initializing Bluetooth..."));

  // Custom BLE config for max file transfer throughput:
  // MTU 247, event_len 100 (125ms max radio time per event),
  // HVN TX queue 10 (up from BANDWIDTH_MAX's 3 — deeper notification pipeline),
  // WrCmd queue 1 (default, we don't use write commands).
  Bluefruit.configPrphConn(247, 100, 10, 1);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  char bleName[32];
  if (getSetting("bluetooth_name", bleName, sizeof(bleName))) {
    debug(F("BLE: Name from settings: "));
    debugln(bleName);
    Bluefruit.setName(bleName);
  } else {
    debugln(F("BLE: WARNING - bluetooth_name not found, using fallback"));
    Bluefruit.setName("DovesDataLogger");
  }

  // Enable connection LED
  Bluefruit.autoConnLed(true);
  Bluefruit.setConnLedInterval(250);

  Bluefruit.Periph.setConnectCallback(bleConnectCallback);
  Bluefruit.Periph.setDisconnectCallback(bleDisconnectCallback);

  // Set connection interval (7.5-15ms)
  Bluefruit.Periph.setConnInterval(6, 12);

  bleSetupFileService();
  bleStartAdvertising();

  bleInitialized = true;
  bleActive = true;

  debugln(F("BLE: Ready for connection!"));
}

void BLE_STOP() {
  if (!bleActive) return;

  debugln(F("BLE: Stopping Bluetooth..."));

  // Mark inactive BEFORE disconnect so the async bleDisconnectCallback
  // knows this was a local stop (not a phone disconnect) and skips reboot.
  bleActive = false;

  // Close any open file and release SD access
  if (bleCurrentFile) {
    bleCurrentFile.close();
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
  }
  bleTransferInProgress = false;

  // Disconnect any connected device
  if (Bluefruit.connected()) {
    Bluefruit.disconnect(Bluefruit.connHandle());
    // BLE disconnect is async; no delay needed - stack handles it
  }

  // Stop advertising
  Bluefruit.Advertising.stop();

  // Turn off the BLE LED
  Bluefruit.autoConnLed(false);
  Bluefruit.setConnLedInterval(0);
  digitalWrite(LED_BLUE, HIGH);

  bleConnected = false;
  // bleActive already set false at top of BLE_STOP()

  debugln(F("BLE: Bluetooth stopped"));
}

void processSettingsCommand() {
  debug(F("BLE: Processing settings cmd: ["));
  debug(settingsCmdBuffer);
  debugln(F("]"));

  if (strcmp(settingsCmdBuffer, "SLIST") == 0) {
    debugln(F("BLE: SLIST - listing all settings"));
    if (!acquireSDAccess(SD_ACCESS_TRACK_PARSE)) {
      debugln(F("BLE: SLIST - SD busy"));
      fileStatusChar.notify((uint8_t*)"SERR:SD_BUSY", 12);
      return;
    }

    File settingsFile;
    settingsFile.open("/SETTINGS.json", O_READ);
    if (!settingsFile) {
      debugln(F("BLE: SLIST - failed to open settings file"));
      releaseSDAccess(SD_ACCESS_TRACK_PARSE);
      fileStatusChar.notify((uint8_t*)"SERR:NO_FILE", 12);
      return;
    }

    char fileBuf[512];
    int bytesRead = settingsFile.read(fileBuf, sizeof(fileBuf) - 1);
    settingsFile.close();
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);

    debug(F("BLE: SLIST - read "));
    debug(bytesRead);
    debugln(F(" bytes"));

    if (bytesRead <= 0) {
      debugln(F("BLE: SLIST - file empty"));
      fileStatusChar.notify((uint8_t*)"SERR:EMPTY", 10);
      return;
    }
    fileBuf[bytesRead] = '\0';

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, fileBuf);
    if (err != DeserializationError::Ok) {
      debug(F("BLE: SLIST - JSON parse error: "));
      debugln(err.c_str());
      fileStatusChar.notify((uint8_t*)"SERR:PARSE", 10);
      return;
    }

    int count = 0;
    for (JsonPair kv : doc.as<JsonObject>()) {
      char entry[64];
      snprintf(entry, sizeof(entry), "SVAL:%s=%s", kv.key().c_str(), kv.value().as<const char*>());
      debug(F("BLE: SLIST - sending: "));
      debugln(entry);
      fileStatusChar.notify((uint8_t*)entry, strlen(entry));
      delay(10);  // BLE notify spacing
      count++;
    }
    debugln(F("BLE: SLIST - sending SEND"));
    fileStatusChar.notify((uint8_t*)"SEND", 4);
    debug(F("BLE: SLIST - done, sent "));
    debug(count);
    debugln(F(" entries"));

  } else if (strncmp(settingsCmdBuffer, "SGET:", 5) == 0) {
    char* key = settingsCmdBuffer + 5;
    debug(F("BLE: SGET - key: ["));
    debug(key);
    debugln(F("]"));

    char valueBuf[48];
    if (getSetting(key, valueBuf, sizeof(valueBuf))) {
      char response[64];
      snprintf(response, sizeof(response), "SVAL:%s=%s", key, valueBuf);
      debug(F("BLE: SGET - responding: "));
      debugln(response);
      fileStatusChar.notify((uint8_t*)response, strlen(response));
    } else {
      debugln(F("BLE: SGET - key not found"));
      fileStatusChar.notify((uint8_t*)"SERR:NOT_FOUND", 14);
    }

  } else if (strncmp(settingsCmdBuffer, "SSET:", 5) == 0) {
    char* payload = settingsCmdBuffer + 5;
    char* eq = strchr(payload, '=');
    if (!eq) {
      debugln(F("BLE: SSET - missing '=' in command"));
      fileStatusChar.notify((uint8_t*)"SERR:BAD_CMD", 12);
      return;
    }
    *eq = '\0';
    char* key = payload;
    char* value = eq + 1;

    debug(F("BLE: SSET - key: ["));
    debug(key);
    debug(F("] value: ["));
    debug(value);
    debugln(F("]"));

    if (setSetting(key, value)) {
      char response[64];
      snprintf(response, sizeof(response), "SOK:%s", key);
      debug(F("BLE: SSET - success: "));
      debugln(response);
      fileStatusChar.notify((uint8_t*)response, strlen(response));
    } else {
      debugln(F("BLE: SSET - write failed"));
      fileStatusChar.notify((uint8_t*)"SERR:WRITE_FAIL", 15);
    }
  } else if (strcmp(settingsCmdBuffer, "SRESET") == 0) {
    debugln(F("BLE: SRESET - resetting all settings to defaults"));
    if (resetSettings()) {
      fileStatusChar.notify((uint8_t*)"SOK:RESET", 9);
      debugln(F("BLE: Settings reset, rebooting in 200ms..."));
      delay(200);  // Let the notification reach the phone
      NVIC_SystemReset();
    } else {
      fileStatusChar.notify((uint8_t*)"SERR:RESET_FAIL", 15);
    }
  } else {
    debug(F("BLE: Unknown settings cmd: ["));
    debug(settingsCmdBuffer);
    debugln(F("]"));
  }
}

void processTrackUpload() {
  debug(F("BLE: Writing track file: ["));
  debug(trackUploadFilename);
  debug(F("] size: "));
  debugln(trackUploadOffset);

  if (trackUploadError) {
    debugln(F("BLE: Track upload too large"));
    fileStatusChar.notify((uint8_t*)"TERR:TOO_LARGE", 14);
    trackUploadActive = false;
    trackUploadComplete = false;
    trackUploadError = false;
    return;
  }

  if (!acquireSDAccess(SD_ACCESS_BLE_TRANSFER)) {
    debugln(F("BLE: SD busy, cannot write track"));
    fileStatusChar.notify((uint8_t*)"TERR:SD_BUSY", 12);
    trackUploadActive = false;
    trackUploadComplete = false;
    return;
  }

  char filepath[FILEPATH_MAX];
  snprintf(filepath, sizeof(filepath), "/TRACKS/%s", trackUploadFilename);

  // Delete existing file if present
  if (SD.exists(filepath)) {
    SD.remove(filepath);
  }

  File32 outFile = SD.open(filepath, FILE_WRITE);
  if (!outFile) {
    debugln(F("BLE: Failed to create track file"));
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
    fileStatusChar.notify((uint8_t*)"TERR:WRITE_FAIL", 15);
    trackUploadActive = false;
    trackUploadComplete = false;
    return;
  }

  size_t written = outFile.write((uint8_t*)trackUploadBuffer, trackUploadOffset);
  outFile.close();
  releaseSDAccess(SD_ACCESS_BLE_TRANSFER);

  if (written != trackUploadOffset) {
    debugln(F("BLE: Track file write incomplete"));
    fileStatusChar.notify((uint8_t*)"TERR:WRITE_FAIL", 15);
  } else {
    debugln(F("BLE: Track file written successfully"));
    fileStatusChar.notify((uint8_t*)"TOK", 3);
    // Refresh in-memory track list
    buildTrackList();
  }

  trackUploadActive = false;
  trackUploadComplete = false;
  trackUploadError = false;
}

void processTrackDelete() {
  debug(F("BLE: Deleting track file: ["));
  debug(trackDeleteFilename);
  debugln(F("]"));

  if (!acquireSDAccess(SD_ACCESS_BLE_TRANSFER)) {
    debugln(F("BLE: SD busy, cannot delete track"));
    fileStatusChar.notify((uint8_t*)"TERR:SD_BUSY", 12);
    trackDeletePending = false;
    return;
  }

  char filepath[FILEPATH_MAX];
  snprintf(filepath, sizeof(filepath), "/TRACKS/%s", trackDeleteFilename);

  if (!SD.exists(filepath)) {
    debugln(F("BLE: Track file not found"));
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
    fileStatusChar.notify((uint8_t*)"TERR:NO_FILE", 12);
    trackDeletePending = false;
    return;
  }

  if (SD.remove(filepath)) {
    debugln(F("BLE: Track file deleted successfully"));
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
    fileStatusChar.notify((uint8_t*)"TOK", 3);
    buildTrackList();
  } else {
    debugln(F("BLE: Failed to delete track file"));
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
    fileStatusChar.notify((uint8_t*)"TERR:WRITE_FAIL", 15);
  }

  trackDeletePending = false;
}

void BLUETOOTH_LOOP() {
  if (!bleActive) return;

  // Process deferred settings commands (thread-safe: runs in main loop)
  if (settingsCmdPending) {
    processSettingsCommand();
    settingsCmdPending = false;
  }

  // Process track upload state machine
  if (trackUploadReady) {
    fileStatusChar.notify((uint8_t*)"TREADY", 6);
    trackUploadReady = false;
  }

  if (trackUploadComplete) {
    processTrackUpload();
  }

  if (trackDeletePending) {
    processTrackDelete();
  }

  // Deferred MTU negotiation - read result 500ms after request
  if (bleWaitingForMTU && millis() - bleMTURequestTime >= 500) {
    bleWaitingForMTU = false;
    BLEConnection* connection = Bluefruit.Connection(bleMTUConnHandle);
    if (connection) {
      bleNegotiatedMtu = connection->getMtu();
      debug(F("BLE: Negotiated MTU: "));
      debugln(bleNegotiatedMtu);
    }
  }

  if (bleTransferInProgress && bleCurrentFile && Bluefruit.connected()) {
    // Use actual negotiated MTU
    uint16_t maxChunk = bleNegotiatedMtu - 3;
    uint8_t buffer[524];
    size_t chunkSize = min(maxChunk, (uint16_t)244);

    // Burst send: read + notify multiple chunks per loop iteration.
    // notify() blocks via semaphore when the SoftDevice TX queue is full,
    // providing natural flow control. This keeps the pipeline fed instead
    // of sending 1 lonely chunk then wasting time on button checks.
    for (int burst = 0; burst < 10 && bleTransferInProgress; burst++) {
      size_t bytesRead = bleCurrentFile.read(buffer, chunkSize);

      if (bytesRead > 0) {
        if (!fileDataChar.notify(buffer, bytesRead)) {
          break;  // Disconnected or error
        }
        bleBytesTransferred += bytesRead;
      } else {
        // Transfer complete
        bleCurrentFile.close();
        bleTransferInProgress = false;
        releaseSDAccess(SD_ACCESS_BLE_TRANSFER);

        debugln(F("BLE: Transfer complete!"));
        fileStatusChar.notify((uint8_t*)"DONE", 4);
        break;
      }
    }
  }
}
