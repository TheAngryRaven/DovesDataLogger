///////////////////////////////////////////
// BLUETOOTH (BLE) MODULE
// All BLE-related functions: callbacks, setup, file transfer, loop
///////////////////////////////////////////

void bleConnectCallback(uint16_t conn_handle) {
  debugln(F("BLE: Device connected!"));
  bleConnected = true;

  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  debug(F("BLE: Initial MTU: "));
  debugln(connection->getMtu());

  // Request MTU exchange
  debugln(F("BLE: Requesting MTU exchange to 247..."));
  if (connection->requestMtuExchange(247)) {
    debugln(F("BLE: MTU exchange requested successfully"));
  } else {
    debugln(F("BLE: MTU exchange request failed!"));
  }

  // Wait for MTU negotiation
  delay(500);

  bleNegotiatedMtu = connection->getMtu();
  debug(F("BLE: Negotiated MTU: "));
  debugln(bleNegotiatedMtu);

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
  // Note: BLE callbacks run in a separate FreeRTOS task from loop().
  // Directory listing is read-only and brief, so we don't acquire exclusive
  // SD access (that would block listing during active logging).
  // If the SD is mid-flush and open fails, we send BUSY gracefully.
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

void bleStartFileTransfer(String filename) {
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

  bleCurrentFile = SD.open(filename.c_str(), FILE_READ);

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

  delay(50);
}

void bleDeleteFile(String filename) {
  debug(F("BLE: Deleting file: ["));
  debug(filename);
  debugln(F("]"));

  if (SD.exists(filename.c_str())) {
    if (SD.remove(filename.c_str())) {
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
  memcpy(buffer, data, min(len, (uint16_t)64));

  String command = String(buffer);
  command.trim();

  debug(F("BLE: Received command: ["));
  debug(command);
  debugln(F("]"));

  if (command.startsWith("LIST")) {
    bleSendFileList();
  } else if (command.startsWith("GET:")) {
    String filename = command.substring(4);
    filename.trim();
    bleStartFileTransfer(filename);
  } else if (command.startsWith("DELETE:")) {
    String filename = command.substring(7);
    filename.trim();
    bleDeleteFile(filename);
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

  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin();
  Bluefruit.setTxPower(4);
  Bluefruit.setName("DovesLapTimer");

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

  // Close any open file and release SD access
  if (bleCurrentFile) {
    bleCurrentFile.close();
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
  }
  bleTransferInProgress = false;

  // Disconnect any connected device
  if (Bluefruit.connected()) {
    Bluefruit.disconnect(Bluefruit.connHandle());
    delay(100);
  }

  // Stop advertising
  Bluefruit.Advertising.stop();

  // Turn off the BLE LED
  Bluefruit.autoConnLed(false);
  Bluefruit.setConnLedInterval(0);
  digitalWrite(LED_BLUE, HIGH);

  bleConnected = false;
  bleActive = false;

  debugln(F("BLE: Bluetooth stopped"));
}

void BLUETOOTH_LOOP() {
  if (!bleActive) return;

  if (bleTransferInProgress && bleCurrentFile && Bluefruit.connected()) {
    // Use actual negotiated MTU
    uint16_t maxChunk = bleNegotiatedMtu - 3;

    uint8_t buffer[524];

    size_t chunkSize = min(maxChunk, (uint16_t)244);
    size_t bytesRead = bleCurrentFile.read(buffer, chunkSize);

    if (bytesRead > 0) {
      if (fileDataChar.notify(buffer, bytesRead)) {
        bleBytesTransferred += bytesRead;

        // Progress update every 10KB
        if (bleBytesTransferred % 10000 == 0) {
          debug(F("BLE: Progress: "));
          debug(bleBytesTransferred);
          debug(F(" / "));
          debug(bleFileSize);
          debug(F(" ("));
          debug((bleBytesTransferred * 100) / bleFileSize);
          debugln(F("%)"));
        }
      }
    } else {
      // Transfer complete
      bleCurrentFile.close();
      bleTransferInProgress = false;
      releaseSDAccess(SD_ACCESS_BLE_TRANSFER);  // Release SD access when transfer completes

      debugln(F("BLE: Transfer complete!"));

      fileStatusChar.notify((uint8_t*)"DONE", 4);
    }
  }
}
