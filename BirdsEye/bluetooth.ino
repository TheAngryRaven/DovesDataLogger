///////////////////////////////////////////
// BLUETOOTH (BLE) MODULE
// All BLE-related functions: callbacks, setup, file transfer, loop
///////////////////////////////////////////

#include "bluetooth.h"
#include "filename_validator.h"
#include "firmware_ota.h"

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

// Deferred file command buffer (BLE callback -> main loop). Carries the
// SD-touching commands (LIST / GET: / DELETE: / TLIST / TGET:) so SdFat is
// only ever driven from the main-loop task — the Bluefruit callback task
// can preempt an in-flight SD write, and SdFat is not thread-safe.
static volatile bool fileCmdPending = false;
static char fileCmdBuffer[65];

// Queue an SD-touching command for BLUETOOTH_LOOP(). Returns false if one
// is already pending — the caller sends its protocol-appropriate busy reply.
// The buffer is stable once fileCmdPending is set: the callback refuses new
// commands until the main loop has processed it and cleared the flag.
static bool deferFileCommand(const char* cmd) {
  if (fileCmdPending) return false;
  strncpy(fileCmdBuffer, cmd, sizeof(fileCmdBuffer) - 1);
  fileCmdBuffer[sizeof(fileCmdBuffer) - 1] = '\0';
  fileCmdPending = true;
  return true;
}

// Set by the disconnect callback; BLUETOOTH_LOOP() performs the SD teardown
// (close transfer/staging file, release SD, abort OTA) and the auto-reboot
// on the main loop, so SdFat is only ever touched by one task.
static volatile bool bleDisconnectCleanupPending = false;

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

  // Pure in-RAM flag resets are safe from this callback (Bluefruit) task.
  bleTransferInProgress = false;
  trackUploadActive = false;
  trackUploadReady = false;
  trackUploadComplete = false;
  trackUploadError = false;
  trackDeletePending = false;
  // Drop any queued-but-unprocessed commands so they can't fire on behalf
  // of a peer that is no longer connected (or after a reconnect).
  fileCmdPending = false;
  settingsCmdPending = false;

  // Everything that touches SdFat — closing the in-flight transfer/staging
  // file, releasing SD access, aborting the OTA — plus the auto-reboot is
  // DEFERRED to BLUETOOTH_LOOP() on the main loop. This callback runs in the
  // Bluefruit task, which can preempt an in-flight SD write in the main loop,
  // and SdFat is not thread-safe. If this was a local BLE_STOP() (which sets
  // bleActive=false before disconnecting), BLE_STOP() already did the
  // teardown on the main loop, so there is nothing to defer.
  if (bleActive) {
    bleDisconnectCleanupPending = true;
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

  // File Request Characteristic. Max length is 244 so the firmware-OTA
  // path can receive ~240-byte raw image chunks (text commands and the
  // legacy 64-byte track-upload chunks fit comfortably inside this).
  fileRequestChar.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  fileRequestChar.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  fileRequestChar.setMaxLen(244);
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
  // Runs on the main loop (deferred via fileCmdBuffer). Hold the SD lock
  // for the entire walk — the delay(10) per entry yields to other tasks,
  // so ownership must be held, not just peeked. The explicit free-check
  // first keeps the idempotent/preempting acquire from piggybacking on an
  // active transfer or stealing a track parse.
  if (currentSDAccess != SD_ACCESS_NONE ||
      !acquireSDAccess(SD_ACCESS_BLE_TRANSFER)) {
    debugln(F("BLE: SD busy, cannot list files"));
    fileListChar.notify((uint8_t*)"BUSY", 4);
    return;
  }

  File32 root = SD.open("/");
  if (!root) {
    debugln(F("BLE: Failed to open root directory"));
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
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
  releaseSDAccess(SD_ACCESS_BLE_TRANSFER);

  fileListChar.notify((uint8_t*)"END", 3);
  debug(F("BLE: File list sent, "));
  debug(fileCount);
  debugln(F(" files"));
}

void bleSendTrackList() {
  // Same locking discipline as bleSendFileList() — see the comment there.
  if (currentSDAccess != SD_ACCESS_NONE ||
      !acquireSDAccess(SD_ACCESS_BLE_TRANSFER)) {
    debugln(F("BLE: SD busy, cannot list tracks"));
    fileStatusChar.notify((uint8_t*)"TERR:SD_BUSY", 12);
    return;
  }

  File32 trackDir2 = SD.open("/TRACKS/");
  if (!trackDir2) {
    debugln(F("BLE: Failed to open TRACKS directory"));
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
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
  releaseSDAccess(SD_ACCESS_BLE_TRANSFER);

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

  // An active transfer holds SD_ACCESS_BLE_TRANSFER, and the same-mode
  // re-acquire below would succeed — guard explicitly so a DELETE can't
  // remove the file being streamed and then drop the transfer's lock.
  if (bleTransferInProgress) {
    debugln(F("BLE: transfer in progress, cannot delete"));
    fileStatusChar.notify((uint8_t*)"BUSY", 4);
    return;
  }
  if (!acquireSDAccess(SD_ACCESS_BLE_TRANSFER)) {
    debugln(F("BLE: SD busy, cannot delete"));
    fileStatusChar.notify((uint8_t*)"BUSY", 4);
    return;
  }

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

  releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
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

  // Firmware OTA image stream: while receiving, every write is raw image
  // data EXCEPT the short FWDONE / FWABORT control tokens. Bounding the
  // token match by length keeps a full binary chunk from being mistaken for
  // a command (mirrors the TPUT/TDONE convention above).
  if (fwReceiving()) {
    if (len <= 8 && (strcmp(buffer, "FWDONE") == 0 || strcmp(buffer, "FWABORT") == 0)) {
      fwHandleCommand(buffer, len);
    } else {
      fwReceiveChunk(data, len);
    }
    return;
  }

  debug(F("BLE: Received command: ["));
  debug(buffer);
  debugln(F("]"));

  // File commands (LIST/GET/DELETE/TLIST/TGET) all touch SD, so they are
  // DEFERRED to BLUETOOTH_LOOP() via deferFileCommand() — SdFat must never
  // run in this Bluefruit callback task. Filename validation is RAM-only
  // and stays here so bad names are rejected immediately.
  if (strncmp(buffer, "LIST", 4) == 0) {
    if (!deferFileCommand(buffer)) {
      fileListChar.notify((uint8_t*)"BUSY", 4);
    }
  } else if (strncmp(buffer, "GET:", 4) == 0) {
    // Skip "GET:" prefix and trim leading whitespace
    char* filename = buffer + 4;
    while (*filename == ' ') filename++;
    // Reject path traversal / FAT-unsafe names before touching SD.
    if (!filename_validator::isValidFilename(filename, filename_validator::kMaxBleFilenameLen)) {
      debugln(F("BLE: GET rejected — bad filename"));
      fileStatusChar.notify((uint8_t*)"ERROR", 5);
      return;
    }
    if (!deferFileCommand(buffer)) {
      fileStatusChar.notify((uint8_t*)"BUSY", 4);
    }
  } else if (strncmp(buffer, "DELETE:", 7) == 0) {
    // Skip "DELETE:" prefix and trim leading whitespace
    char* filename = buffer + 7;
    while (*filename == ' ') filename++;
    if (!filename_validator::isValidFilename(filename, filename_validator::kMaxBleFilenameLen)) {
      debugln(F("BLE: DELETE rejected — bad filename"));
      fileStatusChar.notify((uint8_t*)"NOT_FOUND", 9);
      return;
    }
    if (!deferFileCommand(buffer)) {
      fileStatusChar.notify((uint8_t*)"BUSY", 4);
    }
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
    if (!deferFileCommand(buffer)) {
      fileStatusChar.notify((uint8_t*)"TERR:BUSY", 9);
    }
  } else if (strncmp(buffer, "TGET:", 5) == 0) {
    // The name is spliced into "/TRACKS/%s"; validate it so it can't
    // climb out of /TRACKS via ../ or carry FAT-unsafe characters.
    if (!filename_validator::isValidFilename(buffer + 5, filename_validator::kMaxBleFilenameLen)) {
      debugln(F("BLE: TGET rejected — bad filename"));
      fileStatusChar.notify((uint8_t*)"TERR:BAD_NAME", 13);
      return;
    }
    if (!deferFileCommand(buffer)) {
      fileStatusChar.notify((uint8_t*)"TERR:BUSY", 9);
    }
  } else if (strncmp(buffer, "TPUT:", 5) == 0) {
    if (trackUploadActive || bleTransferInProgress) {
      fileStatusChar.notify((uint8_t*)"TERR:BUSY", 9);
      return;
    }
    if (!filename_validator::isValidFilename(buffer + 5, filename_validator::kMaxBleFilenameLen)) {
      debugln(F("BLE: TPUT rejected — bad filename"));
      fileStatusChar.notify((uint8_t*)"TERR:BAD_NAME", 13);
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
    if (!filename_validator::isValidFilename(buffer + 5, filename_validator::kMaxBleFilenameLen)) {
      debugln(F("BLE: TDEL rejected — bad filename"));
      fileStatusChar.notify((uint8_t*)"TERR:BAD_NAME", 13);
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

  // Firmware OTA commands (FWBEGIN/FWPUT/FWDONE/FWAPPLY/FWABORT). Parsing
  // and synchronous replies happen here; SD writes + apply are deferred to
  // FW_OTA_LOOP() on the main loop.
  } else if (fwIsCommand(buffer)) {
    fwHandleCommand(buffer, len);
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

  // Buttonless OTA DFU. Registers the Secure DFU service so a companion
  // (DovesDataViewer over Web Bluetooth) can write the "enter bootloader"
  // command and reboot the board into the bootloader's Nordic Secure DFU
  // mode — no physical double-tap of reset required. The bootloader then
  // receives the firmware image and flashes it. Added before the app
  // service so it is registered when advertising starts.
  bledfu.begin();

  // Device Information Service (0x180A). Publishes the firmware version
  // via the standard Firmware Revision characteristic (0x2A26) so the
  // companion can read it and compare against the latest GitHub release
  // to decide whether an OTA update is needed.
  bledis.setManufacturer("DovesDataLogger");
  // Model encodes the board variant ("BirdsEye-sense" / "BirdsEye-nonsense")
  // so the companion can pick the matching OTA package.
  bledis.setModel("BirdsEye-" FIRMWARE_VARIANT);
  bledis.setFirmwareRev(FIRMWARE_VERSION);
  bledis.begin();

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

  // Close any open file and release SD access (main-loop context — BLE_STOP()
  // is called from the loop, so SdFat access here is safe). The disconnect
  // callback skips its deferred teardown when bleActive is already false, so
  // this is the single owner of the local-stop teardown.
  if (bleCurrentFile) {
    bleCurrentFile.close();
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
  }
  bleTransferInProgress = false;
  fwReset();  // abort any in-flight OTA (closes staging file, frees SD)

  // Drop queued-but-unprocessed commands so a stale one can't execute on
  // the next BLE session (BLUETOOTH_LOOP stops running once bleActive is
  // false, so nothing would clear them otherwise).
  fileCmdPending = false;
  settingsCmdPending = false;

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

// Execute a deferred file command (main-loop context — the only place
// SdFat may be touched). The filename was validated in the callback and
// the buffer is stable while fileCmdPending is set.
static void processFileCommand() {
  debug(F("BLE: Processing file cmd: ["));
  debug(fileCmdBuffer);
  debugln(F("]"));

  if (strncmp(fileCmdBuffer, "LIST", 4) == 0) {
    bleSendFileList();
  } else if (strncmp(fileCmdBuffer, "GET:", 4) == 0) {
    char* filename = fileCmdBuffer + 4;
    while (*filename == ' ') filename++;
    bleStartFileTransfer(filename);
  } else if (strncmp(fileCmdBuffer, "DELETE:", 7) == 0) {
    char* filename = fileCmdBuffer + 7;
    while (*filename == ' ') filename++;
    bleDeleteFile(filename);
  } else if (strcmp(fileCmdBuffer, "TLIST") == 0) {
    bleSendTrackList();
  } else if (strncmp(fileCmdBuffer, "TGET:", 5) == 0) {
    char filepath[FILEPATH_MAX];
    snprintf(filepath, sizeof(filepath), "/TRACKS/%s", fileCmdBuffer + 5);
    bleStartFileTransfer(filepath);
  }
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

  // Deferred disconnect teardown — runs on the main loop so SdFat is touched
  // by a single task. Closes any in-flight transfer/staging file, releases
  // SD, aborts the OTA, then auto-reboots to apply changed settings.
  if (bleDisconnectCleanupPending) {
    bleDisconnectCleanupPending = false;

    // If a firmware OTA apply has been requested, the web app disconnecting is
    // EXPECTED — it hands the device off to self-flash. We must NOT abort the
    // OTA (fwReset) or reboot here: doing so discards the staged image and
    // boots the OLD firmware. Leave the apply for FW_OTA_LOOP() below, which
    // owns the install and its own reset.
    if (fwApplyRequested()) {
      debugln(F("BLE: disconnect during OTA apply — deferring to FW_OTA_LOOP"));
    } else {
      if (bleCurrentFile) {
        bleCurrentFile.close();
        releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
      }
      bleTransferInProgress = false;
      fwReset();  // abort any in-flight OTA (closes staging file, frees SD)

      if (enableLogging) {
        debugln(F("BLE: Skipping reboot (logging active)"));
      } else {
        debugln(F("BLE: Rebooting to apply settings..."));
        delay(100);  // Brief delay for debug output to flush
        NVIC_SystemReset();
      }
    }
  }

  // Process deferred settings commands (thread-safe: runs in main loop)
  if (settingsCmdPending) {
    processSettingsCommand();
    settingsCmdPending = false;
  }

  // Process deferred file commands (LIST/GET/DELETE/TLIST/TGET) — the only
  // place these touch SdFat. A GET lands here before the burst-send block
  // below, so a transfer still starts in the same loop iteration.
  if (fileCmdPending) {
    processFileCommand();
    fileCmdPending = false;
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

  // Service deferred firmware-OTA work (staging-file writes, CRC verify,
  // apply sequence).
  FW_OTA_LOOP();

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
