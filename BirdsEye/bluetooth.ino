///////////////////////////////////////////
// BLUETOOTH (BLE) MODULE
// All BLE-related functions: callbacks, setup, file transfer, loop
///////////////////////////////////////////

#include "bluetooth.h"
#include "camera_ble.h"
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

// True once a transfer peer actually DROVE the file/settings/OTA service
// (any fileRequestChar write while the transfer owns the radio). Gates the
// auto-reboot-on-disconnect: a bonded camera that connects to the transfer
// advert (the radio has one BD_ADDR, so the X4 can chase it) and vets our
// GATT then drops must NOT reboot the logger out of the user's transfer
// session — repeatedly, if the camera keeps retrying (#1). A peer that never
// touched the service also held no SD, so skipping the teardown is safe.
static volatile bool bleTransferEngaged = false;

void bleConnectCallback(uint16_t conn_handle) {
  // Camera-owned link (the X4 connecting to our remote GATT) — route to the
  // camera module and skip everything below: bleConnected and the MTU/PHY/
  // DLE negotiation are transfer-only.
  if (bleOwner == BLE_OWNER_CAMERA) {
    cameraBleOnConnect(conn_handle);
    return;
  }

  debugln(F("BLE: Device connected!"));
  bleConnected = true;
  bleTransferEngaged = false;  // this peer hasn't used the service yet

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
  // Camera link — route to the camera module and return. This makes the
  // transfer teardown below (and with it the deferred auto-reboot in
  // BLUETOOTH_LOOP()) structurally unreachable for camera links: a camera
  // dropping off must never reboot the logger mid-session. Matched BY
  // HANDLE first, not owner: a teardown-initiated camera disconnect
  // completes asynchronously, so its event can land after ownership has
  // already moved to NONE/TRANSFER (e.g. CAMERA_FORCE_RELEASE()
  // immediately followed by BLE_SETUP() on the transfer page) — owner
  // routing alone would misdeliver it here and reboot the device.
  if (cameraBleOwnsConnHandle(conn_handle) || bleOwner == BLE_OWNER_CAMERA) {
    cameraBleOnDisconnect(conn_handle, reason);
    return;
  }

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
  //
  // Only reboot for a peer that actually USED the transfer service. A bonded
  // camera can land on the transfer advert (shared BD_ADDR) and be routed
  // here as "the phone" when it drops; without this gate its disconnect would
  // reboot the logger mid-transfer, over and over (#1). A never-engaged peer
  // also held no SD, so there is nothing to tear down.
  if (bleActive && bleTransferEngaged) {
    bleDisconnectCleanupPending = true;
  }
  bleTransferEngaged = false;  // reset for the next peer
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

void bleAdvFinalizePadded() {
  // WORKAROUND for a Bluefruit 0.21.0 core bug (fixed upstream in
  // Adafruit 1.7.0, but the Seeed fork ships the broken version):
  // BLEAdvertising::_start() initializes its ble_gap_adv_data_t as a
  // function-local STATIC, so both packet .len fields freeze at whatever
  // the FIRST advert of the boot carried. Any later advert of a
  // different length goes on air truncated or with a stale tail — a
  // malformed PDU every receiver silently discards, while all our API
  // calls report success. (This is what broke the camera wake advert:
  // a 28-byte connect advert first froze the length, then the 31-byte
  // wake PDU lost its last 3 bytes on air.)
  //
  // Defeat it by construction: EVERY advert in this firmware is padded
  // to exactly 31+31 bytes before start(), so the frozen length is
  // always correct. Zero padding after the last AD structure is
  // explicitly legal (BT Core Spec Vol 3 Part C §11: the non-significant
  // part is all-zero octets). Call this after building the payload
  // (additive or raw setData) and immediately before Advertising.start().
  uint8_t buf[BLE_GAP_ADV_SET_DATA_SIZE_MAX] = {0};
  uint8_t n = Bluefruit.Advertising.count();
  memcpy(buf, Bluefruit.Advertising.getData(), n);
  Bluefruit.Advertising.setData(buf, sizeof(buf));

  memset(buf, 0, sizeof(buf));
  n = Bluefruit.ScanResponse.count();
  memcpy(buf, Bluefruit.ScanResponse.getData(), n);
  Bluefruit.ScanResponse.setData(buf, sizeof(buf));
}

void bleApplyTransferAdvertising() {
  // Full rebuild, not an incremental start: the camera module may have
  // owned the advert set (name + payload) since the last transfer session,
  // so drop whatever is there and reconstruct the transfer advert exactly.
  Bluefruit.Advertising.stop();  // safe no-op if not advertising
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  char bleName[32];
  if (getSetting("bluetooth_name", bleName, sizeof(bleName))) {
    debug(F("BLE: Name from settings: "));
    debugln(bleName);
    Bluefruit.setName(bleName);
  } else {
    debugln(F("BLE: WARNING - bluetooth_name not found, using fallback"));
    Bluefruit.setName("DovesDataLogger");
  }

  // Force connectable: the shared Advertising object may have been left
  // non-connectable by a camera wake burst (see kStartWakeBurst in
  // camera_ble.ino), which would otherwise make this transfer advert
  // unconnectable.
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(fileService);
  Bluefruit.Advertising.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  bleAdvFinalizePadded();  // every advert must be 31+31 — see the helper
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
  // Only serve the file service while the transfer page owns the radio. A
  // peer that connects during camera mode must not queue deferred SD work —
  // BLUETOOTH_LOOP() is gated on bleActive and would never drain it.
  if (bleOwner != BLE_OWNER_TRANSFER) return;

  // A write to the request characteristic means this is a genuine transfer
  // peer (the phone app), not a bonded camera vetting our GATT — arm the
  // reboot-on-disconnect gate (#1).
  bleTransferEngaged = true;

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

  // A command longer than the parse buffer was truncated by the memcpy
  // above. The raw-data paths (track upload / OTA image) returned already,
  // so anything this long is a malformed command — never a valid filename
  // command (those are FAT-short). Reject rather than validate and act on a
  // silently-mangled name. (len <= 64 fits buffer[65] with NUL intact.)
  if (len >= sizeof(buffer)) {
    debugln(F("BLE: command too long, rejecting"));
    fileStatusChar.notify((uint8_t*)"ERROR", 5);
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

// Just-Works pairing result trace (Bluefruit pair-complete callback).
// Signature is plain integers, so no Bluefruit-type forward declaration is
// needed. auth_status == BLE_GAP_SEC_STATUS_SUCCESS (0) means bonded.
void blePairCompleteCallback(uint16_t conn_hdl, uint8_t auth_status) {
  (void)conn_hdl;
  debug(F("BLE: pairing complete, auth_status=0x"));
  debugln(auth_status, HEX);
}

void bleCoreEnsureInit() {
  if (bleInitialized) return;

  debugln(F("BLE: Initializing Bluetooth core..."));

  // Custom BLE config for max file transfer throughput:
  // MTU 247, event_len 100 (125ms max radio time per event),
  // HVN TX queue 10 (up from BANDWIDTH_MAX's 3 — deeper notification pipeline),
  // WrCmd queue 1 (default, we don't use write commands).
  Bluefruit.configPrphConn(247, 100, 10, 1);
  // 1 peripheral + 0 central: the camera feature is now a pure PERIPHERAL
  // remote emulation (the camera connects to US and we notify our ce82
  // buttons), so the old central slot for the X4's be80 control link is
  // gone. Both the transfer service and the camera remote are peripherals
  // sharing the single peripheral slot via bleOwner.
  Bluefruit.begin(1, 0);
  Bluefruit.setTxPower(4);

  Bluefruit.Periph.setConnectCallback(bleConnectCallback);
  Bluefruit.Periph.setDisconnectCallback(bleDisconnectCallback);

  // Just-Works pairing acceptance (peripheral). The genuine Insta360 GPS
  // Remote link is encrypted + bonded, and a captured X4 brings up
  // encryption immediately on connect, so the camera (as central) may
  // withhold its ce82 CCCD subscription until the link is secured. Advertise
  // NoInputNoOutput I/O capabilities and no MITM requirement so the
  // SoftDevice completes Just-Works pairing without any on-device prompt.
  // This is link-level only — NO characteristic is marked encrypted
  // (SECMODE_OPEN everywhere), so the file-transfer service keeps working
  // fully open/unbonded. NOTE (Bluefruit 0.21.0 assumption): NoInputNoOutput
  // + MITM-off is already Bluefruit's default and yields Just-Works; setting
  // it explicitly documents intent and guards against a future default
  // change. The pair-complete callback is trace-only.
  Bluefruit.Security.setIOCaps(false, false, false);  // display, yes/no, keyboard
  Bluefruit.Security.setMITM(false);
  Bluefruit.Security.setPairCompleteCallback(blePairCompleteCallback);

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

  // Camera remote GATT (peripheral ce80 + D0FF services) plus the central
  // client objects for the camera's be80 service. GATT services can only
  // be added before advertising starts, so they are registered here even
  // when the user never touches the camera feature.
  cameraBleRegisterServices();

  bleInitialized = true;

  // Deliberately NO advertising, NO device name, NO conn-LED here — the
  // owner (transfer page or camera module) applies its own advert set.
}

void BLE_SETUP() {
  // Parked transfer — bump the SD clock for faster file transfers. Reverted
  // in BLE_STOP() (and by the auto-reboot on phone disconnect).
  sdSetTransferSpeed(true);

  debugln(F("BLE: Starting transfer mode..."));

  bleCoreEnsureInit();

  // Take the radio for the transfer service (main-loop context — the camera
  // module released its links via CAMERA_FORCE_RELEASE() before this page
  // opened). The full advert rebuild below is what makes re-entry correct
  // even when the camera owned the advert set in between.
  bleOwner = BLE_OWNER_TRANSFER;

  // Enable connection LED
  Bluefruit.autoConnLed(true);
  Bluefruit.setConnLedInterval(250); // Blink every 250ms when connected

  bleApplyTransferAdvertising();

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
  bleTransferEngaged = false;  // session is over — no reboot owed
  fwReset();  // abort any in-flight OTA (closes staging file, frees SD)

  // Drop queued-but-unprocessed commands so a stale one can't execute on
  // the next BLE session (BLUETOOTH_LOOP stops running once bleActive is
  // false, so nothing would clear them otherwise).
  fileCmdPending = false;
  settingsCmdPending = false;

  // Disarm auto-restart BEFORE disconnecting. bleApplyTransferAdvertising()
  // set restartOnDisconnect(true) so a mid-session phone drop re-advertises;
  // but here we are deliberately tearing the transfer service down. The
  // disconnect below is async, so if we left it armed Bluefruit's internal
  // handler would restart an ownerless transfer advert AFTER we stop it —
  // the phone would reconnect into a mute session (owner already NONE) and
  // the occupied peripheral slot would block camera auto-record until a
  // power cycle.
  Bluefruit.Advertising.restartOnDisconnect(false);

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

  // Release radio ownership. The camera module re-acquires it on its next
  // advertising action (in CAMERA_LOOP()) — nothing to hand off here.
  bleOwner = BLE_OWNER_NONE;

  // Restore the EMI-safe SD clock now that the transfer session is over.
  sdSetTransferSpeed(false);

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

  // Belt-and-suspenders: buildTrackList() provisions the folder at boot,
  // but re-ensure it before every upload so a missing folder can never
  // fail a TPUT with WRITE_FAIL. Return deliberately ignored.
  sdEnsureTracksFolder();

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
