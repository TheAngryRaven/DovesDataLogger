///////////////////////////////////////////
// BLUETOOTH (BLE) MODULE
// All BLE-related functions: callbacks, setup, file transfer, loop
///////////////////////////////////////////

#include "bluetooth.h"
#include "ble_stream.h"
#include "camera_ble.h"
#include "filename_validator.h"
#include "firmware_ota.h"

// Target connection interval in 1.25 ms units: 12 = 15 ms, the fastest an
// Apple central is permitted to accept from an accessory. See bleTuneLink().
#define BLE_TARGET_INTERVAL_UNITS 12
// Any negotiated link-layer PDU at or above this counts as "DLE took". The
// un-extended default is 27; a successful extension lands at 251 (or whatever
// the peer allows), so the exact value matters less than clearing the floor.
#define BLE_DATA_LENGTH_MIN_EXTENDED 100

// Two-stage post-connect link tuning: 0 = idle, 1 = read + correct at
// +500 ms, 2 = final readback at +1500 ms. Written by the connect callback,
// consumed by the main loop; the stages themselves only ever run on the loop.
static volatile uint8_t bleLinkTuneStage = 0;
static volatile uint32_t bleLinkTuneStartMs = 0;
static volatile uint16_t bleLinkTuneConnHandle = 0;
// Negotiated link-layer PDU (bytes). 27 means DLE never happened, which is
// the single biggest download-throughput tax there is — surfaced on the
// transfer page so it is visible instead of inferred.
static volatile uint16_t bleLinkDataLen = 27;

// File-transfer read-ahead. Notifying straight out of SdFat put a disk read
// in the radio's critical path and cost one single-block SD command per
// 512 bytes (a chunk is not a sector). One aligned 4 KB read is a single
// multi-block transfer, and the notifications then stream out of RAM. The
// index arithmetic lives in the host-tested ble_stream unit; the bytes and
// the file live here. See docs/plans/0008-ble-download-throughput.md.
static uint8_t bleStreamBuf[ble_stream::kReadAheadSize];
static ble_stream::ReadAhead bleStream(ble_stream::kReadAheadSize);

// Live transfer rate, recomputed as chunks go out so the device can show
// KB/s instead of only a percentage — the whole reason this regression took
// a session to notice.
static uint32_t bleTransferStartMs = 0;
static uint32_t bleTransferRate = 0;

// Deferred settings command buffer (BLE callback -> main loop)
static volatile bool settingsCmdPending = false;
static char settingsCmdBuffer[65];  // 64 chars + null

// Track upload state (BLE callback -> main loop)
static volatile bool trackUploadActive = false;
static volatile bool trackUploadReady = false;      // signals main loop to send TREADY
static volatile bool trackUploadComplete = false;    // signals main loop to write file
static volatile bool trackUploadError = false;
static char trackUploadFilename[25];                 // just the filename (e.g. "OKC.json")
// Sized from JSON_BUFFER_SIZE so the largest track the device can PARSE is
// also the largest it can RECEIVE. When these were separate 4 KB constants,
// raising one alone would have produced a device that reads an 8 KB track off
// its own card but answers TERR:TOO_LARGE when the app tries to send one back.
static char trackUploadBuffer[JSON_BUFFER_SIZE];
static volatile uint16_t trackUploadOffset = 0;

// Track delete state (BLE callback -> main loop)
static volatile bool trackDeletePending = false;
static char trackDeleteFilename[25];
// Which folder a pending track upload/delete targets (TRACK_KIND_CIRCUIT /
// TRACK_KIND_SPRINT). Sprint tracks live in /TRACKS/SPRINT — see plan 0002.
static volatile uint8_t trackUploadKind = TRACK_KIND_CIRCUIT;
static volatile uint8_t trackDeleteKind = TRACK_KIND_CIRCUIT;

// Folder for a track kind. The BLE filename validator deliberately rejects
// '/' so a client can never splice a path of its own — the folder is chosen
// here, by opcode, and never comes off the wire.
static const char* trackFolderFor(uint8_t kind) {
  return (kind == TRACK_KIND_SPRINT) ? trackFolderSprint : trackFolder;
}

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
  // Request Data Length Extension (251-byte link-layer PDU). Without it every
  // 244-byte notification fragments into ten 27-byte packets, each paying its
  // own header/CRC/inter-frame space — a 3-4x tax on exactly this transfer.
  //
  // This ask is best-effort and NOT trusted: the SoftDevice runs one
  // link-layer control procedure at a time, so firing it immediately behind
  // the PHY update above can come straight back as NRF_ERROR_BUSY, which
  // Bluefruit swallows into a false return. bleTuneLink() below verifies the
  // result once the link has settled and re-asks if it did not take.
  connection->requestDataLengthUpdate();

  // Link tuning (MTU / DLE / interval readback and retry) is deferred to the
  // main loop rather than blocking this callback with a delay(500).
  bleLinkTuneStartMs = millis();
  bleLinkTuneConnHandle = conn_handle;
  bleLinkTuneStage = 1;
  bleLinkDataLen = 27;  // pre-DLE default; corrected at stage 1

  debug(F("BLE: Connection interval: "));
  debug(connection->getConnectionInterval() * 1.25);
  debugln(F("ms"));
}

// Re-check what the link actually negotiated and fix what did not take.
// Called from BLUETOOTH_LOOP() on a two-stage timer after connect: stage 1
// (+500 ms) reads the settled parameters and issues corrections, stage 2
// (+1500 ms) records the final numbers for the display. Both stages run on
// the main loop, so the SoftDevice calls here never race a callback.
//
// Takes the connection HANDLE, not a BLEConnection* — a Bluefruit type in a
// parameter list would need <bluefruit.h> visible where Arduino inserts its
// generated prototypes, and silently degrades to `int` when it is not (see
// the auto-prototype note in CLAUDE.md's conventions). Returns false when the
// peer is gone.
static bool bleTuneLink(uint16_t conn_handle) {
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  // Bluefruit keeps the connection object around after a drop, so check the
  // link is actually up before tuning it — otherwise a peer that leaves
  // between stages gets requests sent into a dead handle.
  if (!connection || !connection->connected()) return false;

  bleNegotiatedMtu = connection->getMtu();
  bleLinkDataLen = connection->getDataLength();
  uint16_t intervalUnits = connection->getConnectionInterval();

  debug(F("BLE: MTU "));
  debug(bleNegotiatedMtu);
  debug(F(", data length "));
  debug(bleLinkDataLen);
  debug(F(", PHY "));
  debug(connection->getPHY());
  debug(F(", interval "));
  debug(intervalUnits * 1.25);
  debugln(F("ms"));

  if (bleLinkTuneStage != 1) return true;  // stage 2 is readback only

  // DLE did not take (still the 27-byte default). The likeliest cause is the
  // connect-time ask colliding with the PHY update, so ask again now that
  // nothing else is in flight.
  if (bleLinkDataLen < BLE_DATA_LENGTH_MIN_EXTENDED) {
    debugln(F("BLE: data length still default — re-requesting DLE"));
    connection->requestDataLengthUpdate();
  }

  // Connection interval. The advertised preference (setConnInterval(6, 12) =
  // 7.5-15 ms) is deliberately aggressive and desktop/Android centrals honour
  // it — those are already fast and must not be slowed down, so the
  // preference itself stays put. iOS is the problem: Apple's accessory
  // guidelines require a requested Interval Min of at least 15 ms, so a
  // 7.5 ms preference is rejected outright and the link stays on whatever
  // iOS chose at connect (commonly 30 ms) — half the connection events, half
  // the throughput. Only when we find ourselves slower than the target do we
  // make a second, Apple-compliant ask that the central is allowed to accept.
  if (intervalUnits > BLE_TARGET_INTERVAL_UNITS) {
    debug(F("BLE: interval slower than target — requesting "));
    debug(BLE_TARGET_INTERVAL_UNITS * 1.25);
    debugln(F("ms"));
    connection->requestConnectionParameter(BLE_TARGET_INTERVAL_UNITS);
  }

  return true;
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
  bleLinkTuneStage = 0;  // abandon any pending link tuning for this peer
  bleLinkDataLen = 27;   // pre-DLE default

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
    // The whole walk runs inside one loop() iteration: 10 ms per file
    // (plus up to ~100 ms per congested notify) exceeds the ~4 s WDT on a
    // card holding a season of logs — feed it per entry, like the SD
    // formatter does.
    wdtPet();
  }
  root.close();
  releaseSDAccess(SD_ACCESS_BLE_TRANSFER);

  fileListChar.notify((uint8_t*)"END", 3);
  debug(F("BLE: File list sent, "));
  debug(fileCount);
  debugln(F(" files"));
}

void bleSendTrackList(uint8_t kind) {
  // Sprint listings answer with their own tokens (TSFILE:/TSEND) so a client
  // can never confuse a sprint enumeration with a circuit one.
  const bool sprint = (kind == TRACK_KIND_SPRINT);
  const char* fileTok = sprint ? "TSFILE:%s" : "TFILE:%s";
  const char* endTok  = sprint ? "TSEND" : "TEND";
  // Same locking discipline as bleSendFileList() — see the comment there.
  if (currentSDAccess != SD_ACCESS_NONE ||
      !acquireSDAccess(SD_ACCESS_BLE_TRANSFER)) {
    debugln(F("BLE: SD busy, cannot list tracks"));
    fileStatusChar.notify((uint8_t*)"TERR:SD_BUSY", 12);
    return;
  }

  File32 trackDir2 = SD.open(trackFolderFor(kind));
  if (!trackDir2) {
    debugln(F("BLE: Failed to open TRACKS directory"));
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
    fileStatusChar.notify((uint8_t*)endTok, strlen(endTok));
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
      int len = snprintf(msg, sizeof(msg), fileTok, name);
      if (len > 0 && len < (int)sizeof(msg)) {
        fileStatusChar.notify((uint8_t*)msg, len);
        delay(10);
        fileCount++;
      }
    }
    entry.close();
    wdtPet();  // same in-one-iteration walk as bleSendFileList
  }
  trackDir2.close();
  releaseSDAccess(SD_ACCESS_BLE_TRANSFER);

  fileStatusChar.notify((uint8_t*)endTok, strlen(endTok));
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
  // Drop anything the previous transfer left buffered — a stale tail would
  // be prepended to this file.
  bleStream.reset();
  bleTransferStartMs = millis();
  bleTransferRate = 0;
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
  } else if (strcmp(buffer, "TLIST") == 0 || strcmp(buffer, "TSLIST") == 0) {
    if (!deferFileCommand(buffer)) {
      fileStatusChar.notify((uint8_t*)"TERR:BUSY", 9);
    }
  } else if (strncmp(buffer, "TGET:", 5) == 0 || strncmp(buffer, "TSGET:", 6) == 0) {
    // The name is spliced into "<folder>/%s"; validate it so it can't
    // climb out of the tracks folders via ../ or carry FAT-unsafe bytes.
    // The folder itself comes from the opcode, never from the wire.
    if (!filename_validator::isValidFilename(buffer + (buffer[1] == 'S' ? 6 : 5),
                                             filename_validator::kMaxBleFilenameLen)) {
      debugln(F("BLE: TGET rejected — bad filename"));
      fileStatusChar.notify((uint8_t*)"TERR:BAD_NAME", 13);
      return;
    }
    if (!deferFileCommand(buffer)) {
      fileStatusChar.notify((uint8_t*)"TERR:BUSY", 9);
    }
  } else if (strncmp(buffer, "TPUT:", 5) == 0 || strncmp(buffer, "TSPUT:", 6) == 0) {
    if (trackUploadActive || bleTransferInProgress) {
      fileStatusChar.notify((uint8_t*)"TERR:BUSY", 9);
      return;
    }
    const bool putSprint = (buffer[1] == 'S');
    const char* putName = buffer + (putSprint ? 6 : 5);
    if (!filename_validator::isValidFilename(putName, filename_validator::kMaxBleFilenameLen)) {
      debugln(F("BLE: TPUT rejected — bad filename"));
      fileStatusChar.notify((uint8_t*)"TERR:BAD_NAME", 13);
      return;
    }
    trackUploadKind = putSprint ? TRACK_KIND_SPRINT : TRACK_KIND_CIRCUIT;
    strncpy(trackUploadFilename, putName, sizeof(trackUploadFilename) - 1);
    trackUploadFilename[sizeof(trackUploadFilename) - 1] = '\0';
    trackUploadOffset = 0;
    trackUploadError = false;
    trackUploadComplete = false;
    trackUploadReady = true;
    trackUploadActive = true;
    debugln(F("BLE: Track upload started"));
  } else if (strncmp(buffer, "TDEL:", 5) == 0 || strncmp(buffer, "TSDEL:", 6) == 0) {
    if (trackDeletePending) {
      fileStatusChar.notify((uint8_t*)"TERR:BUSY", 9);
      return;
    }
    const bool delSprint = (buffer[1] == 'S');
    const char* delName = buffer + (delSprint ? 6 : 5);
    if (!filename_validator::isValidFilename(delName, filename_validator::kMaxBleFilenameLen)) {
      debugln(F("BLE: TDEL rejected — bad filename"));
      fileStatusChar.notify((uint8_t*)"TERR:BAD_NAME", 13);
      return;
    }
    trackDeleteKind = delSprint ? TRACK_KIND_SPRINT : TRACK_KIND_CIRCUIT;
    strncpy(trackDeleteFilename, delName, sizeof(trackDeleteFilename) - 1);
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
  bleConnLedOff();

  bleConnected = false;
  // bleActive already set false at top of BLE_STOP()

  // Release radio ownership. The camera module re-acquires it on its next
  // advertising action (in CAMERA_LOOP()) — nothing to hand off here.
  bleOwner = BLE_OWNER_NONE;

  // Restore the EMI-safe SD clock now that the transfer session is over.
  sdSetTransferSpeed(false);

  debugln(F("BLE: Bluetooth stopped"));
}

// Manual exit from the Bluetooth transfer page (the on-device Exit button).
// Leaving transfer mode ALWAYS reboots, matching the phone-disconnect
// auto-reboot and the USB mass-storage exit: a reboot is what guarantees
// settings changed over BLE take effect and that no radio/advert/SD state
// leaks from the transfer session into the next driving session. Before
// this, only a peer disconnect rebooted — a manual exit dropped back to the
// menu on the old settings. BLE_STOP() first so the teardown (file close,
// SD release, OTA abort, advert stop) runs cleanly on the main loop before
// the reset. Does not return on hardware; the SIM stub stops the radio and
// returns so the sim's menu walk can continue.
void bleExitTransferMode() {
  BLE_STOP();
  debugln(F("BLE: Transfer mode exited — rebooting..."));
  delay(100);  // let debug output flush (mirrors the disconnect auto-reboot)
  NVIC_SystemReset();
}

// Force the Bluefruit connection LED off and keep it off. Bluefruit drives
// LED_CONN (the XIAO's blue LED, active-low) whenever _led_conn is enabled —
// which is the library DEFAULT, so camera-owned advertising/links blink it
// even though this module never called autoConnLed(true) for them. Disabling
// autoConnLed stops the library re-lighting it; the digitalWrite parks the
// pin high (off) — nRF52 GPIO state is retained in System OFF, so a lit pin
// would stay lit on a "powered off" device.
//
// The Bluefruit calls MUST stay behind bleInitialized: this core's
// setConnLedInterval() passes _led_blink_th to FreeRTOS with no null guard,
// and that timer is only created in Bluefruit.begin() — calling it on a
// never-initialized radio (stock device, BLE fully lazy) hands the timer
// daemon a NULL handle. The pin park below is the only pre-begin-safe part,
// and the only part such a device needs.
void bleConnLedOff() {
  if (bleInitialized) {
    Bluefruit.autoConnLed(false);
    Bluefruit.setConnLedInterval(0);
  }
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, HIGH);
}

// Shutdown-path radio quiesce, UNCONDITIONAL on owner. BLE_STOP() only runs
// for the transfer service (bleActive), so a camera-owned radio — or a peer
// whose async disconnect hasn't been serviced by the Bluefruit task yet —
// used to sail through enterShutdown() with the conn LED still driven (the
// "blue light stays on after sleep" field report). Called from
// enterShutdown() after CAMERA_SLEEP()/BLE_STOP(): stop any advertising,
// drop any surviving link, give the async disconnect a bounded window to be
// serviced (WDT-fed), then force the LED off LAST so nothing re-lights it.
void bleShutdownQuiesce() {
  if (bleInitialized) {
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.stop();
    if (Bluefruit.connected()) {
      Bluefruit.disconnect(Bluefruit.connHandle());
    }
    // Bounded settle: the disconnect (and the library's own LED-off) run on
    // the Bluefruit task; System OFF follows within milliseconds otherwise.
    for (int i = 0; i < 5; i++) {
      if (!Bluefruit.connected()) break;
      wdtPet();
      delay(50);
    }
  }
  bleConnLedOff();  // pre-begin() it only parks the pin (see its guard)
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
    bleSendTrackList(TRACK_KIND_CIRCUIT);
  } else if (strcmp(fileCmdBuffer, "TSLIST") == 0) {
    bleSendTrackList(TRACK_KIND_SPRINT);
  } else if (strncmp(fileCmdBuffer, "TGET:", 5) == 0 ||
             strncmp(fileCmdBuffer, "TSGET:", 6) == 0) {
    const bool sprint = (fileCmdBuffer[1] == 'S');
    char filepath[FILEPATH_MAX];
    snprintf(filepath, sizeof(filepath), "%s/%s",
             trackFolderFor(sprint ? TRACK_KIND_SPRINT : TRACK_KIND_CIRCUIT),
             fileCmdBuffer + (sprint ? 6 : 5));
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
      // A hand-edited SETTINGS.json can hold a non-string value, and
      // as<const char*>() returns NULL for those — %s on NULL streams
      // garbage from address 0 on this core.
      const char* val = kv.value().as<const char*>();
      snprintf(entry, sizeof(entry), "SVAL:%s=%s", kv.key().c_str(),
               val ? val : "");
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
  snprintf(filepath, sizeof(filepath), "%s/%s",
           trackFolderFor(trackUploadKind), trackUploadFilename);

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
  snprintf(filepath, sizeof(filepath), "%s/%s",
           trackFolderFor(trackDeleteKind), trackDeleteFilename);

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

  // Deferred link tuning — see bleTuneLink(). Stage 1 reads what actually
  // negotiated and corrects it, stage 2 records the settled result.
  if (bleLinkTuneStage > 0) {
    uint32_t due = (bleLinkTuneStage == 1) ? 500 : 1500;
    if (millis() - bleLinkTuneStartMs >= due) {
      uint8_t stage = bleLinkTuneStage;
      // Abandon on a peer that went away mid-tune, so a dropped connection
      // can't leave the stage timer re-firing every loop iteration.
      bleLinkTuneStage = bleTuneLink(bleLinkTuneConnHandle)
                             ? ((stage == 1) ? 2 : 0)
                             : 0;
    }
  }

  if (bleTransferInProgress && bleCurrentFile && Bluefruit.connected()) {
    const uint16_t chunk = ble_stream::chunkSize(bleNegotiatedMtu);
    const uint32_t burstStart = millis();

    // Burst send: keep notifying until the wall-clock budget is spent.
    // notify() blocks via semaphore when the SoftDevice TX queue is full,
    // which is the flow control — a blocked notify means the radio is
    // saturated, which is exactly where we want to be. The budget is in
    // milliseconds rather than packets because a fixed packet count is a
    // very different amount of time on a fast link than on a slow one, and
    // the only thing it protects is loop responsiveness (exit button, WDT).
    while (bleTransferInProgress &&
           (millis() - burstStart) < ble_stream::kBurstBudgetMs) {
      bool readFailed = false;

      if (bleStream.needsRefill(chunk)) {
        // Compacting refill: the unsent tail slides to the front so every
        // notify but the file's last one carries a full chunk (without it,
        // 4096 mod 244 = 192 bytes of runt packet at every buffer boundary).
        ble_stream::Move m = bleStream.compact();
        if (m.length) {
          memmove(bleStreamBuf, bleStreamBuf + m.srcOffset, m.length);
        }
        int bytesRead = bleCurrentFile.read(bleStreamBuf + bleStream.fillOffset(),
                                            bleStream.fillSpace());
        if (bytesRead > 0) {
          bleStream.commitFill((uint32_t)bytesRead);
        } else if (bytesRead < 0) {
          // SdFat read error, as distinct from end of file. Both used to fall
          // into the same "no bytes" branch and report DONE, handing the app a
          // truncated session file with a clean status on it.
          readFailed = true;
        }
      }

      if (readFailed) {
        bleCurrentFile.close();
        bleTransferInProgress = false;
        releaseSDAccess(SD_ACCESS_BLE_TRANSFER);

        debugln(F("BLE: SD read failed mid-transfer"));
        fileStatusChar.notify((uint8_t*)"ERROR", 5);
        break;
      }

      uint32_t offset = 0;
      uint16_t slice = bleStream.nextSlice(chunk, &offset);

      if (slice == 0) {
        // Buffer drained and the file had nothing left to give — done.
        bleCurrentFile.close();
        bleTransferInProgress = false;
        releaseSDAccess(SD_ACCESS_BLE_TRANSFER);

        // Recompute before printing: the update at the bottom of the block
        // has not run for this final burst yet.
        bleTransferRate = ble_stream::rateBytesPerSec(
            bleBytesTransferred, millis() - bleTransferStartMs);
        debug(F("BLE: Transfer complete! "));
        debug(bleTransferRate / 1024);
        debugln(F(" KB/s"));
        fileStatusChar.notify((uint8_t*)"DONE", 4);
        break;
      }

      if (!fileDataChar.notify(bleStreamBuf + offset, slice)) {
        // Failed notify (HVN pool starved >100 ms, or disconnect). Nothing to
        // undo: the bytes are still in RAM and the buffer head has not moved,
        // so the identical slice goes out on the next pass. The old
        // read-straight-from-SdFat path had to seekCur() backwards here, and
        // getting that wrong punched a silent hole in a file that still
        // reported DONE.
        break;
      }

      bleStream.consume(slice);
      bleBytesTransferred += slice;
    }

    bleTransferRate = ble_stream::rateBytesPerSec(bleBytesTransferred,
                                                  millis() - bleTransferStartMs);
  }
}

uint32_t bleTransferRateBps() { return bleTransferRate; }
uint16_t bleLinkDataLength() { return bleLinkDataLen; }
uint16_t bleLinkChunkSize() { return ble_stream::chunkSize(bleNegotiatedMtu); }
