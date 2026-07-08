///////////////////////////////////////////
// CAMERA BLE MODULE (Insta360 X4 auto-record)
// Bluefruit glue for the camera remote: hosts the GPS-remote GATT
// (peripheral), drives the central control link to the camera's be80
// service, and executes the actions returned by the host-tested
// camera_fsm pure unit. Frame bytes come from insta360_protocol.
//
// THREADING (mirrors firmware_ota.ino): Bluefruit callbacks only copy
// bytes into RAM and set volatile flags. CAMERA_LOOP() on the main
// loop drains the flags/buffers, steps the FSM, and does all real work
// — including the one setSetting() that persists a captured serial.
//
// Cross-module globals (Bluefruit, bleOwner, bleInitialized, gpsData,
// tachLastReported, gps_speed_mph) are visible via Arduino's .ino
// concatenation — BirdsEye.ino is compiled first, same as bluetooth.ino
// relies on for fileStatusChar / bleActive.
///////////////////////////////////////////

#include "camera_ble.h"

#include <string.h>

#include "project.h"
#include "bluetooth.h"        // bleCoreEnsureInit(), bleOwner ownership model
#include "gps_functions.h"    // getGpsUnixTimestamp()
#include "insta360_protocol.h"
#include "settings.h"

// Forward declarations for every function whose signature mentions a
// Bluefruit type. Arduino's auto-prototype generator inserts prototypes
// BEFORE <bluefruit.h> is included, so without these explicit ones the
// generated prototypes fail with "'BLECharacteristic' has not been
// declared" (same workaround as bleFileRequestCallback in bluetooth.ino
// — arduino-cli skips functions that already have a prototype).
static void cameraCe81WriteCallback(uint16_t conn_hdl, BLECharacteristic* chr,
                                    uint8_t* data, uint16_t len);
static void cameraD0ffIgnoreWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr,
                                          uint8_t* data, uint16_t len);
static void cameraBe82NotifyCallback(BLEClientCharacteristic* chr,
                                     uint8_t* data, uint16_t len);
static void cameraScanCallback(ble_gap_evt_adv_report_t* report);
static void cameraBeginReadChar(BLECharacteristic& chr, const uint8_t* value,
                                uint8_t len);
static void cameraBeginWriteChar(BLECharacteristic& chr);
static void cameraCe82CccdCallback(uint16_t conn_hdl, BLECharacteristic* chr,
                                   uint16_t value);

///////////////////////////////////////////
// MODULE STATE
///////////////////////////////////////////

// The FSM — all temporal behavior lives in the host-tested pure unit.
static camera_fsm::Fsm cameraFsm;

// Stored camera serial: NUL-terminated string + parsed uppercase bytes.
static char storedSerial[insta360_protocol::kSerialLen + 1] = "";
static uint8_t serialBytes[insta360_protocol::kSerialLen] = {0};

// be81 command message counter — incremented per command frame sent
// (start/stop video, GPS frame); wraps naturally.
static uint16_t msgCounter = insta360_protocol::kInitialMsgCounter;

// ---- Link flags (written from Bluefruit-task callbacks -> volatile) ----
static volatile bool     remoteLinkUp = false;   // camera holds our ce80 remote service
static volatile bool     controlLinkUp = false;  // our central link to the camera's be80 is up
static volatile uint16_t remoteConnHandle = BLE_CONN_HANDLE_INVALID;
static volatile uint16_t controlConnHandle = BLE_CONN_HANDLE_INVALID;
static volatile bool     scanHit = false;        // scanner spotted the camera advertising
static volatile int8_t   lastRecordState = -1;   // be82-reported record state (-1 unknown)

// Last-6 characters of the camera's advertised name, captured by the scan
// callback (one-slot latch: buffer written, then ready published). The
// name suffix IS the wake payload's mfg[14..19] bytes — CAMERA_LOOP
// cross-checks it against the stored serial and adopts it on mismatch.
static char          scanNameSuffix[insta360_protocol::kSerialLen + 1] = "";
static volatile bool scanNameReady = false;

// ---- ce81 RX double-buffer (BLE callback fills, CAMERA_LOOP drains) ----
// Mirrors firmware_ota's fill/flush idiom, sized for the short serial /
// status frames the camera writes. Payload arrays are plain RAM; the
// per-slot ready flags + lengths are the synchronization points.
static const uint8_t kCe81BufSize = 32;
static uint8_t          ce81Buf[2][kCe81BufSize];
static volatile uint8_t ce81Len[2] = {0, 0};
static volatile bool    ce81Ready[2] = {false, false};
static volatile uint8_t ce81WriteIdx = 0;  // callback writes (Bluefruit task)
static uint8_t          ce81ReadIdx = 0;   // main loop reads

// ---- One-shot pending events (main-loop context only, consumed by
// CAMERA_LOOP as camera_fsm::Inputs one-shots) ----
static bool sessionEndPending = false;
static bool pairRequestPending = false;
static bool pairCancelPending = false;
static bool pairSerialCapturedPending = false;  // serial captured AND persisted
static bool unpairPending = false;

// Manual bench-test mode (paired-camera Test menu). While set, CAMERA_LOOP()
// suppresses the FSM so it can't fight the tester's manual wake/record/
// power actions — the Bluefruit callbacks still keep both links serviced.
// Main-loop context only; cleared by cameraTeardown() so any release path
// (transfer takeover, sleep) leaves it cleanly.
static bool cameraTestActive = false;

///////////////////////////////////////////
// GATT OBJECTS
///////////////////////////////////////////

// Peripheral: the GPS-remote service the camera pairs against.
static BLEService cameraRemoteService = BLEService(0xCE80);
static BLECharacteristic cameraCe81Char = BLECharacteristic(0xCE81);  // camera -> us (serial/status)
static BLECharacteristic cameraCe82Char = BLECharacteristic(0xCE82);  // us -> camera (button frames)
static BLECharacteristic cameraCe83Char = BLECharacteristic(0xCE83);  // static ID

// Secondary service the real remote exposes (cameras probe it).
// 128-bit base 0000xxxx-3C17-D293-8E48-14FE2E4DA212, little-endian on
// the wire, uuid16 spliced into bytes 12-13.  // X4-VERIFY(sniff)
#define CAMERA_D0FF_UUID128(u16)                                       \
  { 0x12, 0xA2, 0x4D, 0x2E, 0xFE, 0x14, 0x48, 0x8E,                    \
    0x93, 0xD2, 0x17, 0x3C,                                            \
    (uint8_t)((u16) & 0xFF), (uint8_t)(((u16) >> 8) & 0xFF),           \
    0x00, 0x00 }

static const uint8_t kUuidD0ff[16] = CAMERA_D0FF_UUID128(0xD0FF);
static const uint8_t kUuidFfd1[16] = CAMERA_D0FF_UUID128(0xFFD1);
static const uint8_t kUuidFfd2[16] = CAMERA_D0FF_UUID128(0xFFD2);
static const uint8_t kUuidFfd3[16] = CAMERA_D0FF_UUID128(0xFFD3);
static const uint8_t kUuidFfd4[16] = CAMERA_D0FF_UUID128(0xFFD4);
static const uint8_t kUuidFfd5[16] = CAMERA_D0FF_UUID128(0xFFD5);
static const uint8_t kUuidFfd8[16] = CAMERA_D0FF_UUID128(0xFFD8);
static const uint8_t kUuidFff1[16] = CAMERA_D0FF_UUID128(0xFFF1);
static const uint8_t kUuidFff2[16] = CAMERA_D0FF_UUID128(0xFFF2);
static const uint8_t kUuidFfe0[16] = CAMERA_D0FF_UUID128(0xFFE0);

static BLEService d0ffService = BLEService(BLEUuid(kUuidD0ff));
static BLECharacteristic ffd1Char = BLECharacteristic(BLEUuid(kUuidFfd1));
static BLECharacteristic ffd2Char = BLECharacteristic(BLEUuid(kUuidFfd2));
static BLECharacteristic ffd3Char = BLECharacteristic(BLEUuid(kUuidFfd3));
static BLECharacteristic ffd4Char = BLECharacteristic(BLEUuid(kUuidFfd4));
static BLECharacteristic ffd5Char = BLECharacteristic(BLEUuid(kUuidFfd5));
static BLECharacteristic ffd8Char = BLECharacteristic(BLEUuid(kUuidFfd8));
static BLECharacteristic fff1Char = BLECharacteristic(BLEUuid(kUuidFff1));
static BLECharacteristic fff2Char = BLECharacteristic(BLEUuid(kUuidFff2));
static BLECharacteristic ffe0Char = BLECharacteristic(BLEUuid(kUuidFfe0));

// Central: client objects for the camera's own control service.
static BLEClientService cameraControlService = BLEClientService(0xBE80);
static BLEClientCharacteristic cameraBe81Char = BLEClientCharacteristic(0xBE81);  // us -> camera commands
static BLEClientCharacteristic cameraBe82Char = BLEClientCharacteristic(0xBE82);  // camera -> us notifies

///////////////////////////////////////////
// BLUEFRUIT-TASK CALLBACKS
// RAM copies + volatile flags ONLY — no SD, no parsing beyond memcpy
// (be82 record-state parse is RAM-only and explicitly allowed), no
// blocking Bluefruit calls.
///////////////////////////////////////////

// ce81 write: the camera pushing its serial/status frames at us.
static void cameraCe81WriteCallback(uint16_t conn_hdl, BLECharacteristic* chr,
                                    uint8_t* data, uint16_t len) {
  (void)conn_hdl;
  (void)chr;
  // Only accept camera-owned traffic — a transfer peer poking the camera
  // GATT must not feed the pairing pipeline (mirrors the bleOwner guard
  // in bleFileRequestCallback). bleOwner only transitions on the main
  // loop, so this read is stable.
  if (bleOwner != BLE_OWNER_CAMERA) return;

  uint8_t slot = ce81WriteIdx;
  if (ce81Ready[slot]) return;  // both slots full — main loop stalled, drop

  // Drop oversize frames outright rather than truncating: the serial
  // parser is tail-anchored, so a truncated serial frame would yield 6
  // plausible-but-wrong bytes and could pair us to garbage.
  if (len > kCe81BufSize) return;
  memcpy(ce81Buf[slot], data, len);
  ce81Len[slot] = (uint8_t)len;  // publish length first...
  ce81Ready[slot] = true;       // ...then the "ready" signal
  ce81WriteIdx = slot ^ 1;
}

// D0FF write chars: the camera probes these on the real remote; we
// accept and ignore.  // X4-VERIFY(sniff)
static void cameraD0ffIgnoreWriteCallback(uint16_t conn_hdl, BLECharacteristic* chr,
                                          uint8_t* data, uint16_t len) {
  (void)conn_hdl;
  (void)chr;
  (void)data;
  (void)len;
}

// ce82 CCCD write: the camera (as GATT client on our remote service)
// turning button-frame delivery on/off. Trace only — the moment of
// subscription is the key diagnostic for "power-off does nothing"
// (0 = unsubscribed, 1 = notifications, 2 = indications).
static void cameraCe82CccdCallback(uint16_t conn_hdl, BLECharacteristic* chr,
                                   uint16_t value) {
  (void)conn_hdl;
  (void)chr;
  debug(F("CAM: ce82 CCCD write = "));
  debugln(value);
}

// be82 notify: record-state reports from the camera's control service.
// Parse-only (pure unit, RAM) — keep the last known state, don't let a
// non-record-state frame clobber it back to unknown.
static void cameraBe82NotifyCallback(BLEClientCharacteristic* chr,
                                     uint8_t* data, uint16_t len) {
  (void)chr;
  int8_t s = insta360_protocol::parseBe82RecordState(data, len);
  if (s >= 0) lastRecordState = s;
}

// Central connect: discover the camera's be80 service + both chars and
// enable be82 notifications. Doing discovery inside the central connect
// callback is the standard Bluefruit pattern (see the core's
// central_bleuart / client_hrm examples). Discovery failure -> drop the
// link and leave controlLinkUp false; the FSM retries on its timeout.
static void cameraCentralConnectCallback(uint16_t conn_handle) {
  controlConnHandle = conn_handle;

  debugln(F("CAM: central connected, discovering be80..."));
  if (!cameraControlService.discover(conn_handle) ||
      !cameraBe81Char.discover() ||
      !cameraBe82Char.discover() ||
      !cameraBe82Char.enableNotify()) {
    debugln(F("CAM: be80 discovery failed, dropping link"));
    Bluefruit.disconnect(conn_handle);
    return;
  }

  controlLinkUp = true;
  debugln(F("CAM: control link up"));
}

static void cameraCentralDisconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)reason;
  if (conn_handle == controlConnHandle) {
    controlLinkUp = false;
    controlConnHandle = BLE_CONN_HANDLE_INVALID;
    lastRecordState = -1;
    debugln(F("CAM: control link down"));
  }
}

// Scan RX: flag the sighting and connect (standard Bluefruit central
// pattern; a successful connect() stops the scanner). If the connect
// attempt fails (SoftDevice busy / conn count), resume scanning: the
// scanner is left PAUSED after a filtered hit, and a later
// Scanner.start() while paused is a silent INVALID_STATE no-op — without
// the resume, the FSM's whole retry budget would burn on a stalled scan.
//
// Also capture the camera's advertised name ("X4 XXXXXX"): its last six
// characters ARE the wake-payload bytes (community-verified: the wake
// mfg[14..19] is just the BLE-name suffix as ASCII). CAMERA_LOOP cross-
// checks this against the stored serial and adopts it on mismatch, so a
// bad ce81 capture can't leave the wake advert carrying wrong bytes.
// RAM-parse + memcpy only (Bluefruit task rules).
static void cameraScanCallback(ble_gap_evt_adv_report_t* report) {
  char name[32] = {0};
  uint8_t n = Bluefruit.Scanner.parseReportByType(
      report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, (uint8_t*)name,
      sizeof(name) - 1);
  if (n == 0) {
    n = Bluefruit.Scanner.parseReportByType(
        report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, (uint8_t*)name,
        sizeof(name) - 1);
  }
  if (n >= insta360_protocol::kSerialLen && !scanNameReady) {
    memcpy(scanNameSuffix, name + n - insta360_protocol::kSerialLen,
           insta360_protocol::kSerialLen);
    scanNameSuffix[insta360_protocol::kSerialLen] = '\0';
    scanNameReady = true;  // publish last
  }

  scanHit = true;
  if (!Bluefruit.Central.connect(report)) {
    Bluefruit.Scanner.resume();
  }
}

///////////////////////////////////////////
// PERIPHERAL CONNECT ROUTING
// Called from bluetooth.ino's shared connect/disconnect callbacks when
// bleOwner == BLE_OWNER_CAMERA. Bluefruit-task context: flags only.
///////////////////////////////////////////

void cameraBleOnConnect(uint16_t connHandle) {
  remoteConnHandle = connHandle;
  remoteLinkUp = true;
}

void cameraBleOnDisconnect(uint16_t connHandle, uint8_t reason) {
  (void)reason;
  if (connHandle == remoteConnHandle) {
    remoteLinkUp = false;
    remoteConnHandle = BLE_CONN_HANDLE_INVALID;
  }
}

bool cameraBleOwnsConnHandle(uint16_t connHandle) {
  // Match on the handle alone (not remoteLinkUp): cameraTeardown() clears
  // remoteLinkUp synchronously but leaves the handle set so the in-flight
  // disconnect event still routes here instead of to the transfer path.
  // The handle is only invalidated by cameraBleOnDisconnect(), and the
  // SoftDevice cannot reuse it before that event has fired.
  return remoteConnHandle != BLE_CONN_HANDLE_INVALID &&
         connHandle == remoteConnHandle;
}

///////////////////////////////////////////
// GATT REGISTRATION
// Called exactly once from bleCoreEnsureInit() (main loop), before any
// advertising starts.
///////////////////////////////////////////

// D0FF helper: READ char with a small fixed value.
static void cameraBeginReadChar(BLECharacteristic& chr, const uint8_t* value,
                                uint8_t len) {
  chr.setProperties(CHR_PROPS_READ);
  chr.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  chr.setFixedLen(len);
  chr.begin();
  chr.write(value, len);
}

// D0FF helper: WRITE char whose payload we accept and ignore.
static void cameraBeginWriteChar(BLECharacteristic& chr) {
  chr.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  chr.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  chr.setMaxLen(20);
  chr.setWriteCallback(cameraD0ffIgnoreWriteCallback);
  chr.begin();
}

void cameraBleRegisterServices() {
  // ---- Peripheral: 0xCE80 remote service ----
  cameraRemoteService.begin();

  // ce81: camera -> us. Callback copies into the RX double-buffer only.
  cameraCe81Char.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  cameraCe81Char.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  cameraCe81Char.setMaxLen(64);
  cameraCe81Char.setWriteCallback(cameraCe81WriteCallback);
  cameraCe81Char.begin();

  // ce82: us -> camera button frames.
  cameraCe82Char.setProperties(CHR_PROPS_NOTIFY | CHR_PROPS_INDICATE);
  cameraCe82Char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cameraCe82Char.setMaxLen(20);
  cameraCe82Char.setCccdWriteCallback(cameraCe82CccdCallback);
  cameraCe82Char.begin();
  uint8_t ce82Initial = 0x00;
  cameraCe82Char.write(&ce82Initial, 1);

  // ce83: static ID bytes (uint16 LE 0x0201).
  static const uint8_t kCe83Value[2] = {0x01, 0x02};
  cameraCe83Char.setProperties(CHR_PROPS_READ);
  cameraCe83Char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cameraCe83Char.setFixedLen(sizeof(kCe83Value));
  cameraCe83Char.begin();
  cameraCe83Char.write(kCe83Value, sizeof(kCe83Value));

  // ---- Peripheral: D0FF secondary service ----
  // Replicated from reference GPS Action Remote implementations — the
  // camera probes these characteristics on the real remote. READ chars
  // without a documented value serve a single zero byte; WRITE chars
  // accept and ignore.  // X4-VERIFY(sniff)
  static const uint8_t kZeroByte[1] = {0x00};
  static const uint8_t kFfd3Value[4] = {0x01, 0x90, 0x1E, 0x30};
  static const uint8_t kFfd4Value[4] = {0x01, 0x20, 0x00, 0x18};

  d0ffService.begin();
  cameraBeginWriteChar(ffd1Char);
  cameraBeginReadChar(ffd2Char, kZeroByte, sizeof(kZeroByte));
  cameraBeginReadChar(ffd3Char, kFfd3Value, sizeof(kFfd3Value));
  cameraBeginReadChar(ffd4Char, kFfd4Value, sizeof(kFfd4Value));
  cameraBeginReadChar(ffd5Char, kZeroByte, sizeof(kZeroByte));
  cameraBeginWriteChar(ffd8Char);
  cameraBeginReadChar(fff1Char, kZeroByte, sizeof(kZeroByte));
  cameraBeginWriteChar(fff2Char);
  cameraBeginReadChar(ffe0Char, kZeroByte, sizeof(kZeroByte));

  // ---- Central: client objects for the camera's be80 service ----
  cameraControlService.begin();
  cameraBe81Char.begin();
  cameraBe82Char.setNotifyCallback(cameraBe82NotifyCallback);
  cameraBe82Char.begin();

  // Only this module uses the central role, so the central + scanner
  // callbacks live here. Scanner config: active scan (the camera's name
  // rides in the scan response), filtered to the be80 service so the
  // callback only ever sees the camera.
  Bluefruit.Central.setConnectCallback(cameraCentralConnectCallback);
  Bluefruit.Central.setDisconnectCallback(cameraCentralDisconnectCallback);

  Bluefruit.Scanner.setRxCallback(cameraScanCallback);
  Bluefruit.Scanner.restartOnDisconnect(false);
  Bluefruit.Scanner.setInterval(160, 80);  // 100 ms interval / 50 ms window
  Bluefruit.Scanner.useActiveScan(true);
  // X4-VERIFY(sniff): matches only 16-bit UUID list AD fields — if the
  // X4 advertises be80 inside a 128-bit vendor UUID list instead, this
  // filter never fires and the central connect relies on the camera
  // connecting to us first.
  Bluefruit.Scanner.filterUuid(0xBE80);

  debugln(F("CAM: GATT registered (ce80 + D0FF + be80 client)"));
}

///////////////////////////////////////////
// SEND HELPERS (main-loop context)
///////////////////////////////////////////

// Stream a be81 command frame in <= 20-byte chunks. Prefers write-
// without-response so the ~250 Hz main loop stays non-blocking, but the
// choice is gated on the DISCOVERED properties: Bluefruit's
// BLEClientCharacteristic::write() issues a Write Command
// unconditionally (the property VERIFY in the core is commented out),
// so a char that only accepts Write Request would silently discard
// every frame while write() still reported success. Falls back to a
// blocking Write Request when WWR isn't offered (bounded; ~4 chunks max
// at 1 Hz for the GPS frame).
static bool cameraSendBe81(const uint8_t* frame, size_t len) {
  if (!controlLinkUp || !cameraBe81Char.discovered()) return false;

  const bool useWriteCmd =
      (cameraBe81Char.properties() & CHR_PROPS_WRITE_WO_RESP) != 0;

  insta360_protocol::Chunker c;
  c.buf = frame;
  c.len = len;
  c.offset = 0;

  uint8_t chunk[insta360_protocol::kChunkSize];
  size_t n;
  while ((n = insta360_protocol::nextChunk(c, chunk)) > 0) {
    uint16_t wrote = useWriteCmd ? cameraBe81Char.write(chunk, (uint16_t)n)
                                 : cameraBe81Char.write_resp(chunk, (uint16_t)n);
    // Write-command path can still come up short on TX-buffer
    // exhaustion — retry that one chunk as a Write Request.
    if (wrote != n && useWriteCmd) {
      wrote = cameraBe81Char.write_resp(chunk, (uint16_t)n);
    }
    if (wrote != n) return false;
  }
  return true;
}

// Send a ce82 button frame over the remote link, honoring HOW the camera
// subscribed: some centrals enable INDICATIONS on a NOTIFY|INDICATE char,
// and a notification to an indication-only subscriber is silently
// discarded by the stack. Returns false (with the reason in the debug
// trace) instead of failing invisibly — no link, no subscription, and a
// rejected send are all distinguishable now.
static bool cameraSendCe82(const uint8_t* buf, uint16_t n) {
  if (!remoteLinkUp) {
    debugln(F("CAM: ce82 send failed - no remote link (R)"));
    return false;
  }
  const bool notifySub = cameraCe82Char.notifyEnabled();
  const bool indicateSub = cameraCe82Char.indicateEnabled();
  if (!notifySub && !indicateSub) {
    debugln(F("CAM: ce82 send failed - camera never subscribed (CCCD off)"));
    return false;
  }
  bool ok = notifySub ? cameraCe82Char.notify(buf, n)
                      : cameraCe82Char.indicate(buf, n);
  debug(F("CAM: ce82 "));
  debug(notifySub ? F("notify ") : F("indicate "));
  debugln(ok ? F("sent") : F("REJECTED by stack"));
  return ok;
}

// Best-effort ce82 power-off button frame (remote link).
static bool cameraSendPowerOff() {
  uint8_t buf[16];
  size_t n = insta360_protocol::buildPowerOff(buf, sizeof(buf));
  return n > 0 && cameraSendCe82(buf, (uint16_t)n);
}

// Best-effort be81 stop-video (used by the teardown paths).
static void cameraSendStopVideo() {
  uint8_t frame[80];
  size_t n = insta360_protocol::buildStopVideo(frame, sizeof(frame), msgCounter++);
  if (n > 0) cameraSendBe81(frame, n);
}

///////////////////////////////////////////
// ACTION EXECUTION (main-loop context)
///////////////////////////////////////////

// Ensure the core is up and take the advert set for the camera. Returns
// false when the transfer page owns the radio — we never steal it
// (unreachable in practice: CAMERA_LOOP doesn't run while parked).
static bool cameraTakeRadio() {
  bleCoreEnsureInit();  // lazy — first camera action brings BLE up
  if (bleOwner == BLE_OWNER_TRANSFER) {
    debugln(F("CAM: transfer owns radio, skipping advert action"));
    return false;
  }
  bleOwner = BLE_OWNER_CAMERA;
  Bluefruit.Advertising.stop();  // safe no-op if not advertising
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();
  return true;
}

static void cameraExecuteAction(camera_fsm::Action a) {
  switch (a) {
    case camera_fsm::Action::kNone:
      break;

    case camera_fsm::Action::kStartWakeBurst: {
      if (!cameraTakeRadio()) break;
      // A fresh wake must judge the camera on new sightings only — drop
      // any advert hit latched by an earlier scan window.
      scanHit = false;

      uint8_t adv[insta360_protocol::kWakeAdvertLen];
      if (insta360_protocol::buildWakeAdvert(adv, serialBytes) !=
          insta360_protocol::kWakeAdvertLen) {
        break;  // can't happen with a fixed-size out buffer
      }
      // Broadcast setup per the sniffed reference remote (pchwalek):
      //
      //  - CONNECTABLE, not non-connectable: the manuf data powers the
      //    camera on, and the camera then connects BACK to this same
      //    "Insta360 GPS Remote" advert (the R-link). A non-connectable
      //    beacon breaks that reconnect — that rework was wrong.
      //  - The 28-byte manuf AD + flags fill the 31-byte primary PDU, so
      //    the name goes in the SCAN RESPONSE.
      //  - Both packets are set as RAW byte arrays via setData() so the
      //    Bluefruit stack cannot truncate or reshape them — the on-air
      //    payload is byte-identical to buildWakeAdvert()'s golden-tested
      //    output (pchwalek needed a custom ESP32 API for exactly this).
      // A CONNECTABLE start needs the single peripheral slot free. If the
      // camera already holds our R-link it is awake — a wake advert is
      // pointless and start() would fail with NRF_ERROR_CONN_COUNT.
      if (Bluefruit.Periph.connected()) {
        debugln(F("CAM: wake skipped - camera already connected (R-link up)"));
        break;
      }
      Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);
      // Scan response: Complete Local Name "Insta360 GPS Remote" (raw AD:
      // len 0x14 = type + 19 chars). setName() too, so the GAP name
      // characteristic matches once the camera connects.
      static const uint8_t kNameScanRsp[21] = {
          0x14, 0x09, 'I', 'n', 's', 't', 'a', '3', '6', '0', ' ',
          'G',  'P',  'S', ' ', 'R', 'e', 'm', 'o', 't', 'e'};
      Bluefruit.setName("Insta360 GPS Remote");
      Bluefruit.Advertising.setInterval(160, 160);  // 100 ms, fast == slow
      Bluefruit.Advertising.restartOnDisconnect(false);
      // No setFastTimeout(): with equal fast/slow intervals the fast
      // window expiring changes nothing.
      bool ok = Bluefruit.Advertising.setData(adv, insta360_protocol::kWakeAdvertLen);
      ok = Bluefruit.ScanResponse.setData(kNameScanRsp, sizeof(kNameScanRsp)) && ok;
      // Pad both packets to 31 bytes — MANDATORY (Bluefruit 0.21.0
      // freezes packet lengths at the first advert of the boot; a
      // shorter first advert would truncate this PDU on air; see
      // bleAdvFinalizePadded()).
      bleAdvFinalizePadded();
      ok = ok && Bluefruit.Advertising.start(0);
      debug(F("CAM: wake advertising "));
      debugln(ok ? F("STARTED (connectable, name in scanrsp)")
                 : F("FAILED - advert not running"));
      break;
    }

    case camera_fsm::Action::kStartConnectableAdvertising: {
      if (!cameraTakeRadio()) break;
      // Explicit type: the shared Advertising object keeps whatever the
      // last owner set.
      Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);
      // Exact name the camera looks for. Payload kept lean — flags(3) +
      // 0xCE80 service(4) + name(21) = 28 <= 31 fits the primary advert;
      // no TX-power element (the reference remote's advert has none).
      Bluefruit.setName("Insta360 GPS Remote");
      Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
      Bluefruit.Advertising.addService(cameraRemoteService);
      Bluefruit.Advertising.addName();
      Bluefruit.Advertising.restartOnDisconnect(false);
      Bluefruit.Advertising.setInterval(32, 244);
      Bluefruit.Advertising.setFastTimeout(30);
      bleAdvFinalizePadded();  // every advert must be 31+31 — see bluetooth.ino
      debug(F("CAM: connectable advertising "));
      debugln(Bluefruit.Advertising.start(0) ? F("STARTED") : F("FAILED"));
      break;
    }

    case camera_fsm::Action::kStopAdvertising:
      if (bleOwner == BLE_OWNER_CAMERA) {
        Bluefruit.Advertising.stop();
      }
      break;

    case camera_fsm::Action::kStartControlConnect:
      debugln(F("CAM: scanning for camera (be80)"));
      // stop() first: a retry can land while the scanner sits PAUSED
      // after a filtered hit whose connect failed, and start() from
      // paused is a silent INVALID_STATE no-op.
      Bluefruit.Scanner.stop();
      Bluefruit.Scanner.start(0);
      break;

    case camera_fsm::Action::kStopControlConnect:
      Bluefruit.Scanner.stop();
      // A pending CONNECT_IND handshake (advert seen, no handle yet)
      // isn't covered by disconnect() — cancel it explicitly so a late
      // connect can't complete while the FSM is idle or the transfer
      // service owns the radio. Harmless (returns an error) when no
      // connect is pending.
      (void)sd_ble_gap_connect_cancel();
      // Drop a half-open (or aborted) central connection so the retry
      // starts clean.
      if (controlConnHandle != BLE_CONN_HANDLE_INVALID) {
        Bluefruit.disconnect(controlConnHandle);
      }
      break;

    case camera_fsm::Action::kSendStartVideo: {
      uint8_t frame[80];
      size_t n = insta360_protocol::buildStartVideo(frame, sizeof(frame), msgCounter++);
      if (n > 0 && cameraSendBe81(frame, n)) {
        debugln(F("CAM: start-video sent"));
      }
      break;
    }

    case camera_fsm::Action::kSendStopVideo: {
      // Known edge (rare): if the control link dropped in the same
      // instant as a manual session end, this send fails silently and
      // the camera keeps recording. The cooldown's ce82 power-off (or
      // the camera's own auto-off if both links died) closes the file —
      // accepted for v1 rather than adding a stop-retry channel.
      cameraSendStopVideo();
      debugln(F("CAM: stop-video sent"));
      break;
    }

    case camera_fsm::Action::kSendKeepAlive: {
      uint8_t frame[80];
      size_t n = insta360_protocol::buildKeepAlive(frame, sizeof(frame));
      if (n > 0) cameraSendBe81(frame, n);
      break;
    }

    case camera_fsm::Action::kSendGpsFrame: {
      insta360_protocol::GpsSample s;
      s.unixTimeSec = (uint32_t)getGpsUnixTimestamp();
      s.latitudeDeg = gpsData.latitudeDegrees;   // signed degrees
      s.longitudeDeg = gpsData.longitudeDegrees; // signed degrees
      s.speedMps = gpsData.speed * 0.514444;     // knots -> m/s
      s.headingDeg = gpsData.heading;
      s.altitudeM = gpsData.altitude;

      uint8_t frame[80];  // GPS frame is 71 bytes
      size_t n = insta360_protocol::buildGpsFrame(frame, sizeof(frame), msgCounter++, s);
      if (n > 0) cameraSendBe81(frame, n);
      break;
    }

    case camera_fsm::Action::kSendPowerOff:
      cameraSendPowerOff();
      debugln(F("CAM: power-off sent"));
      break;

    case camera_fsm::Action::kDisconnect:
      Bluefruit.Scanner.stop();
      if (controlConnHandle != BLE_CONN_HANDLE_INVALID) {
        Bluefruit.disconnect(controlConnHandle);
      }
      if (remoteLinkUp && remoteConnHandle != BLE_CONN_HANDLE_INVALID) {
        Bluefruit.disconnect(remoteConnHandle);
      }
      break;
  }
}

///////////////////////////////////////////
// ce81 RX DRAIN (main-loop context)
// Classifies each buffered camera write; a serial frame captured while
// pairing is validated, persisted (setSetting — main-loop SD access is
// safe here), and reported to the FSM as the pairSerialCaptured
// one-shot ONLY if the settings write succeeded.
///////////////////////////////////////////

static void cameraDrainCe81() {
  while (ce81Ready[ce81ReadIdx]) {
    uint8_t local[kCe81BufSize];
    uint8_t len = ce81Len[ce81ReadIdx];
    if (len > kCe81BufSize) len = kCe81BufSize;
    memcpy(local, ce81Buf[ce81ReadIdx], len);
    ce81Ready[ce81ReadIdx] = false;  // slot free for the callback again
    ce81ReadIdx ^= 1;

    uint8_t rawSerial[insta360_protocol::kSerialLen];
    insta360_protocol::Ce81Frame frame =
        insta360_protocol::parseCe81Frame(local, len, rawSerial);

    if (frame == insta360_protocol::Ce81Frame::kSerial &&
        cameraFsm.state == camera_fsm::State::kPairing) {
      // Normalize through parseSerial: uppercases and re-validates the
      // charset so a garbled frame can't persist junk.
      char rawStr[insta360_protocol::kSerialLen + 1];
      insta360_protocol::serialToString(rawSerial, rawStr);

      uint8_t normalized[insta360_protocol::kSerialLen];
      if (!insta360_protocol::parseSerial(rawStr, normalized)) {
        debugln(F("CAM: ce81 serial frame failed validation, ignoring"));
        continue;
      }

      char serialStr[insta360_protocol::kSerialLen + 1];
      insta360_protocol::serialToString(normalized, serialStr);

      if (setSetting("camera_serial", serialStr)) {
        memcpy(serialBytes, normalized, sizeof(serialBytes));
        memcpy(storedSerial, serialStr, sizeof(storedSerial));
        pairSerialCapturedPending = true;  // one-shot for the next step
        debug(F("CAM: serial captured: "));
        debugln(serialStr);
      } else {
        // Not captured as far as the FSM is concerned — pairing stays
        // active, the camera will rewrite the frame, we retry the save.
        debugln(F("CAM: serial capture — settings write failed"));
      }
    }
  }
}

// Cross-check the camera's advertised-name suffix (captured by the scan
// callback) against the stored serial, and ADOPT it on mismatch: the
// suffix IS the wake payload's mfg[14..19] (community-verified — e.g.
// "X5 6RER6H" wakes on ASCII "6RER6H"), so it outranks whatever the ce81
// pairing frame carried. Main-loop context (settings write is SD).
static void cameraDrainScanName() {
  if (!scanNameReady) return;

  uint8_t suffixBytes[insta360_protocol::kSerialLen];
  if (insta360_protocol::parseSerial(scanNameSuffix, suffixBytes)) {
    // Only correct an existing pairing — never silently pair to a
    // passing camera the user hasn't chosen.
    if (cameraFsm.serialPresent &&
        memcmp(suffixBytes, serialBytes, insta360_protocol::kSerialLen) != 0) {
      char suffixStr[insta360_protocol::kSerialLen + 1];
      insta360_protocol::serialToString(suffixBytes, suffixStr);
      debug(F("CAM: advert name suffix "));
      debug(suffixStr);
      debug(F(" != stored serial "));
      debug(storedSerial);
      debugln(F(" - adopting the advertised one (wake payload source)"));
      if (setSetting("camera_serial", suffixStr)) {
        memcpy(serialBytes, suffixBytes, insta360_protocol::kSerialLen);
        memcpy(storedSerial, suffixStr, sizeof(suffixStr));
      }
    }
  }
  scanNameReady = false;  // release the latch for the next sighting
}

///////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////

void CAMERA_LOOP() {
  // 1. Drain camera writes first so a captured serial feeds this step.
  cameraDrainCe81();
  cameraDrainScanName();

  // Manual bench-test mode: the tester drives wake/record/power directly
  // (see cameraTest*() below), so the FSM is suppressed here — stepping it
  // would let auto-record fight the manual actions (e.g. re-issue a wake or
  // trip the stop-condition timer). Links stay serviced by the Bluefruit
  // callbacks; ce81 is drained above to keep the RX buffer from wedging.
  if (cameraTestActive) return;

  // 2. Fresh Inputs snapshot.
  camera_fsm::Inputs in;
  in.nowMs = millis();
  in.rpm = tachLastReported;
  in.speedMph = gps_speed_mph;
  in.gpsFixValid = gpsData.fix;
  in.remoteConnected = remoteLinkUp;
  in.controlConnected = controlLinkUp;
  in.cameraAdvertSeen = scanHit;
  in.recordConfirmed = lastRecordState;
  // One-shots: consume (clear) each pending flag as it is handed over.
  in.sessionEndRequested = sessionEndPending;
  sessionEndPending = false;
  in.pairRequested = pairRequestPending;
  pairRequestPending = false;
  in.pairCancelRequested = pairCancelPending;
  pairCancelPending = false;
  in.pairSerialCaptured = pairSerialCapturedPending;
  pairSerialCapturedPending = false;
  in.unpairRequested = unpairPending;
  unpairPending = false;

  // 3. Step the FSM.
  const camera_fsm::State preState = cameraFsm.state;
  camera_fsm::Action action = camera_fsm::step(cameraFsm, in);

  // cameraAdvertSeen is only meaningful in kWaking — clear the latch only
  // once a step has actually CONSUMED a true value (the scan callback can
  // set it between the snapshot read above and here; clearing
  // unconditionally would erase a sighting no step ever saw).
  if (preState == camera_fsm::State::kWaking && in.cameraAdvertSeen) {
    scanHit = false;
  }

  // The kConnecting->kAwaitGps success path returns kNone (the connect
  // itself stopped the scanner), but a retry that overlapped a slow
  // discovery can leave the scanner running — stop it explicitly on the
  // transition so it can't idle-scan for the whole session.
  if (preState == camera_fsm::State::kConnecting &&
      cameraFsm.state == camera_fsm::State::kAwaitGps) {
    Bluefruit.Scanner.stop();
  }

  // WAKING now runs the be80 scanner concurrently with the wake beacon (the
  // FSM emits kStartControlConnect as its entry step 2). Any exit that is NOT
  // heading into CONNECTING — which deliberately keeps the scanner for the
  // in-flight connect — must stop the scanner too: the FSM's single returned
  // action (kStopAdvertising) only stops the beacon.
  if (preState == camera_fsm::State::kWaking &&
      cameraFsm.state != camera_fsm::State::kWaking &&
      cameraFsm.state != camera_fsm::State::kConnecting) {
    Bluefruit.Scanner.stop();
  }

  // 4. Execute the (single) returned action.
  cameraExecuteAction(action);
}

///////////////////////////////////////////
// LIFECYCLE
///////////////////////////////////////////

void CAMERA_SETUP() {
  char buf[16];
  bool serialOk = false;
  if (getSetting("camera_serial", buf, sizeof(buf))) {
    // Treat any parse failure (empty string, wrong length, bad chars)
    // as unpaired.
    serialOk = insta360_protocol::parseSerial(buf, serialBytes);
  }
  if (serialOk) {
    insta360_protocol::serialToString(serialBytes, storedSerial);
    debug(F("CAM: paired camera serial: "));
    debugln(storedSerial);
  } else {
    storedSerial[0] = '\0';
    memset(serialBytes, 0, sizeof(serialBytes));
    debugln(F("CAM: no camera paired"));
  }
  camera_fsm::init(cameraFsm, serialOk);
  // No BLE init here — the SoftDevice comes up lazily on the first
  // advertising action, so unpaired users pay zero RAM/power cost.
}

void CAMERA_NOTIFY_SESSION_END() {
  sessionEndPending = true;
}

// Shared teardown for transfer takeover and sleep entry (main-loop
// context). Best-effort protocol goodbyes, physical link/advert
// teardown, then one forced-idle FSM step. `sendPowerOff` is the sleep
// variant's extra ce82 power-off.
static void cameraTeardown(bool sendPowerOff) {
  if (bleInitialized) {
    // Best-effort stop-video so the camera doesn't record an orphaned
    // clip (chunked, write-without-response preferred).
    if (cameraFsm.state == camera_fsm::State::kRecording && controlLinkUp) {
      cameraSendStopVideo();
    }
    if (sendPowerOff && remoteLinkUp) {
      cameraSendPowerOff();
    }

    Bluefruit.Scanner.stop();
    // Also cancel any pending CONNECT_IND handshake (no handle yet) so a
    // late central connect can't complete after this teardown.
    (void)sd_ble_gap_connect_cancel();
    if (controlConnHandle != BLE_CONN_HANDLE_INVALID) {
      Bluefruit.disconnect(controlConnHandle);
    }
    if (remoteLinkUp && remoteConnHandle != BLE_CONN_HANDLE_INVALID) {
      Bluefruit.disconnect(remoteConnHandle);
      // Clear the link flag NOW — the disconnect completes asynchronously
      // and the FSM must not see a stale remoteConnected after this
      // teardown (a stuck-true flag makes IDLE skip the wake advert
      // forever). The handle itself stays set so the in-flight disconnect
      // event still routes to cameraBleOnDisconnect() via
      // cameraBleOwnsConnHandle() instead of the transfer path.
      remoteLinkUp = false;
    }
    if (bleOwner == BLE_OWNER_CAMERA) {
      Bluefruit.Advertising.stop();
      bleOwner = BLE_OWNER_NONE;
    }
  }

  // Drop latched signals + queued one-shots — none may survive into the
  // next camera session. (ce81 slots too: the callback stopped writing
  // the moment ownership was released above.)
  ce81Ready[0] = false;
  ce81Ready[1] = false;
  scanHit = false;
  lastRecordState = -1;
  sessionEndPending = false;
  pairRequestPending = false;
  pairCancelPending = false;
  pairSerialCapturedPending = false;
  unpairPending = false;
  cameraTestActive = false;  // any release path leaves bench-test mode

  // Force the FSM to IDLE/UNPAIRED with one explicit step. The physical
  // teardown above already happened; the returned action is
  // belt-and-suspenders and idempotent, so execute it anyway.
  camera_fsm::Inputs in;
  in.nowMs = millis();
  in.remoteConnected = remoteLinkUp;    // async disconnects may lag
  in.controlConnected = controlLinkUp;
  in.forceIdleRequested = true;
  cameraExecuteAction(camera_fsm::step(cameraFsm, in));

  debugln(F("CAM: released"));
}

void CAMERA_FORCE_RELEASE() {
  cameraTeardown(false);
}

void CAMERA_SLEEP() {
  cameraTeardown(true);
}

///////////////////////////////////////////
// UI SURFACE (main-loop context)
///////////////////////////////////////////

bool cameraIsPaired() {
  return cameraFsm.serialPresent;
}

camera_fsm::State cameraFsmState() {
  return cameraFsm.state;
}

bool cameraRemoteLinkUp() {
  return remoteLinkUp;
}

bool cameraControlLinkUp() {
  return controlLinkUp;
}

bool cameraAdvertisingUp() {
  return bleInitialized && Bluefruit.Advertising.isRunning();
}

bool cameraCe82Subscribed() {
  return bleInitialized && remoteLinkUp &&
         (cameraCe82Char.notifyEnabled() || cameraCe82Char.indicateEnabled());
}

bool cameraPairedSerial(char* buf, size_t bufSize) {
  if (buf == nullptr || bufSize == 0) return false;
  buf[0] = '\0';
  if (!cameraFsm.serialPresent || storedSerial[0] == '\0') return false;
  snprintf(buf, bufSize, "%s", storedSerial);
  return true;
}

bool cameraRequestPair() {
  // The FSM's guard (UNPAIRED/IDLE only) is authoritative; mirror it
  // here so the return value is honest — e.g. mid-cooldown the request
  // is refused and nothing is queued.
  if (cameraFsm.state != camera_fsm::State::kUnpaired &&
      cameraFsm.state != camera_fsm::State::kIdle) {
    return false;
  }
  pairRequestPending = true;
  return true;
}

void cameraCancelPair() {
  pairCancelPending = true;
}

bool cameraRequestUnpair() {
  if (cameraFsm.state != camera_fsm::State::kUnpaired &&
      cameraFsm.state != camera_fsm::State::kIdle) {
    return false;
  }
  // Persist first: if the settings write fails, change nothing —
  // otherwise the UI would show unpaired while the stored serial
  // re-arms the FSM on the next boot.
  if (!setSetting("camera_serial", "")) {
    debugln(F("CAM: unpair — settings write failed"));
    return false;
  }
  storedSerial[0] = '\0';
  memset(serialBytes, 0, sizeof(serialBytes));
  unpairPending = true;
  debugln(F("CAM: unpaired"));
  return true;
}

bool cameraSetManualSerial(const char* serial6) {
  uint8_t normalized[insta360_protocol::kSerialLen];
  if (!insta360_protocol::parseSerial(serial6, normalized)) return false;

  char serialStr[insta360_protocol::kSerialLen + 1];
  insta360_protocol::serialToString(normalized, serialStr);

  if (!setSetting("camera_serial", serialStr)) return false;

  memcpy(serialBytes, normalized, sizeof(serialBytes));
  memcpy(storedSerial, serialStr, sizeof(storedSerial));
  camera_fsm::init(cameraFsm, true);  // re-arm
  debug(F("CAM: manual serial set: "));
  debugln(storedSerial);
  return true;
}

///////////////////////////////////////////
// BENCH TEST MENU (main-loop context)
// Paired-only manual controls so the camera link can be exercised on the
// bench without staging RPM/GPS to drive the auto-record FSM. Each helper
// reuses the exact cameraExecuteAction() code path the FSM would run, so
// the wire behavior is identical to a real session. While test mode is
// active CAMERA_LOOP() suppresses the FSM (see the guard there).
///////////////////////////////////////////

void cameraTestEnterMode() {
  // Clean baseline first: drop any in-flight session and force the FSM to
  // IDLE (cameraTeardown also clears cameraTestActive), then arm test mode.
  cameraTeardown(false);
  cameraTestActive = true;
  debugln(F("CAM: bench-test mode entered"));
}

void cameraTestExitMode() {
  // Best-effort stop before dropping links so we never leave the camera
  // recording an orphaned clip after a test.
  if (controlLinkUp) cameraSendStopVideo();
  cameraTestActive = false;
  cameraTeardown(false);  // drop links, stop advert/scan, release the radio
  debugln(F("CAM: bench-test mode exited"));
}

bool cameraTestWake() {
  if (!cameraTestActive) return false;
  // Manufacturer-data wake advert. NOTE: this only wakes a camera that is in
  // *standby* (screen off, BLE radio alive, "Bluetooth Wakeup" enabled) — a
  // fully powered-off X4 has its radio off and cannot be woken over BLE.
  cameraExecuteAction(camera_fsm::Action::kStartWakeBurst);
  return true;
}

bool cameraTestConnect() {
  if (!cameraTestActive) return false;
  // Two independent links, established together:
  //   R (remote/peripheral) — present the connectable "Insta360 GPS Remote"
  //     advert so the camera can connect to our ce80 GATT. The camera only
  //     does this after it has been paired from ITS Settings -> Bluetooth
  //     remote menu (capturing the serial is not enough); once connected, the
  //     ce82 power-off frame can reach it.
  //   C (control/central) — scan for the camera's own be80 service and
  //     connect to it for start/stop-video (works without the R link).
  cameraExecuteAction(camera_fsm::Action::kStartConnectableAdvertising);
  cameraExecuteAction(camera_fsm::Action::kStartControlConnect);
  return true;
}

bool cameraTestStartRecording() {
  if (!cameraTestActive) return false;
  cameraExecuteAction(camera_fsm::Action::kSendStartVideo);
  return controlLinkUp;  // frame only reaches the camera when the link is up
}

bool cameraTestStopRecording() {
  if (!cameraTestActive) return false;
  cameraExecuteAction(camera_fsm::Action::kSendStopVideo);
  return controlLinkUp;
}

bool cameraTestPowerOff() {
  if (!cameraTestActive) return false;
  // Direct call (same code path the FSM action takes) so the bench menu
  // gets the REAL send result: false distinguishes no-R-link / camera
  // never subscribed / stack-rejected — all previously invisible.
  return cameraSendPowerOff();
}
