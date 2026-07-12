///////////////////////////////////////////
// CAMERA BLE MODULE (Insta360 X4 auto-record — remote emulation)
//
// This device IS the Insta360 GPS Remote. We are a BLE PERIPHERAL ONLY:
// we host the remote's GATT (service 0xCE80: ce81 WRITE camera->us state,
// ce82 NOTIFY us->camera button frames, ce83 READ static), advertise the
// wake/identity payload, and the camera connects to US as central and
// subscribes to our ce82. ALL control — record (shutter toggle) and
// power-off — is a ce82 notification, byte-for-byte the physical remote's
// frames. We NEVER act as central: no scanning, no connecting to the
// camera's own be80 service, no be81 writes. (The in-camera GPS overlay
// used the be81 central path; its transport is unidentified, so it is
// dropped here. GPS still logs to SD independently.)
//
// All camera behavior is decided by the host-tested camera_fsm pure unit;
// this module builds the Inputs snapshot, executes the returned Action,
// and owns the Bluefruit plumbing. Frame bytes come from insta360_protocol.
//
// THREADING (mirrors firmware_ota.ino): Bluefruit callbacks only copy
// bytes into RAM and set volatile flags. CAMERA_LOOP() on the main loop
// drains the flags/buffers, steps the FSM, and does all real work —
// including the one setSetting() that persists a captured serial and the
// streamed power-off hold.
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

// ce82 button-frame sequence counter — the real remote places this at
// byte[4] and steps it by 2 with every button frame it sends. Kept as a
// single running counter across all ce82 frames (shutter + power-off).
static uint8_t ce82Seq = 0x00;

// Power-off is a HELD button: the real remote streams the 3-second-hold
// frame continuously (fresh seq each frame) until the camera powers off.
// Non-blocking — CAMERA_LOOP feeds it; ends at the deadline or when the
// camera drops the R-link (i.e. it powered off).
static uint32_t ce82HoldUntil = 0;    // millis deadline; 0 = not holding
static uint32_t ce82HoldNextAt = 0;   // next frame due at
constexpr uint32_t kCe82PowerHoldMs   = 3500;  // stream a >3 s hold
constexpr uint32_t kCe82HoldIntervalMs = 30;   // ~30 ms/frame (remote ~25 ms)

// GPS telemetry: the real remote streams RMC on ce82 at 10 Hz the WHOLE
// time the camera is connected+subscribed — it is the remote's liveness
// heartbeat (never go silent, or the camera treats us as dead and drops
// the link). Streamed continuously regardless of recording state, with
// status 'V' when there's no fix (spec §7.6). Paused only during a power-off hold.
static uint32_t gpsStreamNextAt = 0;
constexpr uint32_t kGpsStreamIntervalMs = 100;  // 10 Hz

// ---- Link flags (written from Bluefruit-task callbacks -> volatile) ----
static volatile bool     remoteLinkUp = false;   // camera holds our ce80 remote service
static volatile uint16_t remoteConnHandle = BLE_CONN_HANDLE_INVALID;

// Cached ce82 subscription state. cameraCe82Char.notifyEnabled() is a
// SoftDevice SVC round-trip; calling it ~500x/s (once per loop snapshot +
// once per GPS-stream tick) burned latency in the loop that must sustain
// 25 Hz GPS logging. Instead the ce82 CCCD callback latches this, and while
// linked-but-not-yet-subscribed CAMERA_LOOP re-reads notifyEnabled() at a
// low rate (a bonded peer's sys-attr CCCD restore fires NO CCCD write, so
// the callback alone can miss it). Once true we stop calling the SVC.
static volatile bool ce82NotifyOn = false;
static uint32_t      ce82SubSyncAt = 0;   // next lazy notifyEnabled() re-read (main loop)

// Camera-reported record state, parsed from its 0x10 ce81 display-string
// frames (insta360_protocol::parseRecordingState — the ".HH:MM:SS" timer).
// The camera exposes no clean record flag, so this is how we CONFIRM the
// shutter actually took. Latched by the ce81 drain and read when building
// the FSM Inputs — both main-loop context, so no volatile needed. Fed to
// the FSM only while fresh (a stale observation must not drive the shutter).
static insta360_protocol::RecordObs cameraRecordObs =
    insta360_protocol::RecordObs::kUnknown;
static uint32_t cameraRecordObsAtMs = 0;
constexpr uint32_t kRecordObsFreshMs = 3000;  // older than this -> kUnknown to the FSM

// Bench-test record belief. The FSM is suppressed in bench mode, so its
// recordingActive isn't updated by the manual Record toggles — track the
// toggle here so cameraTestExitMode() can GUARANTEE the camera is left
// stopped (else it records an orphaned clip and inverts the next session).
static bool cameraTestRecording = false;

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
// power actions — the Bluefruit callbacks still keep the R-link serviced.
// Main-loop context only; cleared by cameraTeardown() so any release path
// (transfer takeover, sleep) leaves it cleanly.
static bool cameraTestActive = false;

// Bench mode: keep the connectable remote advert re-armed. Set by
// Test->Wake; while set (and the R-link is down) CAMERA_LOOP restarts the
// advert whenever it isn't running — the camera has been seen connecting
// and dropping sub-second, and after a drop it can only retry if we're
// still on air.
static bool     testAdvertWanted = false;
static uint32_t testAdvertRetryAt = 0;

///////////////////////////////////////////
// GATT OBJECTS
///////////////////////////////////////////

// Peripheral: the GPS-remote service the camera pairs against. The genuine
// remote we captured (nRF Connect) exposes ONLY ce80 (ce81/ce82/ce83) plus
// standard GAP/GATT — no D0FF secondary service — so that is all we host.
static BLEService cameraRemoteService = BLEService(0xCE80);
static BLECharacteristic cameraCe81Char = BLECharacteristic(0xCE81);  // camera -> us (serial/status)
static BLECharacteristic cameraCe82Char = BLECharacteristic(0xCE82);  // us -> camera (button frames)
static BLECharacteristic cameraCe83Char = BLECharacteristic(0xCE83);  // static ID

///////////////////////////////////////////
// BLUEFRUIT-TASK CALLBACKS
// RAM copies + volatile flags ONLY — no SD, no parsing beyond memcpy, no
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

// ce82 CCCD write: the camera (as GATT client on our remote service)
// turning button-frame delivery on/off. Trace only — the moment of
// subscription is the key diagnostic for "power-off does nothing"
// (0 = unsubscribed, 1 = notifications, 2 = indications).
static void cameraCe82CccdCallback(uint16_t conn_hdl, BLECharacteristic* chr,
                                   uint16_t value) {
  (void)conn_hdl;
  (void)chr;
  // Cache the subscription so the hot paths don't SVC into the SoftDevice.
  // Bit 0 (0x0001) = notifications enabled; 0 = unsubscribed.
  ce82NotifyOn = (value & 0x0001) != 0;
  debug(F("CAM: ce82 CCCD write = "));
  debugln(value);
}

///////////////////////////////////////////
// PERIPHERAL CONNECT ROUTING
// Called from bluetooth.ino's shared connect/disconnect callbacks when
// bleOwner == BLE_OWNER_CAMERA (or the handle matches). Bluefruit-task
// context: flags only.
///////////////////////////////////////////

void cameraBleOnConnect(uint16_t connHandle) {
  remoteConnHandle = connHandle;
  remoteLinkUp = true;
  ce82NotifyOn = false;  // fresh link starts unsubscribed until the camera's CCCD write
  // Trace matters: the camera has been seen connecting and dropping
  // faster than the display refresh — without this line those attempts
  // are completely invisible.
  debugln(F("CAM: R-link CONNECTED (camera took our remote advert)"));
}

void cameraBleOnDisconnect(uint16_t connHandle, uint8_t reason) {
  // The HCI reason code says WHO ended it and why: 0x13 = camera chose
  // to disconnect (it vetted our GATT and left), 0x08 = supervision
  // timeout (radio loss), 0x16 = we ended it (teardown), 0x3E = failed
  // to establish.
  debug(F("CAM: R-link DISCONNECTED, reason 0x"));
  debugln(reason, HEX);
  if (connHandle == remoteConnHandle) {
    remoteLinkUp = false;
    remoteConnHandle = BLE_CONN_HANDLE_INVALID;
    ce82NotifyOn = false;  // subscription doesn't survive the link
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

void cameraBleRegisterServices() {
  // ---- Peripheral: 0xCE80 remote service ----
  cameraRemoteService.begin();

  // ce81: camera -> us. Callback copies into the RX double-buffer only.
  cameraCe81Char.setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  cameraCe81Char.setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
  cameraCe81Char.setMaxLen(64);
  cameraCe81Char.setWriteCallback(cameraCe81WriteCallback);
  cameraCe81Char.begin();

  // ce82: us -> camera button frames. NOTIFY-only, matching the genuine
  // remote's GATT (captured); the camera subscribes by CCCD write. We do
  // NOT force encryption on the characteristic (SECMODE_OPEN) — bonding is
  // negotiated at the link level (see bleCoreEnsureInit()'s Just-Works
  // acceptance); the camera may withhold this CCCD write until the link is
  // encrypted, which is why cameraCe82Subscribed() is the real "buttons
  // deliverable" gate.
  cameraCe82Char.setProperties(CHR_PROPS_NOTIFY);
  cameraCe82Char.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  cameraCe82Char.setMaxLen(insta360_protocol::kMaxGpsFrameLen);  // fits the ~90-byte GPS frame
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

  debugln(F("CAM: GATT registered (ce80 peripheral only)"));
}

///////////////////////////////////////////
// SEND HELPERS (main-loop context)
///////////////////////////////////////////

// Send a ce82 button frame over the remote link. ce82 is NOTIFY-only
// (matches the real remote); the camera subscribes by writing the CCCD.
// Without that subscription the notify is discarded by the stack, so
// report the real reason in the trace instead of failing invisibly — no
// link and no subscription are distinguishable now.
static bool cameraSendCe82(const uint8_t* buf, uint16_t n) {
  if (!remoteLinkUp) {
    debugln(F("CAM: ce82 send failed - no remote link (R)"));
    return false;
  }
  if (!ce82NotifyOn) {
    debugln(F("CAM: ce82 send failed - camera never subscribed (CCCD off)"));
    return false;
  }
  return cameraCe82Char.notify(buf, n);
}

// Arm the streamed power-off hold. The real remote does not send a single
// power-off frame — it streams the 3-second-hold frame continuously while
// the button is held, so we do the same (non-blocking; serviced by
// cameraServiceCe82Hold in CAMERA_LOOP). Returns false if the camera
// isn't connected/subscribed, so the bench menu can report the real reason.
static bool cameraSendPowerOff() {
  if (!remoteLinkUp || !ce82NotifyOn) {
    debugln(F("CAM: power-off not armed - no R-link / not subscribed"));
    return false;
  }
  ce82HoldUntil = millis() + kCe82PowerHoldMs;
  ce82HoldNextAt = millis();  // first frame immediately
  debugln(F("CAM: power-off hold armed (streaming ce82)"));
  return true;
}

// Synchronous streamed power-off hold for SLEEP entry. The non-blocking
// cameraServiceCe82Hold() (in CAMERA_LOOP) never runs on the sleep path —
// the loop parks the moment we sleep — so the arm-then-disconnect teardown
// used to transmit ZERO power-off frames and leave the camera running all
// night. Here we stream the hold inline, blocking ~kCe82PowerHoldMs (fine:
// sleep entry is a deliberate action and we're powering down anyway), then
// return so the caller can disconnect. Aborts early if the camera drops the
// link — that drop IS the camera powering off.
static void cameraStreamPowerOffBlocking() {
  if (!remoteLinkUp || !ce82NotifyOn) return;
  const uint32_t deadline = millis() + kCe82PowerHoldMs;
  while ((int32_t)(millis() - deadline) < 0) {
    if (!remoteLinkUp) break;  // camera powered off / dropped the link
    uint8_t buf[9];
    size_t n = insta360_protocol::buildPowerOff(buf, sizeof(buf), ce82Seq);
    ce82Seq += 2;
    if (n > 0) cameraCe82Char.notify(buf, (uint16_t)n);
    wdtPet();  // this hold (~3.5 s) approaches the ~4 s WDT — keep it fed
    delay(kCe82HoldIntervalMs);  // let the SoftDevice TX the notify
  }
  debugln(F("CAM: sleep power-off hold streamed"));
}

// Stream the power-off hold frame while armed. Called every CAMERA_LOOP
// iteration (before any early return) so it runs in both bench-test and
// FSM modes. Stops at the deadline or the moment the camera drops the
// R-link — that drop IS the camera powering off.
static void cameraServiceCe82Hold() {
  if (ce82HoldUntil == 0) return;
  const uint32_t now = millis();
  if (!remoteLinkUp || (int32_t)(now - ce82HoldUntil) >= 0) {
    ce82HoldUntil = 0;
    debugln(F("CAM: power-off hold ended"));
    return;
  }
  if ((int32_t)(now - ce82HoldNextAt) < 0) return;  // rate limit
  ce82HoldNextAt = now + kCe82HoldIntervalMs;
  uint8_t buf[9];
  size_t n = insta360_protocol::buildPowerOff(buf, sizeof(buf), ce82Seq);
  ce82Seq += 2;
  if (n > 0) cameraCe82Char.notify(buf, (uint16_t)n);
}

// Stream the 10 Hz GPS/RMC liveness frame while the camera is connected +
// subscribed (see kGpsStreamIntervalMs). Runs every CAMERA_LOOP iteration
// (before any early return) in both bench-test and FSM modes. Paused
// during a power-off hold so the hold frames aren't diluted. Always emits
// (status 'V' with last-known coords) when there's no fix — never silent.
static void cameraServiceGpsStream() {
  // Rate gate FIRST (cheap), so the ~96% of ticks between 10 Hz frames cost
  // nothing. Subscription comes from the cached flag — no SVC round-trip.
  const uint32_t now = millis();
  if ((int32_t)(now - gpsStreamNextAt) < 0) return;
  gpsStreamNextAt = now + kGpsStreamIntervalMs;
  if (!bleInitialized || !remoteLinkUp || !ce82NotifyOn) return;
  if (ce82HoldUntil != 0) return;  // a power-off hold owns ce82 right now

  insta360_protocol::GpsRmc s;
  s.valid = gpsData.fix;
  s.hour = gpsData.hour;
  s.minute = gpsData.minute;
  s.second = gpsData.seconds;
  s.milli = gpsData.milliseconds;
  s.day = gpsData.day;
  s.month = gpsData.month;
  s.year = (uint8_t)(gpsData.year % 100);  // ddmmyy wants 2-digit year
  s.latitudeDeg = gpsData.latitudeDegrees;
  s.longitudeDeg = gpsData.longitudeDegrees;
  s.speedKnots = gpsData.speed;            // gpsData.speed is already knots
  s.courseDeg = gpsData.heading;

  uint8_t buf[insta360_protocol::kMaxGpsFrameLen];
  size_t n = insta360_protocol::buildGpsRmcFrame(buf, sizeof(buf), s);
  if (n > 0) cameraCe82Char.notify(buf, (uint16_t)n);
}

// Send one ce82 shutter frame — the shutter TOGGLES recording. The FSM
// tracks recordingActive itself, so a single frame per record/stop is all
// the glue emits (no blind re-toggle on reconnect).
static bool cameraSendShutter() {
  uint8_t buf[9];
  size_t n = insta360_protocol::buildShutterToggle(buf, sizeof(buf), ce82Seq);
  ce82Seq += 2;
  if (n == 0) return false;
  return cameraSendCe82(buf, (uint16_t)n);
}

///////////////////////////////////////////
// ADVERTISING (main-loop context)
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

// Present the genuine remote's on-air identity — the ONE advert a real
// GPS Remote transmits, X4-confirmed by capture and phone replay:
// primary = flags 0x05 + serial manufacturer block (this is also the wake
// trigger), scan response = appearance 0x0180 + name. Both set as raw
// bytes (Bluefruit cannot reshape them) and padded to 31+31 (see
// bleAdvFinalizePadded). Caller must have run cameraTakeRadio() and
// checked the peripheral slot. ~100 ms interval (real remote runs ~66).
static bool cameraStartRemoteIdentityAdvert() {
  uint8_t adv[insta360_protocol::kWakeAdvertLen];
  if (insta360_protocol::buildWakeAdvert(adv, serialBytes) !=
      insta360_protocol::kWakeAdvertLen) {
    return false;
  }
  uint8_t rsp[insta360_protocol::kRemoteScanRspLen];
  insta360_protocol::buildRemoteScanResponse(rsp);

  Bluefruit.setName("Insta360 GPS Remote");  // GAP name char, post-connect
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);
  Bluefruit.Advertising.setInterval(160, 160);  // 100 ms, fast == slow
  Bluefruit.Advertising.restartOnDisconnect(false);
  bool ok = Bluefruit.Advertising.setData(adv, sizeof(adv));
  ok = Bluefruit.ScanResponse.setData(rsp, sizeof(rsp)) && ok;
  bleAdvFinalizePadded();
  return ok && Bluefruit.Advertising.start(0);
}

// Unpaired pairing flow: no serial for the mfg payload yet, so use a plain
// discoverable connectable advert with the name + ce80 service in the
// primary PDU so the camera's "add remote" scan can find us.
static bool cameraStartPairingAdvert() {
  Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED);
  Bluefruit.setName("Insta360 GPS Remote");
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addService(cameraRemoteService);
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  bleAdvFinalizePadded();  // every advert must be 31+31 — see bluetooth.ino
  return Bluefruit.Advertising.start(0);
}

// Take the radio and start the appropriate connectable advert. Paired ->
// the real remote's identity (mfg payload + appearance/name scanrsp),
// which is BOTH the wake trigger and the reconnect target (they are the
// same packet on a genuine remote). Unpaired -> the plain pairing advert.
// A CONNECTABLE start needs the single peripheral slot free: if the camera
// already holds our R-link it is awake and connected, so skip.
static bool cameraStartAdvert() {
  if (!cameraTakeRadio()) return false;
  if (Bluefruit.Periph.connected()) {
    debugln(F("CAM: advert skipped - camera already connected (R-link up)"));
    return false;
  }
  if (cameraFsm.serialPresent) {
    return cameraStartRemoteIdentityAdvert();
  }
  return cameraStartPairingAdvert();
}

///////////////////////////////////////////
// ACTION EXECUTION (main-loop context)
///////////////////////////////////////////

static void cameraExecuteAction(camera_fsm::Action a) {
  switch (a) {
    case camera_fsm::Action::kNone:
      break;

    // Wake burst and re-advertise are the same on a pure-peripheral remote:
    // the camera connects back to the identity advert either way. Paired ->
    // identity/wake payload; unpaired pairing -> plain name+service advert.
    case camera_fsm::Action::kStartWakeBurst:
    case camera_fsm::Action::kStartConnectableAdvertising: {
      debug(F("CAM: advertising "));
      debugln(cameraStartAdvert() ? F("STARTED") : F("skipped/FAILED"));
      break;
    }

    case camera_fsm::Action::kStopAdvertising:
      if (bleOwner == BLE_OWNER_CAMERA) {
        Bluefruit.Advertising.stop();
      }
      break;

    case camera_fsm::Action::kSendShutter:
      debug(F("CAM: shutter "));
      debugln(cameraSendShutter() ? F("sent") : F("FAILED (no link/sub)"));
      break;

    case camera_fsm::Action::kSendPowerOff:
      cameraSendPowerOff();  // arms the streamed hold (serviced in CAMERA_LOOP)
      break;

    case camera_fsm::Action::kDisconnect:
      if (remoteLinkUp && remoteConnHandle != BLE_CONN_HANDLE_INVALID) {
        Bluefruit.disconnect(remoteConnHandle);
      }
      if (bleOwner == BLE_OWNER_CAMERA) {
        Bluefruit.Advertising.stop();
      }
      break;
  }
}

///////////////////////////////////////////
// ce81 RX DRAIN (main-loop context)
// Classifies each buffered camera write; a serial frame captured while
// pairing is validated, persisted (setSetting — main-loop SD access is
// safe here), and reported to the FSM as the pairSerialCaptured one-shot
// ONLY if the settings write succeeded.
///////////////////////////////////////////

static void cameraDrainCe81() {
  while (ce81Ready[ce81ReadIdx]) {
    uint8_t local[kCe81BufSize];
    uint8_t len = ce81Len[ce81ReadIdx];
    if (len > kCe81BufSize) len = kCe81BufSize;
    memcpy(local, ce81Buf[ce81ReadIdx], len);
    ce81Ready[ce81ReadIdx] = false;  // slot free for the callback again
    ce81ReadIdx ^= 1;

    // Record-state observation: the camera's 0x10 display-string frame
    // carries a ".HH:MM:SS" timer while recording (the only reliable record
    // signal — see parseRecordingState). Latch it for the FSM reconcile.
    const insta360_protocol::RecordObs obs =
        insta360_protocol::parseRecordingState(local, len);
    if (obs != insta360_protocol::RecordObs::kUnknown) {
      cameraRecordObs = obs;
      cameraRecordObsAtMs = millis();
    }

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

///////////////////////////////////////////
// MAIN LOOP
///////////////////////////////////////////

// Map the latched camera record observation into the FSM's tri-state,
// honouring the freshness window: a stale 0x10 (link idle / dropped) must
// read as kUnknown so it can't drive a shutter toggle.
static camera_fsm::RecordObs cameraObservedRecordForFsm() {
  if (cameraRecordObs == insta360_protocol::RecordObs::kUnknown) {
    return camera_fsm::RecordObs::kUnknown;
  }
  if ((uint32_t)(millis() - cameraRecordObsAtMs) > kRecordObsFreshMs) {
    return camera_fsm::RecordObs::kUnknown;
  }
  return cameraRecordObs == insta360_protocol::RecordObs::kRecording
             ? camera_fsm::RecordObs::kRecording
             : camera_fsm::RecordObs::kIdle;
}

void CAMERA_LOOP() {
  // Reconcile the cached ce82 subscription. The CCCD callback catches an
  // explicit subscribe, but a bonded peer's sys-attr restore enables
  // notifications with NO CCCD write — so while linked but not yet
  // known-subscribed, poll notifyEnabled() at a low rate to catch it. Once
  // subscribed, we never call into the SoftDevice for this again.
  if (remoteLinkUp && !ce82NotifyOn) {
    const uint32_t now = millis();
    if ((int32_t)(now - ce82SubSyncAt) >= 0) {
      ce82SubSyncAt = now + 250;
      if (cameraCe82Char.notifyEnabled()) ce82NotifyOn = true;
    }
  }

  // 1. Drain camera writes first so a captured serial / record observation
  //    feeds this step.
  cameraDrainCe81();
  // Stream an in-flight power-off hold, then the 10 Hz GPS liveness feed,
  // in BOTH bench-test and FSM modes (before the test-mode early return).
  cameraServiceCe82Hold();
  cameraServiceGpsStream();

  // Manual bench-test mode: the tester drives wake/record/power directly
  // (see cameraTest*() below), so the FSM is suppressed here — stepping it
  // would let auto-record fight the manual actions (e.g. re-issue a wake or
  // trip the stop-condition timer). The R-link stays serviced by the
  // Bluefruit callbacks; ce81 is drained above to keep the RX buffer from
  // wedging.
  if (cameraTestActive) {
    // Re-arm the connectable remote advert after an R-link drop (or a
    // start that never connected): the camera retries on ITS schedule and
    // can only succeed if we're still advertising. Rate-limited; stops the
    // moment the camera takes the slot (remoteLinkUp).
    if (testAdvertWanted && bleOwner == BLE_OWNER_CAMERA && !remoteLinkUp &&
        !Bluefruit.Advertising.isRunning() &&
        millis() - testAdvertRetryAt >= 1000) {
      testAdvertRetryAt = millis();
      debugln(F("CAM: re-arming remote advert (R-link free)"));
      cameraExecuteAction(camera_fsm::Action::kStartConnectableAdvertising);
    }
    return;
  }

  // 2. Fresh Inputs snapshot.
  camera_fsm::Inputs in;
  in.nowMs = millis();
  in.rpm = tachLastReported;
  in.speedMph = gps_speed_mph;
  in.gpsFixValid = gpsData.fix;
  in.remoteConnected = remoteLinkUp;
  in.ce82Subscribed = remoteLinkUp && ce82NotifyOn;
  in.recordObserved = cameraObservedRecordForFsm();
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

  // 3. Step the FSM and execute the (single) returned action.
  camera_fsm::Action action = camera_fsm::step(cameraFsm, in);
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
// context). Best-effort protocol goodbyes, physical link/advert teardown,
// then one forced-idle FSM step. `sendPowerOff` is the sleep variant's
// extra ce82 power-off.
static void cameraTeardown(bool sendPowerOff) {
  // Whether we could actually reach the camera to stop/power it off. If not,
  // we must NOT let the forced-idle step below clear recordingActive — the
  // camera may still be rolling, and lying about it would make the next
  // reconnect blind-toggle a live recording OFF (#4).
  const bool wasRecording = cameraFsm.recordingActive;
  bool cameraStopped = false;

  if (bleInitialized) {
    // Best-effort stop recording so the camera doesn't record an orphaned
    // clip. The shutter TOGGLES, so send it only if we believe recording
    // is active (the FSM tracks this) AND we can actually reach the camera.
    if (wasRecording && remoteLinkUp && ce82NotifyOn) {
      cameraSendShutter();
      cameraStopped = true;
    }
    // SLEEP variant: stream the power-off hold SYNCHRONOUSLY here — the loop
    // parks on sleep, so the non-blocking cameraServiceCe82Hold() would never
    // run and no frame would go out. Powering off also stops any recording.
    if (sendPowerOff && remoteLinkUp && ce82NotifyOn) {
      cameraStreamPowerOffBlocking();
      cameraStopped = true;
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
  // next camera session. (ce81 slots too: the callback stopped writing the
  // moment ownership was released above.)
  ce81Ready[0] = false;
  ce81Ready[1] = false;
  sessionEndPending = false;
  pairRequestPending = false;
  pairCancelPending = false;
  pairSerialCapturedPending = false;
  unpairPending = false;
  cameraTestActive = false;    // any release path leaves bench-test mode
  cameraTestRecording = false; // and clears the bench record belief
  testAdvertWanted = false;    // and stops the bench advert re-arm
  cameraRecordObs = insta360_protocol::RecordObs::kUnknown;  // fresh for next session

  // Force the FSM to IDLE/UNPAIRED with one explicit step. The physical
  // teardown above already happened; the returned action is
  // belt-and-suspenders and idempotent, so execute it anyway.
  camera_fsm::Inputs in;
  in.nowMs = millis();
  in.remoteConnected = remoteLinkUp;    // async disconnects may lag
  in.forceIdleRequested = true;
  cameraExecuteAction(camera_fsm::step(cameraFsm, in));

  // forceIdle clears recordingActive; if we could NOT reach the camera to
  // stop it, restore the belief so the next reconnect reconciles against the
  // camera's real 0x10 state instead of blind-toggling a live recording OFF.
  if (wasRecording && !cameraStopped) {
    cameraFsm.recordingActive = true;
  }

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

bool cameraAdvertisingUp() {
  return bleInitialized && Bluefruit.Advertising.isRunning();
}

bool cameraCe82Subscribed() {
  return bleInitialized && remoteLinkUp && ce82NotifyOn;
}

bool cameraGpsStreaming() {
  // The exact gate under which cameraServiceGpsStream() emits an RMC frame.
  return bleInitialized && remoteLinkUp && ce82NotifyOn && ce82HoldUntil == 0;
}

bool cameraObservedRecording() {
  return cameraRecordObs == insta360_protocol::RecordObs::kRecording &&
         (uint32_t)(millis() - cameraRecordObsAtMs) <= kRecordObsFreshMs;
}

bool cameraRecordObservationFresh() {
  return cameraRecordObs != insta360_protocol::RecordObs::kUnknown &&
         (uint32_t)(millis() - cameraRecordObsAtMs) <= kRecordObsFreshMs;
}

bool cameraPairedSerial(char* buf, size_t bufSize) {
  if (buf == nullptr || bufSize == 0) return false;
  buf[0] = '\0';
  if (!cameraFsm.serialPresent || storedSerial[0] == '\0') return false;
  snprintf(buf, bufSize, "%s", storedSerial);
  return true;
}

bool cameraRequestPair() {
  // The FSM's guard (UNPAIRED/IDLE only) is authoritative; mirror it here
  // so the return value is honest — e.g. mid-cooldown the request is
  // refused and nothing is queued.
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
  // otherwise the UI would show unpaired while the stored serial re-arms
  // the FSM on the next boot.
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
// reuses the exact send/advert code paths the FSM would run, so the wire
// behavior is identical to a real session. While test mode is active
// CAMERA_LOOP() suppresses the FSM (see the guard there).
///////////////////////////////////////////

void cameraTestEnterMode() {
  // Clean baseline first: drop any in-flight session and force the FSM to
  // IDLE (cameraTeardown also clears cameraTestActive), then arm test mode.
  cameraTeardown(false);
  cameraTestActive = true;
  debugln(F("CAM: bench-test mode entered"));
}

void cameraTestExitMode() {
  // GUARANTEE the camera is left stopped on exit — otherwise it records an
  // orphaned clip AND the next real session's belief (false) is inverted
  // relative to reality. Bench tracks its own toggle (cameraTestRecording),
  // since the FSM is suppressed and stays IDLE in bench mode.
  if (cameraTestRecording && remoteLinkUp && ce82NotifyOn) {
    cameraSendShutter();  // toggle recording OFF
  }
  cameraTestRecording = false;
  cameraTestActive = false;
  cameraTeardown(false);  // drop link, stop advert, release the radio
  debugln(F("CAM: bench-test mode exited"));
}

bool cameraTestWake() {
  if (!cameraTestActive) return false;
  // Present the wake / remote-identity advert (only reaches an ARMED
  // camera — X4: QuickCapture OFF). Connectable: the woken camera connects
  // back, so keep the advert re-armed (CAMERA_LOOP restarts it after a drop).
  testAdvertWanted = true;
  cameraExecuteAction(camera_fsm::Action::kStartWakeBurst);
  return cameraAdvertisingUp();
}

bool cameraTestRecord() {
  if (!cameraTestActive) return false;
  // ce82 shutter — TOGGLES recording. Same code path the FSM's kSendShutter
  // takes, so the wire bytes match. Returns true only when the frame could
  // actually reach the camera (connected + subscribed to ce82). Track the
  // toggle in the bench belief so cameraTestExitMode() can leave the camera
  // stopped.
  const bool sent = cameraSendShutter();
  if (sent) cameraTestRecording = !cameraTestRecording;
  return sent;
}

bool cameraTestPowerOff() {
  if (!cameraTestActive) return false;
  // Arm the streamed power-off hold (same as the FSM's kSendPowerOff).
  // Returns connected + subscribed so the bench menu reports the real
  // reason on failure.
  return cameraSendPowerOff();
}
