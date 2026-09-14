///////////////////////////////////////////
// SENSOREGG MODULE (wireless EGT pod — observer + GATT central)
//
// Two data paths into ONE surface (plan 0018):
//  - PAIRED egg in range while the egg radio is wanted -> the logger
//    connects as CENTRAL and consumes the PerchWerks Sensor Service
//    (sensoregg_gatt.{h,cpp}, host-tested): self-describing channel
//    table, per-channel Sample batch frames, Clock/boot_id epoch.
//  - Otherwise -> the passive PW-ADV observer exactly as before
//    (sensoregg_protocol.{h,cpp}); it is also the pairing transport.
// Both feed eggReading/eggRxMs/eggSeqMon, so every accessor, DOVEX
// column, race page and LED path is agnostic to the transport. They
// are never live at once: a connected single-link peripheral stops
// advertising.
//
// The camera link still wins every tradeoff: the central connection
// uses skinny parameters (configCentralConn event length 6 = 7.5 ms
// cap, bluetooth.ino), follows the same race/bench gate as the
// scanner, and SENSOREGG_SLEEP() drops it for transfers and shutdown.
//
// Cross-module globals (Bluefruit) are visible via Arduino's .ino
// concatenation — same as camera_ble.ino relies on.
///////////////////////////////////////////

#include "sensoregg.h"

#include <math.h>
#include <string.h>

#include "project.h"

#if BIRDSEYE_ENABLE_SENSOREGG

#include "bluetooth.h"  // bleCoreEnsureInit()
#include "sensoregg_gatt.h"  // PW service decoders + clock fit (plan 0018)
#include "sensoregg_protocol.h"
#include "settings.h"   // getSetting/setSetting — the sensoregg_mac key

// Forward declarations: these callback signatures mention SoftDevice /
// Bluefruit types, and Arduino's auto-prototype generator inserts
// prototypes BEFORE <bluefruit.h> is included — an explicit prototype
// makes arduino-cli skip generating one (same workaround as
// camera_ble.ino).
static void sensoreggScanCallback(ble_gap_evt_adv_report_t* report);
static void sensoreggSampleNotifyCb(BLEClientCharacteristic* chr,
                                    uint8_t* data, uint16_t len);

///////////////////////////////////////////
// MODULE STATE
///////////////////////////////////////////

// Runtime egg-MAC filter (plan 0017), HUMAN byte order. Boots from the
// SENSOREGG_MAC fallback, then SENSOREGG_SETUP() overrides it from the
// "sensoregg_mac" setting when that parses. All-zeros = accept-any
// (unpaired). Written on the main loop (capture / unpair), read in BLE
// task context via sensoreggMacAccepted() — a torn 6-byte read is
// tolerable by construction: capture updates it while eggPairingActive
// still bypasses the filter entirely, and unpair's half-zero transient
// can only mis-route a single report (one-report tolerance, same story
// as the documented raceActive read below).
static uint8_t eggMacFilter[6] = SENSOREGG_MAC;

// ---- pairing / bench state (plan 0017) ----
// The two flags are read in BLE task context (scan gate + callback
// filter bypass): volatile, same care as eggSleeping.
static volatile bool eggPairingActive = false;  // capture window open
static volatile bool eggTestActive = false;     // EGG TEST bench latch
static uint32_t eggPairStartMs = 0;             // main loop only
// Per-slot advertiser address (raw LSB-first), captured alongside the
// payload so the main-loop drain can pair without the callback ever
// learning any protocol. Plain RAM like eggBuf; the ready flags below
// are the synchronization points.
static uint8_t eggPeerMac[2][6];
// Packet-rate meter for the test page (main loop only): accepted parses
// over a 1 s tumbling window.
static uint16_t eggRateCount = 0;
static uint32_t eggRateWindowMs = 0;
static float eggRateHz = 0.0f;

// ---- RX double-buffer (BLE scan callback fills, SENSOREGG_LOOP drains;
// mirrors camera_ble.ino's ce81 idiom: payload arrays are plain RAM, the
// per-slot ready flags are the synchronization points) ----
static uint8_t           eggBuf[2][sensoregg_protocol::kPayloadLenMax];
static volatile uint8_t  eggLen[2] = {0, 0};   // actual bytes captured
static volatile uint32_t eggAtMs[2] = {0, 0};
static volatile bool     eggReady[2] = {false, false};
static volatile uint8_t  eggWriteIdx = 0;  // callback writes (Bluefruit task)
static uint8_t           eggReadIdx = 0;   // main loop reads

// Latest parsed reading (main-loop context only).
static sensoregg_protocol::Reading eggReading;
static uint32_t eggRxMs = 0;
static bool     eggHaveReading = false;
static sensoregg_protocol::SeqMonitor eggSeqMon;  // zombie-egg detection

static bool eggScannerRunning = false;
static bool eggSetupDone = false;   // scanner configured — SENSOREGG_LOOP may (re)start it
// Sleep gate, read by the scan callback (BLE task): a report accepted just
// before SENSOREGG_SLEEP() has its rx callback deferred, and the callback's
// mandatory Scanner.resume() would restart the scan AFTER the stop (the
// core's resume() never checks the running flag) — the scanner then runs
// through the entire charging park. volatile: written on the main loop,
// read in BLE task context.
static volatile bool eggSleeping = false;
static uint32_t eggLastKickMs = 0;      // last scanner self-heal kick (main loop)
static uint32_t eggLastStartTryMs = 0;  // last start attempt (retry throttle)

// Throttle for retrying a start() the SoftDevice refused, so a persistent
// refusal costs one SVC per second instead of one per ~4 ms loop iteration.
static constexpr uint32_t kEggStartRetryMs = 1000;

// ---- GATT LINK state (plan 0018) ------------------------------------------

// Client objects for the egg's PerchWerks service. Same LE UUID byte
// arrays the egg registers (spec section 2; [12]=index low, [13]=high).
// Brace-init: parens around a BLEUuid(array) temp is the most-vexing-
// parse and declares a function instead.
static const uint8_t kPwSvcUuid[16] = {
    0xba, 0x99, 0x4a, 0x31, 0x9c, 0xc1, 0xd7, 0x96,
    0x75, 0x4a, 0x72, 0x4e, 0x57, 0x50, 0x10, 0xe1};  // 0x5057 service
static const uint8_t kPwDescUuid[16] = {
    0xba, 0x99, 0x4a, 0x31, 0x9c, 0xc1, 0xd7, 0x96,
    0x75, 0x4a, 0x72, 0x4e, 0x58, 0x50, 0x10, 0xe1};  // 0x5058 Descriptor
static const uint8_t kPwSampUuid[16] = {
    0xba, 0x99, 0x4a, 0x31, 0x9c, 0xc1, 0xd7, 0x96,
    0x75, 0x4a, 0x72, 0x4e, 0x59, 0x50, 0x10, 0xe1};  // 0x5059 Sample
static const uint8_t kPwClkUuid[16] = {
    0xba, 0x99, 0x4a, 0x31, 0x9c, 0xc1, 0xd7, 0x96,
    0x75, 0x4a, 0x72, 0x4e, 0x5A, 0x50, 0x10, 0xe1};  // 0x505A Clock
static BLEClientService        eggSvc{BLEUuid(kPwSvcUuid)};
static BLEClientCharacteristic eggChrDesc{BLEUuid(kPwDescUuid)};
static BLEClientCharacteristic eggChrSamp{BLEUuid(kPwSampUuid)};
static BLEClientCharacteristic eggChrClk{BLEUuid(kPwClkUuid)};

// Ordered so "engaged" (a connection exists or is being made) is a
// single >= compare: BACKOFF sits between the idle states and
// CONNECTING on purpose.
enum EggLinkState : uint8_t {
  EGG_LINK_IDLE = 0,   // link not wanted (unpaired / gate closed)
  EGG_LINK_WAIT_ADV,   // wanted — the scan callback fires the connect
  EGG_LINK_BACKOFF,    // cooling off after a failure/disconnect
  EGG_LINK_CONNECTING, // sd_ble_gap_connect in flight
  EGG_LINK_BRINGUP,    // discovery/reads running in the callback task
  EGG_LINK_STREAMING,  // notify subscription live, surface fed by GATT
};
static volatile uint8_t  eggLinkState = EGG_LINK_IDLE;
static volatile uint16_t eggConnHandle = BLE_CONN_HANDLE_INVALID;
static volatile uint32_t eggLinkEventMs = 0;  // stamp of the last transition
static constexpr uint32_t kEggConnectTimeoutMs = 10000;
static constexpr uint32_t kEggLinkBackoffMs = 5000;

// Bring-up staging: the central connect callback (Bluefruit callback
// task) runs the BLOCKING discovery/read sequence — a documented
// deviation from "callbacks only copy" (each client op waits a
// connection interval; ~15 round trips on the main loop would stall the
// 25 Hz DOVEX row engine for seconds). It stages raw bytes here,
// ready-flag-LAST; the main loop parses and commits.
static uint8_t           eggDescRaw[512];
static volatile uint16_t eggDescLen = 0;
static uint8_t           eggClkRaw[sensoregg_gatt::kClockLen];
static volatile uint32_t eggClkReqMs = 0, eggClkRspMs = 0;
static volatile bool eggBringupReady = false;
static volatile bool eggBringupFailed = false;

// Notify frame ring (the data plane keeps the copy-only discipline):
// the notify callback memcpys, SENSOREGG_LOOP drains. 32-byte slots
// cover the egg's <=26-byte frames; a fatter future pod's frames are
// counted as drops rather than truncated.
static uint8_t           eggFrameBuf[8][32];
static volatile uint8_t  eggFrameLen[8];
static volatile uint32_t eggFrameAtMs[8];
static volatile bool     eggFrameReady[8] = {false};
static volatile uint8_t  eggFrameW = 0;  // notify callback writes
static uint8_t           eggFrameR = 0;  // main loop reads
static uint32_t          eggFrameDrops = 0;

// Committed pod identity (main loop only, valid while STREAMING).
static sensoregg_gatt::PodDescriptor eggPod;
static int8_t eggRoleIdx[sensoregg_gatt::ROLE_COUNT] = {-1, -1, -1, -1};
static int8_t eggFastestIdx = 0;  // whose frame seq feeds the zombie monitor
static sensoregg_gatt::ClockFit eggClockFit;

// RACE-GATED SCANNER (plan 0012). The scanner's 40-of-every-90 ms radio
// claim is only paid while a race session is running — the one time the EGT
// feed is actually consumed (DOVEX rows + the Temp pages are race-only).
// Outside race mode the radio belongs to whoever needs it clean: the
// always-on scan was measured throttling BLE file downloads to ~33 KB/s by
// denying the transfer link's connection-event extension. raceActive is the
// BirdsEye.ino session flag, visible via the .ino concatenation (same as
// Bluefruit above). Read from the scan callback (BLE task) too — a plain
// bool read is atomic on this core, and a stale read only delays the
// stop/start by one report (the main-loop reconcile owns the real state).
// Plan 0017 widened the gate: the pairing capture window and the EGG
// TEST bench latch also force the scan on — both are explicit user
// actions on the menu, bounded by the 2-minute pairing timeout / the
// test page's Back row, so the plan-0012 "never pay scan duty on the
// menu" rule still holds for every passive path (transfer, replay,
// idle menu).
static bool eggRadioWanted() {
  return (raceActive || eggPairingActive || eggTestActive) && !eggSleeping;
}

// Plan 0018 split the gate three ways. A connection exists or is being
// made:
static bool eggLinkEngaged() {
  return eggLinkState >= EGG_LINK_CONNECTING;
}

// The GATT link is wanted whenever the radio is AND a specific egg is
// paired (an unpaired filter would connect to anyone's pod — the beacon
// path stays promiscuous instead).
static bool eggLinkWanted() {
  return eggRadioWanted() && sensoreggIsPaired();
}

// The scanner runs while the pairing window is open (it must observe
// every egg) or while no link is engaged (unclaimed pod, pre-connect,
// backoff). While STREAMING the 44% scan duty is not paid at all — the
// paired egg is silent anyway (a connected single-link peripheral stops
// advertising). Keeps its name so the callback resume-gate and the
// reconcile below are untouched.
static bool eggScanWanted() {
  return eggRadioWanted() && (eggPairingActive || !eggLinkEngaged());
}

///////////////////////////////////////////
// SCAN CALLBACK (Bluefruit task context — copy bytes, resume, return)
///////////////////////////////////////////

// True when the reporting advertiser is our egg. nRF ble_gap_addr_t
// stores the address LSB-first, the filter is human-ordered (MSB
// first) — macAccepts() compares reversed (host-tested, plan 0017).
// Wildcard filter = accept anyone (the payload magic already filtered).
static bool sensoreggMacAccepted(const uint8_t* peerAddrLsbFirst) {
  return sensoregg_protocol::macAccepts(eggMacFilter, peerAddrLsbFirst);
}

static void sensoreggScanCallback(ble_gap_evt_adv_report_t* report) {
  // NO Serial, NO SD, NO display, NO delay() here — BLE task context.
  uint8_t buf[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
  uint8_t len = Bluefruit.Scanner.parseReportByType(
      report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buf, sizeof(buf));

  // GATT connect trigger (plan 0018): a connectable report from the
  // PAIRED egg, while the main loop has posted WAIT_ADV, becomes the
  // connection instead of a buffered beacon. NO Scanner.resume() on
  // this path — Bluefruit carries the paused scanner's params straight
  // into sd_ble_gap_connect, and resuming would fight it.
  if (eggLinkState == EGG_LINK_WAIT_ADV && !eggPairingActive &&
      report->type.connectable &&
      len >= sensoregg_protocol::kPayloadLen &&
      sensoregg_protocol::matchesMagic(buf, len) &&
      sensoreggMacAccepted(report->peer_addr.addr) &&
      sensoreggIsPaired()) {
    eggLinkState = EGG_LINK_CONNECTING;
    eggLinkEventMs = millis();
    Bluefruit.Central.connect(report);
    return;
  }

  // During the pairing capture window the MAC filter is bypassed so a
  // NEW egg (including a different one while another is paired) can be
  // observed — the main-loop drain does the actual capture.
  if (len >= sensoregg_protocol::kPayloadLen &&
      sensoregg_protocol::matchesMagic(buf, len) &&
      (sensoreggMacAccepted(report->peer_addr.addr) || eggPairingActive)) {
    const uint8_t w = eggWriteIdx;
    // Capture up to the largest known layout; the parser applies the
    // per-version length gate. (The old fixed-14 copy silently truncated
    // v2 frames — bytes 14-15 never reached the parser.)
    const uint8_t copyLen =
        len < sensoregg_protocol::kPayloadLenMax
            ? len
            : (uint8_t)sensoregg_protocol::kPayloadLenMax;
    memcpy(eggBuf[w], buf, copyLen);
    memcpy(eggPeerMac[w], report->peer_addr.addr, 6);
    eggLen[w] = copyLen;
    eggAtMs[w] = millis();
    eggReady[w] = true;
    eggWriteIdx = w ^ 1;
  }

  // MANDATORY: without resume() the scanner halts after the first report
  // — symptom is exactly one reading then permanent silence,
  // indistinguishable from a dead egg. Skipped while the scan is not
  // wanted (sleeping, or race over): this deferred callback may run AFTER
  // a Scanner.stop(), and resume() restarts the scan unconditionally —
  // resurrecting the scanner the stop just killed.
  if (eggScanWanted()) {
    Bluefruit.Scanner.resume();
  }
}

///////////////////////////////////////////
// GATT LINK (plan 0018 — central client callbacks)
///////////////////////////////////////////

// Notify callback (Bluefruit callback task): pure copy into the frame
// ring, ready-flag-last — the data plane keeps the module's copy-only
// discipline. An over-long frame (a fatter future pod at high MTU) is
// counted, never truncated.
static void sensoreggSampleNotifyCb(BLEClientCharacteristic* chr,
                                    uint8_t* data, uint16_t len) {
  (void)chr;
  if (len == 0 || len > sizeof(eggFrameBuf[0])) {
    eggFrameDrops++;
    return;
  }
  const uint8_t w = eggFrameW;
  if (eggFrameReady[w]) {  // ring full — the main loop is behind
    eggFrameDrops++;
    return;
  }
  memcpy(eggFrameBuf[w], data, len);
  eggFrameLen[w] = (uint8_t)len;
  eggFrameAtMs[w] = millis();
  eggFrameReady[w] = true;
  eggFrameW = (uint8_t)((w + 1) & 7);
}

// Central connect callback: runs the whole BLOCKING bring-up sequence
// in the Bluefruit callback task (documented deviation — see the
// staging block's comment; plan 0018). Stages raw bytes; the main loop
// parses and commits. No Serial here — the commit path narrates.
static void sensoreggCentralConnectCb(uint16_t connHandle) {
  eggConnHandle = connHandle;
  eggLinkState = EGG_LINK_BRINGUP;
  eggLinkEventMs = millis();

  bool ok = eggSvc.discover(connHandle);
  if (ok) ok = eggChrDesc.discover();
  if (ok) ok = eggChrClk.discover();
  if (ok) ok = eggChrSamp.discover();
  if (ok) {
    // Best-effort MTU raise: not correctness-critical — the client
    // read() long-reads past 23 on its own, and the egg sizes notify
    // frames to whatever MTU ends up live.
    BLEConnection* conn = Bluefruit.Connection(connHandle);
    if (conn) conn->requestMtuExchange(247);
  }
  if (ok) {
    eggDescLen = eggChrDesc.read(eggDescRaw, sizeof(eggDescRaw));
    ok = eggDescLen >= sensoregg_gatt::kDescHeaderLen;
  }
  if (ok) {
    // The timed Clock read IS the clock-fit anchor: request/response
    // stamps bracket the pod's millis snapshot.
    eggClkReqMs = millis();
    ok = eggChrClk.read(eggClkRaw, sizeof(eggClkRaw)) >=
         sensoregg_gatt::kClockLen;
    eggClkRspMs = millis();
  }
  if (ok) ok = eggChrSamp.enableNotify();

  if (ok) {
    eggBringupReady = true;  // ready-flag-last; main loop commits
  } else {
    eggBringupFailed = true;
    Bluefruit.disconnect(connHandle);
  }
}

static void sensoreggCentralDisconnectCb(uint16_t connHandle,
                                         uint8_t reason) {
  (void)connHandle;
  (void)reason;
  eggConnHandle = BLE_CONN_HANDLE_INVALID;
  eggLinkState = EGG_LINK_BACKOFF;
  eggLinkEventMs = millis();
}

// Called from bleCoreEnsureInit() (flag-gated there): client discovery
// metadata + central callbacks must exist before the central role is
// used. Client begin()s attach the characteristics to the service.
void sensoreggGattClientInit() {
  eggSvc.begin();
  eggChrDesc.begin();
  eggChrSamp.setNotifyCallback(sensoreggSampleNotifyCb);
  eggChrSamp.begin();
  eggChrClk.begin();
  Bluefruit.Central.setConnectCallback(sensoreggCentralConnectCb);
  Bluefruit.Central.setDisconnectCallback(sensoreggCentralDisconnectCb);
}

///////////////////////////////////////////
// LIFECYCLE
///////////////////////////////////////////

void SENSOREGG_SETUP() {
  // Bring the shared BLE core up (idempotent — registers every GATT
  // service before any advertising, so the camera/transfer peripherals
  // are unaffected by the early init).
  bleCoreEnsureInit();

  // Passive observer: 90 ms interval / 40 ms window, ~44% duty (see the
  // constants' comments for the anti-phase-lock and GPS-drop rationale).
  // Passive listening costs the camera link nothing — S140 yields scan
  // windows to connection events automatically.
  Bluefruit.Scanner.setRxCallback(sensoreggScanCallback);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.setInterval(sensoregg_protocol::kScanIntervalUnits,
                                sensoregg_protocol::kScanWindowUnits);
  Bluefruit.Scanner.filterRssi(sensoregg_protocol::kRssiFloorDbm);
  // INLINE manufacturer-ID filter — load-bearing, not an optimization.
  // Bluefruit rejects+resumes filtered packets inside its event handler;
  // an ACCEPTED packet pauses scanning until our deferred rx callback
  // runs and resumes. Without this filter every ambient packet above the
  // RSSI floor (phones, PCs, the X4) took that slow accepted path just to
  // be magic-rejected in our callback, collapsing scan duty in bursts.
  Bluefruit.Scanner.filterMSD(sensoregg_protocol::kCompanyId);
  eggSetupDone = true;

  // Runtime pairing filter (plan 0017): the "sensoregg_mac" setting
  // overrides the SENSOREGG_MAC fallback when it parses; empty or
  // invalid keeps the fallback (all-zeros default = accept-any).
  // SETTINGS_SETUP() ran earlier in setup(), and the scanner cannot be
  // running yet — no concurrency with the callback's filter read.
  {
    char macStr[sensoregg_protocol::kMacStrLen];
    if (getSetting("sensoregg_mac", macStr, sizeof(macStr))) {
      uint8_t mac[6];
      if (sensoregg_protocol::parseMac(macStr, mac)) {
        memcpy(eggMacFilter, mac, sizeof(eggMacFilter));
        debugln(F("SensorEgg: paired MAC loaded from settings"));
      }
    }
  }

  // Deliberately NOT started here (plan 0012). The scanner used to run
  // forever from boot; SENSOREGG_LOOP()'s reconcile now starts it when a
  // race session begins and stops it when the session ends, so the menu,
  // replay and (critically) BLE transfer mode never pay the 44% radio
  // duty. If the SoftDevice ever refuses the race-time start, the known
  // unblock is Bluefruit.begin(1, 1) in bleCoreEnsureInit() (costs one
  // unused central connection slot) — do not change the shared begin()
  // without re-soaking the camera link.
  debugln(F("SensorEgg: passive scanner armed (race-gated)"));
}

void SENSOREGG_SLEEP() {
  // Shutdown path: stop the forever-scan so the SoftDevice radio is quiet
  // before System OFF / the charging loop. Without this the scanner ran
  // through the entire "powered off while charging" park. Idempotent.
  // Gate FIRST: a scan report accepted just before this stop has a deferred
  // rx callback whose resume() would otherwise restart the scan afterwards
  // (see eggSleeping). The stop itself can also no-op while the scanner is
  // paused on an accepted report — the gate covers that hole too, since the
  // paused scanner only resumes through the callback we just muted.
  eggSleeping = true;
  if (eggSetupDone) {
    Bluefruit.Scanner.stop();
  }
  eggScannerRunning = false;

  // Plan 0018: the GATT link goes down with the radio. Cancel a pending
  // connect (it parks the scanner inside the SoftDevice) and drop a
  // live link — this covers BLE transfer mode (BLE_SETUP calls this
  // first) and shutdown (bleShutdownQuiesce()'s bounded settle runs
  // after us and gives the disconnect airtime; its own disconnect only
  // handles the peripheral handle).
  if (eggLinkState == EGG_LINK_CONNECTING) {
    sd_ble_gap_connect_cancel();
  }
  if (eggConnHandle != BLE_CONN_HANDLE_INVALID) {
    Bluefruit.disconnect(eggConnHandle);
  }
  eggLinkState = EGG_LINK_IDLE;
}

void SENSOREGG_WAKE() {
  // Charging-loop soft resume: just clear the sleep gate. The scanner is
  // race-gated (plan 0012) and a charging resume lands on the menu, so
  // there is nothing to start here — SENSOREGG_LOOP()'s reconcile brings
  // the scan up if a race session begins (the config survives a stop).
  eggSleeping = false;
}

void SENSOREGG_LOOP() {
  // Drain everything queued (usually 0 or 1 slots); the newest parse wins.
  while (eggReady[eggReadIdx]) {
    uint8_t local[sensoregg_protocol::kPayloadLenMax];
    uint8_t localPeer[6];
    memcpy(local, eggBuf[eggReadIdx], sizeof(local));
    memcpy(localPeer, eggPeerMac[eggReadIdx], sizeof(localPeer));
    const uint8_t localLen = eggLen[eggReadIdx];
    const uint32_t atMs = eggAtMs[eggReadIdx];
    eggReady[eggReadIdx] = false;
    eggReadIdx ^= 1;

    sensoregg_protocol::Reading r;
    if (sensoregg_protocol::parsePayload(local, localLen, r)) {
      eggReading = r;
      eggRxMs = atMs;
      eggHaveReading = true;
      sensoregg_protocol::seqMonitorFeed(eggSeqMon, r.sequence, atMs);
      eggRateCount++;

      // Pairing capture (plan 0017): first frame whose egg advertises
      // its own pairing-window flag wins. PERSIST FIRST (camera
      // precedent, camera_ble.ino) — a failed SD write leaves the
      // window open so the next frame retries. The RAM filter is
      // updated while eggPairingActive still bypasses it in the
      // callback, so a torn filter read can never reject the captured
      // egg; the flag clears last.
      if (eggPairingActive && r.pairingActive) {
        uint8_t human[6];
        char macStr[sensoregg_protocol::kMacStrLen];
        sensoregg_protocol::macReverse(localPeer, human);
        sensoregg_protocol::formatMac(human, macStr);
        if (setSetting("sensoregg_mac", macStr)) {
          memcpy(eggMacFilter, human, sizeof(eggMacFilter));
          eggPairingActive = false;
          // A NEWLY captured egg supersedes any live GATT link to the
          // old one (plan 0018) — the reconcile then courts the new MAC.
          if (eggConnHandle != BLE_CONN_HANDLE_INVALID) {
            Bluefruit.disconnect(eggConnHandle);
          }
          debugln(F("SensorEgg: paired"));
        } else {
          debugln(F("SensorEgg: pair capture — settings write failed, retrying"));
        }
      }
    }
  }

  // Pairing window timeout (plan 0017): give up after kPairingTimeoutMs
  // without a capture, so a walked-away-from pairing screen doesn't hold
  // the scanner on forever.
  if (eggPairingActive &&
      (uint32_t)(millis() - eggPairStartMs) >=
          sensoregg_protocol::kPairingTimeoutMs) {
    eggPairingActive = false;
    debugln(F("SensorEgg: pairing timed out"));
  }

  // Packet-rate meter for the test page: 1 s tumbling window, so the
  // figure decays to 0 within a second of the egg going quiet.
  {
    const uint32_t now = millis();
    if (eggRateWindowMs == 0) {
      eggRateWindowMs = now;
    } else if ((uint32_t)(now - eggRateWindowMs) >= 1000) {
      eggRateHz =
          (float)eggRateCount * 1000.0f / (float)(now - eggRateWindowMs);
      eggRateCount = 0;
      eggRateWindowMs = now;
    }
  }

  // ---- GATT link (plan 0018): commit, drain, reconcile ----

  // Bring-up failed in the callback task -> cool off.
  if (eggBringupFailed) {
    eggBringupFailed = false;
    eggLinkState = EGG_LINK_BACKOFF;
    eggLinkEventMs = millis();
    debugln(F("SensorEgg: GATT bring-up failed"));
  }

  // Bring-up staged -> parse the descriptor + clock, anchor the fit,
  // start consuming. The staging buffers are quiet once the ready flag
  // is up (the callback task's sequence has finished).
  if (eggBringupReady) {
    eggBringupReady = false;
    sensoregg_gatt::PodDescriptor pd;
    uint8_t clkBoot = 0;
    uint32_t clkPod = 0;
    if (sensoregg_gatt::parseDescriptor(eggDescRaw, eggDescLen, pd) &&
        sensoregg_gatt::parseClock(eggClkRaw, sizeof(eggClkRaw), clkBoot,
                                   clkPod)) {
      eggPod = pd;
      sensoregg_gatt::mapChannels(eggPod, eggRoleIdx);
      eggFastestIdx = sensoregg_gatt::fastestChannel(eggPod);
      sensoregg_gatt::clockFitAnchor(eggClockFit, clkBoot, clkPod,
                                     eggClkReqMs, eggClkRspMs);
      eggLinkState = EGG_LINK_STREAMING;
      debug(F("SensorEgg: GATT link up — fw "));
      debug(eggPod.fwMajor);
      debug(F("."));
      debug(eggPod.fwMinor);
      debug(F(", channels "));
      debugln(eggPod.channelCount);
    } else {
      debugln(F("SensorEgg: descriptor/clock parse failed — dropping link"));
      if (eggConnHandle != BLE_CONN_HANDLE_INVALID) {
        Bluefruit.disconnect(eggConnHandle);
      }
      eggLinkState = EGG_LINK_BACKOFF;
      eggLinkEventMs = millis();
    }
  }

  // Drain the notify frame ring: decode, epoch-check, route the latest
  // sample of each frame into the SAME surface the beacon feeds —
  // everything downstream is transport-agnostic.
  while (eggFrameReady[eggFrameR]) {
    uint8_t local[sizeof(eggFrameBuf[0])];
    const uint8_t localLen = eggFrameLen[eggFrameR];
    const uint32_t atMs = eggFrameAtMs[eggFrameR];
    memcpy(local, eggFrameBuf[eggFrameR], sizeof(local));
    eggFrameReady[eggFrameR] = false;
    eggFrameR = (uint8_t)((eggFrameR + 1) & 7);

    sensoregg_gatt::SampleFrame f;
    if (!sensoregg_gatt::parseSampleFrame(local, localLen, f)) continue;

    if (!sensoregg_gatt::clockFitSameEpoch(eggClockFit, f.bootId)) {
      // The pod's millis restarted under us (watchdog / brownout). An
      // egg reboot drops the physical link anyway — this is the mop-up
      // for frames racing the disconnect. Reconnect re-anchors.
      debugln(F("SensorEgg: pod rebooted (boot_id changed) — re-anchoring"));
      if (eggConnHandle != BLE_CONN_HANDLE_INVALID) {
        Bluefruit.disconnect(eggConnHandle);
      }
      continue;
    }

    int8_t chIdx = -1;
    for (uint8_t i = 0; i < eggPod.channelCount; i++) {
      if (eggPod.ch[i].id == f.channelId) {
        chIdx = (int8_t)i;
        break;
      }
    }
    if (chIdx < 0) continue;

    const int16_t rawLast = f.raw[f.n - 1];
    const float real = sensoregg_gatt::sampleToReal(rawLast, eggPod.ch[chIdx]);
    if (chIdx == eggRoleIdx[sensoregg_gatt::ROLE_EGT]) {
      eggReading.egtC = real;
    } else if (chIdx == eggRoleIdx[sensoregg_gatt::ROLE_CJ]) {
      eggReading.junctionC = real;
    } else if (chIdx == eggRoleIdx[sensoregg_gatt::ROLE_AUX]) {
      eggReading.auxC = real;
    } else if (chIdx == eggRoleIdx[sensoregg_gatt::ROLE_BATT]) {
      // Ratio channel is whole percent (scale 1.0) — route the raw,
      // sentinel/out-of-range -> the surface's 0xFF "unknown".
      eggReading.battery = (rawLast < 0 || rawLast > 100)
                               ? (uint8_t)0xFF
                               : (uint8_t)rawLast;
    }
    eggReading.sequence = f.seq;  // display: the stream's own counter
    // Zombie detection feeds on the FASTEST channel's frame seq only —
    // per-channel u8 counters are independent, and mixing them would
    // alias as "frozen". seqMonitorFeed only tests inequality, so the
    // u8 -> u16 widening is harmless.
    if (chIdx == eggFastestIdx) {
      sensoregg_protocol::seqMonitorFeed(eggSeqMon, f.seq, atMs);
    }
    eggRxMs = atMs;
    eggHaveReading = true;
    eggRateCount++;  // the test page's Hz meter counts GATT frames too
  }

  // Link reconcile: post/withdraw the wanted state, time out a stuck
  // connect, expire the backoff. The scan callback does the actual
  // connecting (it holds the fresh adv report).
  {
    const uint32_t now = millis();
    switch (eggLinkState) {
      case EGG_LINK_IDLE:
        if (eggLinkWanted()) {
          eggLinkState = EGG_LINK_WAIT_ADV;
        }
        break;
      case EGG_LINK_WAIT_ADV:
        if (!eggLinkWanted()) {
          eggLinkState = EGG_LINK_IDLE;
        }
        break;
      case EGG_LINK_BACKOFF:
        if ((uint32_t)(now - eggLinkEventMs) >= kEggLinkBackoffMs) {
          eggLinkState = eggLinkWanted() ? EGG_LINK_WAIT_ADV : EGG_LINK_IDLE;
        }
        break;
      case EGG_LINK_CONNECTING:
        if ((uint32_t)(now - eggLinkEventMs) >= kEggConnectTimeoutMs) {
          sd_ble_gap_connect_cancel();
          eggLinkState = EGG_LINK_BACKOFF;
          eggLinkEventMs = now;
          debugln(F("SensorEgg: connect timed out"));
        }
        break;
      case EGG_LINK_BRINGUP:
      case EGG_LINK_STREAMING:
        // Gate dropped (race over, unpair, sleep already handled) —
        // release the link; the disconnect callback moves us to BACKOFF.
        if (!eggLinkWanted() && eggConnHandle != BLE_CONN_HANDLE_INVALID) {
          Bluefruit.disconnect(eggConnHandle);
        }
        break;
      default:
        break;
    }
  }

  // Race-gate reconcile (plan 0012): the scanner runs only while a race
  // session is active. Runs every iteration so the scan comes up within
  // one loop of race entry and goes down within one loop of the session
  // ending. A start() the SoftDevice refuses (boot race, radio busy)
  // leaves eggScannerRunning false and is retried on the 1 s throttle —
  // this reconcile is now the start-retry path the self-heal below used
  // to be. eggLastKickMs is stamped on every start so a fresh scan gets a
  // full self-heal interval before its first kick.
  if (eggSetupDone) {
    const uint32_t now = millis();
    if (eggScanWanted() && !eggScannerRunning) {
      if (eggLastStartTryMs == 0 ||
          (uint32_t)(now - eggLastStartTryMs) >= kEggStartRetryMs) {
        eggLastStartTryMs = now;
        eggLastKickMs = now;
        eggScannerRunning = Bluefruit.Scanner.start(0);  // 0 = forever
        debugln(eggScannerRunning ? F("SensorEgg: scanner started (race)")
                                  : F("SensorEgg: scanner start refused"));
      }
    } else if (!eggScanWanted() && eggScannerRunning) {
      Bluefruit.Scanner.stop();
      eggScannerRunning = false;
      eggLastStartTryMs = 0;
      debugln(F("SensorEgg: scanner stopped (race over)"));
    }
  }

  // Scanner self-heal. The rx path only resumes scanning when our
  // deferred callback actually runs; a dropped callback would halt the
  // scanner silently and forever. If nothing has been accepted for
  // kScannerSelfHealMs, kick stop+start — harmless when the egg is just
  // off, curative when the scanner wedged. Throttled by its own stamp so
  // an absent egg costs one kick per interval, not one per loop. Gated on
  // eggScannerRunning: with the scanner race-gated, "not running" outside
  // a race is the intended state, and a refused race-time start is retried
  // by the reconcile above — this kick only cures a wedge while running.
  if (eggSetupDone && eggScannerRunning && eggScanWanted()) {
    const uint32_t now = millis();
    const uint32_t lastAlive = eggHaveReading ? eggRxMs : 0;
    if ((uint32_t)(now - lastAlive) >= sensoregg_protocol::kScannerSelfHealMs &&
        (uint32_t)(now - eggLastKickMs) >= sensoregg_protocol::kScannerSelfHealMs) {
      eggLastKickMs = now;
      Bluefruit.Scanner.stop();
      eggScannerRunning = Bluefruit.Scanner.start(0);
    }
  }
}

///////////////////////////////////////////
// DATA SURFACE
///////////////////////////////////////////

bool sensoreggLinkUp() {
  return eggHaveReading && sensoregg_protocol::isFresh(eggRxMs, millis());
}

float sensoreggEgtC() {
  // Stale link -> NaN (never hold a value across a dropout); a fresh
  // link still yields NaN when the egg itself sent the invalid sentinel
  // OR when the egg's app has hung (radio beaconing a frozen payload —
  // a flat line must never masquerade as data).
  if (!sensoreggLinkUp() || sensoreggAppHung()) return NAN;
  return eggReading.egtC;
}

float sensoreggJunctionC() {
  if (!sensoreggLinkUp() || sensoreggAppHung()) return NAN;
  return eggReading.junctionC;
}

float sensoreggAuxC() {
  // Same gating as the EGT: stale link or hung app -> NaN. Also NaN when
  // the egg is v1 (no aux field) or its divider reported the sentinel.
  if (!sensoreggLinkUp() || sensoreggAppHung()) return NAN;
  return eggReading.auxC;
}

uint8_t sensoreggBatteryPct() {
  // 0xFF = unknown: stale/hung link, v1 stub, or the egg's own
  // no-pack-fitted gate. Never report a stale percent as current.
  if (!sensoreggLinkUp() || sensoreggAppHung()) return 0xFF;
  return eggReading.battery;
}

bool sensoreggTcFault() {
  // A frozen payload's fault flag is stale information — suppress it.
  return sensoreggLinkUp() && !sensoreggAppHung() && eggReading.tcFault;
}

bool sensoreggAppHung() {
  // Radio alive (fresh packets) but the application isn't producing new
  // readings (sequence frozen). The egg needs a power cycle.
  return sensoreggLinkUp() &&
         !sensoregg_protocol::seqMonitorLive(eggSeqMon, millis());
}

uint16_t sensoreggSequence() {
  return eggReading.sequence;
}

///////////////////////////////////////////
// PAIRING / BENCH SURFACE (plan 0017)
///////////////////////////////////////////

bool sensoreggIsPaired() {
  return !sensoregg_protocol::macIsWildcard(eggMacFilter);
}

void sensoreggRequestPair() {
  // Idempotent; a re-request restarts the timeout clock.
  eggPairStartMs = millis();
  eggPairingActive = true;
}

void sensoreggCancelPair() {
  eggPairingActive = false;
}

bool sensoreggPairingInProgress() {
  return eggPairingActive;
}

bool sensoreggUnpair() {
  // PERSIST FIRST: refuse the RAM change when the SD write fails so the
  // UI warns instead of silently diverging from disk (camera precedent).
  if (!setSetting("sensoregg_mac", "")) {
    return false;
  }
  memset(eggMacFilter, 0, sizeof(eggMacFilter));  // -> accept-any
  return true;
}

bool sensoreggPairedMac(char* buf, size_t bufSize) {
  if (buf == nullptr || bufSize < sensoregg_protocol::kMacStrLen) {
    return false;
  }
  if (!sensoreggIsPaired()) {
    buf[0] = '\0';
    return false;
  }
  sensoregg_protocol::formatMac(eggMacFilter, buf);
  return true;
}

void sensoreggTestEnterMode() {
  eggTestActive = true;
}

void sensoreggTestExitMode() {
  eggTestActive = false;
}

uint8_t sensoreggProtoVersion() {
  // 0 = stale or never heard — the test page renders "v-".
  if (!sensoreggLinkUp()) return 0;
  return eggReading.protoVersion;
}

bool sensoreggPairingFlag() {
  // The egg's own 30 s window bit, gated like tcFault so a frozen
  // payload's flag can't show as a live window.
  return sensoreggLinkUp() && !sensoreggAppHung() && eggReading.pairingActive;
}

float sensoreggPacketHz() {
  return eggRateHz;
}

uint8_t sensoreggLinkMode() {
  // 2 = GATT stream live; 1 = fresh beacon data; 0 = nothing heard.
  if (eggLinkState == EGG_LINK_STREAMING) return 2;
  return sensoreggLinkUp() ? 1 : 0;
}

uint16_t sensoreggGattMtu() {
  if (eggConnHandle == BLE_CONN_HANDLE_INVALID) return 0;
  BLEConnection* conn = Bluefruit.Connection(eggConnHandle);
  return conn ? conn->getMtu() : 0;
}

#else  // !BIRDSEYE_ENABLE_SENSOREGG

///////////////////////////////////////////
// POC COMPILED OUT (the master/release default — see project.h)
//
// No passive scanner, and nothing here calls bleCoreEnsureInit(), so BLE
// returns to coming up lazily on the first camera/transfer use instead of
// at boot. The accessors keep their contract and simply report "no egg
// ever seen": the Temp1/Junction1 DOVEX columns still get written as
// `nan`, so a log from a SensorEgg build and one from a stock build have
// identical shape. The Temp1 race page is compiled out separately
// (display_pages.ino) rather than left in the rotation showing '---'.
///////////////////////////////////////////

void SENSOREGG_SETUP() {}
void SENSOREGG_LOOP() {}
void SENSOREGG_SLEEP() {}
void SENSOREGG_WAKE() {}

bool sensoreggLinkUp() { return false; }
bool sensoreggAppHung() { return false; }
float sensoreggEgtC() { return NAN; }
float sensoreggJunctionC() { return NAN; }
float sensoreggAuxC() { return NAN; }
uint8_t sensoreggBatteryPct() { return 0xFF; }
bool sensoreggTcFault() { return false; }
uint16_t sensoreggSequence() { return 0; }

bool sensoreggIsPaired() { return false; }
void sensoreggRequestPair() {}
void sensoreggCancelPair() {}
bool sensoreggPairingInProgress() { return false; }
// True: the post-condition "not paired" already holds (removeSetting's
// contract philosophy).
bool sensoreggUnpair() { return true; }
bool sensoreggPairedMac(char* buf, size_t bufSize) {
  if (buf != nullptr && bufSize > 0) buf[0] = '\0';
  return false;
}
void sensoreggTestEnterMode() {}
void sensoreggTestExitMode() {}
uint8_t sensoreggProtoVersion() { return 0; }
bool sensoreggPairingFlag() { return false; }
float sensoreggPacketHz() { return 0.0f; }

void sensoreggGattClientInit() {}
uint8_t sensoreggLinkMode() { return 0; }
uint16_t sensoreggGattMtu() { return 0; }

#endif  // BIRDSEYE_ENABLE_SENSOREGG
