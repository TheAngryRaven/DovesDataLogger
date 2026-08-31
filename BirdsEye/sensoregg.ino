///////////////////////////////////////////
// SENSOREGG MODULE (wireless EGT pod — passive BLE observer)
//
// Receives the DovesSensorEgg's PW-ADV broadcasts (v1 and v2) and
// exposes the latest readings. See sensoregg.h for the role,
// pairing, and threading contracts, and sensoregg_protocol.{h,cpp}
// (host-tested) for the byte layout and staleness rule.
//
// The scanner is a pure passive OBSERVER: no SCAN_REQ, no connection, so
// it cannot contend with the Insta360 X4 camera link for TX airtime.
// S140 time-slices scan windows around existing connection events.
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
#include "sensoregg_protocol.h"
#include "settings.h"   // getSetting/setSetting — the sensoregg_mac key

// Forward declaration: the callback signature mentions a SoftDevice type,
// and Arduino's auto-prototype generator inserts prototypes BEFORE
// <bluefruit.h> is included — an explicit prototype makes arduino-cli
// skip generating one (same workaround as camera_ble.ino).
static void sensoreggScanCallback(ble_gap_evt_adv_report_t* report);

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
static bool eggScanWanted() {
  return (raceActive || eggPairingActive || eggTestActive) && !eggSleeping;
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

#endif  // BIRDSEYE_ENABLE_SENSOREGG
