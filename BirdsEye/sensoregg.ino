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

// Forward declaration: the callback signature mentions a SoftDevice type,
// and Arduino's auto-prototype generator inserts prototypes BEFORE
// <bluefruit.h> is included — an explicit prototype makes arduino-cli
// skip generating one (same workaround as camera_ble.ino).
static void sensoreggScanCallback(ble_gap_evt_adv_report_t* report);

///////////////////////////////////////////
// MODULE STATE
///////////////////////////////////////////

// Hardcoded egg MAC (human order, see sensoregg.h). All-zeros = accept
// any PW-ADV-1 broadcaster (POC: exactly one egg exists).
static const uint8_t kSensorEggMac[6] = SENSOREGG_MAC;

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
static uint32_t eggLastKickMs = 0;  // last scanner self-heal kick (main loop)

///////////////////////////////////////////
// SCAN CALLBACK (Bluefruit task context — copy bytes, resume, return)
///////////////////////////////////////////

// True when the reporting advertiser is our egg. nRF ble_gap_addr_t
// stores the address LSB-first, the #define is human-ordered (MSB
// first) — compare reversed. All-zeros define = accept anyone (the
// payload magic already filtered).
static bool sensoreggMacAccepted(const uint8_t* peerAddrLsbFirst) {
  bool filterActive = false;
  for (int i = 0; i < 6; i++) {
    if (kSensorEggMac[i] != 0x00) {
      filterActive = true;
      break;
    }
  }
  if (!filterActive) return true;
  for (int i = 0; i < 6; i++) {
    if (peerAddrLsbFirst[i] != kSensorEggMac[5 - i]) return false;
  }
  return true;
}

static void sensoreggScanCallback(ble_gap_evt_adv_report_t* report) {
  // NO Serial, NO SD, NO display, NO delay() here — BLE task context.
  uint8_t buf[BLE_GAP_ADV_SET_DATA_SIZE_MAX];
  uint8_t len = Bluefruit.Scanner.parseReportByType(
      report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buf, sizeof(buf));

  if (len >= sensoregg_protocol::kPayloadLen &&
      sensoregg_protocol::matchesMagic(buf, len) &&
      sensoreggMacAccepted(report->peer_addr.addr)) {
    const uint8_t w = eggWriteIdx;
    // Capture up to the largest known layout; the parser applies the
    // per-version length gate. (The old fixed-14 copy silently truncated
    // v2 frames — bytes 14-15 never reached the parser.)
    const uint8_t copyLen =
        len < sensoregg_protocol::kPayloadLenMax
            ? len
            : (uint8_t)sensoregg_protocol::kPayloadLenMax;
    memcpy(eggBuf[w], buf, copyLen);
    eggLen[w] = copyLen;
    eggAtMs[w] = millis();
    eggReady[w] = true;
    eggWriteIdx = w ^ 1;
  }

  // MANDATORY: without resume() the scanner halts after the first report
  // — symptom is exactly one reading then permanent silence,
  // indistinguishable from a dead egg.
  Bluefruit.Scanner.resume();
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
  eggScannerRunning = Bluefruit.Scanner.start(0);  // 0 = forever

  if (eggScannerRunning) {
    debugln(F("SensorEgg: passive scanner started"));
  } else {
    // Known unblock if the installed core refuses an observer without a
    // central slot: Bluefruit.begin(1, 1) in bleCoreEnsureInit() (costs
    // one unused central connection slot). Not taken by default — do not
    // change the shared begin() without re-soaking the camera link.
    debugln(F("SensorEgg: scanner FAILED to start (see begin(1,1) note)"));
  }
}

void SENSOREGG_LOOP() {
  // Drain everything queued (usually 0 or 1 slots); the newest parse wins.
  while (eggReady[eggReadIdx]) {
    uint8_t local[sensoregg_protocol::kPayloadLenMax];
    memcpy(local, eggBuf[eggReadIdx], sizeof(local));
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
    }
  }

  // Scanner self-heal. The rx path only resumes scanning when our
  // deferred callback actually runs; a dropped callback would halt the
  // scanner silently and forever. If nothing has been accepted for
  // kScannerSelfHealMs, kick stop+start — harmless when the egg is just
  // off, curative when the scanner wedged. Throttled by its own stamp so
  // an absent egg costs one kick per interval, not one per loop.
  if (eggScannerRunning) {
    const uint32_t now = millis();
    const uint32_t lastAlive = eggHaveReading ? eggRxMs : 0;
    if ((uint32_t)(now - lastAlive) >= sensoregg_protocol::kScannerSelfHealMs &&
        (uint32_t)(now - eggLastKickMs) >= sensoregg_protocol::kScannerSelfHealMs) {
      eggLastKickMs = now;
      Bluefruit.Scanner.stop();
      Bluefruit.Scanner.start(0);
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

bool sensoreggLinkUp() { return false; }
bool sensoreggAppHung() { return false; }
float sensoreggEgtC() { return NAN; }
float sensoreggJunctionC() { return NAN; }
float sensoreggAuxC() { return NAN; }
uint8_t sensoreggBatteryPct() { return 0xFF; }
bool sensoreggTcFault() { return false; }
uint16_t sensoreggSequence() { return 0; }

#endif  // BIRDSEYE_ENABLE_SENSOREGG
