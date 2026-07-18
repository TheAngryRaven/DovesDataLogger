///////////////////////////////////////////
// SENSOREGG MODULE (wireless EGT pod — passive BLE observer)
//
// Receives the DovesSensorEgg's PW-ADV-1 broadcasts and exposes the
// latest EGT / cold-junction reading. See sensoregg.h for the role,
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

#include <string.h>

#include "project.h"
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
static uint8_t           eggBuf[2][sensoregg_protocol::kPayloadLen];
static volatile uint32_t eggAtMs[2] = {0, 0};
static volatile bool     eggReady[2] = {false, false};
static volatile uint8_t  eggWriteIdx = 0;  // callback writes (Bluefruit task)
static uint8_t           eggReadIdx = 0;   // main loop reads

// Latest parsed reading (main-loop context only).
static sensoregg_protocol::Reading eggReading;
static uint32_t eggRxMs = 0;
static bool     eggHaveReading = false;

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
    memcpy(eggBuf[w], buf, sensoregg_protocol::kPayloadLen);
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

  // Passive observer: 100 ms interval / 60 ms window (see the constants'
  // comments for why the window is wider than the spec's 40 ms). Passive
  // listening costs the camera link nothing — S140 yields scan windows to
  // connection events automatically.
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
    uint8_t local[sensoregg_protocol::kPayloadLen];
    memcpy(local, eggBuf[eggReadIdx], sizeof(local));
    const uint32_t atMs = eggAtMs[eggReadIdx];
    eggReady[eggReadIdx] = false;
    eggReadIdx ^= 1;

    sensoregg_protocol::Reading r;
    if (sensoregg_protocol::parsePayload(local, sizeof(local), r)) {
      eggReading = r;
      eggRxMs = atMs;
      eggHaveReading = true;
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
  // link still yields NaN when the egg itself sent the invalid sentinel.
  if (!sensoreggLinkUp()) return NAN;
  return eggReading.egtC;
}

float sensoreggJunctionC() {
  if (!sensoreggLinkUp()) return NAN;
  return eggReading.junctionC;
}

bool sensoreggTcFault() {
  return sensoreggLinkUp() && eggReading.tcFault;
}

uint16_t sensoreggSequence() {
  return eggReading.sequence;
}
