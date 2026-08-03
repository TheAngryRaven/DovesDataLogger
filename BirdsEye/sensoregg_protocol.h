#pragma once

///////////////////////////////////////////
// SENSOREGG PW-ADV PROTOCOL (v1 + v2)
// Parser for the DovesSensorEgg wireless thermocouple pod's BLE
// advertising payload. The egg is a pure BROADCASTER: it packs its
// readings into a Manufacturer Specific Data AD structure and
// advertises at ~10 Hz; the logger is a passive OBSERVER that never
// connects. This unit owns the byte layout and the staleness/validity
// rules so they are host-testable; the BLE plumbing (Bluefruit.Scanner)
// stays in sensoregg.ino.
//
// v2 (2026-07-27) appends to v1 — bytes 0-13 keep their exact v1
// offsets, so both versions share one decode path. Layout (little-
// endian fields). NOTE: Bluefruit's addManufacturerData() /
// parseReportByType() pass the buffer through RAW — the company ID is
// INSIDE the array, not prepended:
//   0-1   company ID 0xFF 0xFF (SIG test/internal ID)
//   2-3   magic 'P' 'W' (0x50 0x57) — disambiguates other 0xFFFF users
//   4     protocol version (0x01 = 14-byte v1, 0x02 = 16-byte v2)
//   5     flags: bit0 = pairing window active, bit1 = thermocouple fault
//   6-7   EGT, int16 deci-degC (6500 = 650.0 C); 0x8000 = invalid
//   8-9   cold junction, int16 deci-degC; 0x8000 = invalid
//   10    raw MCP9600 STATUS register (0x04)
//   11    battery percent 0-100 (real on v2 eggs; 0xFF = unknown/stub)
//   12-13 free-running sequence counter (wraps)
//   14-15 [v2] aux thermistor (intake air), int16 deci-degC; 0x8000 =
//         invalid. Absent on v1 — parses as NaN.
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

namespace sensoregg_protocol {

// Per-version payload lengths. Shorter-than-the-version's reports are
// rejected; longer ones are accepted (forward compatibility — a future
// egg may append more fields). kPayloadLen doubles as the minimum
// acceptable length for the cheap scan-callback gate; kPayloadLenMax
// sizes RX buffers.
constexpr size_t kPayloadLen = 14;     // v1 (and the common prefix)
constexpr size_t kPayloadLenV2 = 16;   // v2 = v1 + aux thermistor
constexpr size_t kPayloadLenMax = kPayloadLenV2;

// Company ID + magic, exactly as they appear at the start of the payload.
constexpr uint8_t kMagic[4] = {0xFF, 0xFF, 0x50, 0x57};

// Company ID as a value, for Bluefruit's INLINE manufacturer-data filter
// (Scanner.filterMSD). Packets failing an inline filter are rejected and
// resumed inside Bluefruit's event handler; without it, EVERY ambient
// packet above the RSSI floor pauses scanning for a deferred-callback
// round trip through our rx callback, collapsing the scan duty cycle in
// bursts on a BLE-busy bench.
constexpr uint16_t kCompanyId = 0xFFFF;

// Byte 4 — the protocol versions this parser understands. v1 eggs are
// still accepted (their aux temperature parses as NaN), so a fleet can
// mix firmware ages without the logger going blind to either.
constexpr uint8_t kProtocolVersion = 0x01;    // original 14-byte layout
constexpr uint8_t kProtocolVersionV2 = 0x02;  // + aux thermistor

// int16 sentinel for "no valid reading" (the egg emits this instead of
// casting NaN/out-of-range floats to int16, which is UB).
constexpr int16_t kInvalidSentinel = INT16_MIN;

// A reading older than this is stale: consumers must show '---' and log
// NaN rather than hold the last value (a held value draws a flat line
// indistinguishable from real data).
constexpr uint32_t kStalenessMs = 1000;

// Logger scan parameters (0.625 ms units): 90 ms interval / 40 ms window
// (~44% duty), passive-only so the observer never transmits and cannot
// contend with the Insta360 camera link for TX airtime (S140 yields scan
// windows to connection events automatically).
//
// Anti-phase-lock: a scan interval equal to the egg's ~100 ms adv
// interval phase-locks, and with a 40 ms live window the egg could park
// in the deaf zone for seconds (bench-observed flapping). The original
// fix widened the window to 60 ms (60% duty); that radio duty turned
// out to defer the TIMER3 GPS drain enough to drop 25 Hz PVT frames
// (SoftDevice scan-window ISRs outrank every app interrupt). The
// off-100 ms *interval* now carries the anti-lock property instead:
// 90 ms scan vs ~100 ms adv sweeps their relative phase ~10 ms per
// cycle, so a deaf-zone park escapes within ≤5 cycles (~450 ms) —
// comfortably inside kStalenessMs at the egg's ~10 Hz broadcast — and
// the window returns to the spec's 40 ms. (The egg firmware also moves
// its adv interval off 100 ms, sweeping the phases from both ends.)
// Fallback lever if the bench ever shows flapping again: interval 176
// (110 ms) with window 64 still gives ≤36% duty.
constexpr uint16_t kScanIntervalUnits = 144;
constexpr uint16_t kScanWindowUnits = 64;
constexpr int8_t kRssiFloorDbm = -90;

// Scanner self-heal: if no egg packet has been accepted for this long,
// SENSOREGG_LOOP kicks the scanner (stop + start). A lost deferred rx
// callback would otherwise leave the scanner halted forever with no
// error - indistinguishable from a dead egg. Kicking a healthy scanner
// is harmless (a few ms gap), so this also self-limits false positives
// while the egg is simply powered off.
constexpr uint32_t kScannerSelfHealMs = 30000;

struct Reading {
  float egtC = 0.0f;        // NaN when the egg sent the invalid sentinel
  float junctionC = 0.0f;   // NaN when the egg sent the invalid sentinel
  float auxC = 0.0f;        // v2 aux thermistor (intake air); NaN on v1
                            // eggs and on the invalid sentinel
  uint8_t flags = 0;
  bool pairingActive = false;  // flags bit0
  bool tcFault = false;        // flags bit1 (MCP9600 STATUS input-range)
  uint8_t status = 0;          // raw MCP9600 STATUS register
  uint8_t battery = 0;         // percent 0-100 on v2 eggs; 0xFF unknown
  uint16_t sequence = 0;
  uint8_t protoVersion = 0;    // byte 4 of the accepted payload
};

// True when data starts with the 4-byte company-ID+magic prefix (cheap
// filter suitable for the BLE-task scan callback).
bool matchesMagic(const uint8_t* data, size_t len);

// Parse a manufacturer-data payload into `out`. Returns false (out
// untouched) when data is null/short, the magic doesn't match, or the
// protocol version is unknown. Sentinel temperature fields become NaN.
bool parsePayload(const uint8_t* data, size_t len, Reading& out);

// Staleness rule: true while a reading received at `receivedAtMs` is
// still fresh at `nowMs` (age < kStalenessMs, wrap-safe).
bool isFresh(uint32_t receivedAtMs, uint32_t nowMs);

// Display-unit conversion. The on-device display shows Fahrenheit (a C/F
// display setting comes later); DOVEX logging stays Celsius — convert at
// render time only. NaN propagates.
float celsiusToFahrenheit(float c);

// ---- Zombie-egg detection (frozen sequence counter) ---------------------
// BLE radios rebroadcast the last-set advertising buffer autonomously: an
// egg whose application has hung (e.g. a blocking MCP9600 I2C read locked
// up by ignition EMI) keeps beaconing its final payload at ~10 Hz forever.
// Arrival-time freshness alone then reports a live link with a frozen
// value — indistinguishable from real data (2026-07-19 field incident,
// ~3–4 h in). The payload's free-running sequence counter is the tell: a
// live app advances it every update, a hung one never does. Feed every
// accepted packet; the reading is only live while the sequence has changed
// within kStalenessMs. uint16 wrap is naturally handled (any change counts).
struct SeqMonitor {
  uint16_t lastSeq = 0;
  uint32_t lastChangeMs = 0;
  bool haveSeq = false;
};

// Record an accepted packet's sequence at nowMs (stamps a change on the
// first packet and on every packet whose sequence differs from the last).
void seqMonitorFeed(SeqMonitor& m, uint16_t seq, uint32_t nowMs);

// True while the egg's APPLICATION is demonstrably alive: a sequence
// change has been seen within kStalenessMs (wrap-safe timing).
bool seqMonitorLive(const SeqMonitor& m, uint32_t nowMs);

}  // namespace sensoregg_protocol
