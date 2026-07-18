#pragma once

///////////////////////////////////////////
// SENSOREGG PW-ADV-1 PROTOCOL
// Parser for the DovesSensorEgg wireless thermocouple pod's BLE
// advertising payload (spec PW-ADV-1). The egg is a pure BROADCASTER:
// it packs EGT + cold-junction temperatures into a 14-byte Manufacturer
// Specific Data AD structure and advertises at ~10 Hz; the logger is a
// passive OBSERVER that never connects. This unit owns the byte layout
// and the staleness/validity rules so they are host-testable; the BLE
// plumbing (Bluefruit.Scanner) stays in sensoregg.ino.
//
// Payload layout (bytes, all multi-byte fields little-endian). NOTE:
// Bluefruit's addManufacturerData()/parseReportByType() pass the buffer
// through RAW — the company ID is INSIDE the array, not prepended:
//   0-1   company ID 0xFF 0xFF (SIG test/internal ID)
//   2-3   magic 'P' 'W' (0x50 0x57) — disambiguates other 0xFFFF users
//   4     pod type / protocol version (0x01)
//   5     flags: bit0 = pairing window active, bit1 = thermocouple fault
//   6-7   EGT, int16 deci-degC (6500 = 650.0 C); 0x8000 = invalid
//   8-9   cold junction, int16 deci-degC; 0x8000 = invalid
//   10    raw MCP9600 STATUS register (0x04)
//   11    battery percent (stub, always 0xFF)
//   12-13 free-running sequence counter (wraps)
//
// Pure logic — no Arduino headers — so it is exercised by host tests.
///////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

namespace sensoregg_protocol {

// Full payload length. Shorter reports are rejected; longer ones are
// accepted (forward compatibility — a future egg may append fields).
constexpr size_t kPayloadLen = 14;

// Company ID + magic, exactly as they appear at the start of the payload.
constexpr uint8_t kMagic[4] = {0xFF, 0xFF, 0x50, 0x57};

// Company ID as a value, for Bluefruit's INLINE manufacturer-data filter
// (Scanner.filterMSD). Packets failing an inline filter are rejected and
// resumed inside Bluefruit's event handler; without it, EVERY ambient
// packet above the RSSI floor pauses scanning for a deferred-callback
// round trip through our rx callback, collapsing the scan duty cycle in
// bursts on a BLE-busy bench.
constexpr uint16_t kCompanyId = 0xFFFF;

// Byte 4 — the only protocol version this parser understands.
constexpr uint8_t kProtocolVersion = 0x01;

// int16 sentinel for "no valid reading" (the egg emits this instead of
// casting NaN/out-of-range floats to int16, which is UB).
constexpr int16_t kInvalidSentinel = INT16_MIN;

// A reading older than this is stale: consumers must show '---' and log
// NaN rather than hold the last value (a held value draws a flat line
// indistinguishable from real data).
constexpr uint32_t kStalenessMs = 1000;

// Logger scan parameters (0.625 ms units): 100 ms interval / 60 ms window
// (60% duty), passive-only so the observer never transmits and cannot
// contend with the Insta360 camera link for TX airtime (S140 yields scan
// windows to connection events automatically). The window was widened
// from the spec's 40 ms after bench flapping: a 100 ms adv interval
// against a 100 ms scan interval phase-locks, and with only a 40 ms live
// window the egg could park in the 60 ms deaf zone for seconds at a
// time. The egg's adv interval is also moved off 100 ms (see the egg
// firmware) so the phases sweep instead of locking.
constexpr uint16_t kScanIntervalUnits = 160;
constexpr uint16_t kScanWindowUnits = 96;
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
  uint8_t flags = 0;
  bool pairingActive = false;  // flags bit0
  bool tcFault = false;        // flags bit1 (MCP9600 STATUS input-range)
  uint8_t status = 0;          // raw MCP9600 STATUS register
  uint8_t battery = 0;         // stub, 0xFF on current eggs
  uint16_t sequence = 0;
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

}  // namespace sensoregg_protocol
