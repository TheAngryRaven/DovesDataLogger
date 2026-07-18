#pragma once

#include <stdint.h>

///////////////////////////////////////////
// SENSOREGG MODULE (wireless EGT pod — passive BLE observer)
//
// Receives the DovesSensorEgg's PW-ADV-1 advertising broadcasts (see
// sensoregg_protocol.h for the byte layout) and exposes the latest EGT /
// cold-junction reading to the logger and display. POC scope: one egg,
// one temperature channel ("Temp1") plus its cold junction ("Junction1").
//
// RADIO ROLE: pure OBSERVER on the shared SoftDevice. Passive scanning
// only — we never transmit a SCAN_REQ, never connect, and hold no GATT
// link to the egg, so the scanner cannot contend with the Insta360 X4
// camera link (peripheral role) for TX airtime. S140 natively time-slices
// scan windows around existing connection events. The egg is a pure
// broadcaster and accepts no connections. Do not "improve" this into a
// connection — the camera link wins every tradeoff.
//
// PAIRING (POC): hardcoded MAC via SENSOREGG_MAC below. All-zeros (the
// default) = accept any advertiser whose payload matches the PW magic —
// fine while exactly one egg exists.
//
// THREADING (mirrors camera_ble.ino): the Bluefruit scan callback runs in
// BLE task context and only filters, copies bytes into a RAM double
// buffer, and calls Scanner.resume() (mandatory — without it the scanner
// halts after the first report). SENSOREGG_LOOP() on the main loop drains
// the buffer and parses via the host-tested sensoregg_protocol unit. No
// Serial/SD/display work ever happens in the callback.
//
// STALENESS: a reading older than sensoregg_protocol::kStalenessMs (1 s)
// is dead — accessors return NaN/false so the display shows '---' and the
// log writes nan. NEVER hold the last value across a dropout: a held
// value draws a flat line indistinguishable from real data.
///////////////////////////////////////////

// Hardcoded egg MAC, in the human-readable order the egg prints at boot
// (AA:BB:CC:DD:EE:FF -> {0xAA,0xBB,...}). All-zeros = magic-match any
// PW-ADV-1 broadcaster. (nRF ble_gap_addr_t stores bytes LSB-first; the
// match helper handles the reversal — keep this define human-ordered.)
#define SENSOREGG_MAC {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

// ---- lifecycle (called from BirdsEye.ino) ----

// Bring up the BLE core (idempotent) and start the passive scanner.
// Call from setup() after CAMERA_SETUP(). NOTE: this makes the SoftDevice
// come up at boot — previously BLE was lazy (first camera/transfer use).
void SENSOREGG_SETUP();

// Drain the scan double-buffer and parse the newest payload. Call from
// the normal main-loop path. Not called in the bleActive / usbMscActive
// parking branches — a stale reading correctly goes NaN there.
void SENSOREGG_LOOP();

// ---- data surface (display_pages.ino / gps_functions.ino) ----

// True while a reading is fresh (received < 1 s ago).
bool sensoreggLinkUp();

// Latest EGT / cold junction in degC. NaN when the link is stale OR the
// egg reported the invalid sentinel (open probe, sensor fault).
float sensoreggEgtC();
float sensoreggJunctionC();

// True while fresh AND the egg flags a thermocouple fault (open /
// out-of-range probe, MCP9600 STATUS input-range bit).
bool sensoreggTcFault();

// Free-running egg sequence counter from the latest payload (debug).
uint16_t sensoreggSequence();
