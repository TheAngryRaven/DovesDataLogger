#pragma once

#include <stdint.h>

///////////////////////////////////////////
// SENSOREGG MODULE (wireless EGT pod — passive BLE observer)
//
// BUILD FLAG: the whole POC is gated on BIRDSEYE_ENABLE_SENSOREGG
// (project.h) — 0 on master/release, 1 on the beta channel. When it is 0,
// sensoregg.ino compiles to no-op lifecycle calls and NaN accessors (so
// the DOVEX Temp1/Junction1 columns still log `nan`), the Temp1 race page
// is dropped from the rotation, and BLE goes back to lazy init. This
// header's contract below describes the enabled build.
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
// SCANNER ROBUSTNESS: Bluefruit pauses scanning from the moment a report
// is ACCEPTED until our deferred rx callback resumes it, so ambient
// packets must be rejected INLINE — Scanner.filterMSD(0xFFFF) does that
// (Bluefruit self-resumes filtered packets). And because a lost deferred
// callback would halt the scanner silently forever, SENSOREGG_LOOP kicks
// stop+start after kScannerSelfHealMs with no accepted packet.
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

// Stop the passive scanner. Called from enterShutdown() immediately before
// System OFF — after the VBUS/charging branch, so a charging-loop resume
// keeps scanning — so that no BLE role is still running when the chip is
// powered down (Nordic's guidance is to quiesce radio activity first).
// There is no resume counterpart: System OFF wake is a reset, and
// SENSOREGG_SETUP() runs again on the fresh boot.
void SENSOREGG_SLEEP();

// ---- data surface (display_pages.ino / gps_functions.ino) ----

// True while a reading is fresh (received < 1 s ago).
bool sensoreggLinkUp();

// True when packets are arriving but the egg's application has hung: the
// radio beacons the last payload autonomously, so the sequence counter is
// the only sign of life. Readings are NaN while hung; the display shows
// rf:HUNG (the egg needs a power cycle).
bool sensoreggAppHung();

// Latest EGT / cold junction in degC. NaN when the link is stale OR the
// egg reported the invalid sentinel (open probe, sensor fault).
float sensoreggEgtC();
float sensoreggJunctionC();

// True while fresh AND the egg flags a thermocouple fault (open /
// out-of-range probe, MCP9600 STATUS input-range bit).
bool sensoreggTcFault();

// Free-running egg sequence counter from the latest payload (debug).
uint16_t sensoreggSequence();
