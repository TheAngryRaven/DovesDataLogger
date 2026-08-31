#pragma once

#include <stddef.h>
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
// Receives the DovesSensorEgg's PW-ADV advertising broadcasts, v1 and
// v2 (see sensoregg_protocol.h for the byte layouts), and exposes the
// latest readings to the logger and display. Scope: one egg, EGT
// ("Temp1") + cold junction ("Junction1"), and on v2 eggs the aux
// intake-air thermistor ("Temp2") + a real battery percent.
//
// RADIO ROLE: pure OBSERVER on the shared SoftDevice. Passive scanning
// only — we never transmit a SCAN_REQ, never connect, and hold no GATT
// link to the egg, so the scanner cannot contend with the Insta360 X4
// camera link (peripheral role) for TX airtime. S140 natively time-slices
// scan windows around existing connection events. The egg is a pure
// broadcaster and accepts no connections. Do not "improve" this into a
// connection — the camera link wins every tradeoff.
//
// PAIRING (plan 0017): runtime MAC filter, persisted as the
// "sensoregg_mac" setting ("AA:BB:CC:DD:EE:FF"; empty = unpaired =
// accept any advertiser whose payload matches the PW magic).
// SENSOREGG_MAC below is only the fallback when the setting is unset or
// unparsable. Capture is window-gated: the Egg menu opens a 2-minute
// listen window, and the first egg heard with ITS pairing-window flag
// set (egg-side long-press, payload flags bit0) is persisted — physical
// possession is the authorization. Applied live on pair/unpair (no
// reboot), like the camera serial.
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

// FALLBACK egg MAC (used only when the "sensoregg_mac" setting is unset
// or unparsable), in the human-readable order the egg prints at boot
// (AA:BB:CC:DD:EE:FF -> {0xAA,0xBB,...}). All-zeros = magic-match any
// PW-ADV broadcaster. (nRF ble_gap_addr_t stores bytes LSB-first; the
// match helper handles the reversal — keep this define human-ordered.)
#define SENSOREGG_MAC {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

// ---- lifecycle (called from BirdsEye.ino) ----

// Bring up the BLE core (idempotent) and CONFIGURE the passive scanner —
// it is not started here. The scanner is race-gated (plan 0012):
// SENSOREGG_LOOP()'s reconcile starts it when a race session begins and
// stops it when the session ends, so the 44% scan duty is never paid on
// the menu or during a BLE transfer (where it throttled downloads).
// Call from setup() after CAMERA_SETUP(). NOTE: this makes the SoftDevice
// come up at boot — previously BLE was lazy (first camera/transfer use).
void SENSOREGG_SETUP();

// Drain the scan double-buffer, parse the newest payload, and reconcile
// the scanner against the race gate (start on race entry with a 1 s
// retry throttle, stop when the session ends). Call from the normal
// main-loop path. Not called in the bleActive / usbMscActive parking
// branches — a stale reading correctly goes NaN there, and the scanner
// is already down (race over) before either mode can be entered.
void SENSOREGG_LOOP();

// Stop the passive scanner and hold it stopped (shutdown path — a scan
// must not run into System OFF / the charging park — and BLE_SETUP(),
// which guarantees a transfer session is scan-free). Idempotent; no-op
// when the POC is compiled out.
void SENSOREGG_SLEEP();

// Clear the sleep gate after a charging-loop soft resume
// (softResumeFromCharging). Does not start the scanner — the resume
// lands on the menu, and the race gate brings the scan up if a session
// begins. The scanner config survives a stop.
void SENSOREGG_WAKE();

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

// v2 aux thermistor (intake air, "Temp2") in degC. NaN when the link is
// stale, the egg is v1 (no aux field), or the egg reported the invalid
// sentinel (divider open/shorted).
float sensoreggAuxC();

// Egg battery percent 0-100; 0xFF = unknown (stale link, v1 stub, or no
// pack fitted on the egg).
uint8_t sensoreggBatteryPct();

// True while fresh AND the egg flags a thermocouple fault (open /
// out-of-range probe, MCP9600 STATUS input-range bit).
bool sensoreggTcFault();

// Free-running egg sequence counter from the latest payload (debug).
uint16_t sensoreggSequence();

// ---- pairing surface (plan 0017; display_ui.ino / display_pages.ino) ----

// True while a specific egg MAC is stored (runtime filter non-wildcard).
bool sensoreggIsPaired();

// Open the 2-minute capture window (kPairingTimeoutMs): the scanner is
// forced on and EVERY magic-matching egg is observed — the first frame
// carrying the egg's own pairing-window flag wins and is persisted.
// Idempotent; re-requesting restarts the timeout clock.
void sensoreggRequestPair();

// Close the capture window without pairing (user cancel; the timeout
// closes it on its own otherwise).
void sensoreggCancelPair();

// True while the capture window is open.
bool sensoreggPairingInProgress();

// Forget the paired egg. PERSISTS FIRST: returns false when the settings
// write fails (SD trouble) and changes nothing, so the UI can warn
// instead of silently diverging from disk. True = unpaired, back to
// accept-any.
bool sensoreggUnpair();

// Paired MAC as "AA:BB:CC:DD:EE:FF". Returns false (buf = "") when
// unpaired or bufSize < sensoregg_protocol::kMacStrLen (18).
bool sensoreggPairedMac(char* buf, size_t bufSize);

// ---- bench test page (mirrors cameraTestEnterMode/ExitMode) ----

// Latch the passive scanner on outside races so the EGG TEST page (and
// the camera test page's coexistence soak) stream live data on a desk.
// Exit drops the latch; the race gate then owns the scanner again.
void sensoreggTestEnterMode();
void sensoreggTestExitMode();

// ---- test-page telemetry ----

// Protocol version byte of the latest frame (1/2); 0 when stale or no
// egg was ever heard.
uint8_t sensoreggProtoVersion();

// The egg's own pairing-window flag (payload flags bit0), gated fresh &&
// !hung like tcFault so a frozen payload can't show a live window.
bool sensoreggPairingFlag();

// Measured accepted-parse rate over a 1 s tumbling window (~9-10 Hz on a
// healthy bench at the egg's 111.875 ms advertising interval).
float sensoreggPacketHz();
