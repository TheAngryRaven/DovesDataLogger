#pragma once

#include "project.h"

///////////////////////////////////////////
// BLUETOOTH MODULE
// BLE file-transfer + settings + track-management service using
// Bluefruit nRF52. Bluefruit callbacks run in a separate FreeRTOS
// task — anything touching SD has to be deferred into BLUETOOTH_LOOP()
// (which runs on the main loop) so SdFat sees only one writer.
//
// RADIO OWNERSHIP (shared with camera_ble): the one SoftDevice has a
// single advert set and a single peripheral connection slot, shared
// between this transfer service and the camera remote. bleOwner says
// who owns them right now, and the shared connect/disconnect callbacks
// route on it.
//   - bleActive / bleConnected keep their TRANSFER-ONLY meanings:
//     "transfer page is up" / "transfer peer is connected". Camera mode
//     never sets them — the main loop keeps running race processing
//     while a camera is linked.
//   - Owner transitions happen ONLY on the main loop (BLE_SETUP /
//     BLE_STOP here, advertising actions in CAMERA_LOOP()), never in a
//     Bluefruit callback, so the callbacks always see a stable owner.
///////////////////////////////////////////

// Current radio owner (defined in BirdsEye.ino with the other BLE
// state flags). See the ownership model above. volatile: read from
// Bluefruit task callbacks for routing decisions.
extern volatile BleOwner bleOwner;

// One-time Bluefruit core bring-up, idempotent: conn config, begin(1
// peripheral + 1 central on SensorEgg builds — the central slot is the
// egg's PerchWerks GATT link, plan 0018; stock builds begin(1, 0)), TX
// power, connect/disconnect callbacks, conn interval, DFU + DIS
// services, the file service, the camera GATT
// (cameraBleRegisterServices()) and the egg client objects. Registers
// every service BEFORE any advertising starts. Deliberately does NOT
// advertise, set the device name, or touch the conn-LED — those belong
// to whichever owner takes the radio next.
void bleCoreEnsureInit();

// Rebuild the advert set for transfer mode from scratch: stop + clear
// advert/scan-response data, set the name from the bluetooth_name
// setting, add the transfer payload (flags, TX power, 0x1820 service,
// name), and start advertising. The stop/clear makes it correct no
// matter who owned the advert set before (camera or nobody).
void bleApplyTransferAdvertising();

// Pad the current advert + scan-response payloads to exactly 31 bytes
// each (zero padding, spec-legal). MANDATORY before every
// Advertising.start() in this firmware: Bluefruit 0.21.0's _start()
// freezes the packet lengths at the first advert of the boot (function-
// local static), so any later advert of a different length is garbled
// on air. Uniform 31+31 packets make the frozen length always correct.
void bleAdvFinalizePadded();

// Bring up transfer mode: ensure the core is up, take bleOwner for the
// transfer service, and (re)build + start transfer advertising.
void BLE_SETUP();

// Stop advertising, disconnect any peer, close the in-progress
// transfer file, release SD access, and drop bleOwner back to NONE.
void BLE_STOP();

// Manual exit from the Bluetooth transfer page: BLE_STOP() then reboot
// (NVIC_SystemReset), so a manual exit applies changed settings and clears
// session state exactly like the phone-disconnect auto-reboot and the USB
// mass-storage exit. Does not return on hardware; the SIM stub returns
// after stopping the radio.
void bleExitTransferMode();

// Force the Bluefruit connection LED off (autoConnLed disarm + park the
// pin high). Shared by BLE_STOP() and the shutdown quiesce.
void bleConnLedOff();

// Shutdown-path radio quiesce, unconditional on owner: stop advertising,
// drop any surviving link with a bounded WDT-fed settle for the async
// disconnect, then force the conn LED off. Safe when BLE was never
// initialized. Called from enterShutdown() — BLE_STOP() only covers the
// transfer service, so camera-owned radio state needs this.
void bleShutdownQuiesce();

// Service deferred commands from the BLE callback task: settings
// commands, track upload/delete, post-connect link tuning, and the
// burst-send chunk pipeline for any active file transfer.
void BLUETOOTH_LOOP();

// --- Transfer diagnostics (read by the Bluetooth page) ---------------------
// These exist so a download-speed regression is visible on the device instead
// of being inferred from a progress bar. See
// docs/plans/0008-ble-download-throughput.md.

// Live transfer rate in bytes/sec, 0 when nothing is streaming.
uint32_t bleTransferRateBps();

// Negotiated link-layer PDU in bytes. 27 means Data Length Extension never
// happened, which fragments every notification into ten packets — by far the
// largest throughput tax on this link.
uint16_t bleLinkDataLength();

// Payload carried by one notification at the negotiated ATT MTU.
uint16_t bleLinkChunkSize();

// Connection interval in 1.25 ms units, live from the connection object
// (0 = no peer). The central owns this number — the device only requests —
// and 24 units (30 ms) instead of 12 (15 ms) is a silent 2x on every
// download, so it belongs on the page (plan 0012).
uint16_t bleLinkIntervalUnits();

// Radio PHY, live from the connection object: 1 = 1M, 2 = 2M, 4 = Coded,
// 0 = no peer. A link that ignored the 2M request pays double airtime per
// packet (plan 0012).
uint8_t bleLinkPhy();
