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
// state flags). See the ownership model above.
extern BleOwner bleOwner;

// One-time Bluefruit core bring-up, idempotent: conn config,
// begin(1 peripheral + 1 central — the central slot is the camera
// control link), TX power, connect/disconnect callbacks, conn
// interval, DFU + DIS services, the file service, and the camera GATT
// (cameraBleRegisterServices()). Registers every service BEFORE any
// advertising starts. Deliberately does NOT advertise, set the device
// name, or touch the conn-LED — those belong to whichever owner takes
// the radio next.
void bleCoreEnsureInit();

// Rebuild the advert set for transfer mode from scratch: stop + clear
// advert/scan-response data, set the name from the bluetooth_name
// setting, add the transfer payload (flags, TX power, 0x1820 service,
// name), and start advertising. The stop/clear makes it correct no
// matter who owned the advert set before (camera or nobody).
void bleApplyTransferAdvertising();

// Bring up transfer mode: ensure the core is up, take bleOwner for the
// transfer service, and (re)build + start transfer advertising.
void BLE_SETUP();

// Stop advertising, disconnect any peer, close the in-progress
// transfer file, release SD access, and drop bleOwner back to NONE.
void BLE_STOP();

// Service deferred commands from the BLE callback task: settings
// commands, track upload/delete, MTU negotiation tail-read, and the
// burst-send chunk pipeline for any active file transfer.
void BLUETOOTH_LOOP();
