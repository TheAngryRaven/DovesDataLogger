#pragma once

///////////////////////////////////////////
// BLUETOOTH MODULE
// BLE file-transfer + settings + track-management service using
// Bluefruit nRF52. Bluefruit callbacks run in a separate FreeRTOS
// task — anything touching SD has to be deferred into BLUETOOTH_LOOP()
// (which runs on the main loop) so SdFat sees only one writer.
///////////////////////////////////////////

// Bring up Bluefruit (or restart advertising if it's already up).
// Reads the bluetooth_name setting and starts advertising 0x1820.
void BLE_SETUP();

// Stop advertising, disconnect any peer, close the in-progress
// transfer file, and release SD access.
void BLE_STOP();

// Service deferred commands from the BLE callback task: settings
// commands, track upload/delete, MTU negotiation tail-read, and the
// burst-send chunk pipeline for any active file transfer.
void BLUETOOTH_LOOP();
