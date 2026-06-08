#pragma once

#include <stdint.h>

///////////////////////////////////////////
// FIRMWARE OTA MODULE  (SD-staged self-update)
//
// Why this exists: Chrome's Web Bluetooth blocklist bans the Nordic legacy
// DFU service that Adafruit's BLEDfu exposes, so the buttonless BLE-DFU path
// is unreachable from a browser, and our sealed units have no button / SWD
// pins to install a web-allowed Secure-DFU bootloader. So the application
// updates ITSELF: the DovesDataViewer web app streams the new image to the
// SD card over the existing custom 0x1820 file service, the firmware CRC-32
// verifies it, copies it into a free internal-flash region, and a tiny
// RAM-resident flasher swaps it into the application region and resets.
//
// Wire protocol (text commands written to 0x2A3E, text responses notified
// on 0x2A40; image bytes are raw binary writes to 0x2A3E):
//
//   FWBEGIN:<size>,<crc32hex>  -> FWCRC:<crc32hex>   (echo handshake)
//   FWPUT:<size>               -> FWREADY            (then raw chunks...)
//   <raw binary chunks>
//   FWDONE                     -> FWOK:<crc32hex> | FWERR:CRC|SIZE|WRITE
//   FWAPPLY                    -> FWSTAGE:<pct>* , FWAPPLIED | FWERR:...
//
// CRC is CRC-32/IEEE-802.3 (zlib), lowercase 8-char hex (see crc32.h).
//
// THREADING: the BLE request callback runs in the Bluefruit task. As with
// the track-upload path, it only parses commands and copies chunk bytes into
// RAM (never touches SD). All SD writes, CRC verification, and the apply
// sequence happen on the main loop via FW_OTA_LOOP().
///////////////////////////////////////////

// True if `cmd` is one of this module's FW* commands. The BLE callback's
// dispatch chain uses this to decide whether to hand the command to us.
bool fwIsCommand(const char* cmd);

// Handle a text FW* command (BLE callback task). Validates and either
// notifies a synchronous reply (FWCRC/FWERR) or sets a deferred flag for
// FW_OTA_LOOP() to act on (FWPUT/FWDONE/FWAPPLY). `len` is the raw write
// length, used to disambiguate the FWDONE token from image bytes.
void fwHandleCommand(const char* cmd, uint16_t len);

// True while an image is streaming (FWREADY sent, FWDONE not yet seen).
// While true the BLE callback routes raw writes to fwReceiveChunk().
bool fwReceiving();

// Buffer one raw image chunk (BLE callback task). Double-buffered; the main
// loop drains it to SD. Sets an internal error flag on overrun / oversize.
void fwReceiveChunk(const uint8_t* data, uint16_t len);

// Service deferred OTA work on the main loop: open staging file, flush
// chunks to SD, verify CRC, and run the apply sequence. Call from
// BLUETOOTH_LOOP().
void FW_OTA_LOOP();

// Abort any in-flight OTA and free resources (call on BLE disconnect).
void fwReset();
