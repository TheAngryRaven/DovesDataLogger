///////////////////////////////////////////
// FIRMWARE OTA MODULE — SD-staged self-update
// See firmware_ota.h for the protocol contract and the rationale.
///////////////////////////////////////////

#include "firmware_ota.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "crc32.h"

// SoftDevice headers from the core: nrf_soc.h gives sd_power_gpregret_set();
// nrf_sdm.h gives sd_softdevice_disable(); both pull in the nRF device header
// for NRF_NVMC and the NVMC_CONFIG_* fields used by the RAM flasher.
#include <nrf_soc.h>
#include <nrf_sdm.h>

// Adafruit core internal flash HAL — the same one InternalFileSystem uses.
// It transparently routes through the SoftDevice flash API (with the
// async-completion handshake handled for us) while the SD is enabled. We
// forward-declare it here rather than #include "flash/flash_nrf5x.h" so the
// build doesn't depend on that core-internal header's exact path: the symbols
// live in the always-compiled core library and link by name (C linkage). Any
// ABI drift in flash_nrf5x_flush still fails SAFE — the in-flash CRC
// re-verify below rejects a bad staging copy before the app is ever touched.
extern "C" {
  void     flash_nrf5x_erase(uint32_t addr);
  uint32_t flash_nrf5x_write(uint32_t dst_addr, void const* src, uint32_t len);
  void     flash_nrf5x_flush(void);
}

///////////////////////////////////////////
// Memory map (Seeed XIAO nRF52840, SoftDevice S140 7.3.0, Adafruit bootloader)
//
//   0x00000000  MBR + SoftDevice S140 7.3.0
//   0x00027000  CODE_REGION_1_START — application starts here  (FW_APP_BASE)
//   ...         running application (this firmware)
//   0x000A4000  staging region for the incoming image          (FW_STAGE_BASE)
//   0x000F4000  Adafruit bootloader                            (FW_BOOTLOADER_ADDR)
//   0x000FE000  MBR parameter page
//   0x000FF000  bootloader settings page
//
// The staging region [FW_STAGE_BASE, FW_BOOTLOADER_ADDR) is 320 KB and must
// not overlap the running app, which leaves [0x27000, 0xA4000) = 512 KB for
// the app — comfortably larger than the current image. PHASE 0 must confirm
// FW_STAGE_BASE sits above this firmware's actual end (check the .map file)
// and below the installed bootloader on the target units.
///////////////////////////////////////////
#define FW_APP_BASE         0x00027000UL
#define FW_BOOTLOADER_ADDR  0x000F4000UL
#define FW_FLASH_PAGE_SIZE  4096UL
#define FW_MAX_IMAGE_SIZE   (320UL * 1024UL)
#define FW_STAGE_BASE       (FW_BOOTLOADER_ADDR - FW_MAX_IMAGE_SIZE)  // 0xA4000

// Bootloader recovery flag written to GPREGRET before the destructive swap.
// If the swap is interrupted (power loss mid-erase) the app is left invalid;
// the Adafruit bootloader then falls into BLE DFU on its own. Setting the
// OTA-DFU magic additionally forces DFU even if the half-written vector table
// happens to look valid, so a sealed unit is always recoverable over BLE via
// the nRF Connect mobile app. (The design doc calls this the "0xB1 recovery
// flag"; on the stock Adafruit bootloader the value that actually forces OTA
// DFU is DFU_MAGIC_OTA_RESET = 0xA8 — PHASE 0 confirms the installed
// bootloader's magic.)
#define FW_GPREGRET_OTA_DFU 0xA8

// Minimum battery to allow APPLY. A brownout mid-swap bricks until the
// recovery net kicks in, so we gate on a healthy pack. Uses the cached
// reading (lastBatteryVoltage) that the BATT command and main loop maintain.
#define FW_MIN_APPLY_VOLTAGE 3.6f

// Linker-provided markers for the end of this firmware's flash footprint,
// used to refuse an apply that would erase live code (see fwAppFlashEnd).
// Declared weak so a linker script lacking them disables the guard (symbols
// resolve to address 0) instead of failing to link.
extern "C" {
  extern char __etext __attribute__((weak));         // end of .text in flash
  extern char __data_start__ __attribute__((weak));  // .data (RAM) start
  extern char __data_end__ __attribute__((weak));    // .data (RAM) end
}

// Staging file on SD.
static const char* kStagePath = "/fw/pending.bin";

// 16-byte image-descriptor magic. This firmware embeds it (kFwImageDescriptor
// below) so EVERY image built from this codebase carries an identifiable
// variant/version tag for forensics. The variant is NO LONGER inferred from
// these bytes during apply — the web app declares the target variant in the
// FWBEGIN handshake (derived authoritatively from the device's own DIS Model
// Number) and the firmware validates it once, up front, against
// FIRMWARE_VARIANT.
struct FwImageDescriptor {
  char magic[16];    // "DOVESBIRDSEYEFW1" (no NUL terminator — exactly 16 bytes)
  char variant[16];  // FIRMWARE_VARIANT, NUL-padded
  char version[32];  // FIRMWARE_VERSION, NUL-padded (wide enough for beta
                     // strings like "2.2.1-beta.abcdef0")
};

// `used` keeps the descriptor in the linked image even though nothing
// references it at runtime in this translation unit.
__attribute__((used))
const FwImageDescriptor kFwImageDescriptor = {
  {'D','O','V','E','S','B','I','R','D','S','E','Y','E','F','W','1'},
  FIRMWARE_VARIANT,
  FIRMWARE_VERSION,
};

///////////////////////////////////////////
// State machine
///////////////////////////////////////////
enum FwState {
  FW_IDLE = 0,
  FW_HANDSHAKE,  // FWBEGIN received + echoed; awaiting FWPUT
  FW_RECEIVING,  // FWPUT acked with FWREADY; receiving image chunks
  FW_VERIFIED,   // FWDONE received, CRC matched; awaiting FWAPPLY
};

static volatile FwState fwState = FW_IDLE;

// Handshake parameters (set by FWBEGIN).
static uint32_t fwExpectedSize = 0;
static uint32_t fwExpectedCrc = 0;

// Deferred-action flags (set by BLE callback, cleared by FW_OTA_LOOP).
static volatile bool fwPutPending = false;
static volatile bool fwDonePending = false;
static volatile bool fwApplyPending = false;
static volatile bool fwAbortPending = false;

// Receive bookkeeping.
static uint32_t fwBytesReceived = 0;
static volatile bool fwError = false;        // unrecoverable receive error
static const char* fwErrorReason = nullptr;  // FWERR token to emit

// Double buffer: BLE callback fills one half; main loop flushes the other.
#define FW_BUF_SIZE 4096
static uint8_t fwBuf[2][FW_BUF_SIZE];
static volatile int      fwFillIdx = 0;       // buffer the callback fills
static volatile uint16_t fwFillLen = 0;       // bytes in the fill buffer
static volatile int      fwFlushIdx = -1;     // buffer ready to flush, -1 = none
static volatile uint16_t fwFlushLen = 0;

// Staging file kept open across loop iterations during receive.
static File32 fwFile;
static bool fwSdHeld = false;

///////////////////////////////////////////
// Small helpers
///////////////////////////////////////////
static void fwNotify(const char* msg) {
  fileStatusChar.notify((uint8_t*)msg, strlen(msg));
}

static void fwNotifyErr(const char* token) {
  char buf[24];
  int n = snprintf(buf, sizeof(buf), "FWERR:%s", token);
  fileStatusChar.notify((uint8_t*)buf, n);
}

// Case-insensitive whole-string compare of a declared variant token (from the
// FWBEGIN handshake) against this build's compile-time FIRMWARE_VARIANT. Both
// strings must terminate together, so "sense" never matches "nonsense".
static bool fwVariantMatches(const char* declared) {
  const char* self = FIRMWARE_VARIANT;
  while (*declared && *self) {
    if (tolower((unsigned char)*declared) != tolower((unsigned char)*self)) {
      return false;
    }
    ++declared;
    ++self;
  }
  return *declared == '\0' && *self == '\0';
}

// Tear down all receive state, close the file, release SD. Safe to call
// from any path. Does NOT emit a notification.
void fwReset() {
  fwState = FW_IDLE;
  fwPutPending = false;
  fwDonePending = false;
  fwApplyPending = false;
  fwAbortPending = false;
  fwError = false;
  fwErrorReason = nullptr;
  fwBytesReceived = 0;
  fwFillIdx = 0;
  fwFillLen = 0;
  fwFlushIdx = -1;
  fwFlushLen = 0;
  if (fwFile) fwFile.close();
  if (fwSdHeld) {
    releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
    fwSdHeld = false;
  }
}

static void fwFail(const char* token) {
  debug(F("FW: failed: "));
  debugln(token);
  fwNotifyErr(token);
  fwReset();
}

///////////////////////////////////////////
// Command parsing (BLE callback task)
///////////////////////////////////////////
bool fwIsCommand(const char* cmd) {
  return strncmp(cmd, "FW", 2) == 0 &&
         (strncmp(cmd, "FWBEGIN:", 8) == 0 ||
          strncmp(cmd, "FWPUT:", 6) == 0 ||
          strcmp(cmd, "FWDONE") == 0 ||
          strcmp(cmd, "FWAPPLY") == 0 ||
          strcmp(cmd, "FWABORT") == 0);
}

bool fwReceiving() {
  return fwState == FW_RECEIVING;
}

bool fwApplyRequested() {
  return fwApplyPending;
}

void fwHandleCommand(const char* cmd, uint16_t len) {
  (void)len;

  if (strncmp(cmd, "FWBEGIN:", 8) == 0) {
    // FWBEGIN:<size>,<crc32hex>,<variant> — control-channel handshake. Parse
    // all three fields, validate the declared target variant against this
    // build's identity (the ONLY variant check), store size + CRC, and echo
    // the CRC back so the web app can confirm a clean channel before it
    // streams a single byte.
    if (fwState != FW_IDLE) { fwNotifyErr("STATE"); return; }
    const char* p = cmd + 8;
    const char* comma1 = strchr(p, ',');
    if (!comma1) { fwNotifyErr("STATE"); return; }
    const char* comma2 = strchr(comma1 + 1, ',');
    if (!comma2) { fwNotifyErr("STATE"); return; }

    uint32_t size = (uint32_t)strtoul(p, nullptr, 10);

    // The CRC field sits between the two commas. Copy out exactly those bytes
    // so crc32::fromHex (which requires a NUL right after 8 digits) doesn't
    // choke on the trailing ",<variant>".
    if (comma2 - (comma1 + 1) != 8) { fwNotifyErr("CRC"); return; }
    char crcHex[9];
    memcpy(crcHex, comma1 + 1, 8);
    crcHex[8] = '\0';
    uint32_t crc;
    if (!crc32::fromHex(crcHex, crc)) { fwNotifyErr("CRC"); return; }

    if (size == 0 || size > FW_MAX_IMAGE_SIZE) { fwNotifyErr("SIZE"); return; }

    // Declared target variant must match this build's compile-time identity.
    // Fail fast here, before any upload — this is now the only variant gate.
    if (!fwVariantMatches(comma2 + 1)) { fwNotifyErr("VARIANT"); return; }

    fwExpectedSize = size;
    fwExpectedCrc = crc;
    fwState = FW_HANDSHAKE;

    char echo[16];
    char hex[9];
    crc32::toHex(crc, hex);
    int n = snprintf(echo, sizeof(echo), "FWCRC:%s", hex);
    fileStatusChar.notify((uint8_t*)echo, n);
    debug(F("FW: BEGIN size="));
    debug(size);
    debug(F(" crc="));
    debugln(hex);
    return;
  }

  if (strncmp(cmd, "FWPUT:", 6) == 0) {
    if (fwState != FW_HANDSHAKE) { fwNotifyErr("STATE"); return; }
    uint32_t size = (uint32_t)strtoul(cmd + 6, nullptr, 10);
    if (size != fwExpectedSize) { fwNotifyErr("SIZE"); return; }
    // Defer the SD open + FWREADY to the main loop (thread-safe SD).
    fwPutPending = true;
    return;
  }

  if (strcmp(cmd, "FWDONE") == 0) {
    if (fwState != FW_RECEIVING) { fwNotifyErr("STATE"); return; }
    fwDonePending = true;
    return;
  }

  if (strcmp(cmd, "FWAPPLY") == 0) {
    if (fwState != FW_VERIFIED) { fwNotifyErr("STATE"); return; }
    fwApplyPending = true;
    return;
  }

  if (strcmp(cmd, "FWABORT") == 0) {
    // Defer the teardown to FW_OTA_LOOP(): fwReset() closes the staging file
    // and releases SD, which must not run in this BLE callback task.
    fwAbortPending = true;
    return;
  }
}

///////////////////////////////////////////
// Chunk receive (BLE callback task) — RAM only, no SD
///////////////////////////////////////////
void fwReceiveChunk(const uint8_t* data, uint16_t len) {
  if (fwState != FW_RECEIVING || fwError) return;

  if (fwBytesReceived + len > fwExpectedSize) {
    fwError = true;
    fwErrorReason = "SIZE";
    return;
  }

  uint8_t* buf = fwBuf[fwFillIdx];
  if ((uint32_t)fwFillLen + len > FW_BUF_SIZE) {
    // Current fill buffer can't hold this chunk — hand it off to the main
    // loop and switch to the other buffer. If the previous hand-off hasn't
    // been drained yet, the consumer fell behind: flag an overrun.
    if (fwFlushIdx != -1) {
      fwError = true;
      fwErrorReason = "WRITE";
      return;
    }
    fwFlushLen = fwFillLen;   // publish length first...
    fwFlushIdx = fwFillIdx;   // ...then the index (the "ready" signal)
    fwFillIdx ^= 1;
    fwFillLen = 0;
    buf = fwBuf[fwFillIdx];
  }

  memcpy(buf + fwFillLen, data, len);
  fwFillLen += len;
  fwBytesReceived += len;
}

///////////////////////////////////////////
// SD flush + CRC (main loop)
///////////////////////////////////////////
static bool fwWrite(const uint8_t* buf, uint16_t n) {
  if (n == 0) return true;
  size_t w = fwFile.write(buf, n);
  return w == n;
}

// Compute CRC-32 of the staged SD file (reads it back from disk, so this
// verifies what actually landed on the card, not just what we intended).
static bool fwCrcOfStageFile(uint32_t& outCrc) {
  File32 f = SD.open(kStagePath, FILE_READ);
  if (!f) return false;
  uint32_t state = crc32::kInit;
  uint8_t buf[512];
  while (true) {
    int r = f.read(buf, sizeof(buf));
    if (r < 0) { f.close(); return false; }
    if (r == 0) break;
    state = crc32::update(state, buf, (size_t)r);
  }
  f.close();
  outCrc = crc32::finalize(state);
  return true;
}

static void fwFinalizeReceive() {
  // Drain a pending hand-off, then the tail in the fill buffer.
  if (fwFlushIdx != -1) {
    if (!fwWrite(fwBuf[fwFlushIdx], fwFlushLen)) { fwFail("WRITE"); return; }
    fwFlushIdx = -1;
  }
  if (fwFillLen > 0) {
    if (!fwWrite(fwBuf[fwFillIdx], fwFillLen)) { fwFail("WRITE"); return; }
    fwFillLen = 0;
  }
  fwFile.sync();
  fwFile.close();
  releaseSDAccess(SD_ACCESS_BLE_TRANSFER);
  fwSdHeld = false;

  if (fwBytesReceived != fwExpectedSize) { fwFail("SIZE"); return; }

  uint32_t crc;
  if (!fwCrcOfStageFile(crc)) { fwFail("WRITE"); return; }
  if (crc != fwExpectedCrc) { fwFail("CRC"); return; }

  fwState = FW_VERIFIED;
  char hex[9];
  char msg[16];
  crc32::toHex(crc, hex);
  int n = snprintf(msg, sizeof(msg), "FWOK:%s", hex);
  fileStatusChar.notify((uint8_t*)msg, n);
  debug(F("FW: verified, "));
  debug(fwBytesReceived);
  debugln(F(" bytes OK"));
}

///////////////////////////////////////////
// Apply (main loop)
///////////////////////////////////////////

// RAM-resident flasher: erases the application region and copies the staged
// image down from FW_STAGE_BASE using the raw NVMC (the SoftDevice is already
// disabled and this code runs from RAM, because it is erasing the very flash
// the app executes from). It must be wholly self-contained: NO calls into
// flash-resident code, no library, no SoftDevice — a BL from RAM to a flash
// symbol is out of Thumb branch range and won't even link. That is why the
// final reset is an inline write to SCB->AIRCR rather than NVIC_SystemReset()
// (which the core emits as an out-of-line flash function). PHASE 0 must
// confirm this lands in / executes from RAM on the target toolchain (inspect
// the .map; the section attribute places it in .data, which startup copies to
// RAM).
__attribute__((noinline, section(".data")))
static void fwRamFlasher(uint32_t dst, uint32_t src, uint32_t words) {
  // Erase the destination (app) region, page by page.
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);
  for (uint32_t a = dst; a < dst + words * 4U; a += FW_FLASH_PAGE_SIZE) {
    NRF_NVMC->ERASEPAGE = a;
    while (NRF_NVMC->READY == 0) {}
  }
  // Program the new image, word by word.
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);
  volatile uint32_t* d = (volatile uint32_t*)dst;
  const volatile uint32_t* s = (const volatile uint32_t*)src;
  for (uint32_t i = 0; i < words; ++i) {
    d[i] = s[i];
    while (NRF_NVMC->READY == 0) {}
  }
  NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

  // System reset via the Cortex-M SCB, inline (no flash call). Equivalent to
  // NVIC_SystemReset() but reachable from RAM.
  __DSB();
  SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) |
                          (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) |
                          SCB_AIRCR_SYSRESETREQ_Msk);
  __DSB();
  for (;;) {}
}

// Copy the verified SD image into the upper staging flash region and
// re-verify its CRC there. Returns true on success; on any failure the
// running app is still completely intact (nothing destructive has happened).
static bool fwStageToFlash() {
  // Erase the staging region (only the pages we need).
  uint32_t pages = (fwExpectedSize + FW_FLASH_PAGE_SIZE - 1) / FW_FLASH_PAGE_SIZE;
  for (uint32_t i = 0; i < pages; ++i) {
    flash_nrf5x_erase(FW_STAGE_BASE + i * FW_FLASH_PAGE_SIZE);
  }

  // Copy SD -> staging flash in page-sized blocks, padding the final block
  // with 0xFF so the write length is flash-word aligned.
  File32 f = SD.open(kStagePath, FILE_READ);
  if (!f) return false;
  uint8_t blk[FW_FLASH_PAGE_SIZE];
  uint32_t off = 0;
  int lastPct = -1;
  while (off < fwExpectedSize) {
    memset(blk, 0xFF, sizeof(blk));
    uint32_t want = fwExpectedSize - off;
    if (want > FW_FLASH_PAGE_SIZE) want = FW_FLASH_PAGE_SIZE;
    int r = f.read(blk, want);
    if (r != (int)want) { f.close(); return false; }
    uint32_t wlen = (want + 3U) & ~3U;  // round up to whole words
    flash_nrf5x_write(FW_STAGE_BASE + off, blk, wlen);
    off += want;

    int pct = (int)((uint64_t)off * 90 / fwExpectedSize);  // copy = 0..90%
    if (pct != lastPct) {
      char msg[16];
      int n = snprintf(msg, sizeof(msg), "FWSTAGE:%d", pct);
      fileStatusChar.notify((uint8_t*)msg, n);
      lastPct = pct;
    }
  }
  f.close();
  // Commit the last (possibly partial) cached page to flash. NOTE: confirm
  // this matches the pinned Adafruit core — some versions take a bool
  // (flash_nrf5x_flush(true)). This is one of the Phase 0 build checks.
  flash_nrf5x_flush();

  // Re-verify the CRC of what is now sitting in staging flash (catches any
  // flash-write fault before we touch the app region).
  uint32_t state = crc32::kInit;
  state = crc32::update(state, (const void*)FW_STAGE_BASE, fwExpectedSize);
  return crc32::finalize(state) == fwExpectedCrc;
}

// Highest flash address this running firmware occupies = end of code plus the
// initialized-data image stored after it. Returns 0 if the linker didn't
// provide the markers (guard disabled). The cast-to-integer comparison avoids
// a -Waddress warning on the weak-symbol null test.
static uint32_t fwAppFlashEnd() {
  if ((uintptr_t)&__etext == 0) return 0;
  uint32_t dataImage = 0;
  if ((uintptr_t)&__data_end__ != 0 && (uintptr_t)&__data_start__ != 0) {
    dataImage = (uint32_t)(&__data_end__ - &__data_start__);
  }
  return (uint32_t)&__etext + dataImage;
}

static void fwDoApply() {
  if (fwState != FW_VERIFIED) { fwNotifyErr("STATE"); return; }

  // --- Guards (nothing destructive yet) ---
  if (lastBatteryVoltage < FW_MIN_APPLY_VOLTAGE) {
    debugln(F("FW: APPLY refused — battery low"));
    fwNotifyErr("BATTERY");
    return;
  }
  // Variant is validated once, up front, at FWBEGIN (the web app declares the
  // target derived from this device's DIS Model Number). No image-byte scan
  // here anymore.
  // Refuse if this firmware's own flash footprint reaches into the staging
  // region — erasing it (fwStageToFlash) would wipe live code mid-update. The
  // image-size cap keeps the incoming image clear of it, but the running app
  // could grow into it as the codebase expands, so guard at runtime too.
  uint32_t appEnd = fwAppFlashEnd();
  if (appEnd != 0 && appEnd > FW_STAGE_BASE) {
    debugln(F("FW: APPLY refused — app overlaps staging region"));
    fwNotifyErr("FLASH");
    return;
  }

  // --- Stage into upper flash and re-verify (still non-destructive) ---
  debugln(F("FW: staging image to flash..."));
  if (!fwStageToFlash()) {
    debugln(F("FW: stage/verify in flash failed"));
    fwNotifyErr("FLASH");
    return;
  }

  // Point of no return is now imminent. Tell the web app we're committed so
  // it can wait for the disconnect, then give the notification a moment to
  // go out before we tear the radio down.
  fileStatusChar.notify((uint8_t*)"FWSTAGE:100", 11);
  fwNotify("FWAPPLIED");
  delay(250);

  // Arm the bootloader recovery net BEFORE the destructive swap: if power is
  // lost mid-erase, the bootloader comes up in BLE DFU and the unit is
  // re-flashable over the air via nRF Connect — no pins, no opening the box.
  sd_power_gpregret_clr(0, 0xFF);
  sd_power_gpregret_set(0, FW_GPREGRET_OTA_DFU);

  // Hand the radio + SoftDevice down; from here only RAM-resident code runs.
  Bluefruit.Advertising.stop();
  sd_softdevice_disable();
  __disable_irq();

  uint32_t words = (fwExpectedSize + 3U) / 4U;
  fwRamFlasher(FW_APP_BASE, FW_STAGE_BASE, words);  // erases app, copies, resets

  // Unreachable — fwRamFlasher resets the chip.
  NVIC_SystemReset();
}

///////////////////////////////////////////
// Main-loop service
///////////////////////////////////////////
void FW_OTA_LOOP() {
  // Deferred FWABORT: tear everything down (closes staging file, frees SD).
  if (fwAbortPending) {
    fwAbortPending = false;
    fwReset();
    fwNotify("FWABORTED");
    return;
  }

  // Deferred FWPUT: open the staging file, then signal FWREADY.
  if (fwPutPending) {
    fwPutPending = false;
    if (!acquireSDAccess(SD_ACCESS_BLE_TRANSFER)) { fwNotifyErr("WRITE"); fwReset(); return; }
    fwSdHeld = true;
    SD.mkdir("/fw");
    if (SD.exists(kStagePath)) SD.remove(kStagePath);
    fwFile = SD.open(kStagePath, FILE_WRITE);
    if (!fwFile) { fwFail("WRITE"); return; }
    fwBytesReceived = 0;
    fwFillIdx = 0;
    fwFillLen = 0;
    fwFlushIdx = -1;
    fwState = FW_RECEIVING;
    fwNotify("FWREADY");
    debugln(F("FW: receiving image..."));
  }

  // Surface a receive-side error raised in the BLE callback.
  if (fwError) {
    fwFail(fwErrorReason ? fwErrorReason : "WRITE");
    return;
  }

  // Drain a pending buffer hand-off to SD while still receiving.
  if (fwState == FW_RECEIVING && !fwDonePending && fwFlushIdx != -1) {
    int idx = fwFlushIdx;
    uint16_t n = fwFlushLen;
    if (!fwWrite(fwBuf[idx], n)) { fwFail("WRITE"); return; }
    fwFlushIdx = -1;
  }

  // End of stream: flush the remainder, verify CRC.
  if (fwDonePending) {
    fwDonePending = false;
    if (fwState == FW_RECEIVING) fwFinalizeReceive();
  }

  // Apply (install + reboot).
  if (fwApplyPending) {
    fwApplyPending = false;
    fwDoApply();
  }
}
