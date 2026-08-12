///////////////////////////////////////////
// USB MASS STORAGE MODULE
// TinyUSB Mass Storage Class (MSC) glue over SdFat's block device.
// See usb_msc.h for the design overview.
///////////////////////////////////////////

#include "usb_msc.h"

// The MSC interface object. File-scope so only this module touches it.
static Adafruit_USBD_MSC usb_msc;

bool usbMscActive = false;

// Block-callback activity tracking for the exit drain in USB_MSC_DISABLE().
// mscIoInFlight is true while ANY block callback (read, write, or flush) is
// executing on the USBD task; mscLastIoMs is stamped when one finishes. The
// exit must wait for BOTH: a callback mid-writeSectors() can stall 100 ms–2 s
// on SD garbage collection, so a time-window check alone (the old
// mscLastWriteMs, stamped at write ENTRY and ignoring reads entirely) declared
// the bus quiet while a callback was still on it — and the main loop's
// syncDevice() + reset then raced the USBD task inside SdFat.
static volatile bool mscIoInFlight = false;
static volatile uint32_t mscLastIoMs = 0;

// --- TinyUSB block callbacks -------------------------------------------
// These run from the USBD task, NOT the main loop. The SD mutex
// (SD_ACCESS_USB_MSC, held for the whole session) guarantees no other
// subsystem touches the card concurrently, so these can drive the block
// device directly. bufsize is always a multiple of the 512-byte sector.
//
// NOTE: intentionally non-static — Arduino's auto-prototype generator
// conflicts with static functions in .ino files. Names are unique
// across the concatenated sketch.

// A whole, sector-aligned transfer is expected; reject anything else rather
// than silently dropping a partial sector via integer division. Retry up to
// 3x like the rest of the SD code (ignition EMI can glitch a single op).
int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {
  if (bufsize == 0 || (bufsize % 512) != 0) return -1;
  mscIoInFlight = true;
  int32_t result = -1;
  uint32_t sectors = bufsize / 512;
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (SD.card()->readSectors(lba, (uint8_t*) buffer, sectors)) {
      result = (int32_t) bufsize;
      break;
    }
  }
  mscLastIoMs = millis();
  mscIoInFlight = false;
  return result;
}

int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  if (bufsize == 0 || (bufsize % 512) != 0) return -1;
  mscIoInFlight = true;
  int32_t result = -1;
  uint32_t sectors = bufsize / 512;
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (SD.card()->writeSectors(lba, buffer, sectors)) {
      result = (int32_t) bufsize;
      break;
    }
  }
  mscLastIoMs = millis();
  mscIoInFlight = false;
  return result;
}

// Host signalled it is done writing — flush the card and drop SdFat's
// cache so any later firmware FS read sees the host's changes. (In USB
// mode the firmware doesn't touch the filesystem anyway; this is belt
// and suspenders, and matches the canonical Adafruit msc_sdfat example.)
void msc_flush_cb(void) {
  mscIoInFlight = true;
  SD.card()->syncDevice();
  SD.cacheClear();
  mscLastIoMs = millis();
  mscIoInFlight = false;
}

// --- Public API --------------------------------------------------------

void USB_MSC_SETUP() {
  // Vendor (<=8) / Product (<=16) / Revision (<=4) strings shown by the OS.
  usb_msc.setID("Doves", "DataLogger", "1.0");
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);

  // No media and no enumeration yet — the drive only appears once the
  // user opts into USB transfer mode via USB_MSC_ENABLE().
  usb_msc.setUnitReady(false);
}

bool USB_MSC_ENABLE() {
  if (usbMscActive) return true;

  // Require the cable first. With no VBUS there is nothing to mount, and the
  // parked loop reads "VBUS absent" as "cable pulled" on its very first
  // iteration -> instant USB_MSC_DISABLE() -> reset. Bail before taking the
  // SD lock, bumping the clock, or enumerating (isUsbConnected() reads the
  // raw VBUS register; defined in BirdsEye.ino).
  if (!isUsbConnected()) {
    debugln(F("USB MSC: no USB cable present, cannot enter mass-storage mode"));
    return false;
  }

  // Exclusive SD ownership for the whole USB session. Bail if logging,
  // replay, or a BLE transfer is holding the card.
  if (!acquireSDAccess(SD_ACCESS_USB_MSC)) {
    debugln(F("USB MSC: SD busy, cannot enter mass-storage mode"));
    return false;
  }

  // Parked transfer — bump the SD clock for real USB throughput (the 2 MHz
  // EMI-safe clock caps the drive at ~250 KB/s). Restored on the reboot in
  // USB_MSC_DISABLE(); restored explicitly here on the early-bail path.
  sdSetTransferSpeed(true);

  uint32_t sectorCount = SD.card()->sectorCount();
  if (sectorCount == 0) {
    debugln(F("USB MSC: SD sectorCount() == 0, aborting"));
    sdSetTransferSpeed(false);
    releaseSDAccess(SD_ACCESS_USB_MSC);
    return false;
  }

  usb_msc.setCapacity(sectorCount, 512);
  usb_msc.setUnitReady(true);
  if (!usb_msc.begin()) {
    // MSC interface registration failed — don't strand the user on a
    // "drive active" screen with no drive. Undo and report failure.
    debugln(F("USB MSC: begin() failed, aborting"));
    usb_msc.setUnitReady(false);
    sdSetTransferSpeed(false);
    releaseSDAccess(SD_ACCESS_USB_MSC);
    return false;
  }

  // USB is already enumerated (for charging/CDC). Re-enumerate so the
  // host picks up the newly-added MSC interface and mounts the drive.
  // The detach must settle long enough for the host to notice the
  // disconnect before we re-attach — 10 ms is below the USB spec's
  // recommended window and some hosts won't re-enumerate cleanly.
  TinyUSBDevice.detach();
  delay(50);
  TinyUSBDevice.attach();

  usbMscActive = true;
  debugln(F("USB MSC: mass-storage drive active"));
  return true;
}

void USB_MSC_DISABLE() {
  // A reboot is the cleanest exit: it drops the MSC interface (drive
  // disappears) and the firmware remounts a fresh filesystem, so any
  // files the host added/removed are picked up. Mirrors the BLE
  // auto-reboot on disconnect.
  debugln(F("USB MSC: exiting — rebooting to remount filesystem"));
  // Drop media-ready so a still-attached host stops queueing new work, then
  // cut the USB connection entirely. setUnitReady(false) alone only refuses
  // NEW SCSI commands — an already-dispatched READ10/WRITE10 keeps calling
  // the block callbacks on the USBD task, and the host keeps issuing more.
  // Detaching is what actually stops the traffic, so the drain below only
  // has to outlast the ONE callback that may still be executing. (Harmless
  // on the cable-pulled path — the bus is already dead.)
  usb_msc.setUnitReady(false);
  TinyUSBDevice.detach();

  // Drain before we sync + reset: wait until no block callback is executing
  // AND none has finished for a short quiet window. Reads count too — a
  // concurrent readSectors() wedges the shared SPI bus exactly like a write.
  // Without this the main-loop syncDevice() raced a callback still inside
  // SdFat, the exit hung on the wedged bus, and the ~4 s watchdog reset the
  // device instead of the clean reboot (the "crash on USB exit" field bug).
  // Bounded generously — SD garbage collection can stall a single
  // writeSectors() for 100 ms–2 s — and WDT-fed so the wait itself can
  // never trip the watchdog.
  const uint32_t quietMs = 100;
  const uint32_t maxWaitMs = 4000;
  const uint32_t waitStart = millis();
  while (millis() - waitStart < maxWaitMs) {
    wdtPet();
    if (!mscIoInFlight && (millis() - mscLastIoMs >= quietMs)) break;
    delay(5);
  }

  SD.card()->syncDevice();
  delay(50);
  NVIC_SystemReset();
}
