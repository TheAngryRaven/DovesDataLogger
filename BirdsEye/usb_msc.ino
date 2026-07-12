///////////////////////////////////////////
// USB MASS STORAGE MODULE
// TinyUSB Mass Storage Class (MSC) glue over SdFat's block device.
// See usb_msc.h for the design overview.
///////////////////////////////////////////

#include "usb_msc.h"

// The MSC interface object. File-scope so only this module touches it.
static Adafruit_USBD_MSC usb_msc;

bool usbMscActive = false;

// millis() of the last host WRITE10 data phase serviced on the USBD task.
// USB_MSC_DISABLE() waits for this to go quiet before it syncs + resets, so
// a reset can't cut an in-flight writeSectors() and truncate a file / leave
// the FAT inconsistent.
static volatile uint32_t mscLastWriteMs = 0;

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
  uint32_t sectors = bufsize / 512;
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (SD.card()->readSectors(lba, (uint8_t*) buffer, sectors)) {
      return (int32_t) bufsize;
    }
  }
  return -1;
}

int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  if (bufsize == 0 || (bufsize % 512) != 0) return -1;
  mscLastWriteMs = millis();  // mark the write window for the exit quiesce
  uint32_t sectors = bufsize / 512;
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    if (SD.card()->writeSectors(lba, buffer, sectors)) {
      return (int32_t) bufsize;
    }
  }
  return -1;
}

// Host signalled it is done writing — flush the card and drop SdFat's
// cache so any later firmware FS read sees the host's changes. (In USB
// mode the firmware doesn't touch the filesystem anyway; this is belt
// and suspenders, and matches the canonical Adafruit msc_sdfat example.)
void msc_flush_cb(void) {
  SD.card()->syncDevice();
  SD.cacheClear();
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
  // Drop media-ready so the host sees the drive go away, and flush any
  // buffered writes to the card before we reset — the reboot is otherwise
  // a hard cut that would lose a not-yet-synced sector and risk leaving the
  // FAT inconsistent if the host hadn't already ejected.
  usb_msc.setUnitReady(false);

  // Quiesce before we sync + reset. setUnitReady(false) only stops NEW SCSI
  // commands; a WRITE10 already dispatched keeps calling msc_write_cb on the
  // USBD task. Wait until no write has landed for a short window so the reset
  // can't cut an in-flight writeSectors() (truncated file / inconsistent
  // FAT). Bounded so a wedged host can't hang the exit.
  const uint32_t quietMs = 100;
  const uint32_t maxWaitMs = 1000;
  const uint32_t waitStart = millis();
  while (millis() - waitStart < maxWaitMs) {
    if (millis() - mscLastWriteMs >= quietMs) break;  // writes have gone quiet
    delay(5);
  }

  SD.card()->syncDevice();
  delay(50);
  NVIC_SystemReset();
}
