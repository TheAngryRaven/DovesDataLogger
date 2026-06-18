///////////////////////////////////////////
// USB MASS STORAGE MODULE
// TinyUSB Mass Storage Class (MSC) glue over SdFat's block device.
// See usb_msc.h for the design overview.
///////////////////////////////////////////

#include "usb_msc.h"

// The MSC interface object. File-scope so only this module touches it.
static Adafruit_USBD_MSC usb_msc;

bool usbMscActive = false;

// --- TinyUSB block callbacks -------------------------------------------
// These run from the USBD task, NOT the main loop. The SD mutex
// (SD_ACCESS_USB_MSC, held for the whole session) guarantees no other
// subsystem touches the card concurrently, so these can drive the block
// device directly. bufsize is always a multiple of the 512-byte sector.
//
// NOTE: intentionally non-static — Arduino's auto-prototype generator
// conflicts with static functions in .ino files. Names are unique
// across the concatenated sketch.

int32_t msc_read_cb(uint32_t lba, void* buffer, uint32_t bufsize) {
  bool ok = SD.card()->readSectors(lba, (uint8_t*) buffer, bufsize / 512);
  return ok ? (int32_t) bufsize : -1;
}

int32_t msc_write_cb(uint32_t lba, uint8_t* buffer, uint32_t bufsize) {
  bool ok = SD.card()->writeSectors(lba, buffer, bufsize / 512);
  return ok ? (int32_t) bufsize : -1;
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

  // Exclusive SD ownership for the whole USB session. Bail if logging,
  // replay, or a BLE transfer is holding the card.
  if (!acquireSDAccess(SD_ACCESS_USB_MSC)) {
    debugln(F("USB MSC: SD busy, cannot enter mass-storage mode"));
    return false;
  }

  uint32_t sectorCount = SD.card()->sectorCount();
  if (sectorCount == 0) {
    debugln(F("USB MSC: SD sectorCount() == 0, aborting"));
    releaseSDAccess(SD_ACCESS_USB_MSC);
    return false;
  }

  usb_msc.setCapacity(sectorCount, 512);
  usb_msc.setUnitReady(true);
  usb_msc.begin();

  // USB is already enumerated (for charging/CDC). Re-enumerate so the
  // host picks up the newly-added MSC interface and mounts the drive.
  TinyUSBDevice.detach();
  delay(10);
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
  delay(50);
  NVIC_SystemReset();
}
