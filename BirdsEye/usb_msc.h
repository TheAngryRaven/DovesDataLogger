#pragma once

///////////////////////////////////////////
// USB MASS STORAGE MODULE
// Exposes the SD card to a host computer as a USB drive (TinyUSB MSC)
// for plain drag-and-drop file transfer — an alternative to the BLE
// file-transfer service.
//
// The drive is "opt-in": it only enumerates while the user is on the
// USB transfer page (USB_MSC_ENABLE), so normal plug-in/charging is
// unchanged. Leaving the page reboots the device (USB_MSC_DISABLE) so
// the firmware re-reads a clean filesystem after host edits and the
// drive disappears from the host. The parked loop branch in BirdsEye.ino
// also calls USB_MSC_DISABLE() if the cable is unplugged, so the SD lock
// and fast SPI clock can never leak past the end of a transfer session.
//
// While the drive is active the SD mutex is held as SD_ACCESS_USB_MSC,
// keeping logging/replay/BLE off the card — the TinyUSB read/write
// callbacks touch the card from the USBD task, not the main loop.
///////////////////////////////////////////

// Whether the USB mass-storage drive is currently presented to a host.
extern bool usbMscActive;

// Register the MSC callbacks once at boot. Does NOT enumerate a drive —
// the host sees nothing until USB_MSC_ENABLE() is called.
void USB_MSC_SETUP();

// Enter USB mass-storage mode: acquire the SD mutex, set the drive
// capacity, mark the media ready, and force a USB re-enumeration so the
// host mounts the SD as a drive. Returns false (and changes nothing) if
// the SD card is busy with another subsystem.
bool USB_MSC_ENABLE();

// Leave USB mass-storage mode. Reboots the device (NVIC_SystemReset) so
// the host drive drops and the firmware remounts a fresh filesystem.
void USB_MSC_DISABLE();
