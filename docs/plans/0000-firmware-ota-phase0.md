# Firmware SD-staged OTA — Phase 0 findings & apply-strategy decision

This document records the Phase 0 investigation that gates the self-flash
("apply") mechanism for the SD-staged firmware OTA feature, and the design
decisions that flow from it. The protocol, SD staging, and CRC verification
are implemented and host-tested independently of these findings (see
`BirdsEye/firmware_ota.{h,ino}`, `BirdsEye/crc32.{h,cpp}`, and
`tests/crc32_test.cpp`).

> **Status: mostly settled by field use. ONE spike outstanding — #1.**
>
> This document was written *before* the apply path shipped, when all five
> spikes were open and each item recorded a best-evidence answer plus what to
> confirm on a bench unit. The apply path has since shipped and has been
> flashing real loggers over the air for weeks, including a full end-to-end
> run on a bench vehicle. **Ordinary successful updates exercise spikes #2, #3
> and the soft-reset half of #5 every single time** — those are no longer
> open questions, and #4 is moot because it was only ever the fallback for a
> #3 that failed.
>
> **Spike #1 — the recovery net — is the one a successful update can never
> prove.** It only fires when an update is *interrupted*: a half-written
> vector table, or power lost mid-swap. Every update so far has succeeded, so
> that path has never been walked. On sealed units it is the difference
> between a recoverable brick and a brick, which is why it is still called out
> here. It needs one deliberately destructive bench test (below).
>
> Per-spike status is recorded inline. Anyone running the remaining test
> should validate against the CURRENT constants in `BirdsEye/firmware_ota.ino`
> — the staging base moved (see the memory-map note).

## Why we self-flash at all (settled, do not re-litigate)

- Chrome's Web Bluetooth blocklist bans the Nordic **legacy** DFU service
  (`00001530-…-785feabcd123`) that Adafruit `BLEDfu` exposes, so the
  buttonless BLE-DFU path is unreachable from a browser.
- A web-allowed Secure-DFU bootloader needs SWD pins to install; our units
  are sealed (no button, no pins). We therefore do **not** change the
  bootloader for the field flow.
- Instead the app streams the image to SD over the existing custom `0x1820`
  service (not blocklisted), CRC-verifies it, and installs it itself.

## Memory map assumed by the implementation

> **Superseded — the staging base moved.** Plan 0004 piece 2 split the
> app+staging span evenly: `FW_STAGE_BASE` is now `0x0008E000` and
> `FW_MAX_IMAGE_SIZE` is 408 KiB (app region `[0x27000, 0x8E000)` = 412 KiB).
> The table below records the layout as originally specced. Anyone actually
> running these spikes should validate against the CURRENT constants in
> `BirdsEye/firmware_ota.ino` — spike 3 in particular erases the staging
> region by address.

| Region | Address | Notes |
|---|---|---|
| MBR + SoftDevice S140 7.3.0 | `0x00000000`–`0x00026FFF` | |
| **Application** (`FW_APP_BASE`) | `0x00027000` | `CODE_REGION_1_START` |
| … running app … | | current image ≪ 512 KB |
| **Staging region** (`FW_STAGE_BASE`) | `0x000A4000` | 320 KB, `= 0xF4000 − 320 KB` |
| **Bootloader** (`FW_BOOTLOADER_ADDR`) | `0x000F4000` | Adafruit nRF52840 |
| MBR param page | `0x000FE000` | |
| Bootloader settings | `0x000FF000` | |

The split leaves `[0x27000, 0xA4000)` = **512 KB** for the running app and
`[0xA4000, 0xF4000)` = **320 KB** for the staged image. `FWBEGIN`/`FWPUT`
reject any image larger than `FW_MAX_IMAGE_SIZE` (320 KB) with `FWERR:SIZE`.

## Phase 0 spikes

### 1. Recovery net — invalid app ⇒ re-flashable over BLE with no pins
> **STATUS: OPEN — the only one left.** Field use cannot close this. Every
> update so far has *succeeded*, and this path only exists for the ones that
> don't. Closing it needs the destructive test below, deliberately performed.

**Why it matters:** this is what makes self-flashing acceptable on sealed
units. If the swap is interrupted, the unit must still be recoverable.

**Best-evidence answer:** the Adafruit nRF52 bootloader enters DFU mode when
the application is invalid (bad/missing vector table at `FW_APP_BASE`), and it
advertises BLE DFU in that state. nRF Connect (a *native* app, not subject to
the web blocklist) can then push a fresh image. The implementation also writes
`GPREGRET = DFU_MAGIC_OTA_RESET (0xA8)` *before* the destructive swap so DFU is
forced even if a half-written vector table looks plausibly valid.

**Confirm on hardware:** corrupt the app's first page on a bench unit (e.g.
erase one page at `0x27000`), power-cycle, and verify the bootloader
advertises and is re-flashable over BLE via nRF Connect with **no** button /
USB / SWD interaction. This is the make-or-break safety gate.

### 2. App can erase+write the upper free-flash region (SoftDevice flash API)
> **STATUS: CLOSED by field use.** Every OTA stages the incoming image into
> the upper region over a live BLE connection; a failure here would surface as
> `FWERR:FLASH` or a stalled transfer, and updates complete normally. The
> timing concern (page erase starving the connection) is answered by the same
> evidence — transfers hold up.

**Best-evidence answer:** with the SoftDevice enabled, page erase / word write
go through `sd_flash_page_erase` / `sd_flash_write`, which complete
asynchronously via a SoftDevice flash event. The implementation uses the
Adafruit core's `flash_nrf5x_{erase,write,flush}` HAL (the same one
`InternalFileSystem` uses), which wraps that handshake — so we don't hand-roll
event handling. Page size is 4096 B; writes must be word-aligned (the staging
copy pads the final block to a word with `0xFF`).

**Confirm on hardware:** erase + write a known pattern across
`[0xA4000, 0xF4000)` while BLE is connected, read it back, and confirm timing
(≈85 ms/page erase) doesn't starve the BLE connection. Confirm the
`flash/flash_nrf5x.h` include path resolves for a sketch build on the pinned
core version.

### 3. RAM flasher — erase app region + copy staged image, then boot it
> **STATUS: CLOSED by field use.** This was written as "the spike that decides
> whether to ship the RAM flasher". It shipped, and it is what performs every
> update: each successful OTA is a full erase-app-region → copy-staged-image →
> reset → boot-the-new-image cycle. The `.map` inspection and NVMC
> single-stepping below were pre-ship de-risking; the mechanism is now
> demonstrated by the thing itself working repeatedly, on more than one unit.

**Best-evidence answer:** the app-region swap erases the very flash the app
runs from, so it must execute from RAM with the SoftDevice disabled and IRQs
off, touching only the raw NVMC. `fwRamFlasher()` is marked
`__attribute__((section(".data")))` (startup copies `.data` to RAM, and RAM is
executable on Cortex-M with no MPU configured) and is fully self-contained
(`NVIC_SystemReset` is an inlined CMSIS intrinsic; no libc/SoftDevice calls).

**Confirm on hardware (highest risk):** inspect the `.map` to confirm
`fwRamFlasher` is linked into RAM (`0x20000000…`); single-step or scope the
NVMC `READY` waits; verify a full erase+copy+reset cycle boots the new image.
This is the spike that decides whether to ship the RAM flasher or fall back to
the dual-bank alternative (#4).

### 4. Fallback — dual-bank Adafruit bootloader via nRF Connect (no pins)
> **STATUS: MOOT.** This was only ever contingent on #3 failing validation.
> #3 works in the field, so this migration is not happening. Kept for the
> record — if a future flash-layout change (plan 0004 piece 3) destabilises
> the swap, this is still the documented escape route.

**If the RAM flasher proves shaky:** a one-time, over-the-air move to a
dual-bank Adafruit bootloader makes the swap bootloader-managed. The
bootloader update itself is pushed via nRF Connect (native app → buttonless
trigger works) with no pins. Evaluate only if #3 fails validation.

### 5. GPREGRET behavior across soft-reset vs power-loss
> **STATUS: soft-reset half CLOSED by field use** — the normal flow arms the
> flag and soft-resets, and updates land correctly. **The power-loss half is
> not independently testable and does not need to be:** the whole point is
> that GPREGRET is *lost* on brownout, which is precisely why recovery there
> falls through to spike #1. Confirming #1 confirms this.

**Best-evidence answer:** `GPREGRET` is a RETAINED register in the always-on
power domain; it survives a soft reset (`NVIC_SystemReset`) but **not** a full
power loss / brownout. So GPREGRET is the recovery path for the *soft-reset*
case (our normal flow), while the *power-loss* case relies on spike #1
(invalid-app ⇒ DFU). Both paths must hold for the net to be complete.

**Confirm on hardware:** set GPREGRET, soft-reset, confirm the bootloader sees
it; then repeat with a power-cycle and confirm it's cleared (forcing reliance
on the invalid-app fallback).

## Decision

**Leading plan adopted, and since shipped:** SD → upper-flash staging via
`flash_nrf5x`, in-flash CRC re-verify, GPREGRET recovery flag, then a
RAM-resident flasher for the app-region swap (spikes #1–#3, #5). The dual-bank
migration (#4) was the documented fallback if spike #3 failed on hardware; it
did not, so #4 is moot.

This is the path in the field today. What follows are the safety invariants it
enforces — all of them still current, and all of them still load-bearing.

**Safety invariants enforced in code:**
- The app region is **never** erased until the staged copy is CRC-verified
  *in flash* (`fwStageToFlash()` returns false on mismatch → `FWERR:FLASH`,
  app untouched).
- `FWAPPLY` is refused below `FW_MIN_APPLY_VOLTAGE` (3.6 V) → `FWERR:BATTERY`.
- A variant mismatch is refused at the `FWBEGIN` handshake, before any upload
  → `FWERR:VARIANT` (the web app declares the target variant, derived from the
  device's DIS Model Number, and the firmware compares it to its compile-time
  `FIRMWARE_VARIANT`). Every image still embeds `kFwImageDescriptor` for
  forensics.
- The GPREGRET recovery flag is armed *before* the destructive swap.
- Every step before the RAM flasher is abortable; the running firmware stays
  intact until the flasher actually runs.

## The one test still outstanding

Everything else here is answered. This is the whole remaining scope of Phase 0:

**Prove that an interrupted update leaves a recoverable unit** (spike #1).

1. On a bench unit, erase one page at `FW_APP_BASE` (`0x27000`) — enough to
   invalidate the application's vector table.
2. Power-cycle. Do **not** press anything, plug in USB, or attach SWD.
3. Confirm the Adafruit bootloader comes up advertising BLE DFU.
4. Push a fresh image over the air with nRF Connect (native app — not subject
   to the Chrome Web Bluetooth blocklist that forced self-flashing in the
   first place) and confirm the unit boots it.

If that holds, the recovery net is real and Phase 0 is closed. If it does
not, the honest consequence is that a sealed unit interrupted mid-swap is
unrecoverable, and the apply path needs the dual-bank migration (#4) before it
is safe to keep shipping to sealed hardware.

Worth doing on a unit you are willing to lose, and worth doing *before* plan
0004 piece 3 (SD-direct apply) touches the swap — that change would otherwise
land on an unproven recovery net.

## Fleet migration

The first firmware carrying this `FW*` protocol is pushed to already-sealed
units **once** via nRF Connect (native app, buttonless trigger works on the
existing single-bank bootloader — no bootloader swap). After that single push,
all future updates go through the web app. New production ships this firmware
from the factory.
