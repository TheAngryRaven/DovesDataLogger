# SD-Direct OTA Staging — Spike + the FWDFU Pre-Update

> Status: **SPIKE** — the pre-update (`FWDFU`) ships now on both channels;
> the SD-direct apply path is design + hardware validation work, not yet
> implemented. Grew out of the OTA cap incident on PR #116 (image hit
> 100.9% of the 320 KB self-flash cap; it was at 98.2% before sprint mode).

## Problem

The self-flash OTA (plan 0000 / subsystem 11) parks the incoming image in a
**320 KB internal-flash staging region** before the RAM flasher swaps it
into the app region. Staging and app share one 820 KB stretch
(`0x27000..0xF4000`), so the staging region is both the OTA image cap and a
tax on app space. The debug kill switch (DovesLapTimer#48) bought back
~5 KB — the image sits at 99.4% of the cap. Every future feature refights
this.

## Chosen direction: stage from the SD card

The image is **already fully staged and CRC-verified on the SD card**
(`/fw/pending.bin`) before it is ever copied to internal flash — the flash
staging region is a *second* copy that exists only because the RAM flasher
currently can't read SD. Teach the applier to read the image from SD and
the internal staging region disappears entirely:

- App region grows from 512 KB to the full ~820 KB.
- `FWERR:SIZE` and the CI OTA gate stop being about staging and become
  "fits the app region" (~2.5× today's image).
- The web-app protocol (`FW*`) is unchanged — same upload, same CRC, same
  apply command. Only `fwStageToFlash()`/`fwRamFlasher()` change.

### The hard part (why this is a spike)

The apply runs with the SoftDevice disabled and interrupts off, from
RAM-resident code. Reading SD there means a **raw SPI + SD-protocol driver
in the RAM flasher** — no SdFat, no FAT walking at apply time. The planned
shape:

1. **Pre-resolve the file before the destructive phase**: while SdFat is
   still alive, walk `/fw/pending.bin`'s cluster chain and flatten it into
   a sector-run list (start sector + count per contiguous run; a freshly
   written file on a healthy card is usually 1–3 runs). CRC is already
   verified at `FWDONE`; re-verify against the sector list before arming.
2. **RAM flasher loop**: raw single-block SD reads (CMD17 over
   bit-banged-or-minimal SPI) from the sector list → NVMC program of the
   app region, page by page. No filesystem logic at apply time.
3. **Recovery net unchanged**: GPREGRET OTA-DFU flag armed before the
   erase; a failed/interrupted swap still lands in the bootloader's BLE
   DFU. Additionally the UF2 path below is always available.

### Hardware spikes required before field use (Phase-0 style)

- Raw CMD17 reads with SoftDevice off / IRQs off: timing, card
  re-init after SdFat is torn down, SPI clock choice (EMI posture applies —
  apply happens parked, so the 8 MHz transfer clock rationale holds).
- Cards that stall mid-read (SD internal GC) vs. the WDT.
- Fragmented staging file behavior (worst-case sector-run list length).
- Power-loss matrix at each phase boundary.

## The pre-update: `FWDFU` (ships NOW, master + BETA)

The fleet insurance policy, shipped ahead of the rework so devices updated
*today* are ready for anything later:

- New BLE command `FWDFU` → `FWDFU:OK` → reboot with GPREGRET `0x57`
  (`DFU_MAGIC_UF2_RESET`). The **stock** Adafruit/Seeed bootloader then
  enumerates as a USB mass-storage drive; copying a `.uf2` onto it flashes
  the app region directly — **no size cap, no staging, no web app**.
- This makes every future migration (bigger images, the SD-direct rework,
  even a bootloader swap someday) reachable with nothing but a USB cable:
  command the device into UF2 mode over BLE, drag the file.
- The web app can later grow a "prepare for update" button that issues
  `FWDFU` — deliberately out of scope now.
- Release channels should publish a `.uf2` asset alongside the existing
  DFU `.zip` so there is always a file to drag (follow-up to the release
  workflow when the first post-FWDFU release is cut).
- A handful of sealed field units exist whose builders are out of contact;
  any of them that takes this update once (via the current ≤320 KB web OTA
  or nRF Connect) is permanently recoverable/updatable thereafter.

## Sequencing

1. **Now**: `FWDFU` to `master` and `BETA` (this plan's PRs). Test on BETA.
2. **Spike**: the four hardware validations above, on bench hardware.
3. **Implement**: SD-direct apply behind the spike results; delete
   `FW_STAGE_BASE`/staging copy; CI OTA gate re-pointed at the app-region
   bound.
4. **Someday/never**: if the spike fails hard, fall back to raising
   `FW_MAX_IMAGE_SIZE` (staging-layout move, plan 0002 discussion) — the
   options analysis lives in the PR #116 thread.
