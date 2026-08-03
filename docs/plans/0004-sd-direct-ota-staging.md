# SD-Direct OTA Staging — Spike + the FWDFU Pre-Update

> Status: **PARTIALLY SHIPPED**, in three pieces. Grew out of the OTA cap
> incident on PR #116 (image hit 100.9% of the 320 KB self-flash cap; it was
> at 98.2% before sprint mode).
>
> | # | Piece | Cap after | Status |
> |---|---|---|---|
> | 1 | `FWDFU` pre-update (USB UF2 escape hatch) | n/a | **shipped**, master + BETA, field-tested |
> | 2 | Even flash split (staging base `0xA4000` → `0x8E000`) | 320 → **408 KiB** | **shipped**, no hardware spike needed |
> | 3 | SD-direct apply (delete the staging region) | 408 → **~792 KiB** | **spike-gated**, not implemented |
>
> Piece 2 was originally filed below as the "someday/never" fallback. It was
> promoted because it is free: it needs no new code paths, only two
> constants, and it alone took the beta image from 99.0% of the cap to 77.6%.
> Piece 3 is still the endgame and still needs bench hardware.

## Problem

The self-flash OTA (plan 0000 / subsystem 11) parks the incoming image in a
**320 KB internal-flash staging region** before the RAM flasher swaps it
into the app region. Staging and app share one 820 KB stretch
(`0x27000..0xF4000`), so the staging region is both the OTA image cap and a
tax on app space. The debug kill switch (DovesLapTimer#48) bought back
~5 KB — the image sits at 99.4% of the cap. Every future feature refights
this.

## Piece 2 (shipped): split the shared span evenly

Before touching the apply path at all, the existing layout was simply
lopsided. App and staging share `[0x27000, 0xF4000)` = 839 680 B, and
**both** must be able to hold the image — the incoming one is staged up top,
then copied down over the app. So the largest installable image is half the
span. The split was 320 KiB staging against 500 KiB app, which capped OTA at
320 KiB while leaving ~180 KiB of app region no legal image could ever reach.

Splitting evenly, rounded down to a 4 KiB erase page:

```
staging  [0x8E000, 0xF4000) = 417 792 B = 408 KiB   FW_MAX_IMAGE_SIZE
app      [0x27000, 0x8E000) = 421 888 B = 412 KiB   >= the cap
```

Two constants, plus `static_assert`s for page alignment and app-region fit.
No change to the apply sequence, the `FW*` protocol, the CRC, or the web app
(which never enforced a cap of its own — it relies on `FWERR:SIZE`).

**Why this is safe for the existing fleet.** The staging base is read from
the *installed* firmware's constants at apply time; nothing about it is baked
into the image being delivered. A unit still on 3.0.x therefore stages at the
old `0xA4000` and installs a new-layout build normally. And because the new
app region ends at `0x8E000`, **below** the old `0xA4000` staging base, an
image built for this layout can never grow into an old unit's staging region.
Growing the cap is purely additive.

**The one transitional constraint**: a unit on 3.0.x still enforces the old
320 KiB cap, so an image past that is un-installable *on that unit* until it
has taken a new-layout build (its fallback is the USB `FWDFU` → UF2 path).
CI warns on crossing the legacy cap for exactly this reason. Drop the warning
once the stragglers have migrated.

## Piece 3 (spike-gated): stage from the SD card

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

1. ~~**Now**: `FWDFU` to `master` and `BETA` (this plan's PRs). Test on
   BETA.~~ **Done** — merged both channels, flashed twice on beta clean.
2. ~~**Someday/never**: fall back to raising `FW_MAX_IMAGE_SIZE`
   (staging-layout move).~~ **Done, and promoted out of "someday"** — piece 2
   above. It cost two constants and bought 88 KiB, so there was no reason to
   hold it behind the spike. BETA first, then `master`.
3. **Next, no hardware needed**: publish a `.uf2` asset from `release.yml` /
   `beta.yml`. `FWDFU` gets you into UF2 mode but there is currently no
   published file to drag once you are there, which makes the escape hatch
   only half-real.
4. **Spike**: the four hardware validations above, on bench hardware.
5. **Implement**: SD-direct apply behind the spike results; delete
   `FW_STAGE_BASE`/staging copy; CI OTA gate re-pointed at the app-region
   bound. The options analysis lives in the PR #116 thread.

With piece 2 shipped the pressure is off — 93 KiB of headroom against a beta
image that grew ~7.5 KiB for all of sprint mode. Piece 3 is now a
"do it properly when the bench is free" item, not a blocker.
