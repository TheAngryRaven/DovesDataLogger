#pragma once

#include <stdint.h>

///////////////////////////////////////////
// SD BOOT PROBE — "unformatted, dead, or just not ready yet?"
// When the boot-time SD.begin() retries all fail, SD_SETUP() diagnoses
// the card layer by layer: does it answer at the SPI level (cardBegin),
// does it report a size (sectorCount), and does a FAT volume mount
// (volumeBegin)? The answer picks the boot page — the format offer for
// "card answers but no FAT", the FAULT dead-end for "no card".
//
// One probe is not enough evidence to offer an ERASE. A card that is
// answering commands but returning garbage for sector 0 looks exactly
// like a blank one, and there is a real way to get there: the nRF52's
// hardware watchdog survives a soft reset, so a slow post-reset boot
// can be WDT-reset mid-SD-transaction, and with CS grounded and no
// power switch nothing in firmware can reset the card afterwards. The
// 2026-09 field report was exactly that — a freshly formatted, perfectly
// good card offered for formatting on every soft reboot until a power
// cycle. So the volume probe must fail CONSECUTIVELY, with a settle in
// between, before the card is declared unformatted; and any card-layer
// failure along the way is "dead", never "blank" — a flapping card must
// never be offered an erase.
//
// The sketch keeps the SdFat calls, the delay and the WDT pets; this
// unit answers "mounted, retry, unformatted, or dead" so the rule is
// host-tested.
///////////////////////////////////////////

namespace sd_probe {

// Consecutive card-answers-but-no-volume probes before the card is
// declared unformatted and the format page is offered.
constexpr int kVolumeFailuresToDeclare = 2;

// Settle between volume probes (ms). Long enough for a card still busy
// programming (up to ~250 ms after a write per the SD spec) or mid-way
// through an interrupted transfer to finish before it is asked again.
constexpr uint32_t kVolumeRetryDelayMs = 250;

// One probe's raw results, in the order the sketch asks them.
struct Attempt {
  bool cardOk;      // SD.cardBegin() succeeded (SPI-level init + CSD)
  bool hasSectors;  // sectorCount() > 0 (the card reported a size)
  bool volumeOk;    // SD.volumeBegin() succeeded (a FAT volume mounted)
};

enum class Verdict : uint8_t {
  kMounted,      // FAT volume mounted — normal boot
  kRetryVolume,  // card answers, volume did not: settle, probe once more
  kUnformatted,  // card answers, volume failed enough times — offer format
  kDead,         // card layer failed — FAULT page, never an erase
};

// Classify one probe given how many consecutive volume failures preceded
// it (0 for the first probe). kRetryVolume is only ever returned while
// another probe would still count toward kUnformatted.
Verdict classify(const Attempt& a, int volumeFailuresBefore);

}  // namespace sd_probe
