---
name: Bug report
about: Something isn't working as expected
title: "[Bug] "
labels: bug
---

## What happened
<!-- A clear description of the bug and what you expected instead. -->

## Steps to reproduce
1.
2.
3.

## Environment
- **Firmware version / commit:** <!-- e.g. v1.0.0, or the git SHA -->
- **Board:** Seeed XIAO nRF52840 Sense <!-- change if different -->
- **Display:** SH110X / SSD1306
- **SD card:** <!-- size, format (FAT16/FAT32) -->
- **GPS module:** <!-- e.g. Matek SAM-M10Q -->
- **Tachometer connected?** yes / no

## Context that often matters
<!-- Tick / fill in whatever applies -->
- Did it happen while: logging / replaying / BLE transfer / idle / waking from sleep?
- GPS fix acquired at the time? sat count / HDOP if known:
- Was the engine running (RPM present)?
- Does it reproduce every time or intermittently?

## Logs / serial output
<!-- If you can build with HAS_DEBUG defined, paste the serial output.
     Otherwise describe what the OLED showed (any FAULT/WARNING page text). -->

```
```

## Anything else
<!-- A relevant .dovex/.json file, photos of wiring, etc. -->
