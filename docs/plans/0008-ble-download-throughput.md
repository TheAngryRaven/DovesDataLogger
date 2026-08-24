# Getting BLE downloads back off the floor

> Status: **DONE (firmware).** Needs a bench measurement to confirm which of
> the three levers below was actually holding the field device at ~29 KB/s.

## Why this exists

A 3.3 MB session downloaded at **28.8 KB/s** (ETA 1m52s) on an iPad, against a
remembered **~130 KB/s** from an earlier bench run. Same firmware family, same
protocol. Everything else about that day was healthy — the driving loop was
tight enough to drop 5 PVT frames in 20 minutes, and a second session dropped
none — so the loop is not what regressed. Downloads are.

The first suspicion was the SD card: the normal clock is a deliberately slow
2 MHz for ignition EMI, and the fix for that (`sdSetTransferSpeed(true)` →
8 MHz while parked) already shipped in 4.0.0. **It is still wired up
correctly** — `BLE_SETUP()` bumps it, `BLE_STOP()` reverts it. So "we forgot to
put the clock back up" was not the bug. But `sdSetSpiClock()` **silently falls
back to 2 MHz** when the 8 MHz re-init fails, and nothing anywhere surfaced
which clock the session actually got. That silence is a bug in its own right,
and it is fixed here.

Reading the transfer path with the SD theory set aside turned up three real
throughput ceilings, in rough order of expected impact.

## Lever 1 — the Data Length Extension request was fired and never checked

`Bluefruit.configPrphConn(247, ...)` sets the **ATT MTU** to 247, so a
notification carries a 244-byte payload. It says nothing about the **link-layer
PDU**, and if that stays at the 27-byte default every notification is
fragmented into ten link-layer packets, each paying its own
preamble/access-address/header/CRC and inter-frame space. That is roughly a
3–4× tax — the size of the gap being complained about.

The connect callback *does* call `connection->requestDataLengthUpdate()`, so
this looked handled. It is not, for two reasons:

- **It can be rejected on arrival.** The SoftDevice runs one link-layer control
  procedure at a time, and the DLE ask is fired on the line immediately after
  `requestPHY(2M)`. A collision comes back as `NRF_ERROR_BUSY`, which
  `VERIFY_STATUS` turns into a `false` return — and the return value is
  discarded at the call site. Nothing retries.
- **Nothing ever read the result.** `BLEConnection::_data_length` starts at 27
  and only moves on a `BLE_GAP_EVT_DATA_LENGTH_UPDATE`. No code path anywhere
  looked at `getDataLength()`, so a link that silently stayed un-extended was
  indistinguishable from one that did not.

Centrals that initiate DLE themselves — Chrome on desktop and Android do —
paper over this entirely, which is very plausibly why the bench run hit
~130 KB/s while an iOS Web Bluetooth shim did not.

Fix: verify. `bleTuneLink()` reads `getDataLength()` once the link has settled
and re-asks if it is still at the default, with nothing else in flight to
collide with. Nordic's guidance is to extend *after* the ATT MTU exchange
anyway, so the retry lands in the right order for free.

## Lever 2 — the connection-interval request iOS was always going to reject

`Bluefruit.Periph.setConnInterval(6, 12)` advertises a preferred interval of
7.5–15 ms. Apple's accessory guidelines require an accessory's requested
interval to satisfy `Interval Min ≥ 15 ms`, so **iOS rejects this outright** and
leaves the connection on whatever it picked at connect (commonly 30 ms). Half
the connection events, half the throughput.

The naive fix — raise the preferred interval to 15 ms — would slow down every
central that *does* honour 7.5 ms. So the PPCP stays exactly as it is, and the
link-tuning block adds an **adaptive second ask**: read the interval actually in
force, and only if it is slower than 15 ms issue an explicit, Apple-compliant
`requestConnectionParameter(12)`. Centrals already running fast are untouched;
iOS gets an offer it is allowed to accept.

Bluefruit's `requestConnectionParameter()` sets min == max, so the ask is a flat
15 ms with the library's default slave latency 0 and 2 s supervision timeout —
all inside Apple's bounds. Some readings of the guideline also want
`Interval Max ≥ Interval Min + 15 ms`; if a central holds us to that it rejects
this ask too and we are exactly where we already were, so the downside is zero
and the upside is 2×. The bench measurement is what settles it.

## Lever 3 — the SD read was serialised into the radio's critical path

The burst loop read `chunkSize` bytes straight off SdFat and immediately
notified them, ten times per `loop()`. Two costs:

- **Every notification waits on a disk read.** `notify()` blocking on a full
  HVN queue is good flow control — that is the radio being saturated, which is
  the goal. A 244-byte `FatFile::read()` in between is not: it is dead radio
  time, and at 2 MHz (the silent-fallback case) it is *a lot* of dead radio
  time.
- **244 is not 512.** Unaligned sub-sector reads run through SdFat's single
  512-byte cache block, so a large file costs one single-block SD command per
  sector, each with its own command/response round trip.

Fix: a 4 KB read-ahead. One aligned `read()` pulls eight sectors in a single
multi-block transfer, and the notifications then stream out of RAM. The refill
stall (~4.3 ms at 8 MHz) is shorter than a connection interval and is covered by
the ~2.4 KB already sitting in the SoftDevice's 10-deep HVN queue, so the radio
does not go idle across it.

Refills are **compacting** — the unsent tail moves to the front — so every
notification except the file's last one carries a full chunk. Without that, each
4 KB boundary would emit a runt packet (4096 mod 244 = 190).

The burst bound changes from "10 chunks" to "20 ms of wall clock", because the
point of the bound is loop responsiveness, not a packet count: 10 chunks is
2.4 KB, which is a very different amount of time on a fast link than on a slow
one. 20 ms keeps the exit button and the watchdog serviced with three orders of
magnitude to spare.

### Failure handling got simpler, and safer

The old loop reacted to a failed `notify()` by `seekCur(-bytesRead)` — rewinding
the file, because the read had already advanced the position and dropping the
chunk would silently punch a hole in a transfer that still reported `DONE`.
With a read-ahead there is nothing to rewind: the bytes are in RAM and the
buffer head simply does not advance. The hole is structurally impossible rather
than patched.

## Where the logic lives

`ble_stream.{h,cpp}` (host-tested, Arduino-free) owns the index arithmetic —
chunk sizing from the negotiated MTU, the compacting refill, slice/consume, and
the throughput rate. `bluetooth.ino` owns the bytes, the file, and the radio.
That split is what makes the off-by-one that would corrupt a download testable
on a desktop instead of discoverable on a track day.

## Making the next regression visible

The whole reason this took a reading session to notice is that the device
reported a percentage and nothing else. The Bluetooth page now shows live
**KB/s** alongside the percentage, plus a one-line link summary:

```
Transfer: 42%
118KB/s  8M 2M 244
```

— rate, SD clock, link-layer PDU (the DLE result), ATT payload. If downloads are
slow again, that line says which lever slipped without needing a rebuild.
`sdActiveSpiHz()` backs the SD figure with the clock that was actually applied,
not the one that was requested.
