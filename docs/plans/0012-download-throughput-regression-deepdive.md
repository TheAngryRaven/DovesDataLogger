# BLE download throughput regression — transport-stack deep dive

> Status: **FIXES 1+2 IMPLEMENTED** (same branch). Fix 1 landed *wider* than
> first proposed, by owner decision: the egg scanner is **race-gated** — it
> runs only while a race session is active (`SENSOREGG_LOOP()` reconcile on
> `raceActive`), not merely stopped during transfer mode; `BLE_SETUP()` still
> calls `SENSOREGG_SLEEP()` as an explicit guarantee. Fix 2 adds a second
> transfer-page line with the live connection interval + PHY
> (`bleLinkIntervalUnits()` / `bleLinkPhy()`). Fix 3 (the bench matrix) is
> the outstanding step; fix 4 stays contingent on its results.
>
> Field observation (2026-08-24): `Transfer: 77% / 33KB/s 8M 251
> 244` on the device's own transfer page, against a remembered 120+ KB/s.

## The one photo already rules most of the stack out

The diagnostic line plan 0008 added exists for exactly this moment, and it is
doing its job. `33KB/s 8M 251 244` decodes as:

| Field | Value | Meaning | Verdict |
|---|---|---|---|
| `33KB/s` | 33 KB/s | live transfer rate | the complaint |
| `8M` | 8 MHz | SD SPI clock **actually in force** (`sdActiveSpiHz()`) | **SD fast clock is applied — not the bug** |
| `251` | 251 B | negotiated link-layer PDU (`getDataLength()`) | **DLE landed — not the bug** |
| `244` | 244 B | ATT payload per notification (MTU 247) | **MTU landed — not the bug** |

So all three of plan 0008's levers verifiably engaged on this very session.
The regression lives in something the line does *not* show.

### The SD card is exonerated (the direct question)

*"Are we resetting the SD card properly to faster speeds before the
transfer?"* — **Yes, verified two independent ways:**

1. **Code path**: `BLE_SETUP()`'s first statement is
   `sdSetTransferSpeed(true)` (bluetooth.ino), before the radio even comes
   up; `BLE_STOP()` reverts it; the phone-disconnect auto-reboot makes a
   leaked fast clock impossible. `sdSetTransferSpeed()` falls back to 2 MHz
   only if the 8 MHz re-init fails — and then `sdActiveSpiHz()` would show
   `2M` on the page. The photo shows `8M`.
2. **Arithmetic**: the transfer path *reads* through the 4 KB read-ahead
   (`ble_stream::ReadAhead`). One aligned 4 KB multi-sector read at 8 MHz is
   ~4.3 ms → a ~950 KB/s ceiling; even the 2 MHz fallback ceilings at
   ~240 KB/s. Neither can produce a 33 KB/s plateau. The SD card cannot be
   the limiter at any clock the firmware uses.

Also note: downloads never touch the SD **write** path. Recent SD-adjacent
work (SLIST buffer sizing, the format page, course-creator temp-file writes)
is entirely off the `GET:` read path — `git diff` from the plan-0008 commit to
HEAD shows `bluetooth.ino`'s transfer code and `ble_stream.{h,cpp}` are
byte-identical except the SLIST buffer change, and `sd_functions.ino`'s
read/clock path is untouched. **Nothing in the firmware transfer code
regressed since 0008.** Write-hardening work may still be worth doing for the
logging path, but it is not what broke downloads.

### The viewer is exonerated too

`src/lib/ble/fileTransfer.ts` (DovesDataViewer) is unchanged since May. Its
hot path is already lean — push chunk into an array, bump a counter,
rAF-throttled UI — and, more fundamentally, ATT notifications carry no
app-level acknowledgement: a slow receiving app cannot throttle the device's
notify rate. The 33 KB/s is measured **on the device** and reflects what the
radio actually shipped. Nothing in the web app polls the device mid-download
(no BATT/status timers during a transfer).

## What the rate arithmetic says

33 KB/s ÷ 244 B ≈ **138 notifications per second**. Against plausible
connection intervals:

| Interval | Events/s | Notifies **per connection event** |
|---|---|---|
| 15 ms | 66.7 | **~2.1** |
| 30 ms | 33.3 | ~4.2 |
| 48.75 ms (Android default, no priority request) | 20.5 | ~6.8 |

120+ KB/s needs ~500 notifies/s — many packets per event via **connection
event length extension**. ~2 packets per 15 ms event is the classic signature
of an event that closes right after its first exchange: extension denied, or
the central closing early. The firmware's TX pipeline is not the constraint —
the 10-deep HVN queue plus the 20 ms burst loop keeps the SoftDevice saturated
(a blocked `notify()` is the design's flow control), and the parked loop
spends ≥90% of wall clock inside the burst.

**Conclusion: this is a radio-scheduling / link-parameter limit, not a code
path, not the SD card.** Two suspects, both currently invisible.

## Suspect 1 (prime, beta builds): the SensorEgg scanner runs through the whole transfer

On a `BIRDSEYE_ENABLE_SENSOREGG=1` build (every beta image),
`SENSOREGG_SETUP()` starts a **forever passive scan at 44% radio duty**
(40 ms window / 90 ms interval, `sensoregg_protocol.h`) at boot — egg present
or not. The only stop is `SENSOREGG_SLEEP()` in `enterShutdown()`.
**`BLE_SETUP()` never stops it**, so the scanner holds its 40-of-every-90 ms
radio claim for the entire transfer session.

On the S140 SoftDevice, *connection event length extension* — the mechanism
that turns 2 packets/event into 15+ — is one of the lowest scheduler
priorities, below the scanner's timeslots, and a persistently blocked scanner
periodically gets a raised priority to guarantee progress. A 44%-duty scanner
therefore both denies extension for much of every interval and periodically
preempts events outright. This can plausibly cost 2–3× on its own, and it is
the one radio-environment change that maps onto the timeline: the 120+ KB/s
readings, and plan 0008's ~130 KB/s bench memory, predate the egg scanner
being live on the measuring device; every beta image since carries it.

The camera subsystem is *not* a suspect: opening the Bluetooth page calls
`CAMERA_FORCE_RELEASE()` (drops the link, stops camera adverts) before
`BLE_SETUP()`.

**Fix**: pause the scanner for the transfer session — `SENSOREGG_SLEEP()` in
`BLE_SETUP()`. No wake path is needed: every exit from transfer mode reboots
(manual Exit and peer-disconnect both `NVIC_SystemReset()`), so the scanner
comes back with the fresh boot. One line, and the EGT feed loses nothing — the
parked transfer loop never reads it anyway (`SENSOREGG_LOOP()` is skipped
while `bleActive`).

## Suspect 2: the two link parameters the display still hides

Plan 0008's display line shows SD clock, PDU, and ATT payload — the three
levers it fixed. The two levers it *asked for but could not force* are not
shown, so a session where they didn't land is indistinguishable from one
where they did:

- **Connection interval.** The adaptive second ask
  (`requestConnectionParameter(12)`) fires at +500 ms only when the measured
  interval is slower than 15 ms, and Bluefruit's API sends **min == max**.
  Apple's accessory rules are commonly read to also require
  `Interval Max ≥ Interval Min + 15 ms`, so iOS may reject the flat-15 ms ask
  and stay at 30 ms — plan 0008 explicitly accepted that risk as "downside
  zero" *because there was no measurement either way*. There still isn't.
  Android centrals sit near 48.75 ms unless someone requests
  `CONNECTION_PRIORITY_HIGH` — and the native (Tauri) app's BLE backend does
  not (nothing in `lib/loggers/doveslogger` or the Rust side requests it), so
  a native-app download leans entirely on the firmware's second ask landing.
- **PHY.** `requestPHY(2M)` is fired at connect and never read back. A link
  that stayed at 1M doubles every packet's airtime, which halves
  packets-per-event exactly when extension is already being denied.

**Fix**: extend the diagnostic line (there is a spare display row under it):
show `getConnectionInterval()` (ms) and `getPHY()` during a transfer, e.g. a
second line `15ms 2M`. Then the next photo of a slow session names the lever
that slipped — the entire philosophy of the existing line. Optionally: when
the flat-15 ms second ask is rejected (interval unchanged at the +1500 ms
readback), issue a third ask as a range (12–24 units) via
`sd_ble_gap_conn_param_update` directly, which iOS is permitted to accept.

## Ranked fix plan

1. **Stop the egg scanner during transfer mode** (`SENSOREGG_SLEEP()` at the
   top of `BLE_SETUP()`). Highest expected win on beta builds; one line;
   reboot-on-exit makes it self-restoring. Bench A/B: same device, same
   central, before/after — the KB/s line is the instrument.
2. **Add interval + PHY to the transfer page.** Small, no behavior change,
   converts the next regression report from a guess into a diagnosis.
3. **Bench matrix** with (1)+(2) flashed: desktop Chrome, Android app,
   iPad — record `KB/s / SD / PDU / chunk / interval / PHY` for each. This
   retires the last unknowns and finally closes plan 0008's "needs a bench
   measurement" status.
4. Only if iOS is still slow after (1)–(3): the range-form connection-param
   ask (12–24 units) for Apple's `Max ≥ Min + 15` reading.

## Explicitly not the problem

- SD SPI clock switch (verified in code and on-screen; reads ceiling at
  ~950 KB/s at 8 MHz).
- SD write hardening / recent SD changes (transfer path is read-only and
  byte-identical since plan 0008).
- The read-ahead / burst loop (keeps the HVN queue saturated; ~2 pkt/event is
  a scheduler symptom, not a starvation symptom).
- The web viewer (unchanged since May; notifications are unacknowledged at
  ATT level, so the client cannot slow the device).
- The camera link (released before transfer mode opens).
- NeoPixel/profiling in the parked loop (~0.4 ms per 33 ms frame and
  sub-µs brackets respectively — irrelevant at these rates).
