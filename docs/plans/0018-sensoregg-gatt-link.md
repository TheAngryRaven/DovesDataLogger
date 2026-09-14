# 0018 — SensorEgg GATT link: the logger becomes the pod's central

The egg grew the PerchWerks Sensor Service (its repo's roadmap phases
2-3: a self-describing Descriptor, a Clock read carrying a `boot_id`
epoch, and a Sample notify streaming per-channel batch frames stamped
at acquisition). The beacon path we consume today is latest-value-only,
unacknowledged, and unauthenticated beyond the MAC filter. This plan
connects: when a paired egg is in range and the egg radio is wanted,
the logger opens a GATT link and consumes the service; the beacon
remains the unclaimed/disconnected fallback and the pairing transport.

This formally supersedes the old "do not improve this into a
connection" rule (sensoregg.h) — that rule was written when the *egg*
was a pure broadcaster. The egg now accepts connections by design. The
rule's actual content — **the camera link wins every tradeoff** —
stays, expressed as: skinny central parameters (event length 6 = 7.5 ms
cap), the same race/bench gate as the scanner, and `SENSOREGG_SLEEP()`
dropping the link for transfers and shutdown.

## Approach & key decisions

- **`Bluefruit.begin(1, 1)` on SensorEgg builds only** (stock stays
  `(1, 0)`), with `configCentralConn(247, 6, 1, 1)` before it. This is
  the documented spec-§7.2.3 fallback becoming the shipped design.
  *Risk:* the shared `begin()` warning ("re-soak the camera link") —
  the bench checklist ends with a camera connect/record/download soak
  with the egg link streaming.
  - *Rejected:* a second SoftDevice config profile — `configCentralConn`
    is independent of the peripheral knobs; never call
    `configCentralBandwidth` (it re-applies presets).
- **Link gate = scan gate + paired.** `eggLinkWanted() =
  (raceActive || eggPairingActive || eggTestActive) && !eggSleeping &&
  sensoreggIsPaired()`. The scanner runs only while the pairing window
  is open or no link is engaged — while streaming, the 44 % scan duty
  isn't paid at all (the paired egg is silent anyway: a connected
  single-link peripheral stops advertising).
  - *Rejected:* always-connected-when-paired — plan 0012's radio
    discipline stays uniform; transfers/menu keep a clean radio.
- **The scan callback doubles as the connect trigger.** A report from
  the paired MAC with the connectable bit, while a link is wanted and
  not engaged, goes to `Bluefruit.Central.connect(report)` instead of
  the double buffer (and skips `Scanner.resume()` — Bluefruit holds the
  scanner paused into `sd_ble_gap_connect`).
- **Bring-up runs in the Bluefruit callback task** (central connect
  callback): discover service + 3 characteristics → MTU exchange →
  read Descriptor → timed Clock read → enableNotify → ready-flag-last.
  This is a documented deviation from the module's "callbacks only
  copy" rule: every Bluefruit client op is blocking
  (`waitUntilComplete`), and ~10-15 round trips at a 100-200 ms
  connection interval on the main loop would stall the 25 Hz DOVEX row
  engine for seconds. The **data plane** keeps the copy-only
  discipline: the notify callback memcpys frames into an 8-slot ring
  (ready-flag-last), `SENSOREGG_LOOP()` drains and decodes.
- **The GATT stream feeds the exact same surface as the beacon**:
  `eggReading` / `eggRxMs` / `eggHaveReading` / `eggSeqMon` — so every
  accessor, DOVEX column, race page, and LED path is untouched. Values
  route by **descriptor-driven role mapping** (normative names
  EGT/CJ/IAT + quantity 0x08 → battery), never hardcoded channel ids.
  Latest sample of each frame wins (latest-value semantics preserved;
  per-sample row resampling is a future plan — the clock fit is
  implemented and anchored, ready for it).
  - No beacon/GATT double-feed exists by construction: the egg cannot
    advertise while connected.
- **Zombie detection** feeds `eggSeqMon` from the *fastest channel's*
  frame seq only (per-channel u8 counters are independent; mixing them
  could alias as "frozen").
- **Epoch handling:** the Clock read anchors a v1 linear fit (slope
  1.0, offset from the request/response midpoint). A frame whose
  `boot_id` differs from the anchor logs `pod rebooted` and drops the
  link — the reconnect re-anchors. (An egg reboot drops the physical
  link anyway; this path is a mop-up.)
- **Graceful degradations while streaming** (beacon-only fields):
  `sensoreggTcFault()` reads false (frames carry no MCP STATUS — an
  open probe still shows as sentinel → NaN → `---`, the same visible
  outcome); `sensoreggPairingFlag()` / `sensoreggProtoVersion()` hold
  their last beacon values. Pairing capture itself always runs on
  beacons (the window forces the scanner on), and capturing a NEW egg
  while another is connected drops the old link.
- **Security deferred** to the egg's roadmap phase 4 (LESC bonding).
  Today's authorization is the pairing-window MAC capture (plan 0017).

## Link state machine

| State | Entered by | Context |
|---|---|---|
| IDLE | boot / sleep / unpaired | — |
| WAIT_ADV | link wanted | main-loop reconcile |
| CONNECTING | paired connectable report seen → `Central.connect` | scan callback (BLE task) |
| BRINGUP | central connect callback | callback task runs the blocking sequence |
| STREAMING | main loop commits the staged bring-up (parse + map + anchor) | main loop |
| BACKOFF (5 s) | connect timeout (10 s), bring-up failure, disconnect | main loop |

## Touch points

- `BirdsEye/sensoregg_gatt.{h,cpp}` + `tests/sensoregg_gatt_test.cpp` —
  new pure unit: descriptor/frame/clock decode, role mapping, clock
  fit. Fixtures are **byte-identical** to the egg repo's
  `pw_gatt_encode` goldens (the cross-repo pin, exactly like
  `sensoregg_protocol` ↔ `pw_adv_encode` for the beacon).
- `BirdsEye/sensoregg.{h,ino}` — GATT LINK section: client objects,
  state machine, notify ring, drains, gate split
  (`eggLinkWanted`/`eggScanWanted`), `SENSOREGG_SLEEP()` extension
  (cancel/drop the central link — shutdown quiesce only handles the
  peripheral handle), new accessors `sensoreggLinkMode()` /
  `sensoreggGattMtu()` + no-op twins; radio-role prose rewritten.
- `BirdsEye/bluetooth.{h,ino}` — flag-gated `begin(1, 1)` +
  `configCentralConn`; stale be80-era comments fixed.
- `BirdsEye/display_pages.ino` — EGG TEST: `rf:` gains `GATT`, SEQ row
  gains `M<mtu>`.
- `sim/stubs/module_stubs.cpp`, `sim/sim_prototypes.h` — two new stubs.
- `CLAUDE.md` (subsystem 14 + begin() notes), `ARCHITECTURE.md`,
  `CHANGELOG.md`.

## Verification

- Host: `ctest` — the fixture matrix (descriptor parse incl.
  stride-by-28 forward-compat, frame parse, clock parse, role map,
  sampleToReal sentinel, fit anchor/epoch/wrap); `-Werror`.
- Sim: build + goldens **unchanged** (flag-0 invisibility; a diff is a
  gating leak, never regenerate).
- Both flag arms compile locally (the begin split makes flag-0 a real
  compile risk; CI only builds flag-1).
- Bench (owner): flash the phases-2-3 egg first. Pair via the window →
  enter EGG TEST → `rf:GATT M247` with live values within ~2 s;
  power-yank the egg → `---` inside 1 s, auto-reconnect + "pod
  rebooted" line on its return; unpair → beacon fallback (rf:OK);
  transfer mode drops the link and download throughput matches plan
  0012's numbers; **camera soak**: repeated connect/record cycles + one
  full download with the egg link streaming; shutdown/charge park with
  the link up → clean System OFF.

## Status / follow-ups

- Status: shipped with this plan's PR into BETA.
- Follow-ups: per-sample DOVEX row timestamping using the clock fit
  (its anchor + epoch machinery ships here, unused by rows yet);
  LESC bonding when the egg's phase 4 lands; multi-pod (the state
  machine is single-link by design today).
