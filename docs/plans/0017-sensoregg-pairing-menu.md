# 0017 — SensorEgg pairing menu + live-data test page

The egg subsystem's data path is done (v2 parsing, Temp2 page/column,
zombie detection) but its pairing story is still the POC: a compile-time
`#define SENSOREGG_MAC`, all-zeros meaning "accept any egg". That is
wrong the moment a second egg exists at the track — anyone's pod lands
in your log. There is also no on-device way to see the egg's live stream
outside a race session (the scanner is race-gated since plan 0012), so
bench work means a phone running nRF Connect.

This plan adds a main-menu **Egg** entry with a camera-style pairing
page, runtime MAC pairing persisted to settings, and a bench **EGG
TEST** page that latches the scanner on outside races.

## Approach & key decisions

- **Window-gated capture.** The egg's long-press opens a 30 s pairing
  window that sets flags bit0 — parsed as `Reading.pairingActive` since
  the v2 round but never consumed. Pairing = the first frame seen with
  that bit set wins; its advertiser MAC is persisted. Physical
  possession is the authorization ("that's *my* egg").
  - *Rejected:* pair-to-first-egg-seen (grabs a neighbor's pod),
    egg-side BLE bonding (the egg is a pure broadcaster; the
    "do not improve this into a connection" rule stands — the future
    GATT migration is the egg repo's roadmap, not this plan).
- **Settings-backed MAC, `#define` demoted to fallback.** New key
  `sensoregg_mac` = `"AA:BB:CC:DD:EE:FF"` or `""` (unpaired =
  accept-any, today's behavior). Loaded in `SENSOREGG_SETUP()` (settings
  init precedes it); `SENSOREGG_MAC` covers unset/invalid. Applied
  live on pair/unpair — an exception to the settings-need-reboot rule,
  mirroring the camera serial.
  - *Rejected:* manual MAC entry page (12 hex chars on a 3-button
    wheel; deferred — B1 on the unpaired screen stays unbound for it).
- **The scan callback stays protocol-blind.** It filters
  (len/magic/MAC), copies, resumes — unchanged contract. For pairing it
  additionally memcpys the reporting `peer_addr` into a per-slot field
  of the existing double buffer (ready-flag-last ordering preserved).
  The capture decision runs in `SENSOREGG_LOOP()`'s drain on the parsed
  `pairingActive` bit — main-loop context, host-testable logic.
  - *Rejected:* peeking `buf[5]` bit0 in the callback (spreads protocol
    knowledge into BLE task context for no gain).
- **Accept-any during the window.** While pairing, the MAC filter is
  bypassed (`|| eggPairingActive`) so a *different* egg can be captured
  while one is already paired (re-pair without unpairing first).
- **Persist-first ordering** (camera precedent): `setSetting` succeeds
  → update the RAM filter → close the window. A failed SD write leaves
  the window open to retry on the next frame. The RAM filter is written
  while the window flag still bypasses it, so a torn 6-byte read can
  never reject the right egg; the one-report tolerance of a stale
  plain-bool read matches the documented `raceActive` read.
- **Bench latch.** `eggScanWanted()` grows two volatile flags:
  `raceActive || eggPairingActive || eggTestActive`. The reconcile,
  retry-throttle, and self-heal paths need zero changes. The EGG TEST
  page latches `eggTestActive` on entry and drops it on Back. The
  camera test page latches it too — restoring the camera+egg desk-soak
  behavior its comment promised (stale since plan 0012's race gate).
- **Egg row appended after Camera (index 6).** No churn to existing
  select-handler indices; pairing is a rare setup action.
- **Flag-0 sim invisibility = zero golden churn, zero golden coverage.**
  Everything sits behind `BIRDSEYE_ENABLE_SENSOREGG`; the sim builds
  flag-off, so the menu row and pages don't exist there. Golden hashes
  must pass **unchanged** — a diff is a gating leak to fix, never a
  regeneration. Accepted cost: the new pages get no sim coverage
  (an egg-frame injection hook for a flag-1 sim variant is a follow-up).

## State machine

| State | Predicate | Behavior |
|---|---|---|
| UNPAIRED | wildcard filter, window closed | accept-any (POC behavior) |
| PAIRING | `eggPairingActive` | scanner forced on; callback observes every magic egg; first parsed frame with `pairingActive` captures |
| PAIRED | non-wildcard filter, window closed | only the stored egg heard |

Transitions (main loop unless noted): `sensoreggRequestPair()` opens the
window (+ 120 s timeout, `kPairingTimeoutMs`); capture = persist →
filter → close; `sensoreggCancelPair()` closes; `sensoreggUnpair()` =
persist-first `""` → wildcard filter (false on SD failure, UI warns).
Threading: the two window/bench flags are `volatile` (written main loop,
read BLE task), same care as `eggSleeping`.

## Touch points

- `sensoregg_protocol.{h,cpp}` + `tests/sensoregg_protocol_test.cpp` —
  pure MAC helpers (`parseMac`, `formatMac`, `macReverse`,
  `macIsWildcard`, `macAccepts`, `kMacStrLen`, `kPairingTimeoutMs`);
  `sensoreggMacAccepted()` becomes a delegate, putting the LSB-first
  reversal under host test.
- `sensoregg.{h,ino}` — state machine, callback peer-MAC capture,
  packet-rate meter, 12 new public accessors + no-op twins.
- `settings.ino` defaults row (flag-gated), `settings.h` byte budget.
- `BirdsEye.ino` — `PAGE_PAIR_EGG = -20`, `PAGE_EGG_TEST = -21`.
- `display_pages.{h,ino}` — menu row "Egg", `displayPage_pair_egg()`
  (dual-mode), `displayPage_egg_test()`; camera-test soak comment fix.
- `display_ui.ino` — five coupling points (dispatch, insideMenu +
  menuLimit, reverseDirection, select handler, unpaired custom branch);
  camera-test enter/exit latches the egg bench mode.
- `sim/stubs/module_stubs.cpp`, `sim/sim_prototypes.h`.
- `CLAUDE.md` (page ids, subsystem 14 — also fixes two stale blocks
  still describing PW-ADV-1/14-byte-copy — settings + tuning tables),
  `CHANGELOG.md`.

## Verification

- Host: `ctest --test-dir tests/build` (MAC helper matrix, LSB-first
  pin, wildcard semantics, `"00:...:00"` parses to wildcard).
- Sim: build + golden run — hashes unchanged proves the flag gate holds.
- Arduino: compile BOTH flag arms locally (CI only builds flag-on).
- Hardware (owner checklist): pair from the egg's 30 s window; re-pair
  to a second egg while paired; unpair → accept-any; race pages
  unchanged; desk soak on EGG TEST (rate ~9-10 Hz at 111.875 ms adv);
  settings-write-failure path (SD removed) keeps the window open.

## Status / follow-ups

- Status: shipped with this plan's PR into BETA.
- Deferred: manual MAC entry page (B1 reserved); DOVEX egg-battery
  column; sim egg-frame injection + flag-1 golden variant.
