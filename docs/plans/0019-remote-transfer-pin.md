# 0019 — Remote transfer with a paired PIN

Status: **implemented, pending hardware check** (logger side). Companion plans: DovesDataViewer
`docs/plans/0031-remote-transfer-pin.md`; LapWing implements the same
handshake in `loggers/doveslogger/` (no plan folder there).

## Goal

Let the webapp (and LapWing) start a Bluetooth transfer while the logger sits
on its **main menu**, without anyone pressing Transfer → Bluetooth — gated by
the `bluetooth_pin` every logger has generated since first boot but that
nothing ever checked. Not a wake-from-sleep: System OFF has no radio, so the
logger must already be awake on the menu.

The SD card is soldered in on current hardware, so "pull the card and read
`SETTINGS.json`" is gone as a recovery path. The PIN has to be viewable on
the device, and recoverable when lost, without being shown on the transfer
screen all the time.

## Where things stood

- `bluetooth_pin` came from `random(1000, 10000)` seeded with `micros()` —
  not a strong random number.
- No code read it. The file-service characteristics are `SECMODE_OPEN` and
  pairing is Just Works with MITM off.
- `SLIST` / `SGET:bluetooth_pin` handed the PIN to whoever was connected, and
  `SRESET` regenerated it remotely. **A PIN gate is meaningless while the
  protocol reads the PIN out to an unauthenticated peer**, so redaction is
  step one.
- The only access control was standing next to the device and pressing the
  button.

## Two session modes

- **Open (local start)** — Transfer → Bluetooth on the device. Unchanged:
  every command works. This is the *pairing moment*: the app reads the PIN
  with `PINGET` and stores it.
- **Locked (remote start)** — the logger advertises 0x1820 from the main
  menu. A connected peer may only send `AUTH?`, `AUTH:<mac>` and `BATT` (DIS
  reads stay available). Everything else answers `AUTH:REQUIRED`. `AUTH:OK`
  switches the logger into the normal transfer page and code path, labelled
  as a remote session.

## Handshake (challenge-response)

```
app  → AUTH?
log  → AUTH:NONCE:<32 hex>        (locked)   | AUTH:OPEN (open)
                                              | AUTH:LOCKED:<s> | AUTH:CAMERA
app  → AUTH:<first 32 hex of HMAC-SHA256(key = PIN ascii,
                                          msg = "BEAUTH1|" + nonceHex)>
log  → AUTH:OK | AUTH:FAIL:<tries left> | AUTH:LOCKED:<s> | AUTH:NO_NONCE
       | AUTH:BUSY (RNG pool empty, or an answer already queued)
```

- The PIN never crosses the air; a recorded exchange can't be replayed
  because the nonce is single-use (consumed on any answer, right or wrong)
  and a new `AUTH?` discards the previous one.
- Nonce = 16 bytes from the SoftDevice RNG (`sd_rand_application_vector_get`).
- **Not bound to `bluetooth_name`** (changed during implementation). The
  design artifact mixed the name into the MAC, but the nonce is already
  unique to this logger and this attempt, and the advertised name can be
  truncated on air (31-byte advert), so an app could hold a correct PIN and
  still compute the wrong answer. Two correct implementations disagreeing
  buys nothing.
- Golden vector, pinned in all three implementations: PIN `4821`, nonce
  `00112233445566778899aabbccddeeff` → `4936f5dad4502101eb0f65790cca77b4`.
- Lowercase hex on the wire; the logger compares case-insensitively and in
  constant time.

### Other wire changes

| App sends | Logger replies | When |
|---|---|---|
| `PINGET` | `PIN:<digits>` / `AUTH:REQUIRED` | Open mode only |
| `SLIST` | as before, **minus `bluetooth_pin`** | All modes |
| `SGET:bluetooth_pin` | `SERR:PROTECTED` | All modes — use `PINGET` |
| `SSET:bluetooth_pin=…`, `SRESET` | as before | Open, or locked after `AUTH:OK`. PIN must be exactly 4 digits (`SERR:BAD_VALUE`) |
| (unsolicited) | `AUTH:CAMERA` | Right before the logger drops a link because the camera needs the radio |
| (unsolicited) | `AUTH:TIMEOUT` | Right before a peer that never authenticated is dropped (20 s) |

## Logger rules

- **When it advertises:** `PAGE_MAIN_MENU`, radio owner `NONE`, camera
  inactive, no race session, `remote_transfer` setting on. Leaving the menu
  stops the advert. Auto-race and the 5-minute menu idle shutdown are
  unchanged, so standby lasts exactly as long as the menu does.
- **Squatting:** a peer without `AUTH:OK` after 20 s is disconnected.
- **State:** standby holds `bleOwner = TRANSFER` with `bleActive` false —
  nothing parks, and a standby peer's disconnect never reboots. `AUTH:OK`
  promotes into the ordinary transfer page (`bleActive`, reboot on
  disconnect) with the session still locked-and-authenticated, so `PINGET`
  stays off.
- **Guessing:** 5 wrong answers lock auth for 60 s, doubling per lockout up
  to 15 min. RAM-only — a reboot clears it, which needs physical access.
- **PIN generation** moves to the hardware RNG.

## The camera always wins

One peripheral link is shared by the transfer service and the camera remote
(`bleOwner`). Remote transfer only ever gets it when the camera has no use
for it.

- "Camera active" = FSM anywhere except `kUnpaired`/`kIdle`, OR the camera
  test page open, OR `bleOwner == CAMERA`. No standby advert while active.
- Camera turns active during standby → advert stops that iteration; an
  unauthenticated peer gets `AUTH:CAMERA` and is dropped.
- Engine starts with a paired camera → RPM above 500 (the camera wake
  threshold) stops standby, or ends a **remote** transfer (`AUTH:CAMERA`,
  then the usual reboot). This fires ahead of the camera FSM's own 2 s wake
  debounce, and the standby step runs before `CAMERA_LOOP()`, so the camera
  never finds the radio taken. The transfer branch parks `loop()`, so the
  remote guard runs `TACH_LOOP()` itself.
- Remote start **never** calls `CAMERA_FORCE_RELEASE()`. Only a person at
  the logger choosing Transfer → Bluetooth bumps the camera (today's rule).
- Consequence: after a camera session the camera stays in `WATCHING` until
  shutdown, so remote transfer is unavailable until the next power-up.

## Seeing and recovering the PIN

Never on the transfer screen. Every path needs the device in hand.

- **Transfer → PIN** (new row between USB and Back; the Transfer menu now
  scrolls): rows *Show PIN*, *New PIN*, *Back*. Hold Select 3 s on Show to
  reveal for 15 s, then it hides and the digits are wiped from RAM. Hold 3 s
  on New PIN to regenerate it (persist-first, then shown). Apps forget the
  old one on their next `AUTH:FAIL`.
- **Local Bluetooth transfer:** the app re-learns it via `PINGET`.
- **USB drive:** Transfer → USB exposes `SETTINGS.json`.

## Mixed versions

- New app, old firmware: no reply to `AUTH?` in 2 s → legacy, local start only.
- Old app, new firmware: local start works; remote start answers
  `AUTH:REQUIRED`; the settings screen no longer shows the PIN.

## What the PIN does and doesn't protect

It keeps the pits out (weeks of over-the-air guessing with the lockout). It
does **not** stop a sniffer: the link is unencrypted Just Works, so data
after `AUTH:OK` is readable and one captured handshake brute-forces offline
instantly (6 digits would not change that). Real protection needs an
encrypted, bonded link (LE Secure Connections) — a separate project.

## Decisions

- **4-digit PIN** (Dove, 2026-10-07): existing loggers keep their PIN; 6
  digits does not change the sniffer case.
- **`remote_transfer` setting, default on** (`on`/`off`; anything but an
  explicit `off` means on).

## Touch points

- Pure units + host tests: `sha256` (FIPS 180-4 + RFC 4231 HMAC vectors),
  `remote_auth` (nonce/answer/lockout/timeout state machine, command gating,
  protected-key and PIN-format rules, standby/camera predicates),
  `pin_page` (the hold-to-reveal page).
- `bluetooth.ino`: AUTH dispatch, locked-mode gate, settings redaction,
  standby advert + teardown, remote session promotion.
- `settings.ino`: hardware-RNG PIN, `remote_transfer` default.
- `BirdsEye.ino`: standby step in `loop()`, remote parked-branch camera check.
- `display_ui.ino` / `display_pages.ino`: PIN page; `BirdsEye.ino`
  `transferPinPageLoop()`.
- Sim: stubs for the new BLE surface, goldens for the PIN page (the
  transfer menu's hash moved with its fourth row).
- `camera_ble.{h,ino}`: camera-active accessor (+ sim stub).
- Docs: `CLAUDE.md`, `ARCHITECTURE.md`, `CHANGELOG.md`.

## Status

- [x] Plan committed
- [x] PIN redaction, `PINGET`, write gates, hardware RNG
- [x] Auth pure units + tests
- [x] Standby advertising, locked mode, camera priority
- [x] PIN page
- [x] Docs + changelog
- [ ] Hardware check: a phone authenticating from the menu, the lockout,
  and the camera taking the radio back with the engine running
