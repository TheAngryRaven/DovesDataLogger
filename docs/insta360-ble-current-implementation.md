# Insta360 X4 BLE — What the firmware currently does

> Reference dump of the **current implementation** for debugging the two
> things that don't work on real hardware:
> 1. The **wake chirp** never wakes the camera.
> 2. The **R (remote) link** never forms — `cameraRemoteLinkUp()` stays false,
>    so the `ce82` power-off frame is never delivered.
>
> This describes *exactly what we transmit and expose today*, with byte tables
> and code locations, so it can be compared against a live sniff / reference
> implementation. Every camera-facing byte table in the code is marked
> `// X4-VERIFY(sniff)` — i.e. sourced from community projects (pchwalek,
> tsunghowu, btittelbach, arsfabula, xaionaro-go/insta360ctl), **not** yet
> confirmed against our own X4.
>
> Source files: `BirdsEye/camera_ble.ino` (BLE plumbing),
> `BirdsEye/insta360_protocol.cpp` (byte layouts),
> `BirdsEye/camera_fsm.cpp` (state machine).

---

## 0. Two independent BLE links

We run a **dual-role** SoftDevice (`Bluefruit.begin(1, 1)` = 1 peripheral +
1 central). There are **two separate physical connections**, in opposite
roles, doing different jobs. This is the single most important thing to
understand.

| | **R link — "remote"** | **C link — "control"** |
|---|---|---|
| Our BLE role | **Peripheral** (GATT **server**) | **Central** (GATT **client**) |
| Who initiates | **Camera** connects to **us** | **We** connect to the **camera** |
| Service | We host **`0xCE80`** (+ `0xD0FF`) | Camera hosts **`0xBE80`** |
| Data camera→us | `ce81` write (serial + status) | `be82` notify (record state) |
| Data us→camera | `ce82` notify (button frames) | `be81` write (start/stop/GPS/keepalive) |
| Used for | **Power off**, pairing/serial capture | **Start/stop recording**, GPS overlay |
| Status flag | `remoteLinkUp` / `cameraRemoteLinkUp()` | `controlLinkUp` / `cameraControlLinkUp()` |
| On our X4 | ❌ never connects | ✅ works |

**Recording works without the R link** because the state machine advances to
RECORDING as soon as our *scanner* sees the camera advertising `be80`
(`cameraAdvertSeen`), then drives everything over the C link. **Power-off is
the only feature that requires the R link**, and it's the one that fails.

We impersonate the **"Insta360 GPS Action Remote."** In that model the real
remote is the peripheral and the camera is the central that dials into it —
so for the R link, *we advertise and wait for the camera to connect to us.*

---

## 1. The wake "chirp"

### What we broadcast

A fixed **31-byte BLE advertising PDU** — an Apple-iBeacon masquerade carrying
the camera's 6-character serial. Built by
`insta360_protocol::buildWakeAdvert()`
(`insta360_protocol.cpp:130`), broadcast by the `kStartWakeBurst` action
(`camera_ble.ino:436`).

Raw bytes (serial shown as `S0..S5`, ASCII of the 6-char serial):

```
02 01 1A                          AD1: Flags = 0x1A (LE General Disc + BR/EDR not supported)
1B FF                             AD2: length 0x1B=27, type 0xFF (Manufacturer Specific Data)
4C 00                             Company ID 0x004C (Apple, little-endian)
02 15                             iBeacon: sub-type 0x02, length 0x15=21
09 4F 52 42 49 54                 0x09, "ORBIT"  ─┐
09 FF 0F 00                                       ├─ 16-byte iBeacon "proximity UUID"
S0 S1 S2 S3 S4 S5                 <-- serial      │   (bytes 9..24 of the PDU)
                                                 ─┘
00 00                             iBeacon Major = 0x0000
00 00                             iBeacon Minor = 0x0000   (see note)
E4 01                             trailing bytes (TX-power slot + 1 extra)
```

- The 6 serial bytes are written at **absolute PDU offset 19** (i.e. the last
  6 bytes of the 16-byte proximity UUID). Serial comes from the paired
  `camera_serial` setting.
- The `E4 01` tail does not cleanly match a 1-byte iBeacon TX-power field;
  treat the field boundaries after the UUID as **unverified** — the raw bytes
  above are authoritative, the iBeacon interpretation is best-effort.
- Company ID `0x004C` (Apple) + `02 15` is the textbook iBeacon prefix; the
  "ORBIT" ASCII inside the UUID is the Insta360-specific marker.

### How we broadcast it

`kStartWakeBurst` (`camera_ble.ino:436`):

```cpp
cameraTakeRadio();                     // bleCoreEnsureInit(); bleOwner = CAMERA;
                                       // Advertising.stop()/clearData(); ScanResponse.clearData();
scanHit = false;
buildWakeAdvert(adv, serialBytes);
Bluefruit.Advertising.setData(adv, 31);        // raw 31-byte PDU, verbatim
Bluefruit.Advertising.restartOnDisconnect(false);
Bluefruit.Advertising.setInterval(160, 160);   // 100 ms, fast == slow
Bluefruit.Advertising.start(0);                // advertise indefinitely
```

Note `setData()` overwrites the whole PDU with our raw bytes, so the advert
is exactly the 31 bytes above — no flags/name added by Bluefruit.

### Known problem / hypothesis

- **Community finding (high confidence):** a *fully powered-off* X4 has its
  BLE radio off and **cannot be woken by any advertisement**. The iBeacon wake
  only works on a camera in **standby** (screen off, radio alive) with
  **"Bluetooth Wakeup" enabled** in the camera settings.
- The genuine wake-**from-off** is a newer BLE-5.0 feature of the real GPS
  Action Remote and has **not** been replicated by any community project —
  it likely needs the real remote's full bonded handshake, not just an advert.
- **Open questions for the sniff:** Does the real remote's wake advert match
  these bytes exactly (esp. the `09 FF 0F 00`, Major/Minor, and `E4 01` tail)?
  Is the wake advert sent **non-connectably** by the real remote (ours is
  connectable — Bluefruit default)? Does the serial belong at offset 19, or
  is the byte order / placement different?

---

## 2. The R link — the GATT server we advertise (the "remote")

For the camera to connect to us, we (a) expose the remote's GATT, and (b)
advertise **connectably** with the name and service the camera looks for.

### 2a. The connectable advert we present

`kStartConnectableAdvertising` (`camera_ble.ino:468`) — this is what the
camera is supposed to find and connect to for the R link:

```cpp
cameraTakeRadio();
Bluefruit.setName("Insta360 GPS Remote");                              // exact name
Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
Bluefruit.Advertising.addService(cameraRemoteService);                 // advertises 0xCE80
Bluefruit.Advertising.addName();
Bluefruit.Advertising.restartOnDisconnect(false);
Bluefruit.Advertising.setInterval(32, 244);
Bluefruit.Advertising.setFastTimeout(30);
Bluefruit.Advertising.start(0);
```

So the connectable advert = Flags + 16-bit service-UUID list containing
`0xCE80` + Complete Local Name `"Insta360 GPS Remote"`. **This is a different
PDU from the wake chirp** — the wake chirp carries no name and is not what the
camera connects to; this named advert is.

> ⚠️ **In the Test menu, "Connect" is the only action that sends this named
> advert.** The auto-record FSM sends it too (after the 5-s wake burst — see
> §5). The old "Turn On" (pre-#74) never sent it, which is one reason the R
> link never formed on the bench.

### 2b. Service `0xCE80` (registered in `cameraBleRegisterServices()`, `camera_ble.ino:291`)

| Char | UUID | Properties | Permissions | Value / behavior |
|---|---|---|---|---|
| ce81 | `0xCE81` | WRITE, WRITE_NO_RESP | write=OPEN, read=NO_ACCESS | **Camera → us.** Callback copies to RAM only. Carries the serial frame + status frames. `maxLen=64`. |
| ce82 | `0xCE82` | NOTIFY, INDICATE | read=OPEN, write=NO_ACCESS | **Us → camera.** We `notify()` button frames here (shutter/mode/screen/**power-off**). `maxLen=20`, initial value `00`. |
| ce83 | `0xCE83` | READ | read=OPEN | Static 2 bytes `01 02` (uint16 LE `0x0201`). |

### 2c. Secondary service `0xD0FF` (the camera probes these on the real remote)

128-bit UUIDs, base `0000xxxx-3C17-D293-8E48-14FE2E4DA212` (the 16-bit is
spliced into bytes 12–13, little-endian on the wire). All `// X4-VERIFY(sniff)`.

| Char (16-bit) | Properties | Value |
|---|---|---|
| `FFD1` | WRITE / WRITE_NO_RESP | accept & ignore |
| `FFD2` | READ | `00` |
| `FFD3` | READ | `01 90 1E 30` |
| `FFD4` | READ | `01 20 00 18` |
| `FFD5` | READ | `00` |
| `FFD8` | WRITE / WRITE_NO_RESP | accept & ignore |
| `FFF1` | READ | `00` |
| `FFF2` | WRITE / WRITE_NO_RESP | accept & ignore |
| `FFE0` | READ | `00` |

**Security posture:** every characteristic above uses `SECMODE_OPEN` — **no
bonding / no encryption / no passkey** is required or offered. If the real X4
requires a bonded/encrypted link to a GPS remote, this is a prime suspect for
why it refuses to connect or drops immediately.

### 2d. What flows over the R link (once connected)

- **Camera → us on ce81:** the camera writes a **serial frame** and **status
  frames**. Parsed by `insta360_protocol::parseCe81Frame()`
  (`insta360_protocol.cpp:231`):
  - All ce81 frames start `FE EF FE`.
  - **Serial frame:** `FE EF FE 07 00 …` with the **6 ASCII serial bytes at
    the end** of the frame (min length 11). This is how the camera tells us
    its serial during pairing.
  - Anything else classified as a status frame (currently ignored).
- **Us → camera on ce82:** button frames (see §4). Power-off lives here.

### 2e. How the connection is detected in code

Peripheral connect is routed by owner in the **shared** Bluefruit callback:

```
bleConnectCallback(handle)                       // bluetooth.ino:52
  └─ if (bleOwner == BLE_OWNER_CAMERA)
        cameraBleOnConnect(handle)               // camera_ble.ino:236
           remoteConnHandle = handle;
           remoteLinkUp = true;                  // <-- this is what never happens
```

`cameraTakeRadio()` sets `bleOwner = BLE_OWNER_CAMERA` before advertising, so
this routing is armed. **We confirmed the routing is correct** — the flag
simply never gets set because the camera never issues the connection. So the
failure is on the *camera side / air side*, not our dispatch.

### Known problem / hypothesis for the R link

- **Community finding (high confidence):** the X4 only connects to a remote
  **after the user pairs it from the camera's own menu**:
  *camera → Settings → Bluetooth remote → select the device*. **Capturing the
  serial (our on-device pairing screen) is NOT the same as pairing** — that
  only teaches *us* the serial; it does not tell the *camera* to trust/connect
  to us. Until the device is selected in the camera's Bluetooth-remote list,
  the camera ignores our advert.
- **Open questions for the sniff / next investigation:**
  1. Does the camera require **bonding/encryption** on the ce80 link? (We
     offer none.)
  2. Are the **`D0FF`/`FFDx` values** correct? A wrong value the camera reads
     during connection setup could make it reject the remote. `insta360ctl`'s
     `pkg/remote/server.go` / `pkg/protocol/commands_remote.go` are the files
     to pull for the real values.
  3. Does the camera expect a specific **scan-response** payload, or extra AD
     fields (TX power, appearance, 128-bit service UUID) we're not sending?
  4. Does the camera connect to the **named advert** (§2a) or does it only
     react to the **wake chirp** (§1) with the serial? (We stop the connectable
     advert once we see the camera's be80 — see §5 — which may kill a
     half-formed R link.)

---

## 3. The C link — control (this one works)

We are **central**; we scan for and connect to the camera's own `0xBE80`.

### Scanner (`cameraBleRegisterServices()`, `camera_ble.ino:344`)

```cpp
Bluefruit.Scanner.setRxCallback(cameraScanCallback);
Bluefruit.Scanner.restartOnDisconnect(false);
Bluefruit.Scanner.setInterval(160, 80);   // 100 ms interval / 50 ms window
Bluefruit.Scanner.useActiveScan(true);
Bluefruit.Scanner.filterUuid(0xBE80);      // only be80 advertisers reach the callback
```

`kStartControlConnect` (`camera_ble.ino:491`) does `Scanner.stop(); Scanner.start(0);`.
On a hit, `cameraScanCallback` (`camera_ble.ino:223`) sets `scanHit = true` and
calls `Bluefruit.Central.connect(report)`.

### Central connect (`cameraCentralConnectCallback`, `camera_ble.ino:190`)

Discovers `be80` → `be81` → `be82`, enables `be82` notifications, sets
`controlLinkUp = true`. Client chars: `cameraBe81Char` (`0xBE81`, us→camera
write) and `cameraBe82Char` (`0xBE82`, camera→us notify).

### be81 control-frame format (`insta360_protocol.cpp:56`)

16-byte header + payload:

```
bytes 0-3    total frame length, u32 LE (includes the length field)
bytes 4-6    04 00 00
byte  7      command
bytes 8-9    00 02
bytes 10-11  message counter, u16 LE (starts 0x0200, ++ per command)
bytes 12-15  00 80 00 00
byte  16+    payload
```

| Frame | Cmd | Payload | Total len |
|---|---|---|---|
| Start video | `0x04` | `08 01` | 18 |
| Stop video | `0x05` | `10 01` | 18 |
| GPS inject | `0x35` | `0A 35` + 53-byte protobuf-ish body | 71 |
| Keep-alive | — | literal `07 00 00 00 05 00 00` | 7 |

Camera→us record state is parsed by `parseBe82RecordState()`
(`insta360_protocol.cpp:251`): valid frames carry `00 00 04 00 00` at bytes
2–6, code `0x10` at byte 7 = record-state report, state at byte 17.

**These all work on our X4** — good corroboration that the be81 header format
and the start/stop/GPS/keepalive bytes are correct.

---

## 4. Power-off (and the other buttons) — `ce82`, over the R link

Button frames, built by `buildButtonFrame()` (`insta360_protocol.cpp:27`),
sent via `cameraCe82Char.notify()` in `cameraSendPowerOff()`
(`camera_ble.ino:397`). **Guarded by `if (!remoteLinkUp) return;`** — this is
why power-off is a silent no-op today.

Frame layout (9 bytes): `FC EF FE 86 00 03 01 <button> <press>`

| Action | button | press |
|---|---|---|
| Screen toggle | `0x00` | `0x00` |
| Mode cycle | `0x01` | `0x00` |
| Shutter toggle | `0x02` | `0x00` |
| **Power off** | `0x00` | `0x03` (3-second hold) |

**Community finding (high confidence):** there is **no power-off command over
the `be80` control (C) link** in any known reference implementation —
`xaionaro-go/insta360ctl` has a speculative `0x37` but its own author flags it
as an untested guess (officially `0x37` = `GET_SYNC_CAPTURE_MODE`). So
power-off genuinely depends on the R link; the fix is to get the R link up,
not to move power-off to the C link.

---

## 5. How the auto-record FSM sequences the links (`camera_fsm.cpp`)

For reference — the state machine that runs a real session (the Test menu
bypasses this and fires actions directly):

```
IDLE ──(RPM>500 held 2s)──> WAKING
   WAKING:  emit kStartWakeBurst (§1)
            after 5 s (kWakeBurstMs)  -> emit kStartConnectableAdvertising (§2a)
            on (remoteConnected || cameraAdvertSeen) -> CONNECTING, kStopAdvertising
   CONNECTING: emit kStartControlConnect (§3, scan+central connect)
               on controlConnected -> AWAIT_GPS
   AWAIT_GPS -> RECORDING (start-video) -> ... -> COOLDOWN -> POWERING_OFF (ce82)
```

Two things worth noting for the R-link bug:

1. WAKING advances to CONNECTING on **`cameraAdvertSeen` alone** (our scanner
   spotted the camera's be80) — it does **not** require the R link
   (`remoteConnected`) to ever form. So a real session records fine with the R
   link permanently down.
2. On that transition it emits **`kStopAdvertising`** — so we **stop presenting
   the connectable remote advert** the moment we see the camera's be80. If the
   camera hadn't connected to our ce80 yet, that window is closed. Then at
   COOLDOWN the `ce82` power-off has no R link and does nothing — the same bug
   the bench Test menu exposes.

---

## 6. Where everything lives (quick index)

| What | File:line |
|---|---|
| Wake advert bytes | `insta360_protocol.cpp:130` (`buildWakeAdvert`) |
| Wake broadcast | `camera_ble.ino:436` (`kStartWakeBurst`) |
| Connectable "GPS Remote" advert | `camera_ble.ino:468` (`kStartConnectableAdvertising`) |
| ce80 / D0FF GATT server | `camera_ble.ino:291` (`cameraBleRegisterServices`) |
| ce81 parse (serial capture) | `insta360_protocol.cpp:231` (`parseCe81Frame`) |
| ce82 button frames (power-off) | `insta360_protocol.cpp:27` + `camera_ble.ino:397` |
| Peripheral connect → `remoteLinkUp` | `bluetooth.ino:52` → `camera_ble.ino:236` |
| Scanner + central connect (C link) | `camera_ble.ino:190`, `:223`, `:344` |
| be81 control frames | `insta360_protocol.cpp:56`+ |
| FSM sequencing | `camera_fsm.cpp` (`stepWaking` ~:118, `stepConnecting` ~:155) |
| Test-menu actions | `camera_ble.ino` (`cameraTestWake` / `cameraTestConnect` / …) |

---

## 7. Best external references (for the follow-up investigation)

- `xaionaro-go/insta360ctl` — Go, actively targets X3/**X4**/X5; command enum
  from `libOne.so`, protocol docs, and a remote-server implementation:
  - command enum: `pkg/protocol/messagecode/messagecode.go`
  - BLE protocol / topology / wake: `doc/ble_protocol.md`, `doc/command_reference.md`
  - **remote (peripheral) server — for the D0FF/FFDx real values:**
    `pkg/remote/server.go`, `pkg/protocol/commands_remote.go`
- pchwalek — standby-only wake, manufacturer-data format:
  `github.com/pchwalek/insta360_ble_esp32` + Medium write-up.
- tsunghowu — **X4-specific** peripheral / ce82 remote model:
  `github.com/tsunghowu/insta360_ble_rc_rpi_pico_w`.
- Hackaday X3 BLE remote command table (0x20 reboot, etc.).
- Official pairing flow (the in-camera menu step): Insta360 online manual,
  *Connect to remote*.

### Top hypotheses to test with a sniff, in priority order

1. **In-camera-menu pairing is mandatory** — pair the logger from the X4's
   *Settings → Bluetooth remote* list, then watch whether the camera connects
   to our ce80 and `R` flips to `UP`. (Cheapest test; do this first.)
2. **Bonding/encryption required** on ce80 — we currently offer `SECMODE_OPEN`
   only.
3. **Wrong `D0FF`/`FFDx` values or missing characteristics** cause the camera
   to reject the remote — compare against `insta360ctl` `pkg/remote/server.go`.
4. **Wake advert byte mismatch** (Major/Minor/tail/`09 FF 0F 00`, connectable
   vs non-connectable) — compare our §1 bytes against a capture of the genuine
   remote's advert, and confirm the camera must be in *standby*, not off.
