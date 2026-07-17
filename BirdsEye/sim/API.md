# BirdsEye Simulator — WASM API (contract v1)

The canonical, versioned statement of the browser module's surface
(supersedes the draft in the original handoff spec; deltas from that
draft are listed at the bottom). `simApiVersion` is **1**.

## Artifacts (vendored into `DovesDataViewer/public/sim/`)

| File | What |
|---|---|
| `birdseye-sim.mjs` | Public module — hand-written ESM wrapper, import THIS |
| `birdseye-sim-core.mjs` | Emscripten-emitted factory (internal; loaded by the wrapper) |
| `birdseye-sim-core.wasm` | The firmware, compiled (fetched by the core module relative to its own URL) |
| `version.json` | Build provenance: `firmwareSha`, `buildDate`, `simApiVersion`, `dovesLapTimerSha`, `adafruitGfx`, `adafruitSh110x` |
| `test.html` | Standalone harness (serve the folder, open it — boots the firmware, plays a dovex) |

All same-origin; the wasm is `instantiateStreaming`-compatible when the
host serves `application/wasm` (Cloudflare Pages does).

## Usage

```js
import createBirdsEyeSim from '/sim/birdseye-sim.mjs';

const sim = await createBirdsEyeSim();

// ---- lifecycle ----
sim.init();                 // runs firmware setup(); boots to the GPS status page
await sim.reset();          // TRUE fresh boot: re-instantiates the wasm module
                            // (statics can't re-run in place). ASYNC — await it.
                            // Scrub-backward = await reset(), init(), fast-replay.

// ---- time ----
// The sim has NO internal clock; the host owns virtual time.
sim.stepMillis(deltaMs);    // advance + run loop() in ~4 ms quanta.
                            // Returns loop() iteration count, or -1 if the
                            // firmware requested a reboot (see resetRequested()).

// ---- inputs ----
sim.buttonDown(idx);        // 0=Left, 1=Select, 2=Right (real debounce applies:
sim.buttonUp(idx);          //  hold >= ~10 ms of virtual time before releasing)
sim.injectPvt(pvtJson);     // JSON string (schema below); max once per stepMillis
                            // batch; returns false on a parse error
sim.setRpm(rpm);            // synthesizes tach pulses through the real ISR; 0 = off

// ---- outputs ----
sim.getFramebuffer();       // Uint8Array VIEW, 1024 bytes; layout:
                            //   bit = buf[x + (y>>3)*128] >> (y&7) & 1
                            // Re-take after memory growth (each call returns a
                            // fresh view; don't cache long-term).
sim.getFrameHash();         // FNV-1a 32 of those 1024 bytes (redraw dirty-check)
sim.getStateJson();         // object: { page, raceActive, lapCount, bestLapMs,
                            //   lastLapMs, currentLapMs, gpsFix, sats, rpm,
                            //   loggingActive, trackDetected, courseName, millis }
sim.getVersion();           // object: version.json fields
sim.resetRequested();       // true once the firmware called NVIC_SystemReset
sim.readFile(path);         // Uint8Array copy from the in-memory VFS, or null
sim.listFiles();            // string[] of VFS paths (find this run's .dovex)
```

## `injectPvt` JSON schema

Dovex row fields + `fix` + accel (missing numerics default 0):

```json
{ "timestamp": 1775319260240, "lat": 28.4108402, "lng": -81.3793428,
  "sats": 7, "hdop": 2.9, "speed_mph": 0.13, "altitude_m": 34.48,
  "heading_deg": 243.92, "h_acc_m": 0.35, "fix": true,
  "accelX": -0.199, "accelY": -0.451, "accelZ": -1.001 }
```

`fix:false` sends a no-fix frame (fixType 0, no valid bits) — use ~3 s
of these as the boot pre-roll so viewers see the real acquisition UX.

## Deltas from the original handoff-spec draft

1. **`reset()` is async** and implemented as module re-instantiation —
   the only way to truly re-boot statics. Await it before headless
   re-replays (a full 15-min session fast-replays in well under a
   second, so scrub budgets hold).
2. **Four artifacts, not three**: the public `birdseye-sim.mjs` is a
   hand-written wrapper over the Emscripten `birdseye-sim-core.mjs` —
   this is what makes (1) possible while keeping one stable import.
3. `getStateJson()` carries three extra fields beyond the draft
   (`trackDetected`, `courseName`, `millis`) and `getVersion()` carries
   the display-lib + DovesLapTimer provenance.
4. `listFiles()` and `resetRequested()` are additions (no draft
   equivalent).
