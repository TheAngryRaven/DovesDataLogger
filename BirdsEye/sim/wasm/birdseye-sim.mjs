///////////////////////////////////////////
// birdseye-sim.mjs — the public simulator module (API contract v1).
//
// Hand-written ESM wrapper around the Emscripten-emitted core
// (birdseye-sim-core.mjs + birdseye-sim-core.wasm, same directory).
// Vendored VERBATIM alongside the core artifacts — keep it dependency-
// free and boring.
//
//   import createBirdsEyeSim from './birdseye-sim.mjs';
//   const sim = await createBirdsEyeSim();
//   sim.init();
//   sim.stepMillis(16.67);
//   const fb = sim.getFramebuffer();   // Uint8Array view, 1024 bytes
//
// reset() is ASYNC (returns a Promise): a true fresh boot requires
// re-instantiating the wasm module (statics cannot be re-run in
// place), so `await sim.reset()` before headless re-replays. Every
// other method is synchronous. See API.md in the firmware repo.
///////////////////////////////////////////

import createCore from './birdseye-sim-core.mjs';

export default async function createBirdsEyeSim(coreOptions = {}) {
  let core = null;
  let c = null; // cwrap'd calls

  async function instantiate() {
    core = await createCore(coreOptions);
    c = {
      init: core.cwrap('simw_init', null, []),
      stepMillis: core.cwrap('simw_step_millis', 'number', ['number']),
      buttonDown: core.cwrap('simw_button_down', null, ['number']),
      buttonUp: core.cwrap('simw_button_up', null, ['number']),
      injectPvt: core.cwrap('simw_inject_pvt', 'number', ['string']),
      setRpm: core.cwrap('simw_set_rpm', null, ['number']),
      framebuffer: core.cwrap('simw_framebuffer', 'number', []),
      frameHash: core.cwrap('simw_frame_hash', 'number', []),
      resetRequested: core.cwrap('simw_reset_requested', 'number', []),
      stateJson: core.cwrap('simw_state_json', 'string', []),
      versionJson: core.cwrap('simw_version_json', 'string', []),
      readFile: core.cwrap('simw_read_file', 'number',
                           ['string', 'number']),
      listFiles: core.cwrap('simw_list_files', 'string', []),
      free: core.cwrap('simw_free', null, ['number']),
    };
  }

  await instantiate();

  const sim = {
    // ---- lifecycle ----
    init() { c.init(); },
    async reset() { await instantiate(); c.init(); },

    // ---- time ----
    stepMillis(deltaMs) { return c.stepMillis(deltaMs); },

    // ---- inputs ----
    buttonDown(idx) { c.buttonDown(idx); },
    buttonUp(idx) { c.buttonUp(idx); },
    injectPvt(pvtJson) { return c.injectPvt(pvtJson) === 1; },
    setRpm(rpm) { c.setRpm(rpm); },

    // ---- outputs ----
    // Fresh view each call: memory growth can detach older views.
    getFramebuffer() {
      const ptr = c.framebuffer();
      return new Uint8Array(core.HEAPU8.buffer, ptr, 1024);
    },
    getFrameHash() { return c.frameHash() >>> 0; },
    getStateJson() { return JSON.parse(c.stateJson()); },
    getVersion() { return JSON.parse(c.versionJson()); },
    resetRequested() { return c.resetRequested() === 1; },
    listFiles() { return JSON.parse(c.listFiles()); },
    readFile(path) {
      const lenPtr = core._malloc(4);
      const ptr = c.readFile(path, lenPtr);
      const len = core.getValue(lenPtr, 'i32');
      core._free(lenPtr);
      if (!ptr) return null;
      const bytes = new Uint8Array(len);
      bytes.set(core.HEAPU8.subarray(ptr, ptr + len));
      c.free(ptr);
      return bytes;
    },
  };

  return sim;
}
