///////////////////////////////////////////
// smoke.mjs — node smoke test for the WASM artifact (CI gate).
//
// Proves the vendorable module actually works outside a browser: boots
// the firmware, walks off the GPS status page, checks the framebuffer /
// state / version / VFS surfaces, injects a fix + RPM, and verifies two
// independent instances produce identical frame hashes (determinism —
// the property shareable replays depend on).
//
//   node dist/smoke.mjs
///////////////////////////////////////////

import createBirdsEyeSim from './birdseye-sim.mjs';

let failures = 0;
function check(ok, what) {
  console.log(`${ok ? 'PASS' : 'FAIL'} ${what}`);
  if (!ok) failures++;
}

function bootToMenu(sim) {
  sim.init();
  sim.stepMillis(2000);
  sim.buttonDown(1);
  sim.stepMillis(120);
  sim.buttonUp(1);
  sim.stepMillis(500);
}

const sim = await createBirdsEyeSim();

sim.init();
sim.stepMillis(2000);
let st = sim.getStateJson();
check(st.page === 900, `boot lands on GPS status page (page=${st.page})`);

sim.buttonDown(1);
sim.stepMillis(120);
sim.buttonUp(1);
sim.stepMillis(500);
st = sim.getStateJson();
check(st.page === -1, `Select skips to main menu (page=${st.page})`);

const boot = sim.getBootFrames();
check(boot.length >= 2, `boot sequence captured (${boot.length} keyframes)`);
check(boot.every((f) => f.pixels.length === 1024), 'boot frames are 1024 bytes');
check(boot.some((f) => f.pixels.some((b) => b !== 0)), 'boot splash has lit pixels');

const fb = sim.getFramebuffer();
check(fb.length === 1024, 'framebuffer is 1024 bytes');
check(fb.some((b) => b !== 0), 'framebuffer has lit pixels');
const menuHash = sim.getFrameHash();
check(menuHash !== 0, `frame hash nonzero (${menuHash.toString(16)})`);

const ver = sim.getVersion();
check(ver.simApiVersion === 1, `simApiVersion 1 (${JSON.stringify(ver)})`);
check(typeof ver.firmwareSha === 'string' && ver.firmwareSha.length > 0,
      'firmwareSha present');

const settings = sim.readFile('/SETTINGS.json');
check(settings !== null && settings.length > 0, 'readFile(/SETTINGS.json)');
check(sim.readFile('/NOPE.bin') === null, 'readFile miss returns null');
check(sim.listFiles().includes('/TRACKS/OKC.json'), 'VFS lists OKC track');

// A fix frame + engine: state must reflect both.
const ok = sim.injectPvt(JSON.stringify({
  timestamp: 1773500400000, lat: 28.41271, lng: -81.37965, sats: 12,
  hdop: 0.8, speed_mph: 0, altitude_m: 25, heading_deg: 0, h_acc_m: 1.2,
  fix: true, accelX: 0, accelY: 0, accelZ: 1,
}));
check(ok, 'injectPvt(json) accepted');
sim.setRpm(6000);
sim.stepMillis(1000);
st = sim.getStateJson();
check(st.gpsFix === true, 'fix visible in state');
check(st.sats === 12, `sats plumbed (${st.sats})`);
check(st.rpm > 5000, `Kalman RPM spun up (${st.rpm})`);

// Determinism: a second, completely fresh instance replaying the same
// steps must land on the identical frame hash.
const sim2 = await createBirdsEyeSim();
bootToMenu(sim2);
const sim3 = await createBirdsEyeSim();
bootToMenu(sim3);
check(sim2.getFrameHash() === sim3.getFrameHash(),
      `two instances, identical hashes (${sim2.getFrameHash().toString(16)})`);

// reset(): re-instantiates for a true fresh boot.
await sim2.reset();
sim2.stepMillis(2000);
check(sim2.getStateJson().page === 900, 'reset() boots fresh to GPS page');

console.log(failures ? `--- smoke FAILED (${failures}) ---` : '--- smoke ok ---');
process.exit(failures ? 1 : 0);
