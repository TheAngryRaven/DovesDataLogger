#pragma once

/*
 * camera_fsm — Insta360 X4 auto-record state machine (pure logic).
 *
 * REMOTE-EMULATION model: this device IS the Insta360 GPS Remote. We are
 * a BLE peripheral; the camera connects to US (camera = central) and
 * subscribes to our ce82 button characteristic. All control — record
 * (shutter toggle), power-off — is a ce82 notification, exactly as the
 * physical remote works. We never act as central and never touch the
 * camera's own be80 service. (In-camera GPS overlay is NOT part of this
 * model — its transport is unidentified; GPS still logs to SD. See the
 * subsystem doc.)
 *
 * Lifecycle off engine RPM + GPS ground speed + BLE link state: wake the
 * paired camera when the engine starts, start recording once it connects
 * and subscribes (and GPS locks), stop on sustained stationary+engine-off
 * (or manual session end), and power the camera off after a cooldown.
 *
 * The FSM is a read-only consumer of telemetry: the caller builds an
 * Inputs snapshot each loop iteration and executes the single Action
 * step() returns. All timing lives here so the entire temporal behavior
 * is host-testable. Pure logic — no Arduino headers — exercised by
 * tests/camera_fsm_test.cpp. Board-portable core shared with the nRF54
 * ("Falcon") target: nothing here may #ifdef on the platform.
 */

#include <stdint.h>

namespace camera_fsm {

// ---- Camera auto-record tunables (single-point edits) ----
// The race-mode lifecycle is deliberately RPM-driven and simple: wake on
// engine start, record once RPM has held for a few seconds, stop after the
// engine has been off a while (which also ends the log session), then WATCH —
// stay connected, ready to re-record if the engine restarts (stall recovery).
// The camera powers off ONLY on device sleep.
constexpr uint32_t kRecordStartDelayMs  = 5000;    // RPM held at/above the record threshold this long -> start recording
constexpr uint32_t kStopRecordDelayMs   = 30000;   // RPM below OFF this long -> stop recording + end session

// ---- Trigger thresholds (hysteresis band between OFF and ON) ----
// Record-start gate — deliberately FAR above the wake threshold. A pull-start
// registers real ignition pulses (magneto fires while the cord is pulled), so
// cranking blips clear kRpmOnThreshold and can wake the camera; during a
// failed first start of the day they also started a recording (2026-07-19
// field incident). RPM must hold at/above this CONTINUOUSLY for
// kRecordStartDelayMs — any dip below restarts the clock — so only a
// genuinely running engine records. Wake/stop keep the 500/300 band.
constexpr int32_t  kRecordRpmThreshold    = 1500;
constexpr int32_t  kRpmOnThreshold        = 500;   // above = engine running (matches autoRaceModeCheck)
constexpr int32_t  kRpmOffThreshold       = 300;   // below = engine off
constexpr uint32_t kRpmOnDebounceMs       = 2000;  // RPM must hold above ON threshold this long to wake
constexpr uint32_t kRpmGoneAbortMs        = 2000;  // RPM below OFF threshold this long aborts WAKING

// ---- Timeouts / retries ----
// WAKING advertises the wake payload and waits for the camera to CONNECT
// TO US for this window per attempt (the camera's sleep-scan is sparse),
// retrying up to kConnectRetries times before giving up to IDLE.
constexpr uint32_t kConnectTimeoutMs   = 20000;  // per wake attempt: wait for the camera to connect
constexpr uint8_t  kConnectRetries     = 3;      // wake attempts before giving up back to IDLE
// After the camera connects it must subscribe to ce82 before we can send
// any button frame. If it never does, drop and re-advertise — bounded by
// kSubscribeRetries so a camera that connects but never subscribes (stale
// bond / encryption never comes up) eventually gives up to IDLE instead of
// looping connect->timeout->re-advertise forever.
constexpr uint32_t kSubscribeTimeoutMs = 10000;
constexpr uint8_t  kSubscribeRetries   = 3;      // connect-but-never-subscribe cycles before IDLE
constexpr uint32_t kPairingTimeoutMs   = 120000; // pairing screen gives up after 2 min
// RECORDING: if the camera's observed state (0x10 timer) reports IDLE this
// long while we believe we're recording, the start shutter never landed — so
// re-assert it once. Never fires without a fresh observation (kUnknown).
constexpr uint32_t kRecordConfirmMs    = 2500;

enum class State : uint8_t {
  kUnpaired,     // no camera bound; only exit is pairing
  kIdle,         // paired, camera off, armed — waiting for engine start
  kWaking,       // advertising the wake payload, waiting for the camera to connect to us
  kAwaitReady,   // camera connected; waiting for the ce82 subscription
  kRecording,    // recording (shutter sent); monitoring the engine-off stop condition
  kWatching,     // camera ON + connected, not recording — re-records on RPM, powers off only on sleep
  kPairing,      // connectable advertising, waiting to capture a camera serial
};

// Camera record state OBSERVED by the glue from the camera's ce81 0x10
// display-string frames (see insta360_protocol::parseRecordingState). The
// FSM reconciles its recordingActive belief against this so a lost or failed
// shutter is corrected instead of inverted (the shutter is a stateful
// toggle, so a wrong belief flips the camera the wrong way). kUnknown = no
// fresh observation this step (link just up, or the last 0x10 is stale).
enum class RecordObs : uint8_t { kUnknown, kIdle, kRecording };

// step() returns at most one action per call; the glue executes it.
enum class Action : uint8_t {
  kNone,
  kStartWakeBurst,              // CONNECTABLE wake advert (mfg-data serial + remote identity);
                                // the woken camera connects back to it
  kStartConnectableAdvertising, // remote-identity advert (pairing / re-advertise after a drop)
  kStopAdvertising,
  kSendShutter,                 // ce82 shutter button — TOGGLES recording (start or stop)
  kSendPowerOff,                // ce82 power-button 3s hold — NOT emitted by the auto FSM (power-off is
                                // glue-driven on sleep); retained for the bench Test menu's manual path
  kDisconnect,                  // drop the camera link
};

// Telemetry + event snapshot, built fresh by the caller each step().
// The one-shot event flags must be true for exactly one step() call.
struct Inputs {
  uint32_t nowMs = 0;             // millis()
  int32_t  rpm = 0;               // tachLastReported — the ONLY driver of record/stop now
  bool     remoteConnected = false;   // camera connected to our ce80 remote service (THE link)
  bool     ce82Subscribed = false;    // camera wrote our ce82 CCCD — button frames now deliverable
  RecordObs recordObserved = RecordObs::kUnknown;  // camera-reported record state (0x10 timer)
  // one-shot events
  bool sessionEndRequested = false;   // datalogger session ended (manual stop / auto-idle / sleep)
  bool pairRequested = false;         // UI: enter pairing
  bool pairCancelRequested = false;   // UI: leave pairing screen
  bool pairSerialCaptured = false;    // glue: serial captured AND persisted
  bool unpairRequested = false;       // UI: clear the stored serial
  bool forceIdleRequested = false;    // transfer takeover / sleep entry
};

struct Fsm {
  State   state = State::kUnpaired;
  bool    serialPresent = false;
  bool    recordingActive = false;  // we believe the camera is recording (shutter toggle is stateful,
                                     // so track it — don't blind-toggle on a reconnect mid-session)
  // timers: 0 == not running (timestamps are seeded non-zero)
  uint32_t rpmOnSince = 0;          // IDLE: rpm above ON threshold since
  uint32_t rpmGoneSince = 0;        // WAKING: rpm below OFF threshold since
  uint32_t wakeAttemptStarted = 0;  // WAKING: current attempt window start
  uint8_t  wakeAttemptsUsed = 0;
  uint8_t  subscribeAttemptsUsed = 0;  // AwaitReady: connect-but-never-subscribe cycles (persists across re-wake)
  uint32_t connectedSince = 0;      // kAwaitReady: camera-connected since (subscription-wait timer)
  uint32_t recordArmSince = 0;      // WAKING/AWAIT/WATCHING: rpm continuously above ON since (record-start timer)
  uint32_t stopCondSince = 0;       // RECORDING: engine-off hold start
  uint32_t recordIdleSince = 0;     // RECORDING: camera-reports-idle-while-believed-recording since
  bool     recordRetryUsed = false; // RECORDING: single re-assert-shutter latch (re-armed on confirmed recording)
  uint32_t pairingSince = 0;
  State    pairingReturnState = State::kUnpaired;  // where pairing cancel/timeout goes back to
  bool     entryPending = false;    // current state's entry action not yet emitted
};

// Reset to the boot state: kIdle when a serial is stored, else kUnpaired.
void init(Fsm& f, bool serialPresent);

// Advance the machine one tick. Mutates state/timers; returns at most
// one action for the caller to execute. Call at main-loop rate.
Action step(Fsm& f, const Inputs& in);

// Short display label for a state (e.g. "IDLE", "RECORDING") — for the
// status/pairing pages. Never returns null.
const char* stateName(State s);

}  // namespace camera_fsm
