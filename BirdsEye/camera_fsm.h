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
constexpr uint32_t kStopRecordDelayMs   = 60000;   // stationary AND engine-off sustained before stop
constexpr uint32_t kPowerOffDelayMs     = 180000;  // camera stays on this long after recording stops

// ---- Trigger thresholds (hysteresis band between OFF and ON) ----
constexpr int32_t  kRpmOnThreshold        = 500;   // above = engine running (matches autoRaceModeCheck)
constexpr int32_t  kRpmOffThreshold       = 300;   // below = engine off
// Spec says 2.0 km/h; this codebase standardizes on mph and 2.0 mph is
// the exact "stationary" threshold the log auto-idle check already uses,
// so camera-stop and log-idle share one notion of "stopped".
constexpr float    kSpeedStopThresholdMph = 2.0f;
constexpr uint32_t kRpmOnDebounceMs       = 2000;  // RPM must hold above ON threshold this long
constexpr uint32_t kRpmGoneAbortMs        = 2000;  // RPM below OFF threshold this long aborts WAKING

// ---- Timeouts / retries ----
// WAKING advertises the wake payload and waits for the camera to CONNECT
// TO US for this window per attempt (the camera's sleep-scan is sparse),
// retrying up to kConnectRetries times before giving up to IDLE.
constexpr uint32_t kConnectTimeoutMs   = 20000;  // per wake attempt: wait for the camera to connect
constexpr uint8_t  kConnectRetries     = 3;      // wake attempts before giving up back to IDLE
// After the camera connects it must subscribe to ce82 before we can send
// any button frame. If it never does, drop and re-advertise.
constexpr uint32_t kSubscribeTimeoutMs = 10000;
constexpr uint32_t kGpsLockTimeoutMs   = 30000;  // max wait for GPS lock before recording anyway
constexpr uint32_t kPairingTimeoutMs   = 120000; // pairing screen gives up after 2 min
constexpr uint32_t kPowerOffLingerMs   = 5000;   // POWERING_OFF: wait for link drop, then force it

// ---- Behaviour flags ----
constexpr bool kStopOnEither              = false;  // DECIDED: require speed==0 AND rpm==0 to stop
constexpr bool kRecordWithoutGpsOnTimeout = true;   // no lock within timeout -> record anyway
constexpr bool kAutoResumeFromCooldown    = false;  // DEFERRED v1: no auto re-record during cooldown

enum class State : uint8_t {
  kUnpaired,     // no camera bound; only exit is pairing
  kIdle,         // paired, camera off, armed — waiting for engine start
  kWaking,       // advertising the wake payload, waiting for the camera to connect to us
  kAwaitReady,   // camera connected; waiting for ce82 subscription (+ GPS fix) before recording
  kRecording,    // recording (shutter sent); monitoring the stop condition
  kCooldown,     // recording stopped, camera left ON, counting down power-off
  kPoweringOff,  // power-off streaming; waiting for the camera to drop the link
  kPairing,      // connectable advertising, waiting to capture a camera serial
};

// step() returns at most one action per call; the glue executes it.
enum class Action : uint8_t {
  kNone,
  kStartWakeBurst,              // CONNECTABLE wake advert (mfg-data serial + remote identity);
                                // the woken camera connects back to it
  kStartConnectableAdvertising, // remote-identity advert (pairing / re-advertise after a drop)
  kStopAdvertising,
  kSendShutter,                 // ce82 shutter button — TOGGLES recording (start or stop)
  kSendPowerOff,                // ce82 power-button 3s hold (streamed by the glue)
  kDisconnect,                  // drop the camera link
};

// Telemetry + event snapshot, built fresh by the caller each step().
// The one-shot event flags must be true for exactly one step() call.
struct Inputs {
  uint32_t nowMs = 0;             // millis()
  int32_t  rpm = 0;               // tachLastReported
  float    speedMph = 0.0f;       // gps_speed_mph
  bool     gpsFixValid = false;   // gpsData.fix
  bool     remoteConnected = false;   // camera connected to our ce80 remote service (THE link)
  bool     ce82Subscribed = false;    // camera wrote our ce82 CCCD — button frames now deliverable
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
  uint32_t connectedSince = 0;      // kAwaitReady: camera-connected since (subscription-wait timer)
  uint32_t awaitGpsSince = 0;       // kAwaitReady: waiting-for-fix since
  uint32_t stopCondSince = 0;       // RECORDING: stop-condition hold start
  uint32_t cooldownSince = 0;
  uint32_t powerOffSentAt = 0;
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
