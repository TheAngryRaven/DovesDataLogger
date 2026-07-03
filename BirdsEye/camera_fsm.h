#pragma once

/*
 * camera_fsm — Insta360 X4 auto-record state machine (pure logic).
 *
 * Drives the camera session lifecycle off engine RPM, GPS ground speed,
 * and BLE link state: wake the paired camera when the engine starts,
 * start recording once GPS locks, feed GPS at 1 Hz while recording,
 * stop on sustained stationary+engine-off (or manual session end), and
 * power the camera off after a cooldown.
 *
 * The FSM is a read-only consumer of telemetry: the caller builds an
 * Inputs snapshot each loop iteration and executes the single Action
 * step() returns. All timing (debounce, retries, timeouts, decimation)
 * lives here so the entire temporal behavior is host-testable. Pure
 * logic — no Arduino headers — exercised by tests/camera_fsm_test.cpp.
 * This unit is also the board-portable core shared with the nRF54
 * ("Falcon") target: nothing in here may #ifdef on the platform.
 */

#include <stdint.h>

namespace camera_fsm {

// ---- Camera auto-record tunables (single-point edits) ----
constexpr uint32_t kStopRecordDelayMs   = 60000;   // stationary AND engine-off sustained before stop
constexpr uint32_t kPowerOffDelayMs     = 180000;  // camera stays on this long after recording stops
constexpr uint32_t kGpsFeedIntervalMs   = 1000;    // GPS push to camera overlay = 1 Hz
constexpr uint32_t kKeepAliveIntervalMs = 2000;    // be80 control-link keep-alive cadence

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
constexpr uint32_t kWakeBurstMs             = 5000;   // wake mfg-data advert burst, then connectable advert
constexpr uint32_t kConnectTimeoutMs        = 20000;  // per wake attempt: camera must connect within this
constexpr uint8_t  kConnectRetries          = 3;      // wake attempts before giving up back to IDLE
constexpr uint32_t kControlConnectTimeoutMs = 15000;  // per central connect attempt to the camera's be80
constexpr uint8_t  kControlConnectRetries   = 3;
constexpr uint32_t kGpsLockTimeoutMs        = 30000;  // max wait for GPS lock before recording anyway
constexpr uint32_t kPairingTimeoutMs        = 120000; // pairing screen gives up after 2 min
constexpr uint32_t kPowerOffLingerMs        = 5000;   // POWERING_OFF: wait for link drop, then force it

// ---- Behaviour flags ----
constexpr bool kStopOnEither             = false;  // DECIDED: require speed==0 AND rpm==0 to stop
constexpr bool kRecordWithoutGpsOnTimeout = true;  // no lock within timeout -> record anyway
constexpr bool kAutoResumeFromCooldown   = false;  // DEFERRED v1: no auto re-record during cooldown

enum class State : uint8_t {
  kUnpaired,     // no camera bound; only exit is pairing
  kIdle,         // paired, camera off, armed — waiting for engine start
  kWaking,       // advertising wake payload, waiting for the camera to connect
  kConnecting,   // camera awake; connecting centrally to its be80 control service
  kAwaitGps,     // control link up; waiting for a valid GPS fix
  kRecording,    // recording; feeding GPS @ 1 Hz; monitoring stop condition
  kCooldown,     // recording stopped, camera left ON, counting down power-off
  kPoweringOff,  // power-off sent; waiting for the links to drop
  kPairing,      // connectable advertising, waiting to capture a camera serial
};

// step() returns at most one action per call; the glue executes it.
enum class Action : uint8_t {
  kNone,
  kStartWakeBurst,              // non-connectable wake mfg-data advert (serial payload)
  kStartConnectableAdvertising, // connectable "Insta360 GPS Remote" advert
  kStopAdvertising,
  kStartControlConnect,         // scan for the camera + connect central to be80
  kStopControlConnect,          // abort scan/connect attempt
  kSendStartVideo,              // be81 explicit start-video
  kSendStopVideo,               // be81 explicit stop-video
  kSendKeepAlive,               // be81 keep-alive
  kSendGpsFrame,                // be81 GPS telemetry frame (caller packs the fix)
  kSendPowerOff,                // ce82 power-button 3s frame (remote link, best-effort)
  kDisconnect,                  // drop both links
};

// Telemetry + event snapshot, built fresh by the caller each step().
// The one-shot event flags must be true for exactly one step() call.
struct Inputs {
  uint32_t nowMs = 0;             // millis()
  int32_t  rpm = 0;               // tachLastReported
  float    speedMph = 0.0f;       // gps_speed_mph
  bool     gpsFixValid = false;   // gpsData.fix
  bool     remoteConnected = false;   // camera connected to our ce80 remote service
  bool     controlConnected = false;  // our central link to the camera's be80 is up
  bool     cameraAdvertSeen = false;  // scanner spotted the camera advertising (informational wake signal)
  int8_t   recordConfirmed = -1;      // be82-reported record state: -1 unknown, 0 stopped, 1 recording (v1: informational only)
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
  // timers: 0 == not running (timestamps are seeded non-zero)
  uint32_t rpmOnSince = 0;          // IDLE: rpm above ON threshold since
  uint32_t rpmGoneSince = 0;        // WAKING: rpm below OFF threshold since
  uint32_t wakeAttemptStarted = 0;  // WAKING: current attempt window start
  uint8_t  wakeAttemptsUsed = 0;
  bool     wakeBurstPhase = false;  // WAKING: still inside the wake-burst window
  uint32_t controlAttemptStarted = 0;
  uint8_t  controlAttemptsUsed = 0;
  uint32_t awaitGpsSince = 0;
  uint32_t stopCondSince = 0;       // RECORDING: stop-condition hold start
  uint32_t lastGpsFeedAt = 0;
  uint32_t lastKeepAliveAt = 0;
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
