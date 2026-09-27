#pragma once

///////////////////////////////////////////
// SENSOREGG GATT PROTOCOL (plan 0018)
// Decoders for the PerchWerks Sensor Service the egg serves since its
// roadmap phases 2-3 (DovesSensorEgg docs/PW_SENSOR_SERVICE.md +
// PW_CHANNEL_SCHEMA.md): the self-describing Descriptor, per-channel
// Sample batch frames, the Clock read, descriptor-driven role mapping,
// and the v1 clock fit. Pure logic — no Arduino headers — exercised by
// host tests whose fixtures are BYTE-IDENTICAL to the egg repo's
// pw_gatt_encode goldens: encode == decode across the two repos, the
// same discipline sensoregg_protocol already pins for the beacon.
//
// This unit deliberately does NOT extend sensoregg_protocol: that file
// is the PW-ADV-2 beacon contract; this is a different wire spec with
// its own fixture provenance.
//
// All multi-byte fields little-endian; f32 is IEEE-754 binary32 LE.
///////////////////////////////////////////

#include <stddef.h>
#include <stdint.h>

#include "sensoregg_protocol.h"  // Reading + kStalenessMs (the shared surface)

namespace sensoregg_gatt {

constexpr uint8_t kMaxChannels = 21;    // 8 + 21*24 = 512 = ATT max value
constexpr size_t kDescHeaderLen = 8;
constexpr uint8_t kMinRecordLen = 24;   // schema v1; larger = newer schema,
                                        // stride by the declared value
constexpr size_t kSampleHeaderLen = 10;
constexpr size_t kClockLen = 6;
constexpr uint8_t kMaxSamplesPerFrame = 117;  // (247-3-10)/2, spec section 5
// Largest Sample notify the link can carry: ATT_MTU 247 (the logger's
// configCentralConn cap) - 3 = 244 = a full 117-sample frame. Receive
// buffers sized to this take every legal frame at any negotiated MTU.
constexpr size_t kMaxFrameLen = 10 + 2 * (size_t)kMaxSamplesPerFrame;
constexpr int16_t kInvalidSentinel = INT16_MIN;  // 0x8000 = "no valid reading"

// One parsed channel descriptor record (PW_CHANNEL_SCHEMA.md section 5).
struct ChannelInfo {
  uint8_t id = 0;
  uint8_t quantity = 0;
  uint16_t periodMs = 0;  // nominal sample period; 0 = aperiodic
  float scale = 0.0f;
  float offset = 0.0f;
  char name[9] = {0};  // wire is char[8] NUL-PADDED; always terminated here
};

struct PodDescriptor {
  uint8_t schemaVersion = 0;
  uint8_t deviceType = 0;
  uint8_t fwMajor = 0;
  uint8_t fwMinor = 0;
  uint8_t channelCount = 0;
  uint8_t recordLen = 0;
  ChannelInfo ch[kMaxChannels];
};

// Parse a Descriptor characteristic value. Strides by the DECLARED
// record_len (>= 24 accepted — skipping unknown trailing bytes is the
// schema's forward-compat mechanism). Returns false (out untouched) on
// null/short buffers, schema_version != 1, record_len < 24, or a
// channel count outside 1..21.
bool parseDescriptor(const uint8_t* d, size_t len, PodDescriptor& out);

// One parsed Sample notify frame (spec section 5).
struct SampleFrame {
  uint8_t channelId = 0;
  uint8_t bootId = 0;
  uint8_t seq = 0;
  uint32_t baseMs = 0;     // pod-local millis of raw[0], at acquisition
  uint16_t intervalMs = 0; // per-frame authoritative sample spacing
  uint8_t n = 0;
  int16_t raw[kMaxSamplesPerFrame];
};

// Parse a Sample frame. len must be exactly 10 + 2*n for the carried n
// (a notify delivers whole frames); n must be 1..117.
bool parseSampleFrame(const uint8_t* d, size_t len, SampleFrame& out);

// Parse the Clock value (boot_id + pod millis). Accepts len >= 6 and
// reads the first 6 bytes (trailing bytes = a future revision's).
bool parseClock(const uint8_t* d, size_t len, uint8_t& bootId,
                uint32_t& millisNow);

// Engineering conversion: real = raw * scale + offset; the 0x8000
// sentinel becomes NaN BEFORE the conversion (never scaled).
float sampleToReal(int16_t raw, const ChannelInfo& c);

// The surface's battery percent from a battery-role channel's
// engineering value (i.e. AFTER sampleToReal — the descriptor's
// scale/offset apply to the ratio channel like any other): NaN (the
// sentinel) -> 0xFF "unknown"; otherwise clamped to 0-100 and rounded.
// NaN is detected by bit pattern (nan_bits.h) — the device builds with
// -Ofast, where isnan() folds to false and NaN comparisons are UB.
uint8_t batteryPercent(float real);

// Descriptor-driven role mapping — the consumer never hardcodes channel
// ids. Temperatures route by the schema's normative names ("EGT", "CJ",
// "IAT"); the battery routes by quantity 0x08 (ratio). outIdx[] holds
// the CHANNEL-TABLE INDEX for each role, -1 when absent.
enum Role : uint8_t { ROLE_EGT = 0, ROLE_CJ, ROLE_AUX, ROLE_BATT, ROLE_COUNT };
void mapChannels(const PodDescriptor& pd, int8_t outIdx[ROLE_COUNT]);

// Index of the channel with the smallest nonzero period (the zombie
// monitor feeds on its frame seq — the fastest heartbeat). Falls back
// to 0 when every period is 0/absent.
int8_t fastestChannel(const PodDescriptor& pd);

// ---- Clock fit, v1: anchor-only (slope 1.0) -----------------------------
// The pod is time-dumb; the logger owns pod_time -> logger_time. v1
// anchors on the connect-time Clock read: logger time = the
// request/response midpoint (tightest pair available), pod time = the
// value read. boot_id inequality = the pod's millis restarted (epoch).
struct ClockFit {
  bool valid = false;
  uint8_t bootId = 0;
  uint32_t podMs0 = 0;
  uint32_t loggerMs0 = 0;
  uint32_t halfRttMs = 0;  // anchor uncertainty, for diagnostics
};

void clockFitAnchor(ClockFit& f, uint8_t bootId, uint32_t podMillis,
                    uint32_t reqMs, uint32_t rspMs);

bool clockFitSameEpoch(const ClockFit& f, uint8_t frameBootId);

// Wrap-safe in both directions (signed u32 delta): valid for pod times
// within +/- ~24.8 days of the anchor, i.e. always in practice.
uint32_t clockFitPodToLogger(const ClockFit& f, uint32_t podMs);

// ---- Link state machine (review fixes 2026-09) ---------------------------
// The sketch's central-link states, owned here so the transition rules
// that race the Bluefruit callback task are host-tested. Ordered so
// "engaged" (a connection exists or is being made) is a single >=
// compare: BACKOFF sits between the idle states and CONNECTING on
// purpose.
enum LinkState : uint8_t {
  LINK_IDLE = 0,    // link not wanted (unpaired / gate closed)
  LINK_WAIT_ADV,    // wanted — the scan callback fires the connect
  LINK_BACKOFF,     // cooling off after a failure/disconnect
  LINK_CONNECTING,  // sd_ble_gap_connect in flight
  LINK_BRINGUP,     // discovery/reads running in the callback task
  LINK_STREAMING,   // notify subscription live, surface fed by GATT
};

// May the main loop promote a staged bring-up to STREAMING? Only while
// the bring-up it staged is still the live one: state still BRINGUP and
// a connection handle still held. The disconnect callback runs in a
// higher-priority task and can land between the bring-up's ready flag
// and the loop's commit — it has already written BACKOFF and dropped
// the handle, and an unconditional commit would overwrite that with
// STREAMING on a link that no longer exists (scanner held off, EGT NaN
// until shutdown). The caller evaluates this and writes the new state
// inside one critical section.
bool linkMayCommitStreaming(LinkState state, bool handleValid);

// Reconcile guard: a state that claims a connection (BRINGUP or
// STREAMING) while no handle is held is an orphan left by a lost race —
// send it to BACKOFF so the normal retry path (and the scanner) takes
// over. Every other state is returned unchanged.
LinkState linkReconcileOrphan(LinkState state, bool handleValid);

// Should the central connect callback keep the connection it was just
// handed? Only when it is the connect we asked for (state CONNECTING)
// and the link is still wanted and the radio is not asleep. The
// SoftDevice can raise CONNECTED after SENSOREGG_SLEEP() already ran
// (its connect_cancel() is a no-op once the link exists, and the
// handle was not yet known to disconnect) or after the connect timeout
// gave up; accepting then would bring the link up during a transfer
// session / the charging park, where nothing reconciles it. A false
// answer means: disconnect that handle without adopting it (a
// CONNECTING state is retired to BACKOFF; IDLE/BACKOFF stay put).
bool linkAcceptCentralConnect(LinkState state, bool sleeping, bool wanted);

// State after the scan callback asked the SoftDevice to connect. A
// refused request (Central.connect() == false: radio busy, invalid
// params) never produces a connect callback, so staying CONNECTING
// would sit out the full 10 s connect timeout — plus the 5 s backoff —
// with the scanner paused on the report that triggered it. Refused ->
// BACKOFF immediately (and the caller resumes the scanner).
LinkState linkAfterConnectRequest(bool accepted);

// Sustained-drop rule for the notify frame ring. A drop is a frame the
// logger could not keep (ring full — the main loop fell behind — or a
// frame over kMaxFrameLen). One bad window is normal: an SD garbage-
// collection stall parks the main loop for up to ~2 s and the ring
// overflows once, and latest-value semantics lose nothing but
// superseded samples. Drops in kDropWindowsToFail CONSECUTIVE
// kDropWindowMs windows mean the stream is not being consumed; the
// caller then drops the link so the beacon path (and the reconnect)
// takes over rather than streaming into the floor. Feed the cumulative
// drop counter every loop while streaming; a stall spanning several
// windows is judged as one window (windows tumble on loop time).
constexpr uint32_t kDropWindowMs = 1000;
constexpr uint8_t kDropWindowsToFail = 3;

struct DropMonitor {
  bool started = false;
  uint32_t windowStartMs = 0;
  uint32_t dropsAtWindowStart = 0;
  uint8_t badWindows = 0;
};

// Start a fresh evaluation at the current cumulative count.
void dropMonitorReset(DropMonitor& m, uint32_t totalDrops, uint32_t nowMs);

// Returns true once, when the kDropWindowsToFail-th consecutive bad
// window closes (and re-arms the count).
bool dropMonitorUpdate(DropMonitor& m, uint32_t totalDrops, uint32_t nowMs);

// ---- Stream surface rules (review fixes 2026-09) -------------------------
// The GATT stream feeds the same sensoregg_protocol::Reading the beacon
// does, but it only carries the channels the descriptor maps — nothing
// else in that struct is refreshed by a frame, while every frame keeps
// the link's arrival stamp fresh. Two rules keep that from holding a
// value (a held value draws a flat line indistinguishable from data):

// On committing STREAMING, clear everything the stream will (or can
// never) rewrite: temperatures NaN, battery 0xFF (unknown), flags /
// status / tcFault / pairingActive cleared, sequence 0. Sample frames
// carry no MCP STATUS and no pairing bit, so while streaming those
// read false (a fault still reaches the page as the sentinel -> NaN ->
// '---'). protoVersion is left as the last beacon's: it names the
// egg's beacon firmware, which the stream does not contradict.
void readingResetForStream(sensoregg_protocol::Reading& r);

// Per-role receive stamps: each mapped role is fresh only while ITS OWN
// channel keeps arriving, so an EGT channel that stops while CJ carries
// on goes NaN instead of being held by CJ's traffic. A role that was
// never stamped (e.g. a pod with no IAT channel) is never fresh.
//
// Staleness per role = the frame's own cadence with one frame of slack,
// 2 x (n x interval_ms), floored at sensoregg_protocol::kStalenessMs
// (so a fast channel keeps exactly the 1 s rule) and capped at
// kRoleStaleMaxMs (a corrupt interval can't hold a value for minutes).
// The floor alone would be wrong for slow channels: the EGT pod's IAT
// runs at 1 s and its battery at 30 s, so a flat 1 s rule would flap
// IAT and blank the battery forever.
constexpr uint32_t kRoleStaleMaxMs = 60000;

uint32_t roleStaleAfterMs(uint8_t n, uint16_t intervalMs);

struct RoleFreshness {
  bool have[ROLE_COUNT] = {false, false, false, false};
  uint32_t atMs[ROLE_COUNT] = {0, 0, 0, 0};
  uint32_t staleAfterMs[ROLE_COUNT] = {0, 0, 0, 0};
};

void roleFreshnessReset(RoleFreshness& f);
void roleFreshnessStamp(RoleFreshness& f, Role role, uint32_t atMs,
                        uint8_t n, uint16_t intervalMs);
// Wrap-safe: fresh while (now - at) < staleAfter.
bool roleFresh(const RoleFreshness& f, Role role, uint32_t nowMs);

}  // namespace sensoregg_gatt
