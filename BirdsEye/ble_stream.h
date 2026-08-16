#ifndef BLE_STREAM_H
#define BLE_STREAM_H

// BLE file-transfer stream bookkeeping (pure logic, no Arduino headers).
//
// The firmware owns the bytes, the file and the radio; this unit owns the
// index arithmetic that decides how much to read, where to read it, and which
// slice of RAM the next notification carries. See
// docs/plans/0008-ble-download-throughput.md.
//
// Why a read-ahead exists at all: notifying straight out of SdFat put a disk
// read in the radio's critical path and, because a chunk is not a sector,
// spent one single-block SD command per 512 bytes. One large aligned read
// into RAM turns that into a multi-block transfer and lets the notifications
// stream from memory.

#include <stdint.h>

namespace ble_stream {

// fileDataChar.setMaxLen() — the largest payload the data characteristic can
// carry regardless of how generous the negotiated MTU is.
inline constexpr uint16_t kMaxNotifyLen = 244;

// Read-ahead capacity. Eight 512-byte sectors: big enough for SdFat to issue
// one multi-block read, small enough that the refill stall (~4.3 ms at 8 MHz)
// stays well inside a connection interval and is covered by the bytes already
// queued in the SoftDevice.
inline constexpr uint32_t kReadAheadSize = 4096;

// Wall-clock budget for one burst of notifications before the main loop gets
// a turn. Bounded by time rather than packet count: a fixed count is a very
// different amount of time on a fast link than on a slow one, and the only
// thing the bound protects is loop responsiveness (exit button, watchdog).
inline constexpr uint32_t kBurstBudgetMs = 20;

// Payload for one notification at the negotiated ATT MTU (3 bytes of ATT
// header), clamped to what the characteristic can send. Never returns 0 — an
// unnegotiated link still moves 20 bytes at a time.
uint16_t chunkSize(uint16_t negotiatedMtu);

// Transfer rate in bytes/sec. Returns 0 for an empty window rather than
// dividing by zero, so a caller can print it unconditionally.
uint32_t rateBytesPerSec(uint32_t bytes, uint32_t elapsedMs);

// The memmove a compacting refill asks the caller to perform: move `length`
// bytes from `srcOffset` to offset 0 of the buffer.
struct Move {
  uint32_t srcOffset;
  uint32_t length;
};

// Linear read-ahead buffer over a caller-owned byte array.
//
// Refills are COMPACTING: the unsent tail is moved to the front before the
// next read, so every notification except the file's last one carries a full
// chunk. Without that, each buffer boundary would emit a runt packet
// (4096 mod 244 = 190).
//
// A refill is three steps so the caller can do the memmove and the SD read
// itself:
//   ble_stream::Move m = ra.compact();
//   if (m.length) memmove(buf, buf + m.srcOffset, m.length);
//   n = file.read(buf + ra.fillOffset(), ra.fillSpace());
//   ra.commitFill(n);
class ReadAhead {
 public:
  explicit ReadAhead(uint32_t capacity);

  void reset();

  uint32_t capacity() const { return cap_; }
  // Unsent bytes currently held.
  uint32_t pending() const { return filled_ - head_; }
  // Offset of the next unsent byte.
  uint32_t head() const { return head_; }

  // True when the buffer can no longer serve a full chunk. A short tail is
  // refilled rather than sent, so the stream stays at full packet size until
  // the file itself runs out.
  bool needsRefill(uint16_t chunk) const { return pending() < chunk; }

  // Step 1: slide the unsent tail to offset 0. Returns the memmove the caller
  // must perform; `length` is 0 when the buffer is empty or the tail is
  // already at the front, in which case there is nothing to move.
  Move compact();

  // Steps 2a/2b: where a refill read should land, and how many bytes to ask
  // for. Only meaningful immediately after compact().
  uint32_t fillOffset() const { return filled_; }
  uint32_t fillSpace() const { return cap_ - filled_; }

  // Step 3: record what the read actually returned. Clamped to the free
  // space, so a misbehaving read can never push filled_ past capacity.
  void commitFill(uint32_t bytesRead);

  // The next notification slice. Writes the buffer offset to *offsetOut and
  // returns its length, or 0 when nothing is buffered. A tail shorter than
  // `chunk` is returned in full — that is the end of the file, since
  // needsRefill() would otherwise have topped it up.
  uint16_t nextSlice(uint16_t chunk, uint32_t* offsetOut) const;

  // Advance past bytes that were successfully sent. A failed notify simply
  // does not call this, so the bytes are re-sent from RAM on the next pass —
  // there is no file position to rewind and no way to punch a hole in a
  // transfer that still reports DONE.
  void consume(uint16_t n);

 private:
  uint32_t cap_;
  uint32_t head_;
  uint32_t filled_;
};

}  // namespace ble_stream

#endif  // BLE_STREAM_H
