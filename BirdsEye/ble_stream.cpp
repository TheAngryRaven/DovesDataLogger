#include "ble_stream.h"

namespace ble_stream {

uint16_t chunkSize(uint16_t negotiatedMtu) {
  // Three bytes of ATT header (opcode + handle) come off the MTU before the
  // payload. An MTU that never negotiated is the 23-byte default, which still
  // leaves 20 usable bytes — slow, but a working transfer beats a zero-length
  // notify loop that never advances.
  uint16_t payload = (negotiatedMtu > 3) ? (uint16_t)(negotiatedMtu - 3) : 20;
  if (payload > kMaxNotifyLen) payload = kMaxNotifyLen;
  if (payload == 0) payload = 20;
  return payload;
}

uint32_t rateBytesPerSec(uint32_t bytes, uint32_t elapsedMs) {
  if (elapsedMs == 0) return 0;
  // 64-bit intermediate: a 3 MB transfer times 1000 overflows uint32 well
  // before the file does.
  return (uint32_t)(((uint64_t)bytes * 1000u) / elapsedMs);
}

ReadAhead::ReadAhead(uint32_t capacity)
    : cap_(capacity), head_(0), filled_(0) {}

void ReadAhead::reset() {
  head_ = 0;
  filled_ = 0;
}

Move ReadAhead::compact() {
  Move m = {head_, pending()};
  // Nothing to move when the buffer is drained, or when the tail already
  // starts at offset 0 — but the indices still have to collapse so the free
  // space below is measured from the tail's new end.
  head_ = 0;
  filled_ = m.length;
  if (m.srcOffset == 0) m.length = 0;
  return m;
}

void ReadAhead::commitFill(uint32_t bytesRead) {
  const uint32_t space = fillSpace();
  if (bytesRead > space) bytesRead = space;
  filled_ += bytesRead;
}

uint16_t ReadAhead::nextSlice(uint16_t chunk, uint32_t* offsetOut) const {
  const uint32_t avail = pending();
  if (avail == 0 || chunk == 0) return 0;
  if (offsetOut) *offsetOut = head_;
  // A tail shorter than a chunk is the end of the file: needsRefill() would
  // have topped it up otherwise.
  return (avail < chunk) ? (uint16_t)avail : chunk;
}

void ReadAhead::consume(uint16_t n) {
  const uint32_t avail = pending();
  head_ += (n > avail) ? avail : n;
}

}  // namespace ble_stream
