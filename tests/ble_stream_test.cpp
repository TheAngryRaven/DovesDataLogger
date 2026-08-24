// Tests for the BLE file-transfer read-ahead bookkeeping.
//
// The point of this unit is that an off-by-one here corrupts a downloaded
// session file, so the model transfer below actually reassembles bytes and
// compares them, rather than only asserting on indices.

#include <string.h>

#include <vector>

#include "ble_stream.h"
#include "doctest.h"

using namespace ble_stream;

TEST_CASE("chunkSize takes the ATT header off the MTU") {
  CHECK(chunkSize(247) == 244);
  CHECK(chunkSize(185) == 182);  // iOS caps the ATT MTU at 185
  CHECK(chunkSize(23) == 20);    // unnegotiated default
}

TEST_CASE("chunkSize clamps to the characteristic's max length") {
  // A generous MTU cannot make the data characteristic send more than it
  // was configured for.
  CHECK(chunkSize(512) == kMaxNotifyLen);
  CHECK(chunkSize(kMaxNotifyLen + 3) == kMaxNotifyLen);
  CHECK(chunkSize(kMaxNotifyLen + 4) == kMaxNotifyLen);
}

TEST_CASE("chunkSize never returns zero") {
  // A degenerate MTU must not produce a notify loop that sends nothing and
  // never advances the file.
  CHECK(chunkSize(0) == 20);
  CHECK(chunkSize(3) == 20);
  CHECK(chunkSize(4) > 0);
}

TEST_CASE("rateBytesPerSec") {
  CHECK(rateBytesPerSec(0, 0) == 0);       // no window, no divide by zero
  CHECK(rateBytesPerSec(1000, 0) == 0);
  CHECK(rateBytesPerSec(29491, 1000) == 29491);
  CHECK(rateBytesPerSec(1024, 500) == 2048);
}

TEST_CASE("rateBytesPerSec does not overflow on a multi-megabyte transfer") {
  // 3.3 MB * 1000 is far past uint32; the field report that motivated this
  // work was exactly that size.
  CHECK(rateBytesPerSec(3300000u, 114000u) == 28947u);
}

TEST_CASE("a fresh ReadAhead has nothing to send") {
  ReadAhead ra(4096);
  CHECK(ra.capacity() == 4096);
  CHECK(ra.pending() == 0);
  CHECK(ra.head() == 0);
  CHECK(ra.needsRefill(244));

  uint32_t off = 0xFFFFFFFFu;
  CHECK(ra.nextSlice(244, &off) == 0);
}

TEST_CASE("compact on an empty buffer asks for no move and offers the whole buffer") {
  ReadAhead ra(4096);
  Move m = ra.compact();
  CHECK(m.length == 0);
  CHECK(ra.fillOffset() == 0);
  CHECK(ra.fillSpace() == 4096);
}

TEST_CASE("fill then slice hands out full chunks") {
  ReadAhead ra(4096);
  ra.compact();
  ra.commitFill(4096);
  CHECK(ra.pending() == 4096);
  CHECK_FALSE(ra.needsRefill(244));

  uint32_t off = 0;
  CHECK(ra.nextSlice(244, &off) == 244);
  CHECK(off == 0);
  ra.consume(244);

  CHECK(ra.nextSlice(244, &off) == 244);
  CHECK(off == 244);
  CHECK(ra.head() == 244);
  CHECK(ra.pending() == 4096 - 244);
}

TEST_CASE("a short tail is refilled, not sent") {
  // 4096 / 244 = 16 full chunks with 192 bytes left over. Sending that 192
  // would put a runt packet at every buffer boundary.
  ReadAhead ra(4096);
  ra.compact();
  ra.commitFill(4096);
  for (int i = 0; i < 16; i++) ra.consume(244);

  CHECK(ra.pending() == 192);
  CHECK(ra.needsRefill(244));
}

TEST_CASE("compact moves the unsent tail to the front") {
  ReadAhead ra(4096);
  ra.compact();
  ra.commitFill(4096);
  for (int i = 0; i < 16; i++) ra.consume(244);

  uint32_t tail = ra.pending();
  Move m = ra.compact();
  CHECK(m.srcOffset == 16 * 244);
  CHECK(m.length == tail);
  CHECK(ra.head() == 0);
  CHECK(ra.pending() == tail);
  // The refill lands behind the preserved tail and may not overwrite it.
  CHECK(ra.fillOffset() == tail);
  CHECK(ra.fillSpace() == 4096 - tail);
}

TEST_CASE("compact of an already-front-aligned tail asks for no move") {
  ReadAhead ra(64);
  ra.compact();
  ra.commitFill(10);
  Move m = ra.compact();
  CHECK(m.length == 0);  // nothing to memmove — it is already at offset 0
  CHECK(ra.pending() == 10);
  CHECK(ra.fillOffset() == 10);
}

TEST_CASE("a failed notify re-sends the same bytes") {
  // The transport failure model: consume() is simply not called, so the head
  // does not advance and the next pass hands out the identical slice. There
  // is no file position to rewind, so a dropped chunk cannot punch a hole in
  // a transfer that still reports DONE.
  ReadAhead ra(4096);
  ra.compact();
  ra.commitFill(4096);

  uint32_t first = 0, again = 0;
  CHECK(ra.nextSlice(244, &first) == 244);
  // ...notify() fails here, nothing is consumed...
  CHECK(ra.nextSlice(244, &again) == 244);
  CHECK(again == first);
  CHECK(ra.head() == 0);
}

TEST_CASE("commitFill clamps to the free space") {
  ReadAhead ra(100);
  ra.compact();
  ra.commitFill(500);  // a read that lies about how much it returned
  CHECK(ra.pending() == 100);
  CHECK(ra.fillSpace() == 0);
}

TEST_CASE("consume clamps to what is buffered") {
  ReadAhead ra(100);
  ra.compact();
  ra.commitFill(30);
  ra.consume(200);
  CHECK(ra.pending() == 0);
  CHECK(ra.head() == 30);
}

TEST_CASE("reset drops a partially streamed file") {
  ReadAhead ra(4096);
  ra.compact();
  ra.commitFill(4096);
  ra.consume(244);
  ra.reset();
  CHECK(ra.pending() == 0);
  CHECK(ra.head() == 0);
  CHECK(ra.fillSpace() == 4096);
}

// ---------------------------------------------------------------------------
// End-to-end model: stream a file through the real buffer discipline and
// check the bytes that come out the far side are the bytes that went in.
// ---------------------------------------------------------------------------

static std::vector<uint8_t> streamFile(size_t fileSize, uint32_t capacity,
                                       uint16_t chunk, size_t* fullChunksOut) {
  std::vector<uint8_t> file(fileSize);
  for (size_t i = 0; i < fileSize; i++) {
    file[i] = (uint8_t)((i * 31u + (i >> 8)) & 0xFF);
  }

  std::vector<uint8_t> buf(capacity, 0);
  std::vector<uint8_t> sent;
  ReadAhead ra(capacity);
  size_t filePos = 0;
  size_t fullChunks = 0;

  while (true) {
    if (ra.needsRefill(chunk)) {
      Move m = ra.compact();
      if (m.length) memmove(buf.data(), buf.data() + m.srcOffset, m.length);
      size_t want = ra.fillSpace();
      size_t got = (filePos + want <= fileSize) ? want : (fileSize - filePos);
      memcpy(buf.data() + ra.fillOffset(), file.data() + filePos, got);
      filePos += got;
      ra.commitFill((uint32_t)got);
    }

    uint32_t off = 0;
    uint16_t n = ra.nextSlice(chunk, &off);
    if (n == 0) break;  // buffer drained and the file is exhausted
    if (n == chunk) fullChunks++;
    sent.insert(sent.end(), buf.begin() + off, buf.begin() + off + n);
    ra.consume(n);
  }

  if (fullChunksOut) *fullChunksOut = fullChunks;
  return sent;
}

TEST_CASE("a modelled transfer delivers the file byte-for-byte") {
  struct Case {
    size_t size;
    uint16_t chunk;
  };
  // Sizes chosen around the buffer boundary: under one buffer, exactly one,
  // one past, and a large odd size. Chunks cover the negotiated MTUs a real
  // central hands us.
  const Case cases[] = {
      {0, 244},    {1, 244},     {243, 244},   {244, 244},
      {4095, 244}, {4096, 244},  {4097, 244},  {100000, 244},
      {100000, 182}, {100000, 20}, {5000, 182},
  };

  for (const Case& c : cases) {
    CAPTURE(c.size);
    CAPTURE(c.chunk);
    std::vector<uint8_t> sent = streamFile(c.size, 4096, c.chunk, nullptr);
    REQUIRE(sent.size() == c.size);
    for (size_t i = 0; i < c.size; i++) {
      // Regenerate rather than trusting a second copy of the pattern.
      REQUIRE(sent[i] == (uint8_t)((i * 31u + (i >> 8)) & 0xFF));
    }
  }
}

TEST_CASE("compacting refills keep every packet but the last one full") {
  // The whole reason compact() exists. 100000 bytes at 244 = 409 full chunks
  // plus a 236-byte remainder; a non-compacting buffer would additionally emit
  // a runt at each of the ~24 buffer boundaries.
  size_t fullChunks = 0;
  std::vector<uint8_t> sent = streamFile(100000, 4096, 244, &fullChunks);
  CHECK(sent.size() == 100000);
  CHECK(fullChunks == 100000 / 244);
}
