#include "png_dump.h"

#include <stdio.h>
#include <string.h>

#include <vector>

#include "crc32.h"

namespace png_dump {

namespace {

constexpr int kFbWidth = 128;
constexpr int kFbHeight = 64;

void pushU32be(std::vector<uint8_t>& v, uint32_t x) {
  v.push_back((uint8_t)(x >> 24));
  v.push_back((uint8_t)(x >> 16));
  v.push_back((uint8_t)(x >> 8));
  v.push_back((uint8_t)x);
}

void writeChunk(FILE* f, const char type[4], const uint8_t* data,
                size_t len) {
  uint8_t hdr[8];
  hdr[0] = (uint8_t)(len >> 24);
  hdr[1] = (uint8_t)(len >> 16);
  hdr[2] = (uint8_t)(len >> 8);
  hdr[3] = (uint8_t)len;
  memcpy(hdr + 4, type, 4);
  fwrite(hdr, 1, 8, f);
  if (len) fwrite(data, 1, len, f);
  uint32_t crc = crc32::kInit;
  crc = crc32::update(crc, type, 4);
  crc = crc32::update(crc, data, len);
  crc = crc32::finalize(crc);
  uint8_t crcb[4] = {(uint8_t)(crc >> 24), (uint8_t)(crc >> 16),
                     (uint8_t)(crc >> 8), (uint8_t)crc};
  fwrite(crcb, 1, 4, f);
}

uint32_t adler32(const uint8_t* data, size_t len) {
  uint32_t a = 1, b = 0;
  for (size_t i = 0; i < len; i++) {
    a = (a + data[i]) % 65521u;
    b = (b + a) % 65521u;
  }
  return (b << 16) | a;
}

}  // namespace

bool writeFramebufferPng(const char* path, const uint8_t* fb1024, int scale) {
  if (!path || !fb1024 || scale < 1) return false;

  const int w = kFbWidth * scale;
  const int h = kFbHeight * scale;

  // Raw PNG scanline stream: per row, 1 filter byte (0 = None) then w
  // 8-bit grayscale samples.
  std::vector<uint8_t> raw;
  raw.reserve((size_t)h * (w + 1));
  for (int y = 0; y < h; y++) {
    raw.push_back(0);  // filter: None
    const int fy = y / scale;
    for (int x = 0; x < w; x++) {
      const int fx = x / scale;
      const int bit = (fb1024[fx + (fy >> 3) * kFbWidth] >> (fy & 7)) & 1;
      raw.push_back(bit ? 0xFF : 0x00);
    }
  }

  // zlib stream around stored (uncompressed) deflate blocks.
  std::vector<uint8_t> idat;
  idat.push_back(0x78);  // CMF: deflate, 32K window
  idat.push_back(0x01);  // FLG: no dict, fastest (checksum-valid pair)
  size_t off = 0;
  while (off < raw.size()) {
    const size_t n = raw.size() - off > 65535 ? 65535 : raw.size() - off;
    const bool last = (off + n == raw.size());
    idat.push_back(last ? 1 : 0);  // BFINAL, BTYPE=00 (stored)
    idat.push_back((uint8_t)(n & 0xFF));
    idat.push_back((uint8_t)(n >> 8));
    idat.push_back((uint8_t)(~n & 0xFF));
    idat.push_back((uint8_t)((~n >> 8) & 0xFF));
    idat.insert(idat.end(), raw.begin() + off, raw.begin() + off + n);
    off += n;
  }
  pushU32be(idat, adler32(raw.data(), raw.size()));

  FILE* f = fopen(path, "wb");
  if (!f) return false;

  static const uint8_t kSig[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1A,
                                  '\n'};
  fwrite(kSig, 1, 8, f);

  uint8_t ihdr[13];
  ihdr[0] = (uint8_t)(w >> 24);
  ihdr[1] = (uint8_t)(w >> 16);
  ihdr[2] = (uint8_t)(w >> 8);
  ihdr[3] = (uint8_t)w;
  ihdr[4] = (uint8_t)(h >> 24);
  ihdr[5] = (uint8_t)(h >> 16);
  ihdr[6] = (uint8_t)(h >> 8);
  ihdr[7] = (uint8_t)h;
  ihdr[8] = 8;   // bit depth
  ihdr[9] = 0;   // color type: grayscale
  ihdr[10] = 0;  // compression
  ihdr[11] = 0;  // filter
  ihdr[12] = 0;  // interlace
  writeChunk(f, "IHDR", ihdr, sizeof(ihdr));
  writeChunk(f, "IDAT", idat.data(), idat.size());
  writeChunk(f, "IEND", nullptr, 0);

  const bool ok = (fclose(f) == 0);
  return ok;
}

}  // namespace png_dump
