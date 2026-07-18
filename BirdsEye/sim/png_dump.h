#pragma once

#include <stdint.h>

///////////////////////////////////////////
// Debug PNG dump — write a 128x64 1-bit framebuffer (SH110X page
// layout) as an 8-bit grayscale PNG for local eyeballing of frames.
// Dependency-free: the PNG's zlib stream uses stored (uncompressed)
// deflate blocks, and the chunk CRCs reuse the firmware's crc32 unit.
///////////////////////////////////////////

namespace png_dump {

// scale >= 1 writes each framebuffer pixel as a scale x scale block
// (nearest-neighbor), so dumps are eyeball-able without a zoom tool.
// Returns false on I/O failure.
bool writeFramebufferPng(const char* path, const uint8_t* fb1024,
                         int scale = 4);

}  // namespace png_dump
