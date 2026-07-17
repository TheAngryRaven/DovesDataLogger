#pragma once

#include <stddef.h>
#include <stdint.h>

///////////////////////////////////////////
// FNV-1a 32-bit frame hash.
//
// Hashed over the display's raw 1024-byte framebuffer (page layout:
// bit = buf[x + (y>>3)*128] >> (y&7) & 1). Used for the golden display
// fixtures, the viewer's redraw dirty-check, and — kept deliberately
// this cheap — an eventual Pi HIL framebuffer tap can compute the same
// hash over the same bytes.
///////////////////////////////////////////

namespace frame_hash {

constexpr uint32_t kFnvOffsetBasis = 2166136261u;
constexpr uint32_t kFnvPrime = 16777619u;

uint32_t fnv1a(const uint8_t* data, size_t len);

}  // namespace frame_hash
