#include "frame_hash.h"

namespace frame_hash {

uint32_t fnv1a(const uint8_t* data, size_t len) {
  uint32_t h = kFnvOffsetBasis;
  for (size_t i = 0; i < len; i++) {
    h ^= data[i];
    h *= kFnvPrime;
  }
  return h;
}

}  // namespace frame_hash
