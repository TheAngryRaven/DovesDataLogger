#include "crossing_pattern.h"

namespace crossing_pattern {

int frameRects(bool flip, Rect* out, int maxOut) {
  if (out == nullptr || maxOut <= 0) return 0;

  const int cols = kWidth / kCell;    // 8
  const int rows = kHeight / kCell;   // 4
  const int phase = flip ? 1 : 0;     // which column parity is lit
  int n = 0;

  // Only the ODD row bands carry blocks (this is what the original
  // bitmaps did — the even bands were entirely blank).
  for (int cy = 1; cy < rows; cy += 2) {
    for (int cx = phase; cx < cols; cx += 2) {
      if (n >= maxOut) return n;
      out[n].x = cx * kCell;
      out[n].y = cy * kCell;
      out[n].w = kCell;
      out[n].h = kCell;
      n++;
    }
  }
  return n;
}

}  // namespace crossing_pattern
