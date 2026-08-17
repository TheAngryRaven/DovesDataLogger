#pragma once

///////////////////////////////////////////
// REPLAY MODULE
// Instant DOVEX session replay: parses the reserved header at the
// start of a .dovex file (metadata + lap times) and populates
// lapHistory[] for the results page. No streaming, no parsing of
// individual GPS rows — that's why the results screen pops up
// instantly.
///////////////////////////////////////////

#include "SdFat.h"

// Reset replay state (lap history, file handle, SD lock).
void resetReplayState();

// Scan SD root for .dovex files and populate replayFiles[].
// Returns true if any were found.
bool buildReplayFileList();

extern int numReplayFiles;

// Rows on the session browser: one per file plus a trailing Back row.
//
// The single source of truth for that layout — the renderer, the menu's
// row limit and the selection handler all derive from it, so the Back row
// cannot end up drawn in one place and unreachable in another. An empty
// list keeps its one-row "no files" screen (unreachable in practice: the
// page is only opened when buildReplayFileList() found something).
inline int replayItemCount() {
  return numReplayFiles > 0 ? numReplayFiles + 1 : 1;
}

// Read one line from `file` into `buffer` (drops \r, stops at \n /
// EOF). Returns false on EOF when nothing was buffered.
bool readReplayLine(File& file, char* buffer, int bufferSize);

// Parse the reserved 1KB header of a .dovex file (lines 1-4) and
// populate dovexReplay* globals plus lapHistory[]. Returns false if
// the header is empty/incomplete (e.g. session ended in a crash).
bool parseDovexHeader(const char* filename);
