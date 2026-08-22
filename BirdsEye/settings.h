#pragma once

///////////////////////////////////////////
// SETTINGS MODULE
// Persistent JSON key/value store at /SETTINGS.json on the SD card.
// Always reads fresh from disk (no in-RAM cache) — getSetting() and
// setSetting() each take a brief SD lock via acquireSDAccess.
///////////////////////////////////////////

#include <stddef.h>

///////////////////////////////////////////
// SETTINGS JSON CAPACITY — the single source of truth.
//
// Both the read buffer and the ArduinoJson document that parse
// /SETTINGS.json must be this size, in EVERY module that parses the file.
// There are two such parsers: settings.ino (getSetting/setSetting) and
// bluetooth.ino's `SLIST` handler, which enumerates the file for the
// companion app.
//
// This constant exists because those two drifted. Plan 0010 raised
// settings.ino's pair 512 -> 1024 when nine new keys took the default file
// from 329 to 538 bytes; bluetooth.ino's copy stayed at 512, so `SLIST`
// read 511 bytes of a 538-byte file, deserializeJson() returned
// IncompleteInput, and the handler answered SERR:PARSE on every device
// instead of listing a single key. Nothing coupled the two numbers, so
// nothing caught it. Now they cannot drift.
//
// TWO separate walls, both sized by this: the READ CAP (a file longer than
// capacity-1 parses as IncompleteInput and every key read fails) and the
// DOCUMENT CAPACITY (22 string pairs need JSON_OBJECT_SIZE(22); a <512>
// document returns NoMemory regardless of how much was read).
//
// Adding settings keys is not free — see the measureJson() guard in
// setSettingInner(), and subsystem 8 in CLAUDE.md.
///////////////////////////////////////////
#define SETTINGS_JSON_CAPACITY 1024

// Initialize the settings file (creates defaults on first boot,
// adds any missing keys on upgrade). Call once from setup() AFTER
// SD_SETUP().
bool SETTINGS_SETUP();

// Read a setting by key into a caller-provided buffer.
// Returns true if found, false on any failure (buf set to "").
bool getSetting(const char* key, char* buf, size_t bufSize);

// Read-modify-write a single setting. Returns true on success.
bool setSetting(const char* key, const char* value);

// Delete the settings file and re-create with fresh defaults
// (rolls a new random BLE name and PIN).
bool resetSettings();
