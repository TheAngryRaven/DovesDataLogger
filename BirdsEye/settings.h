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
// DOCUMENT CAPACITY (24 string pairs need JSON_OBJECT_SIZE(24); a <512>
// document returns NoMemory at 22 pairs regardless of how much was read).
//
// Adding settings keys is not free — see the measureJson() guard in
// setSettingInner(), and subsystem 8 in CLAUDE.md. As of plan 0017 the
// default file measures 570 bytes on a stock build (23 keys) and 611 on
// a SensorEgg one (25 — sensoregg_mac added), rising to 599/657 with the
// longest values every key accepts, against a 1023-byte read cap. That
// is roughly fifteen average keys of headroom. Each key costs
// len(key) + len(value) + 6.
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

// Delete a single key. True when the key is gone afterwards, INCLUDING
// when it was already absent — the caller's post-condition is "not
// there", not "I removed something". False on any SD/parse failure, so a
// migration can hold off dropping an old key until the new one is
// safely written. Deliberately does NOT heal a corrupt file the way
// setSetting() does: dropping a key is never urgent enough to justify
// quarantining the user's whole settings file behind their back.
bool removeSetting(const char* key);

// Delete the settings file and re-create with fresh defaults
// (rolls a new random BLE name and PIN).
bool resetSettings();
