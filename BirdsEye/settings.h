#pragma once

///////////////////////////////////////////
// SETTINGS MODULE
// Persistent JSON key/value store at /SETTINGS.json on the SD card.
// Always reads fresh from disk (no in-RAM cache) — getSetting() and
// setSetting() each take a brief SD lock via acquireSDAccess.
///////////////////////////////////////////

#include <stddef.h>

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
