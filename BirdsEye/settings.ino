///////////////////////////////////////////
// SETTINGS MODULE
// Persistent JSON key-value store on SD card (/SETTINGS.json)
// Provides getSetting() and setSetting() for any subsystem.
///////////////////////////////////////////

#include "settings.h"

static const char SETTINGS_FILE_PATH[] = "/SETTINGS.json";
// Quarantine target for a corrupt (non-empty, unparseable) settings file —
// typically a hand-edit gone wrong. The bad file is KEPT (renamed, not
// deleted) so its contents can be inspected/salvaged on a computer; a
// pre-existing .bad from an earlier quarantine is overwritten.
static const char SETTINGS_BAD_PATH[] = "/SETTINGS.json.bad";
// Sized for the whole settings file plus its NUL, with real headroom:
// every read path below caps at sizeof(settingsFileBuffer) - 1, so a
// file larger than this parses as IncompleteInput and EVERY key read
// fails. That failure is self-inflicting — SETTINGS_SETUP quarantines
// the "corrupt" file and regenerates it, ensureDefaultSettings grows it
// back over the cap, and the device loses its settings on a loop.
// Raised 512 -> 1024 in plan 0010: the 18-key file was already 436 B and
// the four new keys put it at 543. The document is the other wall — 22
// string pairs need JSON_OBJECT_SIZE(22) and a <512> doc returns
// NoMemory. Keep the two numbers equal, and see the measureJson() guard
// in setSettingInner() before adding more keys.
static char settingsFileBuffer[SETTINGS_JSON_CAPACITY];
static StaticJsonDocument<SETTINGS_JSON_CAPACITY> settingsJson;

// 12 racing-adjacent words used to build a friendly default device name so
// logs dumped from a fleet of devices stay distinguishable. The literals
// live in flash and the pointer table is tiny (~48 bytes), so this costs
// almost no RAM. A default name is two of these picked at random.
static const char* const kDeviceNameWords[] = {
  "Apex", "Turbo", "Nitro", "Drift", "Slick", "Boost",
  "Piston", "Vortex", "Camber", "Draft", "Redline", "Gearbox"
};

/**
 * @brief Compose a random default device name (e.g. "ApexTurbo") by picking
 *        two distinct racing words from kDeviceNameWords.
 * @param buf Caller-provided buffer for the name (null-terminated)
 * @param bufSize Size of buf
 */
static void generateDeviceName(char* buf, size_t bufSize) {
  const int count = (int)(sizeof(kDeviceNameWords) / sizeof(kDeviceNameWords[0]));
  int i = (int)random(0, count);
  int j = (int)random(0, count - 1);
  if (j >= i) j++;  // map onto the remaining words so the two always differ
  snprintf(buf, bufSize, "%s%s", kDeviceNameWords[i], kDeviceNameWords[j]);
}

/**
 * @brief Generate default settings file on first boot
 * @return true if file created successfully
 */
bool createDefaultSettings() {
  if (!acquireSDAccess(SD_ACCESS_TRACK_PARSE)) {
    debugln(F("Settings: Cannot acquire SD access for defaults"));
    return false;
  }

  settingsJson.clear();

  // Generate random BLE name: DovesDataLogger-XXX
  char nameSuffix[4];
  int randVal = random(0, 1000);
  snprintf(nameSuffix, sizeof(nameSuffix), "%03d", randVal);

  char defaultName[32];
  snprintf(defaultName, sizeof(defaultName), "DovesDataLogger-%s", nameSuffix);
  settingsJson["bluetooth_name"] = defaultName;

  // Generate random 4-digit PIN
  char defaultPin[5];
  snprintf(defaultPin, sizeof(defaultPin), "%04d", (int)random(1000, 10000));
  settingsJson["bluetooth_pin"] = defaultPin;

  // Random racing-themed device name so multi-device log dumps stay sorted
  char deviceName[32];
  generateDeviceName(deviceName, sizeof(deviceName));
  settingsJson["device_name"] = deviceName;

  // New default settings
  settingsJson["driver_name"] = "Driver";
  settingsJson["lap_detection_distance"] = "7";
  settingsJson["waypoint_detection_distance"] = "30";
  settingsJson["waypoint_speed"] = "30";

  // Camera pairing: empty until an Insta360 serial is captured/entered
  settingsJson["camera_serial"] = "";

  File settingsFile;
  settingsFile.open(SETTINGS_FILE_PATH, O_WRITE | O_CREAT | O_TRUNC);
  if (!settingsFile) {
    debugln(F("Settings: Failed to create file"));
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return false;
  }

  serializeJson(settingsJson, settingsFile);
  settingsFile.close();
  releaseSDAccess(SD_ACCESS_TRACK_PARSE);

  debug(F("Settings: Created defaults - name: "));
  debug(defaultName);
  debug(F(", pin: "));
  debugln(defaultPin);

  return true;
}

/**
 * @brief Ensure all expected settings keys exist, adding missing ones with defaults.
 * Called when settings file already exists to handle firmware upgrades that add new keys.
 */
static void ensureDefaultSettings() {
  // Plan 0013 renamed `rev_limit` to `target_rpm`. Migrate BEFORE the
  // defaults table runs: the table only writes a key that is missing, so
  // once target_rpm exists it is left alone — but if we let the table go
  // first it would stamp the 15000 default over a user's tuned 7550 and
  // the old value would be gone. The old key is only dropped once the
  // new one is confirmed written, so a failed or refused write leaves the
  // device exactly as it was rather than silently resetting its shift
  // point.
  {
    char legacyBuf[48];
    if (!getSetting("target_rpm", legacyBuf, sizeof(legacyBuf)) &&
        getSetting("rev_limit", legacyBuf, sizeof(legacyBuf))) {
      debug(F("Settings: migrating rev_limit -> target_rpm = "));
      debugln(legacyBuf);
      if (setSetting("target_rpm", legacyBuf)) {
        removeSetting("rev_limit");
      }
    }
  }

  // Table of all expected settings with their defaults
  static const struct { const char* key; const char* defaultValue; } defaults[] = {
    { "driver_name", "Driver" },
    { "lap_detection_distance", "7" },
    { "waypoint_detection_distance", "30" },
    { "waypoint_speed", "30" },
    { "camera_serial", "" },  // empty = no Insta360 paired
    { "race_mode", "circuit" },  // tiebreak pref when circuit AND sprint tracks are in range
    // Engine geometry for true RPM (plans 0003, 0014). spark_mode is the ONLY
    // one of these two that reaches the RPM math — one clamp on one plug wire
    // sees one cylinder's ignition regardless of what the engine has.
    { "spark_mode", "wasted" },   // "wasted" = 1 spark/rev (2T or 4T wasted); "single" = 1 per 2 revs
    // The engine's ACTUAL cylinder count — no pickup-placement mind game, and
    // no longer a divider (plan 0014). Descriptive: >1 means crank speed is
    // inferred from one cylinder's firing rate, which is what the settings UI
    // warns about.
    { "cylinder_count", "1" },
    // RPM estimator (plan 0009). "smooth" = outlier gate + RPM-aware noise
    // model; "legacy" = the pre-0009 filter for A/B against older logs;
    // "raw" = no filtering at all, so a track session shows exactly what the
    // pickup delivers. Anything else reads as "smooth".
    { "tach_filter", "smooth" },
    // Panel colours. "normal" is lit-on-black, exactly as shipped; "inverted"
    // swaps lit and unlit pixels for glare/daylight readability.
    { "display_invert", "normal" },
    // Race-rotation debug pages (GPS/RF DEBUG + GPS STATS). "hide" (default)
    // starts the rotation at the speed page — end users never see the
    // diagnostic counters; "show" restores them for development/tuning.
    { "debug_pages", "hide" },
    // NeoPixel strip (plan 0006). Read on every channel since 4.1.0 —
    // BIRDSEYE_ENABLE_NEOPIXEL defaults to 1, so these are live
    // settings on a stock logger, not beta-only bookkeeping.
    { "led_brightness", "64" },  // global cap 0-255; 0 = LEDs disabled
    // The SHIFT/warning point, not a limiter (plan 0013 renamed it from
    // "rev_limit", which taught every new user the wrong thing —
    // overrev_limit below is the actual problem limit). LED scale
    // ceiling + the `rpm` status-LED flasher.
    { "target_rpm", "15000" },
    // Plan 0007: the PROBLEM limit (0 = disabled) — whole LED chain
    // flashes red past it, and the tach page's OVER REV header.
    { "overrev_limit", "0" },
#if BIRDSEYE_ENABLE_SENSOREGG
    // Temp1 alert threshold in Celsius. SensorEgg builds only (plan
    // 0013): with the POC compiled out there is no probe to have a
    // threshold for, and the key would just spend settings-file bytes
    // on every stock device.
    { "temp1_alert_c", "650" },
    // Paired egg MAC "AA:BB:CC:DD:EE:FF"; empty = unpaired = accept any
    // PW-ADV egg (plan 0017). Written by the Egg menu's window-gated
    // capture (persist-first); applied LIVE on pair/unpair — one of the
    // two settings exempt from the next-boot rule, like camera_serial.
    { "sensoregg_mac", "" },
#endif
    // Plan 0013: what each status LED shows, and the speed the 9-px bar
    // scales against on a session with no tachometer. Mode tokens are
    // led_status::modeName(): off / rpm / speed / gps / camera / lap /
    // sector / egt. `egt` renders dark on a build without SensorEgg
    // support — the setting still round-trips to the companion app.
    { "led_status_left", "rpm" },
#if BIRDSEYE_ENABLE_SENSOREGG
    { "led_status_right", "egt" },
#else
    { "led_status_right", "lap" },
#endif
    { "target_speed_mph", "60" },  // mph on device; the app converts
    // Plan 0010: minutes east of UTC (US Central standard = -360, India
    // = 330). PRESENTATION ONLY — logged timestamps stay UTC. The only
    // consumer today is the LED day/night brightness swap below, which
    // needs "7am" to mean the driver's 7am. NO DST: a fixed offset walks
    // an hour twice a year, which is noise for a dim-after-dark gate.
    { "utc_offset_min", "0" },
    // Night brightness cap and the LOCAL wall-clock hours the swap
    // happens on. Equal hours disable the swap (led_brightness applies
    // around the clock). 0 = strip dark at night, but the 5 V rail
    // stays up — only led_brightness 0 cuts the rail.
    { "led_brightness_night", "16" },
    { "led_day_start_hour", "7" },
    { "led_night_start_hour", "19" },
  };

  char buf[48];
  for (int i = 0; i < (int)(sizeof(defaults) / sizeof(defaults[0])); i++) {
    if (!getSetting(defaults[i].key, buf, sizeof(buf))) {
      setSetting(defaults[i].key, defaults[i].defaultValue);
    }
  }

  // device_name needs random generation, not a static default
  if (!getSetting("device_name", buf, sizeof(buf))) {
    char dev[32];
    generateDeviceName(dev, sizeof(dev));
    setSetting("device_name", dev);
  }

  // BLE keys need random generation, not static defaults
  if (!getSetting("bluetooth_name", buf, sizeof(buf))) {
    char name[32];
    snprintf(name, sizeof(name), "DovesDataLogger-%03d", (int)random(0, 1000));
    setSetting("bluetooth_name", name);
  }
  if (!getSetting("bluetooth_pin", buf, sizeof(buf))) {
    char pin[5];
    snprintf(pin, sizeof(pin), "%04d", (int)random(1000, 10000));
    setSetting("bluetooth_pin", pin);
  }
}

/**
 * @brief Quarantine a corrupt settings file and regenerate defaults.
 * The unparseable file is renamed to /SETTINGS.json.bad (overwriting any
 * earlier quarantine) so it can be inspected on a computer, then a fresh
 * default file is created. Before this, a corrupt file was a dead end:
 * setSetting() aborted on the parse error and createDefaultSettings() only
 * ran when the file didn't exist, so every setting silently failed forever
 * with no on-device recovery.
 * @return true when a fresh default file is in place
 */
static bool settingsQuarantineCorrupt() {
  debugln(F("Settings: corrupt file — quarantining to /SETTINGS.json.bad"));
  if (!acquireSDAccess(SD_ACCESS_TRACK_PARSE)) {
    debugln(F("Settings: Cannot acquire SD for quarantine"));
    return false;
  }
  if (SD.exists(SETTINGS_BAD_PATH)) {
    SD.remove(SETTINGS_BAD_PATH);  // one quarantine slot — overwrite the old one
  }
  if (!SD.rename(SETTINGS_FILE_PATH, SETTINGS_BAD_PATH)) {
    // Rename failing (odd FAT state) must not leave the corrupt file in
    // place — a fresh default file matters more than preserving the bad one.
    SD.remove(SETTINGS_FILE_PATH);
  }
  releaseSDAccess(SD_ACCESS_TRACK_PARSE);
  return createDefaultSettings();
}

/**
 * @brief True when /SETTINGS.json is readable and parses as JSON. An empty
 * or missing file counts as fine — those self-heal through the existing
 * default-population paths; only a non-empty unparseable file is corrupt.
 */
static bool settingsFileParses() {
  if (!acquireSDAccess(SD_ACCESS_TRACK_PARSE)) return true;  // can't check now
  File settingsFile;
  settingsFile.open(SETTINGS_FILE_PATH, O_READ);
  if (!settingsFile) {
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return true;
  }
  int bytesRead = settingsFile.read(settingsFileBuffer, sizeof(settingsFileBuffer) - 1);
  settingsFile.close();
  releaseSDAccess(SD_ACCESS_TRACK_PARSE);
  if (bytesRead <= 0) return true;  // empty: ensureDefaultSettings rebuilds it
  settingsFileBuffer[bytesRead] = '\0';
  settingsJson.clear();
  return deserializeJson(settingsJson, settingsFileBuffer) == DeserializationError::Ok;
}

/**
 * @brief Initialize settings subsystem. Creates default file on first boot.
 * Call once from setup() after SD is initialized.
 * @return true if settings file exists (or was created)
 */
bool SETTINGS_SETUP() {
  // Seed random for generating default BLE name and PIN
  // WARNING: Do NOT use analogRead() on ANY pin here! On nRF52840, analogRead()
  // permanently disables the digital input buffer on the target pin:
  //   A0 = D0 (tach ISR), A1-A3 = buttons, A4 = SDA, A5 = SCL
  // micros() provides sufficient entropy — this only runs on first boot.
  randomSeed(micros());

  if (!sdSetupSuccess) {
    debugln(F("Settings: SD not available, skipping"));
    return false;
  }

  if (!SD.exists(SETTINGS_FILE_PATH)) {
    debugln(F("Settings: First boot, creating defaults"));
    return createDefaultSettings();
  }

  // A corrupt (hand-edited) file used to poison every get/set forever —
  // quarantine it and start fresh before the missing-key pass.
  if (!settingsFileParses()) {
    return settingsQuarantineCorrupt();
  }

  debugln(F("Settings: File exists, checking for missing keys"));
  ensureDefaultSettings();
  return true;
}

/**
 * @brief Read a setting value by key
 * @param key JSON key to look up
 * @param buf Caller-provided buffer for the value (null-terminated)
 * @param bufSize Size of buf (value truncated to bufSize-1 if needed)
 * @return true if key found and copied, false on any failure
 */
bool getSetting(const char* key, char* buf, size_t bufSize) {
  buf[0] = '\0';

  if (!sdSetupSuccess) return false;

  if (!acquireSDAccess(SD_ACCESS_TRACK_PARSE)) {
    debugln(F("Settings: Cannot acquire SD for read"));
    return false;
  }

  File settingsFile;
  settingsFile.open(SETTINGS_FILE_PATH, O_READ);
  if (!settingsFile) {
    debugln(F("Settings: Cannot open file for read"));
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return false;
  }

  int bytesRead = settingsFile.read(settingsFileBuffer, sizeof(settingsFileBuffer) - 1);
  settingsFile.close();
  releaseSDAccess(SD_ACCESS_TRACK_PARSE);

  if (bytesRead <= 0) {
    debugln(F("Settings: Empty or unreadable file"));
    return false;
  }
  settingsFileBuffer[bytesRead] = '\0';

  settingsJson.clear();
  DeserializationError err = deserializeJson(settingsJson, settingsFileBuffer);
  if (err != DeserializationError::Ok) {
    debug(F("Settings: JSON parse error: "));
    debugln(err.c_str());
    return false;
  }

  if (!settingsJson.containsKey(key)) {
    debug(F("Settings: Key not found: "));
    debugln(key);
    return false;
  }

  const char* value = settingsJson[key];
  if (value == nullptr) return false;

  strncpy(buf, value, bufSize - 1);
  buf[bufSize - 1] = '\0';
  return true;
}

/**
 * @brief Write or update a setting value
 * @param key JSON key to set
 * @param value String value to store
 * @return true on success, false on failure
 */
/**
 * @brief Delete settings file and recreate with fresh defaults (new random BLE name/PIN).
 * @return true on success
 */
bool resetSettings() {
  if (!sdSetupSuccess) return false;

  if (!acquireSDAccess(SD_ACCESS_TRACK_PARSE)) {
    debugln(F("Settings: Cannot acquire SD for reset"));
    return false;
  }

  if (SD.exists(SETTINGS_FILE_PATH)) {
    SD.remove(SETTINGS_FILE_PATH);
  }
  releaseSDAccess(SD_ACCESS_TRACK_PARSE);

  debugln(F("Settings: Deleted, recreating defaults"));
  return createDefaultSettings();
}

static bool setSettingInner(const char* key, const char* value, bool healCorrupt) {
  if (!sdSetupSuccess) return false;

  if (!acquireSDAccess(SD_ACCESS_TRACK_PARSE)) {
    debugln(F("Settings: Cannot acquire SD for write"));
    return false;
  }

  // Read existing file first
  settingsJson.clear();

  File settingsFile;
  settingsFile.open(SETTINGS_FILE_PATH, O_READ);
  if (settingsFile) {
    int bytesRead = settingsFile.read(settingsFileBuffer, sizeof(settingsFileBuffer) - 1);
    settingsFile.close();
    if (bytesRead > 0) {
      settingsFileBuffer[bytesRead] = '\0';
      DeserializationError err = deserializeJson(settingsJson, settingsFileBuffer);
      if (err != DeserializationError::Ok) {
        debug(F("Settings: Parse error on read-modify-write: "));
        debugln(err.c_str());
        releaseSDAccess(SD_ACCESS_TRACK_PARSE);
        // Corrupt mid-session (boot already healed once): quarantine +
        // regenerate, then apply this write to the fresh file. Single
        // retry — the regenerated file always parses, so no recursion.
        if (healCorrupt && settingsQuarantineCorrupt()) {
          return setSettingInner(key, value, false);
        }
        return false;
      }
    }
  }

  // Update the key
  settingsJson[key] = value;

  // Refuse a write that could not be read back afterwards. Two ways it
  // could quietly destroy the whole file:
  //   overflowed() - the document ran out of slots, so the assignment
  //     above silently did nothing and serializing now would drop the
  //     key. (The flag is sticky, but settingsJson.clear() above resets
  //     it, so here it can only mean THIS parse+assign.)
  //   measureJson() - the result exceeds settingsFileBuffer. Every read
  //     path caps at sizeof(settingsFileBuffer) - 1, so such a file
  //     parses as IncompleteInput and destroys ALL settings, not just
  //     this key — and the boot-time corrupt-file heal then loops
  //     forever. measureJson() needs no scratch buffer, which matters
  //     here: settingsFileBuffer is off limits because the zero-copy
  //     parse above left the document pointing INTO it.
  // Refusing one write leaves the existing file intact and readable.
  if (settingsJson.overflowed() ||
      measureJson(settingsJson) > sizeof(settingsFileBuffer) - 1) {
    debug(F("Settings: REFUSING write, would not be readable back: "));
    debugln(key);
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return false;
  }

  // Write back (truncate and rewrite)
  settingsFile.open(SETTINGS_FILE_PATH, O_WRITE | O_CREAT | O_TRUNC);
  if (!settingsFile) {
    debugln(F("Settings: Failed to open file for write"));
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return false;
  }

  serializeJson(settingsJson, settingsFile);
  settingsFile.close();
  releaseSDAccess(SD_ACCESS_TRACK_PARSE);

  debug(F("Settings: Saved "));
  debug(key);
  debug(F(" = "));
  debugln(value);

  return true;
}

bool setSetting(const char* key, const char* value) {
  return setSettingInner(key, value, /*healCorrupt=*/true);
}

bool removeSetting(const char* key) {
  if (!sdSetupSuccess) return false;

  if (!acquireSDAccess(SD_ACCESS_TRACK_PARSE)) {
    debugln(F("Settings: Cannot acquire SD for remove"));
    return false;
  }

  settingsJson.clear();

  File settingsFile;
  settingsFile.open(SETTINGS_FILE_PATH, O_READ);
  if (!settingsFile) {
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return false;  // nothing to remove from
  }
  int bytesRead = settingsFile.read(settingsFileBuffer, sizeof(settingsFileBuffer) - 1);
  settingsFile.close();
  if (bytesRead <= 0) {
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return false;
  }
  settingsFileBuffer[bytesRead] = '\0';
  DeserializationError err = deserializeJson(settingsJson, settingsFileBuffer);
  if (err != DeserializationError::Ok) {
    debug(F("Settings: Parse error on remove: "));
    debugln(err.c_str());
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return false;  // deliberately NOT healed here — see the header note
  }

  if (!settingsJson.containsKey(key)) {
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return true;  // already gone: the caller's post-condition holds
  }
  settingsJson.remove(key);

  // No size guard needed: a removal can only shrink the file, so it can
  // never cross the read cap that setSettingInner() has to defend.
  settingsFile.open(SETTINGS_FILE_PATH, O_WRITE | O_CREAT | O_TRUNC);
  if (!settingsFile) {
    debugln(F("Settings: Failed to open file for remove"));
    releaseSDAccess(SD_ACCESS_TRACK_PARSE);
    return false;
  }
  serializeJson(settingsJson, settingsFile);
  settingsFile.close();
  releaseSDAccess(SD_ACCESS_TRACK_PARSE);

  debug(F("Settings: Removed "));
  debugln(key);
  return true;
}
