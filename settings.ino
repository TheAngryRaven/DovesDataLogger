///////////////////////////////////////////
// SETTINGS MODULE
// Persistent JSON key-value store on SD card (/SETTINGS.json)
// Provides getSetting() and setSetting() for any subsystem.
///////////////////////////////////////////

static const char SETTINGS_FILE_PATH[] = "/SETTINGS.json";
static char settingsFileBuffer[512];
static StaticJsonDocument<512> settingsJson;

/**
 * @brief Generate default settings file on first boot
 * @return true if file created successfully
 */
static bool createDefaultSettings() {
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

  // New default settings
  settingsJson["driver_name"] = "Driver";
  settingsJson["lap_detection_distance"] = "7";
  settingsJson["waypoint_detection_distance"] = "30";
  settingsJson["use_legacy_csv"] = "false";
  settingsJson["waypoint_speed"] = "30";

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
  // Table of all expected settings with their defaults
  static const struct { const char* key; const char* defaultValue; } defaults[] = {
    { "driver_name", "Driver" },
    { "lap_detection_distance", "7" },
    { "waypoint_detection_distance", "30" },
    { "use_legacy_csv", "false" },
    { "waypoint_speed", "30" },
  };

  char buf[48];
  for (int i = 0; i < (int)(sizeof(defaults) / sizeof(defaults[0])); i++) {
    if (!getSetting(defaults[i].key, buf, sizeof(buf))) {
      setSetting(defaults[i].key, defaults[i].defaultValue);
    }
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
bool setSetting(const char* key, const char* value) {
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
        debug(F("Settings: Parse error on read-modify-write, aborting: "));
        debugln(err.c_str());
        releaseSDAccess(SD_ACCESS_TRACK_PARSE);
        return false;
      }
    }
  }

  // Update the key
  settingsJson[key] = value;

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
