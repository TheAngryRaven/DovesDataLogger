#pragma once

///////////////////////////////////////////
// sim_prototypes.h — hand-written replacement for Arduino's
// auto-generated function prototypes.
//
// The Arduino build concatenates the .ino files and injects prototypes
// for every function so definition order never matters. The sim TU
// (sim_main.cpp) replicates the concatenation; this header replicates
// the prototypes. Two known ordering landmines, mirrored from
// BirdsEye.ino's own top-of-file comments:
//   - project.h must precede everything that names its types.
//   - The SparkFun header must precede the onPVTReceived prototype.
//
// SD_FAT_TYPE note: BirdsEye.ino #defines SD_FAT_TYPE before including
// SdFat.h. The sim's SdFat shim ignores SD_FAT_TYPE entirely (one File32
// type), so this header including it "early" is harmless.
///////////////////////////////////////////

#include <Arduino.h>

#include "project.h"

#include <SparkFun_u-blox_GNSS_v3.h>

#include <CourseManager.h>
#include <DovesLapTimer.h>
#include <SprintTimer.h>

#include "SdFat.h"

#include "course_creator.h"
#include "gps_status_page.h"
#include "sd_format_page.h"
#include "wake_cause.h"

// ---- BirdsEye.ino ----
float getBatteryVoltage();
int getBatteryPercent(float voltage);
void checkForNewLapData();
void wdtSetup();
void wdtPet();
void setup();
void loop();
DovesLapTimer* getActiveTimerDLT();
WaypointLapTimer* getActiveTimerWLT();
bool activeTimerRaceStarted();
bool activeTimerCrossing();
int activeTimerLaps();
unsigned long activeTimerCurrentLapTime();
unsigned long activeTimerLastLapTime();
unsigned long activeTimerBestLapTime();
int activeTimerBestLapNumber();
float activeTimerPaceDifference();
float activeTimerTotalDistance();
unsigned long activeTimerOptimalLapTime();
bool activeTimerSectorsConfigured();
SprintTimer* getActiveTimerSprint();
bool sprintModeIsActive();
bool activeTimerRunActive();
bool createSprintSession();
void trackDetectionLoop();
void endRaceSession();
void createLapAnythingCourseManager();
void checkAutoIdle();
void autoRaceModeCheck();
void gpsStatusPageLoop();
void sdFormatPageLoop();
int courseCreatorPage();
bool courseCreatorActive();
bool courseCreatorEnter();
void courseCreatorSave();
void courseCreatorSelect();
void courseCreatorLoop();
void updateGpsLockHold();
void writeDovexHeader();
bool isUsbConnected();
void enterShutdown();

// ---- accelerometer.ino ----
void ACCEL_SETUP();
void ACCEL_LOOP();

// ---- display_pages.ino ----
void displayPage_boot();
void displayPage_gps_status();
void displayPage_main_menu();
void displayPage_bluetooth();
void displayPage_transfer_menu();
void displayPage_usb_storage();
void displayPage_pair_camera();
void displayPage_camera_test();
void displayPage_course_track();
void displayPage_course_type();
void displayPage_course_lines();
void displayPage_course_line();
void displayPage_course_point();
void displayPage_camera_serial_entry();
void displayPage_replay_file_select();
void displayPage_replay_results();
void displayPage_replay_exit();
void displayPage_gps_stats();
void displayPage_gps_speed();
void displayPage_gps_lap_time();
void displayPage_gps_pace();
void displayPage_gps_best_lap();
void displayPage_tachometer();
void displayPage_sensorTemp();
void displayPage_sensorTemp2();
void displayPage_optimal_lap();
void displayPage_gps_lap_list();
void displayPage_stop_logging();
void displayPage_stop_logging_confirm();
void displayPage_gps_debug();
void displayPage_internal_fault();
void displayPage_sd_format();
void displayPage_sd_format_progress(const __FlashStringHelper* line1,
                                    const __FlashStringHelper* line2);
void displayPage_internal_warning();
void displayPage_sleep_charging();
void displayCrossing();

// ---- display_ui.ino ----
void i2cBusRecover();
void safeDisplayUpdate();
void setupButtons();
void readButtons();
void resetButtons();
void updateButtonHoldState();
bool isButtonHeld(int btnNum, unsigned long durationMs);
bool anyButtonPressed();
void resetButton(ButtonState* button);
bool readButtonMultiSample(int pin);
void checkButton(ButtonState* button);
void resetDisplay();
void forceDisplayRefresh();
void switchToDisplayPage(int newDisplayPage);
void displaySetup();
void handleMenuPageSelection();
void handleRunningPageSelection();
void displayLoop();

// ---- gps_functions.ino ----
void startGpsSerialTimer();
void stopGpsSerialTimer();
unsigned long getGpsTimeInMilliseconds();
unsigned long getGpsUnixTimestamp();
unsigned long long getGpsUnixTimestampMillis();
void onPVTReceived(UBX_NAV_PVT_data_t* pvt);
void onNAVSATReceived(UBX_NAV_SAT_data_t* sat);
void GPS_SETUP();
bool gpsRetriesExhausted();
void GPS_STATUS_RETRY_LOOP();
void GPS_LOOP();
void GPS_SLEEP();
void GPS_RECONFIGURE();
void gpsEnterStatusMode();
void gpsEnterRaceMode();
bool GPS_BAUD_RECOVERY();
void GPS_WAKE();
void calculateGPSFrameRate();
uint32_t gpsStatsDroppedPvt();
uint32_t gpsStatsRingFullEvents();
uint32_t gpsStatsIsrLatencyMaxUs();
uint16_t gpsStatsDrainMaxBytes();
uint32_t gpsStatsCoreSatEvents();

// ---- replay.ino ----
void resetReplayState();
bool buildReplayFileList();
bool readReplayLine(File& file, char* buffer, int bufferSize);
bool parseDovexHeader(const char* filename);

// ---- sd_functions.ino ----
bool acquireSDAccess(int mode);
void releaseSDAccess(int mode);
void forceReleaseSDAccess();
void makeFullTrackPath(const char* trackName, char* filepath, uint8_t kind);
bool scanTrackDir(const char* folder, uint8_t kind);
bool sdSetSpiClock(uint32_t maxSck);
void sdSetTransferSpeed(bool fast);
bool SD_SETUP();
bool sdEnsureTracksFolder();
void sdPerformFormat();
bool buildTrackList();
int parseTrackFile(char* filepath);

// ---- settings.ino ----
bool createDefaultSettings();
bool SETTINGS_SETUP();
bool getSetting(const char* key, char* buf, size_t bufSize);
bool resetSettings();
bool setSetting(const char* key, const char* value);

// ---- tachometer.ino ----
void TACH_COUNT_PULSE();
void TACH_LOOP();
