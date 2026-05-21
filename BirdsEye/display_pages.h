#pragma once

///////////////////////////////////////////
// DISPLAY PAGES MODULE
// Per-page render functions. Each is called from displayLoop() in
// display_ui.cpp/.ino based on currentPage. No I/O outside of display
// writes (and read-only reads of shared GPS / timer / battery state).
///////////////////////////////////////////

// Menu / chrome.
void displayPage_boot();
void displayPage_main_menu();
void displayPage_bluetooth();
void displayPage_internal_fault();
void displayPage_internal_warning();
void displayPage_sleep_charging();

// Replay flow (DOVEX instant replay).
void displayPage_replay_file_select();
void displayPage_replay_results();
void displayPage_replay_exit();

// Live racing pages.
void displayPage_gps_stats();
void displayPage_gps_speed();
void displayPage_tachometer();
void displayPage_gps_lap_time();
void displayPage_gps_pace();
void displayPage_gps_best_lap();
void displayPage_optimal_lap();
void displayPage_gps_lap_list();
void displayPage_gps_debug();

// Stop-logging confirmation.
void displayPage_stop_logging();
void displayPage_stop_logging_confirm();

// Animated overlay shown while DovesLapTimer is mid-crossing.
void displayCrossing();
