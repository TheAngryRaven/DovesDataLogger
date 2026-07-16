///////////////////////////////////////////
// native_main.cpp — Phase-1 native driver.
//
// Boots the firmware, presses Select once to leave the boot GPS-status
// page (no fix is ever injected in Phase 1), then runs 60 seconds of
// virtual time and prints a state line every virtual second.
//
// The output is fully deterministic (virtual clock, fixed VFS assets):
// the done-criteria check runs this binary twice and diffs the output.
///////////////////////////////////////////

#include <cstdio>
#include <cstring>

#include "sim_host.h"

static void printState(const char* tag) {
  std::printf("[%6u ms] %s page=%d race=%d laps=%d rpm=%d fix=%d logging=%d\n",
              sim_millis(), tag, sim_current_page(), sim_race_active(),
              sim_lap_count(), sim_rpm(), sim_gps_fix(),
              sim_logging_active());
}

int main(int argc, char** argv) {
  const bool quiet = (argc > 1 && std::strcmp(argv[1], "--quiet") == 0);
  (void)quiet;

  std::printf("--- sim boot ---\n");
  sim_init();
  printState("boot");

  // 2 s on the GPS status page, then a Select press skips it (any button
  // skips — gps_status_page::Exit::kToMenu, engine off).
  sim_step_millis(2000);
  printState("pre-skip");

  sim_button_down(1);
  sim_step_millis(120);  // > the 3-sample debounce + one loop pass
  sim_button_up(1);
  sim_step_millis(100);
  printState("post-skip");

  // Balance of the 60 s soak on the main menu.
  for (int sec = 3; sec <= 60; sec++) {
    if (sim_step_millis(1000) < 0) {
      std::printf("UNEXPECTED firmware reset at %u ms\n", sim_millis());
      return 1;
    }
    if (sec % 5 == 0) printState("soak");
  }

  printState("final");

  if (sim_reset_requested()) {
    std::printf("FAIL: firmware requested reset during soak\n");
    return 1;
  }
  std::printf("--- sim ok ---\n");
  return 0;
}
