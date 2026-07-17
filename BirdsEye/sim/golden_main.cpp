///////////////////////////////////////////
// golden_main.cpp — Phase-2 golden display fixtures.
//
// Boots the firmware and walks REAL menu navigation (button presses
// through the pin map, virtual time between them), capturing the
// FNV-1a hash of the real Adafruit framebuffer at fixed virtual times.
// Each fixture also asserts the page id it expects, so the goldens lock
// both the pixels AND the navigation that produced them.
//
// Modes:
//   (none)            compare against golden/golden_hashes.txt (CI mode)
//   --print           print the fixture table (regeneration aid)
//   --dump <dir>      also write a 4x-scaled PNG per fixture (eyeballing)
//
// To update goldens after an intentional display change:
//   birdseye_sim_golden --print > sim/golden/golden_hashes.txt
///////////////////////////////////////////

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "png_dump.h"
#include "sim_host.h"

namespace {

struct Fixture {
  std::string name;
  int page;
  uint32_t hash;
};

std::vector<Fixture> g_fixtures;
const char* g_dumpDir = nullptr;

// One debounced press+release: hold long enough for the 3-sample read
// in at least one loop pass, release, then wait out the 200 ms refire
// lockout so consecutive presses all register.
void press(int idx) {
  sim_button_down(idx);
  sim_step_millis(120);
  sim_button_up(idx);
  sim_step_millis(280);
}

void capture(const char* name, int expectPage) {
  // One display period (3 Hz) has always elapsed inside press(); make
  // sure a render definitely happened at a fixed virtual time anyway.
  sim_step_millis(400);
  Fixture f;
  f.name = name;
  f.page = sim_current_page();
  f.hash = sim_frame_hash();
  g_fixtures.push_back(f);
  if (expectPage != f.page) {
    std::printf("FIXTURE %s: expected page %d, got %d\n", name, expectPage,
                f.page);
    std::exit(1);
  }
  if (g_dumpDir) {
    char path[512];
    std::snprintf(path, sizeof(path), "%s/%s.png", g_dumpDir, name);
    if (!png_dump::writeFramebufferPng(path, sim_framebuffer(), 4)) {
      std::printf("FIXTURE %s: PNG dump failed (%s)\n", name, path);
      std::exit(1);
    }
  }
}

// Page ids duplicated from BirdsEye.ino's constants (the driver is
// outside the firmware TU). The golden test breaks loudly if they drift.
constexpr int kPageGpsStatus = 900;
constexpr int kPageMainMenu = -1;
constexpr int kPageTransferMenu = -4;
constexpr int kPageBluetooth = -2;
constexpr int kPagePairCamera = -6;
constexpr int kPageCameraSerialEntry = -7;
constexpr int kPageWarning = 100;

void runScript() {
  sim_init();

  // Boot lands on the GPS status page; no fix is ever injected here.
  sim_step_millis(2000);
  capture("gps_status_no_fix", kPageGpsStatus);

  // Any button skips the status page -> main menu (Race highlighted).
  press(1);
  capture("main_menu_race", kPageMainMenu);

  // Main menu renders top-to-bottom; btn3 walks down the list.
  press(2);
  press(2);
  capture("main_menu_transfer", kPageMainMenu);

  // Select -> Transfer submenu (Bluetooth / USB).
  press(1);
  capture("transfer_menu", kPageTransferMenu);

  // Select Bluetooth -> BLE page (stub radio: waiting-for-connection UI).
  press(1);
  capture("bluetooth_waiting", kPageBluetooth);

  // Exit BLE page -> menu; walk down once -> Replay; select -> the VFS
  // has no .dovex files, so the firmware shows the warning page.
  press(1);
  press(2);
  press(1);
  capture("warning_no_dovex", kPageWarning);

  // Any button dismisses the warning back to the menu; walk to Camera
  // (index 3) and select -> unpaired pairing screen.
  press(1);
  press(2);
  press(2);
  press(2);
  press(1);
  capture("pair_camera_unpaired", kPagePairCamera);

  // Left on the unpaired pairing screen opens the manual 6-char serial
  // entry page (custom button branch) — the eighth fixture.
  press(0);
  capture("camera_serial_entry", kPageCameraSerialEntry);
}

}  // namespace

int main(int argc, char** argv) {
  bool printMode = false;
  const char* goldenPath = nullptr;
  for (int i = 1; i < argc; i++) {
    if (!std::strcmp(argv[i], "--print")) {
      printMode = true;
    } else if (!std::strcmp(argv[i], "--dump") && i + 1 < argc) {
      g_dumpDir = argv[++i];
    } else if (!std::strcmp(argv[i], "--golden") && i + 1 < argc) {
      goldenPath = argv[++i];
    } else {
      std::fprintf(stderr,
                   "usage: %s [--print] [--dump <dir>] [--golden <file>]\n",
                   argv[0]);
      return 2;
    }
  }

  runScript();

  if (printMode || !goldenPath) {
    for (const auto& f : g_fixtures) {
      std::printf("%s %d %08x\n", f.name.c_str(), f.page, f.hash);
    }
    if (printMode) return 0;
  }

  if (!goldenPath) {
    std::fprintf(stderr, "no --golden file given; printed table only\n");
    return 2;
  }

  FILE* gf = std::fopen(goldenPath, "r");
  if (!gf) {
    std::fprintf(stderr, "cannot open golden file %s\n", goldenPath);
    return 2;
  }
  std::vector<Fixture> want;
  char name[128];
  int page;
  unsigned hash;
  while (std::fscanf(gf, "%127s %d %x", name, &page, &hash) == 3) {
    want.push_back({name, page, (uint32_t)hash});
  }
  std::fclose(gf);

  bool ok = want.size() == g_fixtures.size();
  if (!ok) {
    std::printf("golden count mismatch: want %zu, got %zu\n", want.size(),
                g_fixtures.size());
  }
  for (size_t i = 0; ok && i < want.size(); i++) {
    if (want[i].name != g_fixtures[i].name ||
        want[i].page != g_fixtures[i].page ||
        want[i].hash != g_fixtures[i].hash) {
      std::printf("golden mismatch [%zu]: want %s %d %08x, got %s %d %08x\n",
                  i, want[i].name.c_str(), want[i].page, want[i].hash,
                  g_fixtures[i].name.c_str(), g_fixtures[i].page,
                  g_fixtures[i].hash);
      ok = false;
    }
  }

  if (!ok) {
    std::printf(
        "golden fixtures FAILED — if the display change is intentional, "
        "regenerate with --print\n");
    return 1;
  }
  std::printf("golden fixtures OK (%zu pages)\n", g_fixtures.size());
  return 0;
}
