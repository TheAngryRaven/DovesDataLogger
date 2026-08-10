#pragma once

#include "Arduino.h"

#include <string>
#include <vector>

///////////////////////////////////////////
// SdFat shim — in-memory VFS (sim build).
//
// Implements exactly the SdFat subset the firmware calls (enumerated
// from sd_functions.ino / settings.ino / gps_functions.ino / replay.ino
// / BirdsEye.ino), backed by a std::map filesystem in sim_vfs.cpp.
// Preloaded at boot with assets/SETTINGS.json (fixed values, so boots
// are deterministic) and assets/TRACKS/OKC.json. DOVEX logs written
// during a run land in the VFS and evaporate with it — persistence is
// explicitly out of the demo's scope.
//
// This is NOT real SdFat over a RAM block device (deliberately — see the
// sim handoff spec): "formatting" just wipes the VFS, "SPI clocks" are
// accepted and ignored, and every card-level probe succeeds.
///////////////////////////////////////////

// ---- open flags (values match SdFat's FatLib O_* constants) ----
#define O_READ 0x01
#define O_WRITE 0x02
#define O_RDWR 0x03
#define O_CREAT 0x40
#define O_TRUNC 0x200
#define O_APPEND 0x400
typedef int oflag_t;

// ---- SPI config surface (accepted, ignored) ----
#define SD_SCK_MHZ(mhz) ((uint32_t)(mhz)*1000000UL)
#define SHARED_SPI 0

struct SdSpiConfig {
  SdSpiConfig(int cs, int opts, uint32_t maxSck) {
    (void)cs;
    (void)opts;
    (void)maxSck;
  }
};

// ------------------------------------------------------------------ File32
class File32 : public Print {
 public:
  File32() {}

  // Open by absolute path ("/SETTINGS.json", "/TRACKS/x.json", "/").
  bool open(const char* path, oflag_t oflag = O_READ);
  // Open the next entry of an open directory (SdFat's two spellings).
  bool openNext(File32* dir, oflag_t oflag = O_READ);
  File32 openNextFile();

  void close();
  bool isOpen() const { return open_; }
  explicit operator bool() const { return open_; }

  bool isDirectory() const { return isDir_; }
  bool isDir() const { return isDir_; }

  void rewind() { dirIndex_ = 0; }

  size_t getName(char* name, size_t size) const;

  int read();                          // single byte, -1 on EOF
  int read(void* buf, size_t count);   // -1 on error, else bytes read
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* buf, size_t size) override;
  size_t write(const void* buf, size_t count) {
    return write(static_cast<const uint8_t*>(buf), count);
  }
  using Print::write;

  bool seekSet(uint32_t pos);
  uint32_t position() const { return pos_; }
  uint32_t size() const;
  uint32_t fileSize() const { return size(); }
  int available();

  void flush() {}
  bool sync() { return true; }

 private:
  bool openDir(const std::string& path);
  bool openFile(const std::string& path, oflag_t oflag);

  bool open_ = false;
  bool isDir_ = false;
  bool writable_ = false;
  std::string path_;  // normalized absolute path
  uint32_t pos_ = 0;
  // Directory iteration: snapshot of child names taken at open()/rewind().
  std::vector<std::string> dirEntries_;
  size_t dirIndex_ = 0;

  friend class SdFat;
};

typedef File32 File;

// ------------------------------------------------------------------ SdCard
// Card-level view. The VFS has no sectors; report a plausible 8 GB card.
class SimSdCard {
 public:
  uint32_t sectorCount() { return 8UL * 1024UL * 1024UL * 2UL; }
  bool readSectors(uint32_t, uint8_t*, size_t) { return false; }
  bool writeSectors(uint32_t, const uint8_t*, size_t) { return false; }
  bool syncDevice() { return true; }
};

// ------------------------------------------------------------------- SdFat
class SdFat {
 public:
  bool begin(int csPin, uint32_t maxSck);
  bool cardBegin(const SdSpiConfig&) { return true; }
  bool volumeBegin() { return true; }
  SimSdCard* card() { return &card_; }
  void cacheClear() {}

  bool exists(const char* path);
  bool mkdir(const char* path);
  bool remove(const char* path);
  // Used by the on-device course creator's write-to-temp-then-swap, so a
  // power loss mid-rewrite can't leave a truncated track file behind.
  bool rename(const char* oldPath, const char* newPath);
  File32 open(const char* path, oflag_t oflag = O_READ);

  // On-device format: wipes the VFS clean (no FAT to build).
  bool format(Print* progress);

 private:
  SimSdCard card_;
};

// ---- host/VFS control surface (sim glue + native driver) ----
namespace sim_vfs {

// Wipe and re-preload the bundled assets (fresh boot).
void reset();

// Raw file access for the host API (readFile in the sim contract).
bool readFileBytes(const char* path, std::vector<uint8_t>& out);
bool writeFileBytes(const char* path, const uint8_t* data, size_t len);
std::vector<std::string> listFiles();

}  // namespace sim_vfs
