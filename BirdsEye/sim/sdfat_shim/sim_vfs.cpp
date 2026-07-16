#include "SdFat.h"

#include <algorithm>
#include <map>
#include <set>

///////////////////////////////////////////
// In-memory VFS backing the SdFat shim.
//
// Files:       absolute normalized path -> byte vector.
// Directories: explicit set (root "/" always present) so exists()/mkdir()
//              and directory listings behave like a FAT volume's.
// Preload:     the bundled assets (embedded at build time by
//              cmake/bin2h.cmake -> sim_assets.h) so every boot sees the
//              same fixed SETTINGS.json + example track.
///////////////////////////////////////////

#include "sim_assets.h"  // generated: kAssetSettingsJson / kAssetTrackOkcJson

namespace {

std::map<std::string, std::vector<uint8_t>>& files() {
  static std::map<std::string, std::vector<uint8_t>> f;
  return f;
}

std::set<std::string>& dirs() {
  static std::set<std::string> d{"/"};
  return d;
}

// "/TRACKS/x.json", "TRACKS/x.json", "//TRACKS//x.json" -> "/TRACKS/x.json"
std::string normalize(const char* raw) {
  std::string p = raw ? raw : "";
  std::string out = "/";
  size_t i = 0;
  while (i < p.size()) {
    while (i < p.size() && p[i] == '/') i++;
    size_t start = i;
    while (i < p.size() && p[i] != '/') i++;
    if (i > start) {
      if (out.back() != '/') out += '/';
      out += p.substr(start, i - start);
    }
  }
  return out;
}

std::string parentOf(const std::string& path) {
  size_t slash = path.find_last_of('/');
  if (slash == 0 || slash == std::string::npos) return "/";
  return path.substr(0, slash);
}

std::string basenameOf(const std::string& path) {
  size_t slash = path.find_last_of('/');
  return path.substr(slash + 1);
}

}  // namespace

namespace sim_vfs {

void reset() {
  files().clear();
  dirs().clear();
  dirs().insert("/");
  dirs().insert("/TRACKS");
  files()["/SETTINGS.json"] = std::vector<uint8_t>(
      kAssetSettingsJson, kAssetSettingsJson + kAssetSettingsJsonLen);
  files()["/TRACKS/OKC.json"] = std::vector<uint8_t>(
      kAssetTrackOkcJson, kAssetTrackOkcJson + kAssetTrackOkcJsonLen);
}

bool readFileBytes(const char* path, std::vector<uint8_t>& out) {
  auto it = files().find(normalize(path));
  if (it == files().end()) return false;
  out = it->second;
  return true;
}

bool writeFileBytes(const char* path, const uint8_t* data, size_t len) {
  files()[normalize(path)] = std::vector<uint8_t>(data, data + len);
  return true;
}

std::vector<std::string> listFiles() {
  std::vector<std::string> out;
  for (const auto& kv : files()) out.push_back(kv.first);
  return out;
}

}  // namespace sim_vfs

// ------------------------------------------------------------------ File32

bool File32::openDir(const std::string& path) {
  open_ = true;
  isDir_ = true;
  writable_ = false;
  path_ = path;
  pos_ = 0;
  dirIndex_ = 0;
  dirEntries_.clear();
  // Snapshot direct children (files then subdirs, name-sorted — a stable,
  // deterministic order; real FAT order is insertion order, which for the
  // sim's fixed assets is equivalent in practice).
  std::set<std::string> names;
  for (const auto& kv : files()) {
    if (parentOf(kv.first) == path) names.insert(basenameOf(kv.first));
  }
  for (const auto& d : dirs()) {
    if (d != "/" && parentOf(d) == path) names.insert(basenameOf(d));
  }
  dirEntries_.assign(names.begin(), names.end());
  return true;
}

bool File32::openFile(const std::string& path, oflag_t oflag) {
  auto& fs = files();
  auto it = fs.find(path);
  const bool wantWrite = (oflag & O_WRITE) != 0;

  if (it == fs.end()) {
    if (!(oflag & O_CREAT)) return false;
    // Parent must exist (SdFat's open() never creates parent dirs).
    const std::string parent = parentOf(path);
    if (parent != "/" && dirs().find(parent) == dirs().end()) return false;
    fs[path] = {};
  } else if (oflag & O_TRUNC) {
    it->second.clear();
  }

  open_ = true;
  isDir_ = false;
  writable_ = wantWrite;
  path_ = path;
  pos_ = (oflag & O_APPEND) ? (uint32_t)fs[path].size() : 0;
  return true;
}

bool File32::open(const char* path, oflag_t oflag) {
  if (open_) return false;  // matches SdFat: opening an open file fails
  const std::string p = normalize(path);
  if (dirs().count(p)) return openDir(p);
  return openFile(p, oflag);
}

bool File32::openNext(File32* dir, oflag_t oflag) {
  if (open_ || !dir || !dir->open_ || !dir->isDir_) return false;
  while (dir->dirIndex_ < dir->dirEntries_.size()) {
    const std::string& name = dir->dirEntries_[dir->dirIndex_++];
    const std::string full =
        (dir->path_ == "/") ? "/" + name : dir->path_ + "/" + name;
    if (dirs().count(full)) return openDir(full);
    if (files().count(full)) return openFile(full, oflag);
    // entry vanished since the snapshot — skip it
  }
  return false;
}

File32 File32::openNextFile() {
  File32 f;
  f.openNext(this, O_READ);
  return f;
}

void File32::close() {
  open_ = false;
  isDir_ = false;
  writable_ = false;
  path_.clear();
  pos_ = 0;
  dirEntries_.clear();
  dirIndex_ = 0;
}

size_t File32::getName(char* name, size_t size) const {
  if (!name || size == 0) return 0;
  if (!open_) {
    name[0] = '\0';
    return 0;
  }
  const std::string base = (path_ == "/") ? "/" : basenameOf(path_);
  strncpy(name, base.c_str(), size - 1);
  name[size - 1] = '\0';
  return strlen(name);
}

int File32::read() {
  uint8_t c;
  return read(&c, 1) == 1 ? c : -1;
}

int File32::read(void* buf, size_t count) {
  if (!open_ || isDir_) return -1;
  auto it = files().find(path_);
  if (it == files().end()) return -1;
  const auto& data = it->second;
  if (pos_ >= data.size()) return 0;
  const size_t n = std::min(count, data.size() - pos_);
  memcpy(buf, data.data() + pos_, n);
  pos_ += (uint32_t)n;
  return (int)n;
}

size_t File32::write(uint8_t c) { return write(&c, 1); }

size_t File32::write(const uint8_t* buf, size_t size) {
  if (!open_ || isDir_ || !writable_) return 0;
  auto it = files().find(path_);
  if (it == files().end()) return 0;
  auto& data = it->second;
  if (pos_ + size > data.size()) data.resize(pos_ + size);
  memcpy(data.data() + pos_, buf, size);
  pos_ += (uint32_t)size;
  return size;
}

bool File32::seekSet(uint32_t pos) {
  if (!open_ || isDir_) return false;
  if (pos > size()) return false;
  pos_ = pos;
  return true;
}

uint32_t File32::size() const {
  if (!open_ || isDir_) return 0;
  auto it = files().find(path_);
  return it == files().end() ? 0 : (uint32_t)it->second.size();
}

int File32::available() {
  const uint32_t s = size();
  return pos_ < s ? (int)(s - pos_) : 0;
}

// ------------------------------------------------------------------- SdFat

bool SdFat::begin(int, uint32_t) { return true; }

bool SdFat::exists(const char* path) {
  const std::string p = normalize(path);
  return files().count(p) > 0 || dirs().count(p) > 0;
}

bool SdFat::mkdir(const char* path) {
  const std::string p = normalize(path);
  if (files().count(p)) return false;
  const std::string parent = parentOf(p);
  if (parent != "/" && dirs().find(parent) == dirs().end()) return false;
  dirs().insert(p);
  return true;
}

bool SdFat::remove(const char* path) {
  return files().erase(normalize(path)) > 0;
}

File32 SdFat::open(const char* path, oflag_t oflag) {
  File32 f;
  f.open(path, oflag);
  return f;
}

bool SdFat::format(Print* progress) {
  if (progress) progress->write((uint8_t)'.');
  files().clear();
  dirs().clear();
  dirs().insert("/");
  return true;
}
