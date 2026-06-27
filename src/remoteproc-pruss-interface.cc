/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2026 Leonardo Romor <leonardo.romor@gmail.com>
 *
 * This file is part of BeagleG. http://github.com/hzeller/beagleg
 *
 * BeagleG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BeagleG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
 */

// PRU hardware interface using the kernel's remoteproc subsystem for
// firmware lifecycle and the uio_pruss-irq driver for memory mapping
// and event delivery. Works on stock BeagleBone Debian images without
// device-tree overrides.
//
// Firmware ELF is expected at /lib/firmware/beagleg-pru0-fw.

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>

#include "common/logging.h"
#include "pru-hardware-interface.h"

namespace {

constexpr const char *kRemoteprocRoot = "/sys/class/remoteproc";
constexpr const char *kUioDev = "/dev/uio0";
constexpr const char *kPru0DramSizePath =
  "/sys/class/uio/uio0/maps/map1/size";
constexpr const char *kFirmwareName = "beagleg-pru0-fw";

// Write `value` to `path` as a single string. Returns true on success.
bool WriteFile(const char *path, const char *value) {
  const int fd = open(path, O_WRONLY);
  if (fd < 0) {
    Log_error("open(%s) failed: %s", path, strerror(errno));
    return false;
  }
  const ssize_t len = (ssize_t)strlen(value);
  const ssize_t n = write(fd, value, len);
  close(fd);
  if (n != len) {
    Log_error("write(%s) failed: %s", path, strerror(errno));
    return false;
  }
  return true;
}

// Read first line of `path` into `out` (NUL-terminated, trimmed). Returns
// true on success.
bool ReadLine(const char *path, char *out, size_t out_size) {
  FILE *f = fopen(path, "r");
  if (!f) return false;
  if (!fgets(out, (int)out_size, f)) {
    fclose(f);
    return false;
  }
  fclose(f);
  // Strip trailing whitespace.
  size_t n = strlen(out);
  while (n && (out[n - 1] == '\n' || out[n - 1] == '\r' || out[n - 1] == ' ')) {
    out[--n] = '\0';
  }
  return true;
}

// Find the /sys/class/remoteproc/remoteprocN node whose `name` file
// matches "*.pru" with PRU0 selected by the smallest matching index.
// Returns a malloc'd path on success, NULL on failure.
char *FindPru0Node() {
  DIR *dir = opendir(kRemoteprocRoot);
  if (!dir) {
    Log_error("opendir(%s) failed: %s", kRemoteprocRoot, strerror(errno));
    return NULL;
  }
  char *best = NULL;
  struct dirent *de;
  while ((de = readdir(dir)) != NULL) {
    if (strncmp(de->d_name, "remoteproc", 10) != 0) continue;
    char name_path[256];
    snprintf(name_path, sizeof(name_path), "%s/%s/name", kRemoteprocRoot,
             de->d_name);
    char name[64];
    if (!ReadLine(name_path, name, sizeof(name))) continue;
    // PRU0 nodes are usually named like "4a334000.pru" on AM335x. PRU1
    // is at 4a338000. Pick the first one we encounter that looks like a
    // PRU and ends in "334000.pru" (PRU0 address). If absent, accept any
    // ".pru".
    const char *dot = strrchr(name, '.');
    if (!dot || strcmp(dot, ".pru") != 0) continue;
    if (best && strstr(best, "334000") != NULL) continue;
    free(best);
    best = (char *)malloc(strlen(kRemoteprocRoot) + 1 + strlen(de->d_name) + 1);
    sprintf(best, "%s/%s", kRemoteprocRoot, de->d_name);
    if (strstr(name, "334000") != NULL) break;  // exact PRU0 match
  }
  closedir(dir);
  if (!best) Log_error("No PRU remoteproc node found under %s", kRemoteprocRoot);
  return best;
}

}  // namespace

RemoteprocPruInterface::RemoteprocPruInterface()
    : uio_fd_(-1), mmap_(NULL), mmap_size_(0), rproc_dir_(NULL) {}

RemoteprocPruInterface::~RemoteprocPruInterface() {
  if (mmap_ != NULL) munmap(mmap_, mmap_size_);
  if (uio_fd_ >= 0) close(uio_fd_);
  free(rproc_dir_);
}

bool RemoteprocPruInterface::Init() {
  rproc_dir_ = FindPru0Node();
  if (!rproc_dir_) return false;
  uio_fd_ = open(kUioDev, O_RDWR | O_SYNC);
  if (uio_fd_ < 0) {
    Log_error("open(%s) failed: %s", kUioDev, strerror(errno));
    return false;
  }
  return true;
}

bool RemoteprocPruInterface::AllocateSharedMem(void **pru_mmap, size_t size) {
  // PRU0 DRAM is the second uio memory region (map1). The mmap offset on
  // /dev/uio0 is page_index * page_size; map1 lives at offset 1*pagesize.
  const long page_size = sysconf(_SC_PAGESIZE);
  *pru_mmap =
    mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, uio_fd_, page_size);
  if (*pru_mmap == MAP_FAILED) {
    Log_error("mmap PRU DRAM via %s failed: %s", kUioDev, strerror(errno));
    *pru_mmap = NULL;
    return false;
  }
  mmap_ = *pru_mmap;
  mmap_size_ = size;
  memset(*pru_mmap, 0x00, size);
  return true;
}

bool RemoteprocPruInterface::StartExecution() {
  char path[320];
  snprintf(path, sizeof(path), "%s/firmware", rproc_dir_);
  if (!WriteFile(path, kFirmwareName)) return false;
  snprintf(path, sizeof(path), "%s/state", rproc_dir_);
  return WriteFile(path, "start");
}

unsigned RemoteprocPruInterface::WaitEvent() {
  uint32_t event_count = 0;
  // Re-arm the uio interrupt.
  const uint32_t enable = 1;
  if (write(uio_fd_, &enable, sizeof(enable)) != (ssize_t)sizeof(enable)) {
    Log_error("uio rearm failed: %s", strerror(errno));
    return 0;
  }
  if (read(uio_fd_, &event_count, sizeof(event_count)) !=
      (ssize_t)sizeof(event_count)) {
    Log_error("uio event read failed: %s", strerror(errno));
    return 0;
  }
  return event_count;
}

bool RemoteprocPruInterface::Shutdown() {
  char path[320];
  snprintf(path, sizeof(path), "%s/state", rproc_dir_);
  return WriteFile(path, "stop");
}

void RemoteprocPruInterface::Halt() {
  char path[320];
  snprintf(path, sizeof(path), "%s/state", rproc_dir_);
  WriteFile(path, "stop");
}

void RemoteprocPruInterface::Restart() {
  char path[320];
  snprintf(path, sizeof(path), "%s/state", rproc_dir_);
  WriteFile(path, "start");
}
