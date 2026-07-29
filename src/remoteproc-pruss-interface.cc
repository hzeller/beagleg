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

// PRU hardware interface for the kernel's remoteproc stack, as found on
// stock BeagleBone Debian images (PRU-RPROC device-tree overlay):
//  - firmware lifecycle via /sys/class/remoteproc/remoteprocN/{firmware,state}
//  - shared motion queue via /dev/mem, mapping PRU0 DRAM directly
//  - per-segment completion events via rpmsg: the firmware announces the
//    "rpmsg-pru" channel on port 30 and sends one message per finished
//    queue slot; WaitEvent() is a blocking read on /dev/rpmsg_pru30.
//
// The rpmsg channel needs a one-byte handshake write after every PRU
// (re)start so the firmware learns our address before its first send.

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstdint>

#include "common/logging.h"
#include "pru-hardware-interface.h"

namespace {

constexpr const char *kRemoteprocRoot = "/sys/class/remoteproc";
constexpr const char *kFirmwareName = "beagleg-pru0-fw";
constexpr const char *kRpmsgDev = "/dev/rpmsg_pru30";
constexpr const char *kMemDev = "/dev/mem";

// AM335x PRU-ICSS: PRU0 data RAM as seen from the ARM.
constexpr off_t kPru0DramPhys = 0x4A300000;
constexpr size_t kPru0DramSize = 8 * 1024;

// How long to wait for /dev/rpmsg_pru30 to appear after starting the PRU.
constexpr int kRpmsgWaitMs = 5000;

// Write "value" into "path". Returns true on success.
bool WriteSysfs(const char *path, const char *value) {
  const int fd = open(path, O_WRONLY);
  if (fd < 0) {
    Log_error("open(%s): %s", path, strerror(errno));
    return false;
  }
  const ssize_t len = (ssize_t)strlen(value);
  const ssize_t n = write(fd, value, len);
  close(fd);
  if (n != len) {
    Log_error("write(%s <- '%s'): %s", path, value, strerror(errno));
    return false;
  }
  return true;
}

bool ReadLine(const char *path, char *out, size_t out_size) {
  FILE *f = fopen(path, "r");
  if (!f) return false;
  const bool ok = fgets(out, (int)out_size, f) != nullptr;
  fclose(f);
  if (!ok) return false;
  size_t n = strlen(out);
  while (n && (out[n - 1] == '\n' || out[n - 1] == '\r' || out[n - 1] == ' '))
    out[--n] = '\0';
  return true;
}

// Find the remoteproc node of PRU0. On AM335x the node's "name" file
// contains the PRU core's device address, 4a334000.pru for PRU0.
// Returns a malloc'd "/sys/class/remoteproc/remoteprocN" or nullptr.
char *FindPru0Node() {
  DIR *dir = opendir(kRemoteprocRoot);
  if (!dir) {
    Log_error(
      "opendir(%s): %s. Is this kernel built with remoteproc, and is the "
      "PRU-RPROC device-tree overlay enabled in /boot/uEnv.txt?",
      kRemoteprocRoot, strerror(errno));
    return nullptr;
  }
  char *result = nullptr;
  struct dirent *de;
  while (!result && (de = readdir(dir)) != nullptr) {
    if (strncmp(de->d_name, "remoteproc", 10) != 0) continue;
    char name_path[256];
    snprintf(name_path, sizeof(name_path), "%s/%.64s/name", kRemoteprocRoot,
             de->d_name);
    char name[64];
    if (!ReadLine(name_path, name, sizeof(name))) continue;
    if (strstr(name, "4a334000") == nullptr) continue;  // not PRU0
    const size_t len = strlen(kRemoteprocRoot) + 1 + strlen(de->d_name) + 1;
    result = (char *)malloc(len);
    snprintf(result, len, "%s/%s", kRemoteprocRoot, de->d_name);
  }
  closedir(dir);
  if (!result) {
    Log_error(
      "No PRU0 (4a334000.pru) node under %s; check that the pru_rproc "
      "module is loaded (lsmod | grep pru)",
      kRemoteprocRoot);
  }
  return result;
}

}  // namespace

RemoteprocPruInterface::RemoteprocPruInterface()
    : rpmsg_fd_(-1), mmap_(nullptr), mmap_size_(0), rproc_dir_(nullptr) {}

RemoteprocPruInterface::~RemoteprocPruInterface() {
  if (mmap_) munmap(mmap_, mmap_size_);
  if (rpmsg_fd_ >= 0) close(rpmsg_fd_);
  free(rproc_dir_);
}

bool RemoteprocPruInterface::Init() {
  rproc_dir_ = FindPru0Node();
  if (!rproc_dir_) return false;
  // Make sure we start from a clean, stopped PRU; ignore failure ("stop"
  // on an offline PRU returns EINVAL).
  char path[300];
  snprintf(path, sizeof(path), "%s/state", rproc_dir_);
  WriteSysfs(path, "stop");
  return true;
}

bool RemoteprocPruInterface::AllocateSharedMem(void **pru_mmap, size_t size) {
  if (size > kPru0DramSize) {
    Log_error("PRU shared memory request %zu > DRAM size %zu", size,
              kPru0DramSize);
    return false;
  }
  const int fd = open(kMemDev, O_RDWR | O_SYNC);
  if (fd < 0) {
    Log_error("open(%s): %s (BeagleG needs to run as root)", kMemDev,
              strerror(errno));
    return false;
  }
  mmap_ = mmap(nullptr, kPru0DramSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
               kPru0DramPhys);
  close(fd);  // mapping stays valid.
  if (mmap_ == MAP_FAILED) {
    mmap_ = nullptr;
    Log_error("mmap PRU0 DRAM (%s @ 0x%llx): %s", kMemDev,
              (unsigned long long)kPru0DramPhys, strerror(errno));
    return false;
  }
  mmap_size_ = kPru0DramSize;
  memset(mmap_, 0x00, size);
  *pru_mmap = mmap_;
  return true;
}

// Wait for the firmware's rpmsg channel device, open it and send the
// one-byte handshake that tells the firmware our address.
bool RemoteprocPruInterface::EstablishRpmsgChannel() {
  if (rpmsg_fd_ >= 0) {
    close(rpmsg_fd_);
    rpmsg_fd_ = -1;
  }
  for (int waited_ms = 0;; waited_ms += 10) {
    rpmsg_fd_ = open(kRpmsgDev, O_RDWR);
    if (rpmsg_fd_ >= 0) break;
    if (waited_ms >= kRpmsgWaitMs) {
      Log_error(
        "%s did not appear within %dms of starting the PRU. Is the "
        "rpmsg_pru module available (modinfo rpmsg_pru)?",
        kRpmsgDev, kRpmsgWaitMs);
      return false;
    }
    usleep(10 * 1000);
  }
  const char token = 0;
  if (write(rpmsg_fd_, &token, 1) != 1) {
    Log_error("rpmsg handshake write(%s): %s", kRpmsgDev, strerror(errno));
    close(rpmsg_fd_);
    rpmsg_fd_ = -1;
    return false;
  }
  return true;
}

bool RemoteprocPruInterface::StartExecution() {
  char path[300];
  snprintf(path, sizeof(path), "%s/firmware", rproc_dir_);
  if (!WriteSysfs(path, kFirmwareName)) return false;
  snprintf(path, sizeof(path), "%s/state", rproc_dir_);
  if (!WriteSysfs(path, "start")) return false;
  return EstablishRpmsgChannel();
}

unsigned RemoteprocPruInterface::WaitEvent() {
  char buf[512];
  const ssize_t n = read(rpmsg_fd_, buf, sizeof(buf));
  if (n < 0) {
    Log_error("rpmsg read(%s): %s", kRpmsgDev, strerror(errno));
    return 0;
  }
  return 1;
}

bool RemoteprocPruInterface::Shutdown() {
  char path[300];
  snprintf(path, sizeof(path), "%s/state", rproc_dir_);
  WriteSysfs(path, "stop");
  // The rpmsg channel dies with the PRU.
  if (rpmsg_fd_ >= 0) {
    close(rpmsg_fd_);
    rpmsg_fd_ = -1;
  }
  return true;
}
