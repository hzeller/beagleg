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

// PRU hardware interface that needs no PRU-specific kernel driver, for
// stock mainline-kernel BeagleBone images that ship neither uio_pruss
// nor PRU rpmsg support. The device-tree overlay in
// dts/BEAGLEG-PRU-IRQ.dts binds the PRU to the kernel's uio_pdrv_genirq
// driver, which exposes the PRU0 memories as mmap regions (firmware
// loading, start/stop and the shared motion queue go directly through
// them) and the per-segment completion interrupt as a blocking read.
// Everything runs through one /dev/uioN, so a udev rule on that node is
// all it takes to run without root -- no /dev/mem involved.

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>

#include "common/logging.h"
#include "pru-hardware-interface.h"

// Generated PRU code from motor-interface-pru.p
#include "motor-interface-pru_bin.h"

namespace {
// The three UIO mmap regions, in the order the reg entries of
// dts/BEAGLEG-PRU-IRQ.dts declare them. Their physical addresses live
// in that overlay; region sizes per the AM335x Technical Reference
// Manual (TI document SPRUH73), section 4.3.2, Table 4-8 "Global
// Memory Map": https://www.ti.com/lit/pdf/spruh73#page=207 (anchor
// per revision Q). Linux on the AM335x uses 4 KiB pages.
constexpr size_t kPageSize = 4096;

struct MapRange {
  int index;  // Which UIO map region == reg-entry position in the dts.
  size_t size;
};
constexpr MapRange kDataRamMap{.index = 0, .size = 2 * kPageSize};  // 8 KiB.
constexpr MapRange kCtrlMap{.index = 1, .size = kPageSize};
constexpr MapRange kIramMap{.index = 2, .size = 2 * kPageSize};  // 8 KiB.

// Bits of the PRU0 CTRL register (TRM section 4.5.1.1, Table 4-41,
// https://www.ti.com/lit/pdf/spruh73#page=275): SOFT_RST_N (bit 0)
// resets the core when written as 0 and self-sets the next cycle --
// a pulse, not a held state; EN (bit 1) makes the core run.
//
// The writes below set the whole register instead of read-modify-write,
// mirroring libprussdrv. Nothing else owns this register (the overlay
// keeps pru_rproc off PRU0), and 0 is the value we want in every other
// writable field: PCTR_RST_VAL (bits 31:16) = 0 starts execution at
// instruction 0, SINGLE_STEP (bit 8) = 0 runs free, CTR_EN (bit 3) = 0
// keeps the cycle counter off, SLEEPING (bit 2) = 0 not sleeping;
// RUNSTATE (bit 15) is read-only. A read-modify-write would instead
// inherit whatever a previous occupant left behind, e.g. a stale
// nonzero start address in PCTR_RST_VAL.
constexpr uint32_t kCtrlSoftRstN = 1 << 0;
constexpr uint32_t kCtrlEnable = 1 << 1;

// Our node name in dts/BEAGLEG-PRU-IRQ.dts; the kernel-reported UIO
// device name may carry an @unit-address suffix, so accept exactly
// this name or this name followed by '@'.
constexpr const char *kUioName = "beagleg_pru_irq";

// The mmap offset on a UIO device is not a byte offset into one flat
// space; it selects which memory region to map, and the kernel maps
// that region from its start: "To map the memory of mapping N, you
// have to use N times the page size as your offset"
// (https://www.kernel.org/doc/html/latest/driver-api/uio-howto.html).
// Enforced in drivers/uio/uio.c, uio_find_mem_index(): vm_pgoff is
// consumed as the map index, never as a position -- so mappings of
// different regions cannot overlap.
void *MapUioRegion(int uio_fd, const MapRange &range) {
  void *m = mmap(nullptr, range.size, PROT_READ | PROT_WRITE, MAP_SHARED,
                 uio_fd, (off_t)range.index * kPageSize);
  return (m == MAP_FAILED) ? nullptr : m;
}

// Find and open the /dev/uioN backed by our device-tree node.
int OpenUioDevice() {
  // Buffers sized for the longest possible directory entry (NAME_MAX).
  char dev[sizeof("/dev/") + NAME_MAX] = "";
  DIR *dir = opendir("/sys/class/uio");
  struct dirent *de;
  while (dir && !dev[0] && (de = readdir(dir)) != nullptr) {
    if (de->d_name[0] == '.') continue;  // ".", "..", hidden entries.
    char path[sizeof("/sys/class/uio/") + NAME_MAX + sizeof("/name")];
    char name[64] = "";
    snprintf(path, sizeof(path), "/sys/class/uio/%s/name", de->d_name);
    FILE *f = fopen(path, "r");
    if (!f) continue;
    const size_t prefix_len = strlen(kUioName);
    if (fgets(name, sizeof(name), f) &&
        strncmp(name, kUioName, prefix_len) == 0 &&
        (name[prefix_len] == '\0' || name[prefix_len] == '\n' ||
         name[prefix_len] == '@')) {
      snprintf(dev, sizeof(dev), "/dev/%s", de->d_name);
    }
    fclose(f);
  }
  if (dir) closedir(dir);
  if (!dev[0]) {
    Log_error(
      "No '%s' UIO device found. Is the BEAGLEG-PRU-IRQ device-tree "
      "overlay loaded? See INSTALL.md: uboot_overlay_pru and "
      "uio_pdrv_genirq.of_id=generic-uio in /boot/uEnv.txt.",
      kUioName);
    return -1;
  }
  const int fd = open(dev, O_RDWR);
  if (fd < 0) Log_error("open(%s): %s", dev, strerror(errno));
  return fd;
}
}  // namespace

UioGenirqInterface::UioGenirqInterface()
    : uio_fd_(-1), data_ram_(nullptr), ctrl_(nullptr), iram_(nullptr) {}

UioGenirqInterface::~UioGenirqInterface() {
  if (iram_) munmap(iram_, kIramMap.size);
  if (ctrl_) munmap((void *)ctrl_, kCtrlMap.size);
  if (data_ram_) munmap(data_ram_, kDataRamMap.size);
  if (uio_fd_ >= 0) close(uio_fd_);
}

bool UioGenirqInterface::Init() {
  uio_fd_ = OpenUioDevice();
  if (uio_fd_ < 0) return false;
  data_ram_ = MapUioRegion(uio_fd_, kDataRamMap);
  ctrl_ = (volatile uint32_t *)MapUioRegion(uio_fd_, kCtrlMap);
  iram_ = MapUioRegion(uio_fd_, kIramMap);
  if (!data_ram_ || !ctrl_ || !iram_) {
    Log_error("mmap of PRU0 memories via uio: %s", strerror(errno));
    return false;
  }
  return true;
}

bool UioGenirqInterface::AllocateSharedMem(void **pru_mmap, const size_t size) {
  if (size > kDataRamMap.size) {
    Log_error("PRU shared memory request %zu > data RAM size %zu", size,
              kDataRamMap.size);
    return false;
  }
  memset(data_ram_, 0x00, size);
  *pru_mmap = data_ram_;
  return true;
}

bool UioGenirqInterface::StartExecution() {
  static_assert(sizeof(PRUcode) <= kIramMap.size,
                "PRU firmware exceeds the 8 KiB PRU0 IRAM");
  // IRAM is host-writable only while the core is halted, and a wedged
  // core (RUNSTATE stuck with EN off, as left behind by a crashed
  // occupant) silently drops IRAM writes -- so pulse the reset first to
  // force a clean halt, and read the code back to turn that failure
  // mode into a hard error.
  *ctrl_ = 0;    // SOFT_RST_N pulse (self-clears), EN off: clean halt.
  usleep(1000);  // Let the reset pulse and core state settle.
  // Copy and verify word-wise: the IRAM host port is 32 bits wide and
  // the mapping is uncached Device memory, where libc memcpy may use
  // multi-word or NEON accesses that are not permitted -- same reason
  // prussdrv_pru_write_memory copies as a 32-bit loop.
  volatile uint32_t *iram_words = (volatile uint32_t *)iram_;
  const size_t words = sizeof(PRUcode) / sizeof(uint32_t);
  for (size_t i = 0; i < words; ++i) iram_words[i] = PRUcode[i];
  for (size_t i = 0; i < words; ++i) {
    if (iram_words[i] != PRUcode[i]) {
      Log_error("PRU0 IRAM readback mismatch after loading the firmware.");
      return false;
    }
  }
  // EN with another SOFT_RST_N pulse: the core starts from instruction
  // 0 (PCTR_RST_VAL = 0).
  *ctrl_ = kCtrlEnable;
  return true;
}

unsigned UioGenirqInterface::WaitEvent() {
  // Re-arm first: uio_pdrv_genirq masks the interrupt whenever it
  // fires. An event raised while masked stays latched in the PRUSS
  // interrupt controller and fires the moment we unmask, so no event is
  // lost between read() and the next call.
  uint32_t value = 1;
  if (write(uio_fd_, &value, sizeof(value)) != sizeof(value)) {
    Log_error("uio interrupt re-arm: %s", strerror(errno));
    usleep(100 * 1000);  // Keep a broken fd from spinning our callers hot.
    return 0;
  }
  ssize_t r;
  do {
    r = read(uio_fd_, &value, sizeof(value));
  } while (r < 0 && errno == EINTR);  // A signal is not an event error.
  if (r != (ssize_t)sizeof(value)) {
    Log_error("uio interrupt wait: %s", strerror(errno));
    usleep(100 * 1000);  // Keep a broken fd from spinning our callers hot.
    return 0;
  }
  return value;  // Running event count, same semantics as prussdrv.
}

bool UioGenirqInterface::Shutdown() {
  // ENABLE cleared, reset not asserted: halt the core.
  if (ctrl_) *ctrl_ = kCtrlSoftRstN;
  return true;
}
