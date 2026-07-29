/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2016 Henner Zeller <h.zeller@acm.org>,
 *          Leonardo Romor <leonardo.romor@gmail.com>,
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
#ifndef BEAGLEG_PRU_HARDWARE_INTERFACE_
#define BEAGLEG_PRU_HARDWARE_INTERFACE_

#include <cstddef>

// Pru hardware controls
class PruHardwareInterface {
 public:
  virtual ~PruHardwareInterface() {}

  // Initialize the hardware, interrupts, etc...
  // return value: returns if the operation was successful.
  virtual bool Init() = 0;

  // Retrieve the pointer of the pru mapping and initialize the memory.
  virtual bool AllocateSharedMem(void **pru_mmap, size_t size) = 0;

  // Enable the PRU and start predetermined program.
  virtual bool StartExecution() = 0;

  // Wait for a beagleg-mapped event. Return number of events that have occured.
  virtual unsigned WaitEvent() = 0;

  // Halt the PRU
  virtual bool Shutdown() = 0;
};

class UioPrussInterface : public PruHardwareInterface {
 public:
  bool Init() final;
  bool AllocateSharedMem(void **pru_mmap, size_t size) final;
  bool StartExecution() final;
  unsigned WaitEvent() final;
  bool Shutdown() final;
};

// PRU hardware control via the kernel's remoteproc stack (stock
// BeagleBone Debian images with the default PRU-RPROC overlay):
// firmware lifecycle through /sys/class/remoteproc/, the shared motion
// queue through a /dev/mem mapping of PRU0 DRAM, and per-segment
// completion events through the firmware's rpmsg channel
// (/dev/rpmsg_pru30).
class RemoteprocPruInterface : public PruHardwareInterface {
 public:
  RemoteprocPruInterface();
  ~RemoteprocPruInterface() final;

  bool Init() final;
  bool AllocateSharedMem(void **pru_mmap, size_t size) final;
  bool StartExecution() final;
  unsigned WaitEvent() final;
  bool Shutdown() final;

 private:
  // (Re-)open /dev/rpmsg_pru30 and send the address handshake; needed
  // after every PRU start since the channel dies across stop/start.
  bool EstablishRpmsgChannel();

  int rpmsg_fd_;
  void *mmap_;
  size_t mmap_size_;
  char *rproc_dir_;  // e.g. "/sys/class/remoteproc/remoteproc1"
};

#endif  // BEAGLEG_PRU_HARDWARE_INTERFACE_
