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
#include <cstdint>

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

// PRU control via the uio_pruss kernel driver and TI's libprussdrv:
// the historical BeagleG setup, for TI-flavor kernels booted with the
// PRU-UIO device-tree overlay (see INSTALL.md, "Enable PRU"). Default
// backend, PRU_BACKEND=uio.
class UioPrussInterface final : public PruHardwareInterface {
 public:
  bool Init() final;
  bool AllocateSharedMem(void **pru_mmap, size_t size) final;
  bool StartExecution() final;
  unsigned WaitEvent() final;
  bool Shutdown() final;
};

// PRU hardware control without any PRU-specific kernel driver, for
// kernels >= 5.10 that carry the pruss interrupt-controller driver --
// notably the mainline-flavor kernels of current BeagleBone images,
// which ship neither uio_pruss nor PRU rpmsg. Firmware load, start/stop
// and the shared motion queue go through the memory maps of a generic
// UIO device, per-segment completion interrupts through its blocking
// read, all wired up by the device-tree overlay in
// dts/BEAGLEG-PRU-IRQ.dts (see INSTALL.md). PRU_BACKEND=genirq.
class UioGenirqInterface final : public PruHardwareInterface {
 public:
  UioGenirqInterface();
  ~UioGenirqInterface() final;

  bool Init() final;
  bool AllocateSharedMem(void **pru_mmap, size_t size) final;
  bool StartExecution() final;
  unsigned WaitEvent() final;
  bool Shutdown() final;

 private:
  // Volatile is applied at the dereference points, not the storage:
  // ctrl_ is MMIO, so every access must really happen and in order.
  // The queue memory is only memset here before the PRU starts, then
  // handed out via AllocateSharedMem(); the consumer accesses it
  // through the volatile PRUCommunication fields (pru-motion-queue.cc)
  // while the PRU concurrently writes it. iram_ is bulk-copied
  // (memcpy cannot take volatile) only while the core is halted after
  // the reset pulse, so it never has a concurrent accessor. The uio
  // mappings
  // are non-cached device memory, so the CPU preserves the access
  // order the compiler emits -- the same contract the uio_pruss
  // mapping always provided.
  int uio_fd_;
  void *data_ram_;           // Motion queue, shared with the PRU.
  volatile uint32_t *ctrl_;  // PRU0 control register.
  void *iram_;               // PRU0 instruction RAM.
};

#endif  // BEAGLEG_PRU_HARDWARE_INTERFACE_
