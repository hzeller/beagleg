/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2014 Henner Zeller <h.zeller@acm.org>
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
#ifndef _BEAGLEG_MOTION_QUEUE_H_
#define _BEAGLEG_MOTION_QUEUE_H_

#include <stdint.h>
#include "common/container.h"

#include "pru-hardware-interface.h"

// Number of motors handled by motion segment.
// TODO: this and BEAGLEG_NUM_MOTORS should be coming from the same place.
#define MOTION_MOTOR_COUNT 8

// Lowlevel interface allowing to enqueue MotionSegments to the hardware.
// The motion planning running on the host-OS prepares these parameters
// to be interpreted by some realtime hardware.
// Both systems are connected by this queue.
//
// This contains the pre-calculated parameters for efficient hardware-generation
// of the accleration profile.
// These are the parameters enqueued between motor-interface and the
// processor/hardware unit creating the realtime accurate motion profile.
//
// There are different implementations
//  - The main implementation uses the BeagleBone PRU.
//  - There is a simulation implementation that mimicks the operation in the hardware
//    and outputs some graphs (sim-firmware.{h,c})
//  - A 'dummy' implementation does nothing. Good for dry-run situations.
// operations are basic enough to be executed by any realtime implementation
// using a microcontroller or FPGA.
// Also useful for testing.

struct MotionSegment {
  // Queue header
  uint8_t state;           // see motor-interface-constants.h STATE_* constants.

  uint8_t direction_bits;

  // TravelParameters (needs to match TravelParameters in motor-interface-pru.p)
  uint16_t loops_accel;    // Phase 1: loops spent in acceleration
  uint16_t loops_travel;   // Phase 2: lops spent in travel
  uint16_t loops_decel;    // Phase 3: loops spent in deceleration
  uint16_t aux;            // all 16 bits can be used
  uint32_t accel_series_index;  // index in taylor

  uint32_t hires_accel_cycles;  // acceleration delay cycles.
  uint32_t travel_delay_cycles; // travel delay cycles.

  uint32_t fractions[MOTION_MOTOR_COUNT]; // fixed point fractions to add each step.

#if JERK_EXPERIMENT
  /*
   * The following not handled yet in PRU, just experimental in sim right now.
   */
  uint16_t jerk_start;
  uint16_t jerk_stop;
  float jerk_motion;
#endif
} __attribute__((packed));

namespace internal {
// Layout of the status register
// Assuming atomicity of 32 bit boundaries
// This 32 bit value will be a copy of the R28 register of the PRU.
// First 0-23 bits are assigned to the counter, top 24-31 bits to the index.
// This is an internal implementation detail of the PRUMotionQueue.
struct QueueStatus {
  uint32_t counter : 24; // remaining number of cycles to be performed
  uint32_t index : 8;    // represent the executing slot [0 to QUEUE_LEN - 1]
};
}

typedef FixedArray<int, MOTION_MOTOR_COUNT> MotorsRegister;

// Low level motion queue operations.
class MotionQueue {
public:
  virtual ~MotionQueue() {}

  // Enqueue a motion segment into queue. Blocks until capacity of queue allows
  // to receive more elements.
  // Might change values in MotionSegment.
  // Returns true if segment was added, false if PRU abort was detected
  virtual bool Enqueue(MotionSegment *segment) = 0;

  // Block and wait for queue to be empty.
  virtual void WaitQueueEmpty() = 0;

  // Immediately enable motors, indepenent of queue.
  virtual void MotorEnable(bool on) = 0;

  // Shutdown. If !flush_queue: immediate, even if motors are still moving.
  virtual void Shutdown(bool flush_queue) = 0;

  // Returns the number of motion segments that are pending in the queue
  // behind the one currently executing. So if we are currently executing
  // the last element or are idle after the last element, this will return
  // zero.
  // The return parameter head_item_progress is set to the number
  // of not yet executed loops in the item currenly being executed.
  virtual int GetPendingElements(uint32_t *head_item_progress) = 0;
};

// Standard implementation.
// Create queue towards PRU and initialize operation pointers.
// Returns 0 on success. Requires to run as root to initialize.
// There is only one PRU, so it can only be initialzed once until shutdown() is
// called.
class HardwareMapping;
struct PRUCommunication;
class PRUMotionQueue : public MotionQueue {
public:
  PRUMotionQueue(HardwareMapping *hw, PruHardwareInterface *pru);
  ~PRUMotionQueue();

  bool Enqueue(MotionSegment *segment);
  void WaitQueueEmpty();
  void MotorEnable(bool on);
  void Shutdown(bool flush_queue);
  int GetPendingElements(uint32_t *head_item_progress);

private:
  bool Init();

  void ClearPRUAbort(unsigned int idx);

  HardwareMapping *const hardware_mapping_;
  PruHardwareInterface *const pru_interface_;

  volatile struct PRUCommunication *pru_data_;
  unsigned int queue_pos_;
};


// Queue that does nothing. For testing purposes.
class DummyMotionQueue : public MotionQueue {
public:
  bool Enqueue(MotionSegment *segment) { return true; }
  void WaitQueueEmpty() {}
  void MotorEnable(bool on) {}
  void Shutdown(bool flush_queue) {}
  int GetPendingElements(uint32_t *head_item_progress) {
    if (head_item_progress)
      *head_item_progress = 0;
    return 1;
  }
};

#endif  // _BEAGLEG_MOTION_QUEUE_H_
