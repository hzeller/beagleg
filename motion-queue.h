/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
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
  uint16_t aux;            // right now: only lowest 2 bits.
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

// Available operations towards the queue.
struct MotionQueue {
  // Enqueue motion segment into queue.
  void (*enqueue)(struct MotionSegment *segment);
  void (*wait_queue_empty)();     // block and wait for queue to be empty.
  void (*motor_enable)(char on);  // immediately enable motor, indepenent of queue.
  void (*shutdown)(char flush_queue); // Shutdown. If !flush_queue: immediate.
};

// Standard implementation.
// Create queue towards PRU and initialize operation pointers.
// Returns 0 on success. Requires to run as root to initialize.
// There is only one PRU, so it can only be initialzed once until shutdown() is
// called.
int init_pru_motion_queue(struct MotionQueue *queue);

// Ignores all commands.
void init_dummy_motion_queue(struct MotionQueue *queue);

#endif  // _BEAGLEG_MOTION_QUEUE_H_
