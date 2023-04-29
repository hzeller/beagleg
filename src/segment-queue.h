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
#ifndef _BEAGLEG_MOTOR_OPERATIONS_H_
#define _BEAGLEG_MOTOR_OPERATIONS_H_

#include <stdint.h>

enum { BEAGLEG_NUM_MOTORS = 8 };

// The movement command send to the segment queue either changes speed, or
// provides a steady speed. Already low-level broken down for motors.
struct LinearSegmentSteps {
  // Speed is steps/s. If initial speed and final speed differ, the motor will
  // accelerate or decelerate to reach the final speed within the given number
  // of alotted steps of the axis with the most number of steps; all other axes
  // are scaled accordingly. Uses jerk-settings to increase/decrease
  // acceleration; the acceleration is zero at the end of the move.
  float v0;  // initial speed
  float v1;  // final speed

  // Bits that are set in parallel with the motor control that should be
  // set at the beginning of the motor movement.
  uint16_t aux_bits;  // Aux-bits to switch.

  int steps[BEAGLEG_NUM_MOTORS];  // Steps for axis. Negative for reverse.
};

// Struct used to return data about the currently executed steps
// and the status of the auxes.
struct PhysicalStatus {
  int pos_steps[BEAGLEG_NUM_MOTORS];  // Absolute position in steps.
  uint16_t aux_bits;                  // Auxes status
};

class SegmentQueue {
 public:
  virtual ~SegmentQueue() {}

  // Enqueue a coordinated move command.
  // If there is space in the ringbuffer, this function returns immediately,
  // otherwise it waits until a slot frees up or an abort happens.
  // If "err_stream" is non-NULL, prints error message there.
  // Automatically enables motors if not already.
  // Returns true if the move was added, false if aborted
  virtual bool Enqueue(const LinearSegmentSteps &segment) = 0;

  // Waits for the queue to be empty and Enables/disables motors according to
  // the given boolean value (Right now, motors cannot be individually
  // addressed).
  virtual void MotorEnable(bool on) = 0;

  // Wait, until all elements in the ring-buffer are consumed.
  virtual void WaitQueueEmpty() = 0;

  // Get the absolute position and auxes status the motors currently
  // in, and the end of the exeuction queue.
  // Returns 'true' if the status was available and is updated.
  virtual bool GetPhysicalStatus(PhysicalStatus *status) = 0;

  // Set absolute position of given axis as provided from some external
  // source (e.g. homing). This will allow accurate reporting of the
  // PhysicalStatus.
  virtual void SetExternalPosition(int axis, int position_steps) = 0;

  // Immediately stop any motion and clear the internal queue.
  // Current physical status will be lost.
  // Returns true if the queue was successfully cleared,
  // false if an internal error occurred.
  virtual bool Clear() = 0;
};

#endif  // _BEAGLEG_MOTOR_OPERATIONS_H_
