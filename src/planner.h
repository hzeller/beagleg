/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2016 Henner Zeller <h.zeller@acm.org>
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
#ifndef _BEAGLEG_PLANNER_H_
#define _BEAGLEG_PLANNER_H_

#include "gcode-parser/gcode-parser.h"  // AxesRegister

struct MachineControlConfig;
class HardwareMapping;
class MotorOperations;

// The planner receives a sequence of desired target positions.
// It then plans acceleration and speed profile for the physical
// machine, and emits these to the MotorOperations backend.
class Planner {
public:
  // The planner writes out motor operations to the backend.
  Planner(const MachineControlConfig *config,
          HardwareMapping *hardware_mapping,
          MotorOperations *motor_backend);
  ~Planner();

  // Enqueue a new target position to go to in a linear movement from
  // the current position.
  // Returns true if successful, false if aborted
  bool Enqueue(const AxesRegister &target_pos, float speed);

  // Flush the queue and wait until all remaining motor
  // operations have been flushed.
  void BringPathToHalt();

  // Get the latest position enqueued to the motors.
  // TODO(Leonardo): get actual position of the motor at this moment.
  void GetCurrentPosition(AxesRegister *pos);

  // Drive an axis directly. Should only be used for cases such as
  // homing which require direct motor driving.
  //
  // Precondition: path needs to be halted, so call BringPathToHalt() first.
  // After one or a series of DirectDrive() calls, SetExternalPosition()
  // must be called to inform the Planner what new absolute position we
  // are in.
  //
  // Returns the number of steps the stepmotor for that axis did.
  int DirectDrive(GCodeParserAxis axis, float distance, float v0, float v1);

  // Set the current absolute position of the given axis from an
  // machine move outside of the control of the Planner.
  // Precondition: BringPathToHalt() had been called before.
  void SetExternalPosition(GCodeParserAxis axis, float pos);

private:
  class Impl;
  Impl *const impl_;
};
#endif
