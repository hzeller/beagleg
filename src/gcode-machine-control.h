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
#ifndef _BEAGLEG_GCODE_MACHINE_CONTROL_H_
#define _BEAGLEG_GCODE_MACHINE_CONTROL_H_

#include "gcode-parser/gcode-parser.h"
#include "common/container.h"
#include "hardware-mapping.h"

#include <string>

class MotorOperations;
class ConfigParser;
class Spindle;
typedef AxesRegister FloatAxisConfig;

/* Configuration constants for the controller.
 * Parameters in the arrays are always indexed by logical axes, e.g. AXIS_X.
 * The output mapping to the physical driver is controlled by output_mapping
 */
struct MachineControlConfig {
  MachineControlConfig();

  // Read values from configuration file.
  bool ConfigureFromFile(ConfigParser *parser);

  // Arrays with values for each axis
  FloatAxisConfig steps_per_mm;   // Steps per mm for each logical axis.
  FloatAxisConfig move_range_mm;  // Range of axes in mm (0..range[axis]). -1: no limit

  FloatAxisConfig max_feedrate;   // Max feedrate for axis (mm/s)
  FloatAxisConfig acceleration;   // Max acceleration for axis (mm/s^2)

  FloatAxisConfig max_probe_feedrate; // Max probe feedrate for axis (mm/s)

  float speed_factor;         // Multiply feed with. Should be 1.0 by default.
  float threshold_angle;      // Threshold angle to ignore speed changes
  float speed_tune_angle;     // Angle added to the angle between vectors for speed tuning

  std::string home_order;        // Order in which axes are homed.

  FixedArray<HardwareMapping::AxisTrigger, GCODE_NUM_AXES> homing_trigger;

  int auto_motor_disable_seconds; // disable motors automatically after these seconds.
  int auto_fan_disable_seconds; // Disable fan automatically after these seconds.
  int auto_fan_pwm;             // PWM value to automatically enable fan with.
  bool acknowledge_lines;       // Respond w/ 'ok' on each command on msg_stream.
  bool require_homing;          // Require homing before any moves.
  bool range_check;             // Do machine limit checks. Default 1.
  std::string clamp_to_range;   // Clamp these axes to machine range before
                                // range check. Dangerous.
                                // Support only "" or "Z" right now.
  bool debug_print;             // Print step-tuples to output_fd if 1.
  bool synchronous;             // Don't queue, wait for command to finish if 1.
  bool enable_pause;            // Enable pause switch detection. Default 0.
};

// A class that controls a machine via gcode.
class GCodeMachineControl {
 public:
  enum class EStopState {  NONE,  SOFT,  HARD  };

  // The three levels of homing confidence. If we ever switch off
  // power to the motors after homing, we can't be sure.
  enum class HomingState {
    NEVER_HOMED,
    HOMED_BUT_MOTORS_UNPOWERED,
    HOMED
  };

  // Factor to create a GCodeMachineControl.
  // The MotorOperations provide the low-level motor control ops.
  // msg_stream, if non-NULL, sends back return messages on the GCode channel.
  // Returns NULL on failure.
  static GCodeMachineControl *Create(const MachineControlConfig &config,
                                     MotorOperations *motor_backend,
                                     HardwareMapping *hardware_mapping,
                                     Spindle *spindle,
                                     FILE *msg_stream);

  ~GCodeMachineControl();

  // Set where messages should go.
  void SetMsgOut(FILE *msg_stream);

  // Get the physical home position of this machine which depend
  // on the position of the endstops configured for homing.
  // return in *pos register.
  void GetHomePos(AxesRegister *pos);

  // Return the receiver for parse events. The caller must not assume ownership
  // of the returned pointer.
  GCodeParser::EventReceiver *ParseEventReceiver();

  // Return the E-Stop status.
  // Can only be called in the same thread that also handles gcode updates.
  EStopState GetEStopStatus();

  // Return the Homing status.
  // Can only be called in the same thread that also handles gcode updates.
  HomingState GetHomeStatus();

  // Return the motors enabled status.
  // Can only be called in the same thread that also handles gcode updates.
  bool GetMotorsEnabled();

  // Return current position relative to origin.
  // Can only be called in the same thread that also handles gcode updates.
  void GetCurrentPosition(AxesRegister *pos);

 private:
  class Impl;

  // The MotorOperations struct provide the low-level motor control ops.
  // msg_stream, if non-NULL, sends back return messages on the GCode channel.
  GCodeMachineControl(Impl *Impl);

  Impl *const impl_;  // opaque state.
};

#endif //  _BEAGLEG_GCODE_MACHINE_CONTROL_H_
