/*
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

enum HomeType {
  HOME_POS_NONE     = -1,  // Axis does not do homing.
  HOME_POS_ORIGIN   = 0,   // Home position is at origin '0' for this axis
  HOME_POS_ENDRANGE = 1,   // Home position is at end of range for this axis.
};

/* Configuration constants for the controller. */
struct MachineControlConfig {
  // Arrays with values for each axis
  float steps_per_mm[8];        // Steps per mm for each axis.
  float move_range_mm[8];       // Available of axes in mm (0..range[axis])
                                // Only values > 0 are actively clipped.
  enum HomeType home_switch[8]; // Home position for axes. Axis with
                                // HOME_POS_NONE don't participate in homing.
  float max_feedrate[8];        // Max feedrate for axis (mm/s)
  float acceleration[8];        // Max acceleration for axis (mm/s^2)
  float speed_factor;           // Speed up everything. Should be 1.0 by default.
  const char *output_mapping;   // Mapping of axis to actual output motor channel
                                // Assumed "XYZEABC" if NULL. Not mentioned
                                // axes are not handled.
  char dry_run;                 // Don't actually send motor commands if 1.
  char debug_print;             // Print step-tuples to output_fd if 1.
  char synchronous;             // Don't queue, wait for command to finish if 1.
};


// Initialize the motor control with the given configuration.
// This internally creates a copy of the configuration, so no need for
// the value to stay around after this call.
// Returns 0 on success.
int gcode_machine_control_init(const struct MachineControlConfig *config);

// To be called after use.
void gcode_machine_control_exit();

// Read gcode from the "gcode_fd" filedescriptor and operate machinery with
// it.
// This reads until it reached End-of-File. The gcode_fd is closed when done.
//
// If "output_fd" is >=0, error messages and other output is written there; this
// file-descriptor is _not_ closed.
//
// Only one thread at a time can be using this function.
// Returns 0 on success.
int gcode_machine_control_from_stream(int gcode_fd, int output_fd);
