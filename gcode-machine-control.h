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
#ifndef _BEAGLEG_GCODE_MACHINE_CONTROL_H_
#define _BEAGLEG_GCODE_MACHINE_CONTROL_H_
#include "gcode-parser.h"

enum HomeType {
  HOME_POS_NONE     = 0,  // Axis does not do homing.
  HOME_POS_ORIGIN   = 1,  // Home position is at origin '0' for this axis
  HOME_POS_ENDRANGE = 2,  // Home position is at end of range for this axis.
};

/* Configuration constants for the controller.
 * Parameters in the arrays are always indexed by logical axes, e.g. AXIS_X.
 * The output mapping to the physical driver is controlled by output_mapping
 */
struct MachineControlConfig {
  // Arrays with values for each axis
  float steps_per_mm[GCODE_NUM_AXES];   // Steps per mm for each logical axis.
  float move_range_mm[GCODE_NUM_AXES];  // Range of axes in mm (0..range[axis])
  enum HomeType home_switch[GCODE_NUM_AXES]; // Home position for axes.
                 // Axes with HOME_POS_NONE don't participate in homing.

  float max_feedrate[GCODE_NUM_AXES];   // Max feedrate for axis (mm/s)
  float acceleration[GCODE_NUM_AXES];   // Max acceleration for axis (mm/s^2)

  float speed_factor;         // Multiply feed with. Should be 1.0 by default.

  // The follwing two parameters determine which logical axis ends up
  // on which physical plug location. To make things easier to
  // digest, this is done in two steps.
  //
  // The 'channel_layout' maps how the driver channels (internally some GPIO
  // bits) ends up being mapped into the pysical sequence of connectors on
  // the board. The pysical location is given as position in the string,
  // the value at that point describes the internal channel.
  // A channel_layout mapping of "021" for instance means, that on the very
  // left is channel zero, followed by channel two in the middle and channel
  // 1 at the right. Due to layout reasons, the Bumps board
  // (github.com/hzeller/bumps) has the mapping "23140".
  //
  // The 'axis_mapping' determines how to map a logical axis (e.g. 'X') to
  // a connector position. So again, the string position represents the
  // position on the board (sequence of connectors), while the character at
  // that position describes the logical axis. Typicaly, this is just
  // "XZYEABC"; for reasons such as using a double-connector, one might
  // have a different mapping, e.g. "XZE_Y". Underscores represent axis that
  // are not mapped.
  //
  // Of course, these two mappings could be done in one shot, but it would be
  // a bit mind-twisting.
  const char *channel_layout;   // Mapping of driver channel (character in
                                // string) to physical location (position in
                                // string). This typically is a constant of
                                // the physical cape used.
                                // If NULL, default is "23140" (Bumps board).
  const char *axis_mapping;     // Mapping of axis-name (character in string)
                                // to physical location (position in string).
                                // Assumed "XYZEABC" if NULL.
                                // Axis name '_' for skipped placeholder.
                                // Not mentioned axes are not handled.

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

#endif //  _BEAGLEG_GCODE_MACHINE_CONTROL_H_
