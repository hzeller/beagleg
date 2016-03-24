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
#ifndef _BEAGLEG_DETERMINE_PRINT_STATS_H
#define _BEAGLEG_DETERMINE_PRINT_STATS_H

struct MachineControlConfig;   // gcode-machine-control.h

struct BeagleGPrintStats {
  float total_time_seconds;      // Total time real execution would take.
  float last_x, last_y, last_z;  // last coordinate.
  float last_z_extruding;        // Last z with extrusion = printed height.
  float filament_len;            // total filament length

};

// Given the input file-descriptor (which is read to EOF and then closed)
// and the given constraints, determine statistics about the gcode-file.
// Returns true on success.
bool determine_print_stats(int input_fd,
                           const MachineControlConfig &config,
                           struct BeagleGPrintStats *result);
#endif // _BEAGLEG_DETERMINE_PRINT_STATS_H
