/*
 * (c) 2013, 1014 Henner Zeller <h.zeller@acm.org>
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

struct BeagleGPrintStats {
  float total_time_seconds;      // Total time real execution would take.
  // If feedrate was too high after speed-factor: this was the highest feedrate
  // seen, that was capped to the provided max feedrate.
  float max_G1_feedrate;         // Max feedrate for G1 seen.
  float last_x, last_y, last_z;  // last coordinate.
  float filament_len;            // total filament length

};

// Given the input file-descriptor (which is read to EOF and then closed)
// and the given constraints, determine statistics about the gcode-file.
// Returns 0 on success.
int determine_print_stats(int input_fd, float max_feedrate, float speed_factor,
			  struct BeagleGPrintStats *result);
