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

#include <fcntl.h>
#include <getopt.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "determine-print-stats.h"
#include "gcode-machine-control.h"

int usage(const char *prog) {
  fprintf(stderr, "Usage: %s [options] <gcode-file> [<gcode-file> ..]\n"
	  "Options:\n"
	  "\t-m <max-feedrate> : Maximum feedrate in mm/s\n"
	  "\t-f <factor>       : Speedup-factor for print\n"
          "\t-H                : Toggle print header line\n"
	  "Use filename '-' for stdin.\n", prog);
  return 1;
}

static void print_file_stats(const char *filename, int indentation,
			     const MachineControlConfig &config) {
  struct BeagleGPrintStats result;
  int fd = strcmp(filename, "-") == 0 ? STDIN_FILENO : open(filename, O_RDONLY);
  if (determine_print_stats(fd, config, &result)) {
    // Filament length looks a bit high, is this input or extruded ?
    printf("%-*s %10.0f %12.1f %14.1f",
           indentation, filename,
           result.total_time_seconds, result.last_z_extruding,
           result.filament_len);
    printf("\n");
  } else {
    printf("#%s not-processed\n", filename);
  }
}

int main(int argc, char *argv[]) {
  struct MachineControlConfig config;

  int max_feedrate = 200;  // mm/s
  float factor = 1.0;        // print speed factor.
  char print_header = 1;

  // TODO: read other parameters for the MachineControlConfig from long options.

  int opt;
  while ((opt = getopt(argc, argv, "f:m:H")) != -1) {
    switch (opt) {
    case 'f':
      factor = (float)atof(optarg);
      if (factor <= 0) return usage(argv[0]);
      break;
    case 'm':
      max_feedrate = atoi(optarg);
      if (max_feedrate <= 0) return usage(argv[0]);
      break;
    case 'H':
      print_header = !print_header;
      break;
    default:
      return usage(argv[0]);
    }
  }

  if (optind >= argc)
    return usage(argv[0]);

  config.max_feedrate[AXIS_X] = max_feedrate;
  config.max_feedrate[AXIS_Y] = max_feedrate;
  config.speed_factor = factor;

  int longest_filename = strlen("#[filename]"); // table header
  for (int i = optind; i < argc; ++i) {
    int len = strlen(argv[i]);
    if (len > longest_filename) longest_filename = len;
  }
  if (print_header) {
    printf("%-*s %10s %12s %14s\n", longest_filename,
           "#[filename]", "[time{s}]", "[height{mm}]",
           "[filament{mm}]");
  }
  for (int i = optind; i < argc; ++i) {
    print_file_stats(argv[i], longest_filename, config);
  }
  return 0;
}
