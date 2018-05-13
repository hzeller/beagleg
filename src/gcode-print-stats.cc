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

#include "common/logging.h"

#include "determine-print-stats.h"
#include "gcode-machine-control.h"
#include "config-parser.h"

int usage(const char *prog) {
  fprintf(stderr, "Usage: %s [options] <gcode-file> [<gcode-file> ..]\n"
          "Options:\n"
          "\t-c <config>       : Machine config\n"
          "\t-f <factor>       : Speedup-factor for feedrate.\n"
          "\t-H                : Toggle print header line\n"
          "Use filename '-' for stdin.\n", prog);
  return 1;
}

static void print_file_stats(const char *filename, int indentation,
                             FILE *msg_out,
                             const MachineControlConfig &config) {
  struct BeagleGPrintStats result;
  int fd = strcmp(filename, "-") == 0 ? STDIN_FILENO : open(filename, O_RDONLY);
  if (determine_print_stats(fd, config, msg_out, &result)) {
    // Filament length looks a bit high, is this input or extruded ?
    printf("%-*s %10.0f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f %7.1f",
           indentation, filename,
           result.total_time_seconds,
           result.x_min, result.x_max,
           result.y_min, result.y_max,
           result.z_min, result.z_max,
           result.last_z_extruding,
           result.filament_len);
    printf("\n");
  } else {
    printf("#%s not-processed\n", filename);
  }
}

int main(int argc, char *argv[]) {
  struct MachineControlConfig config;

  float factor = 1.0;        // print speed factor.
  char print_header = 1;
  const char *config_file = NULL;
  const char *msg_out_file = "/dev/null";

  int opt;
  while ((opt = getopt(argc, argv, "c:f:Hv")) != -1) {
    switch (opt) {
    case 'c':
      config_file = strdup(optarg);
      break;
    case 'f':
      factor = (float)atof(optarg);
      if (factor <= 0) return usage(argv[0]);
      break;
    case 'H':
      print_header = !print_header;
      break;
    case 'v':
      msg_out_file = "/dev/stderr";
      break;
    default:
      return usage(argv[0]);
    }
  }

  if (optind >= argc)
    return usage(argv[0]);

  if (!config_file) {
    fprintf(stderr, "Expected config file -c <config>\n");
    return 1;
  }

  FILE *msg_out = fopen(msg_out_file, "w");
  Log_init("/dev/null");

  ConfigParser config_parser;
  if (!config_parser.SetContentFromFile(config_file)) {
    fprintf(stderr, "Cannot read config file '%s'\n", config_file);
    return 1;
  }
  if (!config.ConfigureFromFile(&config_parser)) {
    fprintf(stderr, "Exiting. Parse error in configuration file '%s'\n", config_file);
    return 1;
  }

  config.speed_factor = factor;
  config.range_check = false;  // don't care about clipping.

  // This is not connected to any machine. Don't assume homing.
  config.require_homing = false;
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    config.homing_trigger[i] = HardwareMapping::TRIGGER_NONE;
  }

  int longest_filename = strlen("#[filename]"); // table header
  for (int i = optind; i < argc; ++i) {
    int len = strlen(argv[i]);
    if (len > longest_filename) longest_filename = len;
  }
  if (print_header) {
    printf("%-*s %10s %7s %7s %7s %7s %7s %7s %7s %7s\n", longest_filename,
           "#[filename]", "time",
           "min_x", "max_x", "min_y", "max_y", "min_z", "max_z",
           "z-last", "filament-mm");
  }
  for (int i = optind; i < argc; ++i) {
    print_file_stats(argv[i], longest_filename, msg_out, config);
  }
  fclose(msg_out);
  return 0;
}
