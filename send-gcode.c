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

// get usleep()
#define _XOPEN_SOURCE 500

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <getopt.h>

#include "gcode-parser.h"
#include "motor-interface.h"
#include "determine-print-stats.h"
#include "gcode-machine-control.h"

// Some default settings.
#define DEFAULT_MAX_FEEDRATE_MM_PER_SEC 600
static const int kStepsPerMM[] = { 160, 160, 160, 40, 0, 0, 0, 0 };

static void print_file_stats(const char *filename,
			     struct MachineControlConfig *config) {
  struct BeagleGPrintStats result;
  if (determine_print_stats(open(filename, O_RDONLY),
			    config->max_feedrate, config->speed_factor,
			    &result) == 0) {
    printf("----------------------------------------------\n");
    printf("Print time: %.3f seconds; %.1fmm height; "
	   "%.1fmm filament used.\n",
	   result.total_time_seconds, result.last_z, result.filament_len);
    if (result.highest_capped_feedrate > 0) {
      printf("Max feedrate requested %.1f mm/s, but capped to %.1f mm/s "
	     "(change max-feedrate with -m)\n",
	     result.highest_capped_feedrate, config->max_feedrate);
    }
    printf("----------------------------------------------\n");
  }
}

static int usage(const char *prog) {
   fprintf(stderr, "Usage: %s [options] <gcode-filename>\n"
	   "Options:\n"
	   "  -f <factor> : Print speed factor (Default 1.0).\n"
	   "  -m <rate>   : Max. feedrate (Default %dmm/s).\n"
	   "  -p          : Print motor commands to console (Default: off).\n"
	   "  -n          : Dryrun; don't send to motors (Default: off).\n"
	   "  -s          : Synchronous: don't queue (Default: off).\n"
	   "  -l          : Loop file forever.\n",
	   prog, DEFAULT_MAX_FEEDRATE_MM_PER_SEC);
   return 1;
}

static void send_file_to_printer(const char *filename, char do_loop) {
  do {
    int fd = open(filename, O_RDONLY);
    gcode_machine_control_from_stream(fd, STDERR_FILENO);
  } while (do_loop);
}

int main(int argc, char *argv[]) {
  struct MachineControlConfig config;
  // Per axis X, Y, Z, E (Z and E: need to look up)
  memcpy(config.axis_steps_per_mm, kStepsPerMM,
	 sizeof(config.axis_steps_per_mm));
  config.max_feedrate = DEFAULT_MAX_FEEDRATE_MM_PER_SEC;
  config.speed_factor = 1;
  config.dry_run = 0;
  config.debug_print = 0;
  config.synchronous = 0;
  char do_loop = 0;
  int opt;
  while ((opt = getopt(argc, argv, "spnlf:m:")) != -1) {
    switch (opt) {
    case 'f':
      config.speed_factor = atof(optarg);
      if (config.speed_factor <= 0) return usage(argv[0]);
      break;
    case 'm':
      config.max_feedrate = atoi(optarg);
      if (config.max_feedrate <= 0) return usage(argv[0]);
      break;
    case 'n':
      config.dry_run = 1;
      break;
    case 'p':
      config.debug_print = 1;
      break;
    case 's':
      config.synchronous = 1;
      break;
    case 'l':
      do_loop = 1;
      break;
    default:
      return usage(argv[0]);
    }
  }

  if (optind >= argc) return usage(argv[0]);  // expected filename.

  const char *filename = argv[optind];
  print_file_stats(filename, &config);

  if (gcode_machine_control_init(&config) != 0) {
    return 1;
  }
  send_file_to_printer(filename, do_loop);
  gcode_machine_control_exit();
  return 0;
}
