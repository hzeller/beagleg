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

#define DEFAULT_MAX_FEEDRATE_MM_PER_SEC 600

// Per axis X, Y, Z, E (Z and E: need to look up)
static int kStepsPerMM[] = { 160, 160, 160, 40 };

struct PrintConfig {
  float max_feedrate;
  float speed_factor;
  char dry_run;
  char debug_print;
  char synchronous;
};

void print_file_stats(const char *filename, struct PrintConfig *config) {
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

struct PrinterState {
  struct PrintConfig config;
  float current_feedrate_mm_per_sec;
  int axes_pos[4];  // Absolute position in steps for each of the 4 axis
};

static void printer_feedrate(void *userdata, float fr) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  state->current_feedrate_mm_per_sec = state->config.speed_factor * fr/60;
  if (state->current_feedrate_mm_per_sec > state->config.max_feedrate) {
    state->current_feedrate_mm_per_sec = state->config.max_feedrate;
  }
}

static int choose_max_abs(int a, int b) {
  return abs(a) > abs(b) ? abs(a) : abs(b);
}
static void printer_move(void *userdata, float feedrate, const float *axis) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  int new_axes_pos[4];
  new_axes_pos[AXIS_X] = axis[AXIS_X] * kStepsPerMM[AXIS_X];
  new_axes_pos[AXIS_Y] = axis[AXIS_Y] * kStepsPerMM[AXIS_Y];
  new_axes_pos[AXIS_Z] = axis[AXIS_Z] * kStepsPerMM[AXIS_Z];
  new_axes_pos[AXIS_E] = axis[AXIS_E] * kStepsPerMM[AXIS_E];
  struct bg_movement command;
  bzero(&command, sizeof(command));
  command.steps[AXIS_X] = new_axes_pos[AXIS_X] - state->axes_pos[AXIS_X];
  command.steps[AXIS_Y] = new_axes_pos[AXIS_Y] - state->axes_pos[AXIS_Y];
  command.steps[AXIS_Z] = new_axes_pos[AXIS_Z] - state->axes_pos[AXIS_Z];
  command.steps[AXIS_E] = new_axes_pos[AXIS_E] - state->axes_pos[AXIS_E];
  if (command.steps[AXIS_X] == 0 && command.steps[AXIS_Y] == 0
      && command.steps[AXIS_Z] == 0 && command.steps[AXIS_E] == 0)
    return;		       

  int min_feedrate_relevant_steps_per_mm = -1;
#if 0
  for (int i = 0; i < 4; ++i) {
    // The axis with the lowest number of steps per mm ultimately determines
    // the maximum feedrate in steps/second (TODO: do relative to distance this
    // axis has to travel)
    if (command.steps[i] != 0 &&
	(min_feedrate_relevant_steps_per_mm < 0
	 || min_feedrate_relevant_steps_per_mm > kStepsPerMM[i])) {
      min_feedrate_relevant_steps_per_mm = kStepsPerMM[i];
    }
  }
#else
  // For now: set that to x/y speed.
  min_feedrate_relevant_steps_per_mm = kStepsPerMM[AXIS_X];
#endif

  int max_axis_steps = choose_max_abs(command.steps[AXIS_X],
				      command.steps[AXIS_Y]);
  if (max_axis_steps > 0) {
    double euklid_steps = sqrt(command.steps[AXIS_X] * command.steps[AXIS_X]
			       + command.steps[AXIS_Y] * command.steps[AXIS_Y]);
    command.travel_speed = max_axis_steps * min_feedrate_relevant_steps_per_mm
      * feedrate / euklid_steps;
  } else {
    command.travel_speed = min_feedrate_relevant_steps_per_mm * feedrate;
  }

  // This is now our new position.
  memcpy(state->axes_pos, new_axes_pos, sizeof(new_axes_pos));

  if (!state->config.dry_run) {
    if (state->config.synchronous) beagleg_wait_queue_empty();
    beagleg_enqueue(&command);
  }

  if (state->config.debug_print) {
    if (command.steps[2] != 0) {
      printf("(%6d, %6d) Z:%-3d E:%-2d step kHz:%-8.3f (%.1f mm/s)\n",
	     command.steps[0], command.steps[1], command.steps[2],
	     command.steps[3], command.travel_speed / 1000.0, feedrate);
    } else {
      printf("(%6d, %6d)       E:%-3d step kHz:%-8.3f (%.1f mm/s)\n",
	     command.steps[0], command.steps[1],
	     command.steps[3], command.travel_speed / 1000.0, feedrate);
    }
  }
}

static void printer_coordinated_move(void *userdata, const float *axis) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  printer_move(userdata, state->current_feedrate_mm_per_sec, axis);
}

static void printer_rapid_move(void *userdata, const float *axis) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  printer_move(userdata, state->config.max_feedrate, axis);
}
static void printer_dwell(void *userdata, float value) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (!state->config.dry_run) beagleg_wait_queue_empty();
  usleep((int) (value * 1000));
}

static void dummy_home(void *userdata, unsigned char x) {}

void send_to_printer(const char *filename, char do_loop,
		     struct PrintConfig *config) {
  if (!config->dry_run) beagleg_init();

  struct GCodeParserCb callbacks;
  bzero(&callbacks, sizeof(callbacks));
  callbacks.set_feedrate = &printer_feedrate;
  callbacks.coordinated_move = &printer_coordinated_move;
  callbacks.rapid_move = &printer_rapid_move;
  callbacks.go_home = &dummy_home;
  callbacks.dwell = &printer_dwell;

  do {
    struct PrinterState state;
    bzero(&state, sizeof(state));
    state.config = *config;
    GCodeParser_t *parser = gcodep_new(&callbacks, &state);
    FILE *f = fopen(filename, "r");
    if (f == NULL) {
      perror("Opening file for printing");
      return;
    }
    char buffer[1024];
    while (fgets(buffer, sizeof(buffer), f)) {
      gcodep_parse_line(parser, buffer);
    }
    gcodep_delete(parser);
    fclose(f);
  } while (do_loop);

  if (!config->dry_run) beagleg_exit();
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

int main(int argc, char *argv[]) {
  struct PrintConfig print_config;
  print_config.max_feedrate = DEFAULT_MAX_FEEDRATE_MM_PER_SEC;
  print_config.speed_factor = 1;
  print_config.dry_run = 0;
  print_config.debug_print = 0;
  print_config.synchronous = 0;
  char do_loop = 0;
  int opt;
  while ((opt = getopt(argc, argv, "spnlf:m:")) != -1) {
    switch (opt) {
    case 'f':
      print_config.speed_factor = atof(optarg);
      if (print_config.speed_factor <= 0) return usage(argv[0]);
      break;
    case 'm':
      print_config.max_feedrate = atoi(optarg);
      if (print_config.max_feedrate <= 0) return usage(argv[0]);
      break;
    case 'n':
      print_config.dry_run = 1;
      break;
    case 'p':
      print_config.debug_print = 1;
      break;
    case 's':
      print_config.synchronous = 1;
      break;
    case 'l':
      do_loop = 1;
      break;
    default:
      return usage(argv[0]);
    }
  }

  if (optind >= argc) return usage(argv[0]);

  const char *filename = argv[optind];

  print_file_stats(filename, &print_config);

  if (!print_config.dry_run && geteuid() != 0) {
    fprintf(stderr, "Need to run as root to access GPIO pins\n");
    return 1;
  }

  send_to_printer(filename, do_loop, &print_config);
}
