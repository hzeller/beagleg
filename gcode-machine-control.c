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
// Use fdopen()
#define _XOPEN_SOURCE 500

#include "gcode-machine-control.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/types.h>
#include <unistd.h>

#include "motor-interface.h"
#include "gcode-parser.h"

struct PrinterState {
  struct MachineControlConfig cfg;
  GCodeParser_t *parser;
  float current_feedrate_mm_per_sec;
  int machine_position[GCODE_NUM_AXES];  // Absolute position in steps.
  FILE *msg_stream;
};

// Since there is only one machine, we just keep this as a singleton.
static struct PrinterState *s_mstate = NULL;

// Dummy implementations of callbacks not yet handled.
static void dummy_set_temperature(void *userdata, float f) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream,
	    "MachineControl: set_temperature(%.1f) not implemented.\n", f);
  }
}
static void dummy_set_fanspeed(void *userdata, float speed) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream,
	    "MachineControl: set_fanspeed(%.0f) not implemented.\n", speed);
  }
}
static void dummy_wait_temperature(void *userdata) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream,
	    "MachineControl: wait_temperature() not implemented.\n");
  }
}
static void dummy_disable_motors(void *userdata) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream,
	    "MachineControl: disable_motors() not implemented.\n");
  }
}
static const char *dummy_unprocessed(void *userdata, char letter, float value,
				     const char *remaining) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream,
	    "MachineControl: didn't understand ('%c', %d, '%s')\n",
	    letter, (int) value, remaining);
  }
  return NULL;
}

static int choose_max_abs(int a, int b) {
  return abs(a) > abs(b) ? abs(a) : abs(b);
}

// Move the given number of machine steps for each axis.
static void move_machine_steps(struct PrinterState *state, float feedrate,
			       int machine_steps[]) {
  struct bg_movement command;
  bzero(&command, sizeof(command));
  char any_work = 0;
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    command.steps[i] = machine_steps[i];
    if (command.steps[i] != 0) any_work = 1;
  }

  if (!any_work) {
    return;  // Nothing to do.
  }

  // The axis with the lowest number of steps per mm ultimately determines
  // the maximum feedrate in steps/second (TODO: do relative to distance this
  // axis has to travel)
  int min_feedrate_relevant_steps_per_mm = -1;
  // But for now: set that to x/y speed.
  min_feedrate_relevant_steps_per_mm = state->cfg.axis_steps_per_mm[AXIS_X];

  int max_axis_steps = choose_max_abs(command.steps[AXIS_X],
				      command.steps[AXIS_Y]);
  if (max_axis_steps > 0) {
    double euclid_steps = sqrt(command.steps[AXIS_X] * command.steps[AXIS_X]
			       + command.steps[AXIS_Y] * command.steps[AXIS_Y]);
    command.travel_speed = max_axis_steps * min_feedrate_relevant_steps_per_mm
      * feedrate / euclid_steps;
  } else {
    command.travel_speed = min_feedrate_relevant_steps_per_mm * feedrate;
  }

  if (!state->cfg.dry_run) {
    if (state->cfg.synchronous) beagleg_wait_queue_empty();
    beagleg_enqueue(&command, state->msg_stream);
  }
  
  if (state->cfg.debug_print && state->msg_stream) {
    if (command.steps[2] != 0) {
      fprintf(state->msg_stream,
	      "(%6d, %6d) Z:%-3d E:%-2d step kHz:%-8.3f (%.1f mm/s)\n",
	      command.steps[0], command.steps[1], command.steps[2],
	      command.steps[3], command.travel_speed / 1000.0, feedrate);
    } else {
      fprintf(state->msg_stream,
	      "(%6d, %6d)       E:%-3d step kHz:%-8.3f (%.1f mm/s)\n",
	      command.steps[0], command.steps[1],
	      command.steps[3], command.travel_speed / 1000.0, feedrate);
    }
  }
}

static void printer_move(void *userdata, float feedrate, const float axis[]) {
  struct PrinterState *state = (struct PrinterState*)userdata;

  // Real world -> machine coordinates
  int new_machine_position[GCODE_NUM_AXES];
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    new_machine_position[i] = axis[i] * state->cfg.axis_steps_per_mm[i];
  }

  int differences[GCODE_NUM_AXES];
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    differences[i] = new_machine_position[i] - state->machine_position[i];
  }

  // TODO: for acceleration planning, we need to do a whole bunch more here.

  move_machine_steps(state, feedrate, differences);

  // This is now our new position.
  memcpy(state->machine_position, new_machine_position,
	 sizeof(state->machine_position));
}

static void printer_G1(void *userdata, float feed, const float *axis) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (feed > 0) {
    state->current_feedrate_mm_per_sec = state->cfg.speed_factor * feed;
    if (state->current_feedrate_mm_per_sec > state->cfg.max_feedrate) {
      state->current_feedrate_mm_per_sec = state->cfg.max_feedrate;
    }
  }
  printer_move(userdata, state->current_feedrate_mm_per_sec, axis);
}

static void printer_G0(void *userdata, float feed, const float *axis) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  float rapid_feed = state->cfg.max_feedrate;
  const float given = state->cfg.speed_factor * feed;
  if (feed > 0 && given < state->cfg.max_feedrate)
    rapid_feed = given;

  printer_move(userdata, rapid_feed, axis);
}

static void printer_dwell(void *userdata, float value) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (!state->cfg.dry_run) beagleg_wait_queue_empty();
  usleep((int) (value * 1000));
}

static void printer_home(void *userdata, unsigned char axes_bitmap) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  int machine_pos_differences[GCODE_NUM_AXES];
  bzero(machine_pos_differences, sizeof(machine_pos_differences));

  // Goal is to bring back the machine the negative amount of steps.
  for (int i = 0; i <= GCODE_NUM_AXES; ++i) {
    if ((1 << i) & axes_bitmap) {
      if (i != AXIS_E) {  // 'homing' of filament never makes sense.
	machine_pos_differences[i] = -state->machine_position[i];
      }
      state->machine_position[i] = 0;
    }
  }

  // We don't have endswitches yet, so homing brings us in a bad situation with
  // two bad solutions:
  //  (a) just 'assume' we're home. This really only works well the first time
  //      if the machine was manually homed. Followups are considering the last
  //      position as home, which might be ... uhm .. worse.
  //  (b) Rapid move to position 0 of the requested axes. This will work multiple
  //      times but still assumes that we were at 0 initially and it is subject
  //      to machine shift.
  // Solution (b) is what we're doing.
  // TODO: do this with endswitches.
  if (state->msg_stream) {
    fprintf(state->msg_stream, "MachineControl: Homing requested (0x%02x), but "
	    "don't have endswitches, so move difference steps (%d, %d, %d)\n",
	    axes_bitmap, machine_pos_differences[AXIS_X],
	    machine_pos_differences[AXIS_Y], machine_pos_differences[AXIS_Z]);
  }
  move_machine_steps(state, state->cfg.max_feedrate, machine_pos_differences);
}

int gcode_machine_control_init(const struct MachineControlConfig *config) {
  if (s_mstate != NULL) {
    fprintf(stderr, "gcode_machine_control_init(): already initialized.\n");
    return 1;
  }

  if (!config->dry_run) {
    if (geteuid() != 0) {
      // TODO: running as root is generally not a good idea. Setup permissions
      // to just access these GPIOs.
      fprintf(stderr, "Need to run as root to access GPIO pins. "
	      "(use the dryrun option -n to not write to GPIO)\n");
      return 1;
    }
    beagleg_init();
  }

  s_mstate = (struct PrinterState*) malloc(sizeof(struct PrinterState));
  bzero(s_mstate, sizeof(*s_mstate));
  s_mstate->cfg = *config;
  s_mstate->current_feedrate_mm_per_sec = config->max_feedrate / 10;

  struct GCodeParserCb callbacks;
  bzero(&callbacks, sizeof(callbacks));
  callbacks.coordinated_move = &printer_G1;
  callbacks.rapid_move = &printer_G0;
  callbacks.go_home = &printer_home;
  callbacks.dwell = &printer_dwell;

  // Not yet implemented
  callbacks.set_fanspeed = &dummy_set_fanspeed;
  callbacks.set_temperature = &dummy_set_temperature;
  callbacks.wait_temperature = &dummy_wait_temperature;
  callbacks.disable_motors = &dummy_disable_motors;
  callbacks.unprocessed = &dummy_unprocessed;

  // The parser keeps track of the real-world coordinates (mm), while we keep
  // track of the machine coordinates (steps). So it has the same life-cycle.
  s_mstate->parser = gcodep_new(&callbacks, s_mstate);

  return 0;
}

void gcode_machine_control_exit() {
  if (!s_mstate) {
    fprintf(stderr, "gcode_machine_control_exit() called without init.\n");
    return;
  }
  if (!s_mstate->cfg.dry_run)
    beagleg_exit();
  gcodep_delete(s_mstate->parser);
  free(s_mstate);
  s_mstate = NULL;
}

int gcode_machine_control_from_stream(int gcode_fd, int output_fd) {
  if (!s_mstate) {
    fprintf(stderr, "Machine control not initialized.\n");
    return 1;
  }

  if (output_fd >= 0) {
    s_mstate->msg_stream = fdopen(output_fd, "w");
    if (s_mstate->msg_stream) {
      // Output needs to be unbuffered, otherwise they'll never make it.
      setvbuf(s_mstate->msg_stream, NULL, _IONBF, 0);
    }
  }
  FILE *gcode_stream = fdopen(gcode_fd, "r");
  if (gcode_stream == NULL) {
    perror("Opening gcode stream");
    return 1;
  }

  char buffer[1024];
  while (fgets(buffer, sizeof(buffer), gcode_stream)) {
    gcodep_parse_line(s_mstate->parser, buffer, s_mstate->msg_stream);
  }

  if (s_mstate->msg_stream) {
    fflush(s_mstate->msg_stream);
    s_mstate->msg_stream = NULL;
  }
  fclose(gcode_stream);

  return 0;
}
