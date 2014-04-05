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
#include "gcode-machine-control.h"

#include <ctype.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/types.h>
#include <unistd.h>

#include "motor-interface.h"
#include "gcode-parser.h"

// In case we get a zero feedrate, send this frequency to motors instead.
#define ZERO_FEEDRATE_OVERRIDE_HZ 5

#define VERSION_STRING "PROTOCOL_VERSION:0.1 FIRMWARE_NAME:BeagleG "    \
  "FIRMWARE_URL:http%3A//github.com/hzeller/beagleg"

struct PrinterState {
  const struct MachineControlConfig cfg;
  // Derived configuration
  float g0_feedrate_mm_per_sec;          // Highest of all axes; used for G0
                                         // (will be trimmed if needed)
  // Pre-calcualted per axis limits in steps/s, steps/s^2
  // All arrays are indexed by axis.
  float max_axis_speed[GCODE_NUM_AXES];  // max travel speed hz
  float max_axis_accel[GCODE_NUM_AXES];  // acceleration hz/s
  float highest_accel;                   // hightest accel of all axes.

  int axis_to_driver[GCODE_NUM_AXES];    // Which axis is mapped to which
                                         // physical output driver. This allows
                                         // to have a logical axis (e.g. X, Y,
                                         // Z) output to any physical driver.
  GCodeParser_t *parser;

  // Current machine state
  float current_feedrate_mm_per_sec;
  float prog_speed_factor;               // Speed factor set by program (M220)
  int machine_position[GCODE_NUM_AXES];  // Absolute position in steps.
  FILE *msg_stream;
};

// Since there is only one machine, we just keep this as a singleton.
static struct PrinterState *s_mstate = NULL;

// It is usually good to shut down gracefully, otherwise the PRU keeps running.
// So we're intercepting signals and exit gcode_machine_control_from_stream()
// cleanly.
static volatile char caught_signal = 0;
static void receive_signal() {
  caught_signal = 1;
  static char msg[] = "Caught signal. Shutting down ASAP.\n";
  (void)write(STDERR_FILENO, msg, sizeof(msg)); // void, but gcc still warns :/
}
static void arm_signal_handler() {
  caught_signal = 0;
  signal(SIGTERM, &receive_signal);  // Regular kill
  signal(SIGINT, &receive_signal);   // Ctrl-C
}
static void disarm_signal_handler() {
  signal(SIGTERM, SIG_DFL);  // Regular kill
  signal(SIGINT, SIG_DFL);   // Ctrl-C
}

// Dummy implementations of callbacks not yet handled.
static void dummy_set_temperature(void *userdata, float f) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream,
	    "// BeagleG: set_temperature(%.1f) not implemented.\n", f);
  }
}
static void dummy_set_fanspeed(void *userdata, float speed) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream,
	    "// BeagleG: set_fanspeed(%.0f) not implemented.\n", speed);
  }
}
static void dummy_wait_temperature(void *userdata) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (state->msg_stream) {
    fprintf(state->msg_stream,
	    "// BeagleG: wait_temperature() not implemented.\n");
  }
}
static void motors_enable(void *userdata, char b) {  
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (!state->cfg.dry_run) beagleg_motor_enable(b);
}

static const char *special_commands(void *userdata, char letter, float value,
				    const char *remaining) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (!state->msg_stream)
    return NULL;
  switch ((int) value) {
  case 105: fprintf(state->msg_stream, "ok T-300\n"); break;  // no temp yet.
  case 114:
    fprintf(state->msg_stream, "ok C: X:%.3f Y:%.3f Z%.3f E%.3f\n",
	    (1.0f * state->machine_position[AXIS_X]
	     / state->cfg.steps_per_mm[AXIS_X]),
	    (1.0f * state->machine_position[AXIS_Y]
	     / state->cfg.steps_per_mm[AXIS_Y]),
	    (1.0f * state->machine_position[AXIS_Z]
	     / state->cfg.steps_per_mm[AXIS_Z]),
	    (1.0f * state->machine_position[AXIS_E]
	     / state->cfg.steps_per_mm[AXIS_E]));
    break;
  case 115: fprintf(state->msg_stream, "ok %s\n", VERSION_STRING); break;
  default:  fprintf(state->msg_stream,
		    "// BeagleG: didn't understand ('%c', %d, '%s')\n",
		    letter, (int) value, remaining);
    break;
  }
  return NULL;
}
static double euklid_distance(double x, double y, double z) {
  return sqrt(x*x + y*y + z*z);
}

// Move the given number of machine steps for each axis.
static void move_machine_steps(struct PrinterState *state,
			       float requested_feedrate_mm_s,
			       int machine_steps[]) {
  struct bg_movement command;
  bzero(&command, sizeof(command));
  char any_work = 0;
  int axis_steps[GCODE_NUM_AXES];
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    axis_steps[i] = machine_steps[i];
    if (axis_steps[i] != 0) any_work = 1;
  }

  if (!any_work) {
    return;  // Nothing to do.
  }

  // The defining axis is the axis that requires to go the most number of steps.
  // it defines the frequency to go.
  enum GCodeParserAxes defining_axis = AXIS_X;
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (abs(axis_steps[i]) > abs(axis_steps[defining_axis]))
      defining_axis = (enum GCodeParserAxes) i;
  }

  command.travel_speed
    = requested_feedrate_mm_s * state->cfg.steps_per_mm[defining_axis];
  command.acceleration = state->highest_accel;  // Trimmed below.

  // If we're in the euklidian space, choose the step-frequency according to
  // the relative feedrate of the defining axis.
  // (A straight 200mm/s should be the same as a diagnoal 200mm/s)
  if (defining_axis == AXIS_X
      || defining_axis == AXIS_Y
      || defining_axis == AXIS_Z) {
    // We need to calculate the feedrate in real-world coordinates as each
    // axis can have a different amount of steps/mm
    const float total_xyz_length = 
      euklid_distance(axis_steps[AXIS_X] / state->cfg.steps_per_mm[AXIS_X],
		      axis_steps[AXIS_Y] / state->cfg.steps_per_mm[AXIS_Y],
		      axis_steps[AXIS_Z] / state->cfg.steps_per_mm[AXIS_Z]);
    const float steps_per_mm = state->cfg.steps_per_mm[defining_axis];  
    const float defining_axis_length = axis_steps[defining_axis]/steps_per_mm;
    const float euklid_fraction = fabsf(defining_axis_length) / total_xyz_length;
    command.travel_speed *= euklid_fraction;
  }

  // Now: range limiting. We trim speed and acceleration to what the weakest
  // involved axis can handle.
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (axis_steps[i] == 0)
      continue;
    // We only get this fraction of steps, so this is how our speed is scaled.
    float fraction = fabs(1.0 * axis_steps[i] / axis_steps[defining_axis]);
    if (command.travel_speed * fraction > state->max_axis_speed[i])
      command.travel_speed = state->max_axis_speed[i] / fraction;
    // Acceleration can be set to a value <= 0 to mean 'infinite'.
    if (state->max_axis_accel[i] > 0
	&& command.acceleration * fraction > state->max_axis_accel[i])
      command.acceleration = state->max_axis_accel[i] / fraction;
  }
  
  if (command.travel_speed == 0) {
    // In case someone choose a feedrate of 0, set something smallish.
    if (state->msg_stream) {
      fprintf(state->msg_stream,
	      "// Ignoring speed of 0, setting to %.6f mm/s\n",
	      (1.0f * ZERO_FEEDRATE_OVERRIDE_HZ
	       / state->cfg.steps_per_mm[defining_axis]));
    }
    command.travel_speed = ZERO_FEEDRATE_OVERRIDE_HZ;
  }

  // Now map axis steps to actual motor driver
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    const int motor_for_axis = state->axis_to_driver[i];
    if (motor_for_axis < 0) continue;  // no mapping.
    command.steps[motor_for_axis] = axis_steps[i];
  }

  if (!state->cfg.dry_run) {
    if (state->cfg.synchronous) beagleg_wait_queue_empty();
    beagleg_enqueue(&command, state->msg_stream);
  }
  
  if (state->cfg.debug_print && state->msg_stream) {
    float defining_feedrate
      = command.travel_speed / state->cfg.steps_per_mm[defining_axis];
    float defining_accel
      = command.acceleration / state->cfg.steps_per_mm[defining_axis];
    if (axis_steps[AXIS_Z] != 0) {
      fprintf(state->msg_stream,
	      "// (%6d, %6d) Z:%-3d E:%-2d step kHz:%-8.3f "
	      "(main axis: %.1f mm/s, %.1fmm/s^2)\n",
	      axis_steps[AXIS_X], axis_steps[AXIS_Y],
	      axis_steps[AXIS_Z], axis_steps[AXIS_E],
	      command.travel_speed / 1000.0, defining_feedrate, defining_accel);
    } else {
      fprintf(state->msg_stream,  // less clutter, when there is no Z
	      "// (%6d, %6d)       E:%-3d step kHz:%-8.3f "
	      "(main axis: %.1f mm/s, %.1fmm/s^2)\n",
	      axis_steps[AXIS_X], axis_steps[AXIS_Y],
	      axis_steps[AXIS_E], command.travel_speed / 1000.0,
	      defining_feedrate, defining_accel);
    }
  }
}

static void machine_move(void *userdata, float feedrate, const float axis[]) {
  struct PrinterState *state = (struct PrinterState*)userdata;

  // Real world -> machine coordinates
  int new_machine_position[GCODE_NUM_AXES];
  int differences[GCODE_NUM_AXES];
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    new_machine_position[i] = roundf(axis[i] * state->cfg.steps_per_mm[i]);
    differences[i] = new_machine_position[i] - state->machine_position[i];
  }

  // TODO: for acceleration planning, we need to do a whole bunch more here.

  move_machine_steps(state, feedrate, differences);

  // This is now our new position.
  memcpy(state->machine_position, new_machine_position,
	 sizeof(state->machine_position));
}

static void machine_G1(void *userdata, float feed, const float *axis) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (feed > 0) {
    state->current_feedrate_mm_per_sec = state->cfg.speed_factor * feed;
  }
  float feedrate = state->prog_speed_factor * state->current_feedrate_mm_per_sec;
  machine_move(userdata, feedrate, axis);
}

static void machine_G0(void *userdata, float feed, const float *axis) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  float rapid_feed = state->g0_feedrate_mm_per_sec;
  const float given = state->cfg.speed_factor * state->prog_speed_factor * feed;
  machine_move(userdata, given > 0 ? given : rapid_feed, axis);
}

static void machine_dwell(void *userdata, float value) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (!state->cfg.dry_run) beagleg_wait_queue_empty();
  usleep((int) (value * 1000));
}

static void machine_set_speed_factor(void *userdata, float value) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  if (value < 0) {
    value = 1.0f + value;   // M220 S-10 interpreted as: 90%
  }
  if (value < 0.005) {
    if (state->msg_stream) fprintf(state->msg_stream,
				   "// M220: Not accepting speed "
				   "factors < 0.5%% (got %.1f%%)\n",
				   100.0f * value);
    return;
  }
  state->prog_speed_factor = value;
}

static void machine_home(void *userdata, unsigned char axes_bitmap) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  int machine_pos_differences[GCODE_NUM_AXES];
  bzero(machine_pos_differences, sizeof(machine_pos_differences));

  // TODO(hzeller): use home_switch info.
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
    fprintf(state->msg_stream, "// BeagleG: Homing requested (0x%02x), but "
	    "don't have endswitches, so move difference steps (%d, %d, %d)\n",
	    axes_bitmap, machine_pos_differences[AXIS_X],
	    machine_pos_differences[AXIS_Y], machine_pos_differences[AXIS_Z]);
  }
  move_machine_steps(state, state->g0_feedrate_mm_per_sec,
		     machine_pos_differences);
}

// Cleanup whatever is allocated. Return 1 for convenience in early exit.
static int cleanup_state() {
  free(s_mstate);
  s_mstate = NULL;
  return 1;
}

int gcode_machine_control_init(const struct MachineControlConfig *config) {
  if (s_mstate != NULL) {
    fprintf(stderr, "gcode_machine_control_init(): already initialized.\n");
    return 1;
  }

  // Initialize basic state and derived configuration.
  s_mstate = (struct PrinterState*) malloc(sizeof(struct PrinterState));
  bzero(s_mstate, sizeof(*s_mstate));
  // Here we assign it to the 'const' cfg, all other accesses will check for
  // the readonly ness. So some nasty override here: we know what we're doing.
  *((struct MachineControlConfig*) &s_mstate->cfg) = *config;
  s_mstate->current_feedrate_mm_per_sec = config->max_feedrate[AXIS_X] / 10;
  float lowest_accel
    = config->max_feedrate[AXIS_X] * config->steps_per_mm[AXIS_X];
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (config->max_feedrate[i] > s_mstate->g0_feedrate_mm_per_sec)
      s_mstate->g0_feedrate_mm_per_sec = config->max_feedrate[i];
    s_mstate->max_axis_speed[i]
      = config->max_feedrate[i] * config->steps_per_mm[i];
    float accel = config->acceleration[i] * config->steps_per_mm[i];
    s_mstate->max_axis_accel[i] = accel;
    if (accel > s_mstate->highest_accel)
      s_mstate->highest_accel = accel;
    if (accel < lowest_accel)
      lowest_accel = accel;
  }
  s_mstate->prog_speed_factor = 1.0f;

  // Mapping axes to physical motors. We might have a larger set of logical
  // axes of which we map a subset to actual motors.
  // We do this in two steps: One identifies which io-pin actually goes to which
  // physical location (a property of the actual cape), the second maps
  // logical axes (e.g. 'X') to the location on the board.
  // This double mapping is done, so that it is intuitive for users to map
  // (as the first is a hardware property that doesn't really change and the
  // second the mapping the user wants).

  // Mapping of connector position on cape to driver ID (the axis in the
  // motor interface). This might differ due to physical board layout reasons.
  int pos_to_driver[BEAGLEG_NUM_MOTORS];
  for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
    pos_to_driver[i] = -1;
  }
  const char *physical_mapping = config->channel_layout;
  if (physical_mapping == NULL) physical_mapping = "23140";  // bumps board.
  if (strlen(physical_mapping) > BEAGLEG_NUM_MOTORS) {
    fprintf(stderr, "Physical mapping string longer than available motors. "
            "('%s', max axes=%d)\n", physical_mapping, BEAGLEG_NUM_MOTORS);
    return cleanup_state();
  }
  for (int pos = 0; *physical_mapping; pos++, physical_mapping++) {
    const int mapped_driver = *physical_mapping - '0';
    if (mapped_driver >= 0 && mapped_driver < BEAGLEG_NUM_MOTORS) {
      pos_to_driver[pos] = mapped_driver;
    }
    else {
      fprintf(stderr, "Invalid character '%c' in channel-layout mapping. "
              "Can be characters '0'..'%d'\n",
              *physical_mapping, BEAGLEG_NUM_MOTORS - 1);
      return cleanup_state();
    }
  }

  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    s_mstate->axis_to_driver[i] = -1;
  }
  const char *axis_mapping = config->axis_mapping;
  if (axis_mapping == NULL) axis_mapping = "XYZEABC";
  if (strlen(axis_mapping) > BEAGLEG_NUM_MOTORS) {
    fprintf(stderr, "Axis mapping string longer than available connectors."
            "('%s', max axes=%d)\n", axis_mapping, BEAGLEG_NUM_MOTORS);
    return cleanup_state();
  }
  for (int pos = 0; *axis_mapping; pos++, axis_mapping++) {
    switch (toupper(*axis_mapping)) {
    case 'X': s_mstate->axis_to_driver[AXIS_X] = pos_to_driver[pos]; break;
    case 'Y': s_mstate->axis_to_driver[AXIS_Y] = pos_to_driver[pos]; break;
    case 'Z': s_mstate->axis_to_driver[AXIS_Z] = pos_to_driver[pos]; break;
    case 'E': s_mstate->axis_to_driver[AXIS_E] = pos_to_driver[pos]; break;
    case 'A': s_mstate->axis_to_driver[AXIS_A] = pos_to_driver[pos]; break;
    case 'B': s_mstate->axis_to_driver[AXIS_B] = pos_to_driver[pos]; break;
    case 'C': s_mstate->axis_to_driver[AXIS_C] = pos_to_driver[pos]; break;
    case '_': break;  // skip.
    default:
      fprintf(stderr, "Illegal axis->connector mapping character '%c' in '%s' "
	      "(Only valid axis letter or '_' to skip a connector)\n",
	      toupper(*axis_mapping), config->axis_mapping);
      return cleanup_state();
    }
  }

  struct GCodeParserCb callbacks;
  bzero(&callbacks, sizeof(callbacks));
  callbacks.coordinated_move = &machine_G1;
  callbacks.rapid_move = &machine_G0;
  callbacks.go_home = &machine_home;
  callbacks.dwell = &machine_dwell;
  callbacks.set_speed_factor = &machine_set_speed_factor;
  callbacks.motors_enable = &motors_enable;
  callbacks.unprocessed = &special_commands;

  // Not yet implemented
  callbacks.set_fanspeed = &dummy_set_fanspeed;
  callbacks.set_temperature = &dummy_set_temperature;
  callbacks.wait_temperature = &dummy_wait_temperature;

  // The parser keeps track of the real-world coordinates (mm), while we keep
  // track of the machine coordinates (steps). So it has the same life-cycle.
  s_mstate->parser = gcodep_new(&callbacks, s_mstate);

  // Init motor control.
  if (!config->dry_run) {
    if (geteuid() != 0) {
      // TODO: running as root is generally not a good idea. Setup permissions
      // to just access these GPIOs.
      fprintf(stderr, "Need to run as root to access GPIO pins. "
	      "(use the dryrun option -n to not write to GPIO)\n");
      return cleanup_state();
    }
    if (beagleg_init(lowest_accel) != 0) {
      return cleanup_state();
    }
  }

  return 0;
}

void gcode_machine_control_exit() {
  if (!s_mstate) {
    fprintf(stderr, "gcode_machine_control_exit() called without init.\n");
    return;
  }
  if (!s_mstate->cfg.dry_run) {
    if (caught_signal) {
      fprintf(stderr, "Skipping potential remaining queue.\n");
      beagleg_exit_nowait();
    } else {
      beagleg_exit();
    }
  }
  gcodep_delete(s_mstate->parser);
  cleanup_state();
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

  arm_signal_handler();
  char buffer[1024];
  while (!caught_signal && fgets(buffer, sizeof(buffer), gcode_stream)) {
    gcodep_parse_line(s_mstate->parser, buffer, s_mstate->msg_stream);
  }
  disarm_signal_handler();

  if (s_mstate->msg_stream) {
    fflush(s_mstate->msg_stream);
    s_mstate->msg_stream = NULL;
  }
  fclose(gcode_stream);

  return caught_signal ? 2 : 0;
}
