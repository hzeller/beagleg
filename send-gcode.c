#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>

#include "gcode-parser.h"
#include "motor-interface.h"

/* Little sample code to determine the approximate total print time */
struct DurationData {
  float total_time;
  float current_feedrate_mm_per_sec;
  float last_x, last_y, last_z;  // last coordinate.
  float filament_len;
};

static void dummy_home(void *userdata, unsigned char x) {}
static const char *dummy_unprocessed(void *userdata, char letter, float value,
				     const char *remaining) { return NULL; }

static void duration_feedrate(void *userdata, float fr) {
  ((struct DurationData*)userdata)->current_feedrate_mm_per_sec = fr/60;
}
static void duration_move(void *userdata, const float *axis) {
  struct DurationData *state = (struct DurationData*)userdata;
  const float distance
    = sqrtf((axis[AXIS_X] - state->last_x)*(axis[AXIS_X] - state->last_x)
	    + (axis[AXIS_Y] - state->last_y)*(axis[AXIS_Y] - state->last_y)
	    + (axis[AXIS_Z] - state->last_z)*(axis[AXIS_Z] - state->last_z));
  // we're ignoring acceleration and assume full feedrate
  state->total_time += distance / state->current_feedrate_mm_per_sec;
  state->last_x = axis[AXIS_X];
  state->last_y = axis[AXIS_Y];
  state->last_z = axis[AXIS_Z];
  state->filament_len = axis[AXIS_E];
}

void determine_duration(const char *filename) {
  struct DurationData state;
  bzero(&state, sizeof(state));
  struct GCodeParserCb callbacks;
  bzero(&callbacks, sizeof(callbacks));
  callbacks.set_feedrate = &duration_feedrate;
  callbacks.coordinated_move = &duration_move;
  callbacks.go_home = &dummy_home;
  callbacks.unprocessed = &dummy_unprocessed;
  GCodeParser_t *parser = gcodep_new(&callbacks, &state);
  FILE *f = fopen(filename, "r");
  char buffer[1024];
  while (fgets(buffer, sizeof(buffer), f)) {
    gcodep_parse_line(parser, buffer);
  }
  gcodep_delete(parser);
  printf("Total original print time: ~%.1f seconds; %.1fmm filament used\n",
	 state.total_time, state.filament_len);
}

// Per axis X, Y, Z, E (Z and E: need to look up)
static int kStepsPerMM[] = { 160, 160, 160, 40 };
struct PrinterState {
  float speed_factor;
  float current_feedrate_mm_per_sec;
  int axes_pos[4];  // Absolute position in steps for each of the 4 axis
};
static void printer_feedrate(void *userdata, float fr) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  state->current_feedrate_mm_per_sec = state->speed_factor * fr/60;
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

  // For now: set that to x/y speed.
  min_feedrate_relevant_steps_per_mm = kStepsPerMM[AXIS_X];

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

  beagleg_enqueue(&command);
  printf("x=%4d y=%4d z=%4d e=%4d speed=%6.2f\n",
	 command.steps[0], command.steps[1], command.steps[2],
	 command.steps[3], command.travel_speed);
}

static void printer_coordinated_move(void *userdata, const float *axis) {
  struct PrinterState *state = (struct PrinterState*)userdata;
  printer_move(userdata, state->current_feedrate_mm_per_sec, axis);
}

static void printer_rapid_move(void *userdata, const float *axis) {
  printer_move(userdata, 600, axis);
}

void send_to_printer(const char *filename, float factor) {
  beagleg_init();
  struct PrinterState state;
  bzero(&state, sizeof(state));
  state.speed_factor = factor;
  struct GCodeParserCb callbacks;
  bzero(&callbacks, sizeof(callbacks));
  callbacks.set_feedrate = &printer_feedrate;
  callbacks.coordinated_move = &printer_coordinated_move;
  callbacks.rapid_move = &printer_rapid_move;
  callbacks.go_home = &dummy_home;
  GCodeParser_t *parser = gcodep_new(&callbacks, &state);
  FILE *f = fopen(filename, "r");
  char buffer[1024];
  while (fgets(buffer, sizeof(buffer), f)) {
    gcodep_parse_line(parser, buffer);
  }
  gcodep_delete(parser);
  beagleg_exit();
}

static int usage(const char *prog) {
   fprintf(stderr, "Usage: %s <filename> [<speed-factor>]\n", prog);
   return 1;
}

int main(int argc, char *argv[]) {
  if (argc < 2 || argc > 3) {
    return usage(argv[0]);
  }
  const char *filename = argv[1];
  float factor = 1.0;
  if (argc >= 3) {
    factor = atof(argv[2]);
    if (factor <= 0) return usage(argv[0]);
  }
  determine_duration(filename);
  if (geteuid() != 0) {
    fprintf(stderr, "Need to run as root to access GPIO pins\n");
    return 1;
  }
  send_to_printer(filename, factor);
}
