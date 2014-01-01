#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <math.h>

#include "gcode-parser.h"

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
  printf("Total print time: ~%.1f seconds; %.1fmm filament used\n",
	 state.total_time, state.filament_len);
}

int main(int argc, char *argv[]) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s <filename>\n", argv[0]);
    return 1;
  }
  const char *filename = argv[1];
  determine_duration(filename);
}
