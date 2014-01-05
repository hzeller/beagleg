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

#include "determine-print-stats.h"

#include <math.h>
#include <strings.h>
#include <stdio.h>

#include "gcode-parser.h"

struct StatsData {
  float max_feedrate;
  float speed_factor;
  float current_feedrate;  // mm/s
  struct BeagleGPrintStats *stats;
};

static void dummy_home(void *userdata, unsigned char x) {}
static const char *dummy_unprocessed(void *userdata, char letter, float value,
				     const char *remaining) { return remaining; }
static void dummy_setvalue(void *userdata, float v) {}
static void dummy_noparam(void *userdata) {}
static void duration_feedrate(void *userdata, float fr) {
  struct StatsData *data = (struct StatsData*)userdata;
  data->current_feedrate = data->speed_factor * fr/60;
  if (data->current_feedrate > data->max_feedrate) {
    if (data->stats->highest_capped_feedrate < data->current_feedrate) {
      data->stats->highest_capped_feedrate = data->current_feedrate;
    }
    data->current_feedrate = data->max_feedrate;
  }
}
static void duration_move(void *userdata, const float *axis) {
  struct StatsData *d = (struct StatsData*)userdata;
  const float distance
    = sqrtf((axis[AXIS_X] - d->stats->last_x)*(axis[AXIS_X] - d->stats->last_x)
	    + (axis[AXIS_Y] - d->stats->last_y)*(axis[AXIS_Y] - d->stats->last_y)
	    + (axis[AXIS_Z] - d->stats->last_z)*(axis[AXIS_Z] - d->stats->last_z)
	    );
  // We're ignoring acceleration and assume full feedrate
  d->stats->total_time_seconds += distance / d->current_feedrate;
  d->stats->last_x = axis[AXIS_X];
  d->stats->last_y = axis[AXIS_Y];
  d->stats->last_z = axis[AXIS_Z];
  d->stats->filament_len = axis[AXIS_E];
}
static void duration_dwell(void *userdata, float value) {
  struct StatsData *data = (struct StatsData*)userdata;
  data->stats->total_time_seconds += value / 1000.0f;
}

int determine_print_stats(int input_fd,
			  float max_feedrate, float speed_factor,
			  struct BeagleGPrintStats *result) {
  struct StatsData data;
  bzero(&data, sizeof(data));
  data.max_feedrate = max_feedrate;
  data.speed_factor = speed_factor;
  bzero(result, sizeof(*result));
  data.stats = result;

  struct GCodeParserCb callbacks;
  bzero(&callbacks, sizeof(callbacks));
  callbacks.set_feedrate = &duration_feedrate;
  callbacks.coordinated_move = &duration_move;
  callbacks.go_home = &dummy_home;
  callbacks.dwell = &duration_dwell;
  callbacks.unprocessed = &dummy_unprocessed;
  callbacks.set_fanspeed = &dummy_setvalue;
  callbacks.set_temperature = &dummy_setvalue;
  callbacks.disable_motors = &dummy_noparam;
  callbacks.wait_temperature = &dummy_noparam;

  FILE *f = fdopen(input_fd, "r");
  if (f == NULL) {
    perror("Couldn't determine print stats");
    return 1;
  }
  GCodeParser_t *parser = gcodep_new(&callbacks, &data);
  char buffer[1024];
  while (fgets(buffer, sizeof(buffer), f)) {
    gcodep_parse_line(parser, buffer);
  }
  fclose(f);
  gcodep_delete(parser);

  return 0;
}
