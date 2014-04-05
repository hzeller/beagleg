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
#include "determine-print-stats.h"

#include <math.h>
#include <strings.h>
#include <stdio.h>

#include "gcode-parser.h"

struct StatsData {
  float max_feedrate;
  float cfg_speed_factor;   // speed factor set from commandline
  float prog_speed_factor;  // factor set from program
  float current_G1_feedrate;   // mm/s
  struct BeagleGPrintStats *stats;
};

static void dummy_home(void *userdata, unsigned char x) {}
static const char *dummy_unprocessed(void *userdata, char letter, float value,
				     const char *remaining) { return remaining; }
static void dummy_setvalue(void *userdata, float v) {}
static void dummy_noparam(void *userdata) {}
static void dummy_motors_enable(void *userdata, char b) {}

static void duration_move(struct BeagleGPrintStats *stats,
			  float feedrate, const float axis[]) {
  const float distance
    = sqrtf((axis[AXIS_X] - stats->last_x)*(axis[AXIS_X] - stats->last_x)
	    + (axis[AXIS_Y] - stats->last_y)*(axis[AXIS_Y] - stats->last_y)
	    + (axis[AXIS_Z] - stats->last_z)*(axis[AXIS_Z] - stats->last_z));
  // We're ignoring acceleration and assume full feedrate
  stats->total_time_seconds += distance / feedrate;
  stats->last_x = axis[AXIS_X];
  stats->last_y = axis[AXIS_Y];
  stats->last_z = axis[AXIS_Z];
  stats->filament_len = axis[AXIS_E];
}

static void duration_set_speed_factor(void *userdata, float value) {
  struct StatsData *data = (struct StatsData*)userdata;
  if (value < 0) {
    value = 1.0f + value;   // M220 S-10 interpreted as: 90%
  }
  if (value < 0.005) return;
  data->prog_speed_factor = value;
}

static void duration_G0(void *userdata, float feed, const float axis[]) {
  struct StatsData *data = (struct StatsData*)userdata;
  float rapid_feed = data->max_feedrate;
  const float given = data->cfg_speed_factor * data->prog_speed_factor * feed;
  if (feed > 0 && given < data->max_feedrate)
    rapid_feed = given;

  // Feedrate for G0 we only obey once, but don't remember
  duration_move(data->stats, rapid_feed, axis);
}

static void duration_G1(void *userdata, float feed, const float axis[]) {
  struct StatsData *data = (struct StatsData*)userdata;
  if (feed > 0) {
    // Change current feedrate.
    data->current_G1_feedrate = data->cfg_speed_factor * feed;
  }
  float feedrate = data->current_G1_feedrate * data->prog_speed_factor;
  if (feedrate > data->max_feedrate) {
    feedrate = data->max_feedrate;  // limit.
  }
  if (feedrate > data->stats->max_G1_feedrate) {
    data->stats->max_G1_feedrate = feedrate;
  }
  duration_move(data->stats, feedrate, axis);
}

static void duration_dwell(void *userdata, float value) {
  struct StatsData *data = (struct StatsData*)userdata;
  data->stats->total_time_seconds += value / 1000.0f;
}

int determine_print_stats(int input_fd, float max_feedrate, float speed_factor,
			  struct BeagleGPrintStats *result) {
  struct StatsData data;
  bzero(&data, sizeof(data));
  data.max_feedrate = max_feedrate;
  data.cfg_speed_factor = speed_factor;
  data.prog_speed_factor = 1.0f;
  data.current_G1_feedrate = max_feedrate / 10; // some reasonable default.
  bzero(result, sizeof(*result));
  data.stats = result;

  struct GCodeParserCb callbacks;
  bzero(&callbacks, sizeof(callbacks));
  callbacks.rapid_move = &duration_G0;
  callbacks.coordinated_move = &duration_G1;
  callbacks.dwell = &duration_dwell;
  callbacks.set_speed_factor = &duration_set_speed_factor;

  // Not implemented
  callbacks.go_home = &dummy_home;
  callbacks.unprocessed = &dummy_unprocessed;
  callbacks.set_fanspeed = &dummy_setvalue;
  callbacks.set_temperature = &dummy_setvalue;
  callbacks.motors_enable = &dummy_motors_enable;
  callbacks.wait_temperature = &dummy_noparam;

  FILE *f = fdopen(input_fd, "r");
  if (f == NULL) {
    perror("Couldn't determine print stats");
    return 1;
  }
  GCodeParser_t *parser = gcodep_new(&callbacks, &data);
  char buffer[1024];
  while (fgets(buffer, sizeof(buffer), f)) {
    gcodep_parse_line(parser, buffer, stderr);
  }
  fclose(f);
  gcodep_delete(parser);

  return 0;
}
