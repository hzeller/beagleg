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
#include "determine-print-stats.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#include "gcode-machine-control.h"
#include "gcode-parser.h"
#include "motor-operations.h"

struct StatsData {
  struct BeagleGPrintStats *stats;
  struct GCodeParserCb machine_delegatee;
};

// GCode parser event receivers, that forward calls to the delegate
// but also determine relevant height information.
static void forwarding_set_speed_factor(void *userdata, float f) {
  struct StatsData *data = (struct StatsData*)userdata;
  data->machine_delegatee.set_speed_factor(data->machine_delegatee.user_data, f);
}
static void forwarding_set_temperature(void *userdata, float f) {
  struct StatsData *data = (struct StatsData*)userdata;
  data->machine_delegatee.set_temperature(data->machine_delegatee.user_data, f);
}
static void forwarding_set_fanspeed(void *userdata, float speed) {
  struct StatsData *data = (struct StatsData*)userdata;
  data->machine_delegatee.set_fanspeed(data->machine_delegatee.user_data, speed);
}
static void forwarding_wait_temperature(void *userdata) {
  struct StatsData *data = (struct StatsData*)userdata;
  data->machine_delegatee.wait_temperature(data->machine_delegatee.user_data);
}
static void forwarding_dwell(void *userdata, float value) {
  struct StatsData *data = (struct StatsData*)userdata;
  data->stats->total_time_seconds += value / 1000.0f;
  // We call the original dell() with zero time as we don't want to spend _real_ time.
  data->machine_delegatee.dwell(data->machine_delegatee.user_data, 0);
}
static void forwarding_motors_enable(void *userdata, char b) {
  struct StatsData *data = (struct StatsData*)userdata;
  data->machine_delegatee.motors_enable(data->machine_delegatee.user_data, b);
}

static void update_coordinate_stats(struct BeagleGPrintStats *stats,
                                    const float *axis) {
  stats->last_x = axis[AXIS_X];
  stats->last_y = axis[AXIS_Y];
  stats->last_z = axis[AXIS_Z];
  if (axis[AXIS_E] > stats->filament_len) {
    stats->last_z_extruding = stats->last_z;
  }
  stats->filament_len = axis[AXIS_E];
}

static char forwarding_rapid_move(void *userdata, float feed, const float *axes) {
  struct StatsData *data = (struct StatsData*)userdata;
  update_coordinate_stats(data->stats, axes);
  return data->machine_delegatee.rapid_move(data->machine_delegatee.user_data, feed, axes);
}
static char forwarding_coordinated_move(void *userdata, float feed, const float *axes) {
  struct StatsData *data = (struct StatsData*)userdata;
  update_coordinate_stats(data->stats, axes);
  return data->machine_delegatee.coordinated_move(data->machine_delegatee.user_data, feed, axes);
}
static void forwarding_go_home(void *userdata, AxisBitmap_t axes) {
  struct StatsData *data = (struct StatsData*)userdata;
  data->machine_delegatee.go_home(data->machine_delegatee.user_data, axes);
}
static const char *forwarding_unprocessed(void *userdata, char letter, float value,
                                          const char *remaining) {
  struct StatsData *data = (struct StatsData*)userdata;
  return data->machine_delegatee.unprocessed(data->machine_delegatee.user_data,
                                             letter, value, remaining);
}

static void forwarding_info_origin_offset(void *userdata, const float *val) {
  struct StatsData *s = (struct StatsData*)userdata;
  s->machine_delegatee.inform_origin_offset(s->machine_delegatee.user_data, val);
}

namespace {
class StatsMotorOperations : public MotorOperations {
public:
  StatsMotorOperations(BeagleGPrintStats *stats) : print_stats_(stats) {}

  virtual int Enqueue(const MotorMovement &param, FILE *err_stream) {
    int max_steps = 0;
    for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
      int steps = abs(param.steps[i]);
      if (steps > max_steps)
        max_steps = steps;
    }

    // max_steps = a/2*t^2 + v0*t; a = (v1-v0)/t
    print_stats_->total_time_seconds += 2 * max_steps / (param.v0 + param.v1);
    //printf("HZ:v0=%7.1f v1=%7.1f steps=%d\n", param->v0, param->v1, max_steps);
    return 0;
  }

  virtual void MotorEnable(bool on) {}
  virtual void WaitQueueEmpty() {}

private:
  BeagleGPrintStats *const print_stats_;
};
}

int determine_print_stats(int input_fd, const MachineControlConfig &config,
			  struct BeagleGPrintStats *result) {
  struct StatsData data;
  bzero(&data, sizeof(data));
  bzero(result, sizeof(*result));
  data.stats = result;

  // Motor control that just determines the time spent turning the motor.
  StatsMotorOperations stats_motor_ops(result);
  GCodeMachineControl *machine_control
    = GCodeMachineControl::Create(config, &stats_motor_ops, NULL);
  assert(machine_control);

  machine_control->FillEventCallbacks(&data.machine_delegatee);

  struct GCodeParserConfig parser_config;
  bzero(&parser_config, sizeof(parser_config));

  parser_config.callbacks.user_data = &data;

  parser_config.callbacks.go_home = &forwarding_go_home;
  parser_config.callbacks.set_speed_factor = forwarding_set_speed_factor;
  parser_config.callbacks.set_fanspeed = forwarding_set_fanspeed;
  parser_config.callbacks.set_temperature = forwarding_set_temperature;
  parser_config.callbacks.wait_temperature = forwarding_wait_temperature;
  parser_config.callbacks.rapid_move = &forwarding_rapid_move;
  parser_config.callbacks.coordinated_move = &forwarding_coordinated_move;
  parser_config.callbacks.dwell = &forwarding_dwell;
  parser_config.callbacks.motors_enable = &forwarding_motors_enable;
  parser_config.callbacks.unprocessed = &forwarding_unprocessed;
  parser_config.callbacks.inform_origin_offset = &forwarding_info_origin_offset;

  GCodeParser_t *parser = gcodep_new(&parser_config);
  gcodep_parse_stream(parser, input_fd, stderr);
  gcodep_delete(parser);

  return 0;
}
