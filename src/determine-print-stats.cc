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

#include "gcode-parser/gcode-parser.h"

#include "gcode-machine-control.h"
#include "motor-operations.h"
#include "hardware-mapping.h"
#include "spindle-control.h"

namespace {
// An event receiver for GCodeParser events that are intercepted, stats
// recorded, then further delegated to an original GCodeParser::Events
// receiver
class StatsCollectingEventDelegator : public GCodeParser::EventReceiver {
public:
  StatsCollectingEventDelegator(BeagleGPrintStats *stats,
                                GCodeParser::EventReceiver *delegatee)
    : stats_(stats), delegatee_(delegatee) {
  }

  virtual void gcode_start(GCodeParser *p) { delegatee_->gcode_start(p); }
  virtual void gcode_finished(bool eos) { delegatee_->gcode_finished(eos); }

  // GCode parser event receivers, that forward calls to the delegate
  // but also determine relevant height information.
  virtual void set_speed_factor(float f) { delegatee_->set_speed_factor(f);  }
  virtual void set_temperature(float f) { delegatee_->set_temperature(f); }
  virtual void set_fanspeed(float speed) { delegatee_->set_fanspeed(speed);  }
  virtual void wait_temperature() { delegatee_->wait_temperature(); }
  virtual void motors_enable(bool b) { delegatee_->motors_enable(b); }
  virtual void go_home(AxisBitmap_t axes) { /* ignore */ }
  virtual void inform_origin_offset(const AxesRegister& axes) {
    delegatee_->inform_origin_offset(axes);
  }

  virtual void dwell(float value) {
    stats_->total_time_seconds += value / 1000.0f;
    // We call the original dell() with zero time as we don't want to spend _real_ time.
    delegatee_->dwell(0);
  }

  virtual bool rapid_move(float feed, const AxesRegister &axes) {
    update_coordinate_stats(axes);
    return delegatee_->rapid_move(feed, axes);
  }
  virtual bool coordinated_move(float feed, const AxesRegister &axes) {
    update_coordinate_stats(axes);
    return delegatee_->coordinated_move(feed, axes);
  }

  virtual const char *unprocessed(char letter, float value, const char *remain) {
    return delegatee_->unprocessed(letter, value, remain);
  }

private:
  void update_coordinate_stats(const AxesRegister &axis) {
    stats_->last_x = axis[AXIS_X];
    stats_->last_y = axis[AXIS_Y];
    stats_->last_z = axis[AXIS_Z];
    if (axis[AXIS_E] > stats_->filament_len) {
      stats_->last_z_extruding = stats_->last_z;
    }
    stats_->filament_len = axis[AXIS_E];
  }

  struct BeagleGPrintStats *const stats_;
  GCodeParser::EventReceiver *const delegatee_;
};

class StatsMotorOperations : public MotorOperations {
public:
  StatsMotorOperations(BeagleGPrintStats *stats) : print_stats_(stats) {}

  virtual void Enqueue(const LinearSegmentSteps &param) {
    int max_steps = 0;
    for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
      int steps = abs(param.steps[i]);
      if (steps > max_steps)
        max_steps = steps;
    }

    // max_steps = a/2*t^2 + v0*t; a = (v1-v0)/t
    print_stats_->total_time_seconds += 2 * max_steps / (param.v0 + param.v1);
    //printf("HZ:v0=%7.1f v1=%7.1f steps=%d\n", param.v0, param.v1, max_steps);
  }

  virtual void MotorEnable(bool on) {}
  virtual void WaitQueueEmpty() {}

private:
  BeagleGPrintStats *const print_stats_;
};
}

bool determine_print_stats(int input_fd, const MachineControlConfig &config,
                           FILE *msg_out,
                           struct BeagleGPrintStats *result) {
  bzero(result, sizeof(*result));

  HardwareMapping hardware;  // We never initialize, just sim mode.
  Spindle spindle;

  // Motor control that just determines the time spent turning the motor.
  // We do that by intercepting the motor operations by replacing the
  // implementation with our own.
  StatsMotorOperations stats_motor_ops(result);
  GCodeMachineControl *machine_control
    = GCodeMachineControl::Create(config, &stats_motor_ops,
                                  &hardware, &spindle, NULL);
  if (!machine_control)
    return false;

  // We intercept gcode events to update some stats, then pass on to
  // machine event receiver.
  StatsCollectingEventDelegator
    stats_event_receiver(result, machine_control->ParseEventReceiver());
  GCodeParser::Config parser_cfg;
  GCodeParser::Config::ParamMap parameters;
  parser_cfg.parameters = &parameters;
  GCodeParser parser(parser_cfg, &stats_event_receiver, false);
  const bool success = parser.ParseStream(input_fd, msg_out) == 0
    && parser.error_count() == 0;
  delete machine_control;
  return success;
}
