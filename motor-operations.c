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

#include "motor-operations.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>

#include "motor-interface-constants.h"
#include "motion-queue.h"

// We need two loops per motor step (edge up, edge down),
// So we need to multiply step-counts by 2
// This could be more, if we wanted to implement sub-step resolution with
// more than one bit output per step (probably only with hand-built drivers).
#define LOOPS_PER_STEP (1 << 1)

// TODO: don't store this singleton like, but keep in user_data of the MotorOperations
static float hardware_frequency_limit_;

static float sq(float x) { return x * x; }  // square a number

// delay loops per second.
static double cycles_per_second() { return 100e6; } // two cycles per loop.

// Clip speed to maximum we can reach with hardware.
static float clip_hardware_frequency_limit(float v) {
  return v < hardware_frequency_limit_ ? v : hardware_frequency_limit_;
}

static double calcAccelerationCurveValueAt(int index, double acceleration) {
  // counter_freq * sqrt(2 / accleration)
  const double accel_factor = cycles_per_second()
    * (sqrt(LOOPS_PER_STEP * 2.0 / acceleration)) / LOOPS_PER_STEP;
  // The approximation is pretty far off in the first step; adjust.
  const double c0 = (index == 0) ? accel_factor * 0.67605 : accel_factor;
  return c0 * (sqrt(index + 1) - sqrt(index));
}

#if 0
// Is acceleration in acceptable range ?
static char test_acceleration_ok(float acceleration) {
  if (acceleration <= 0)
    return 1;  // <= 0: always full speed.

  // Check that the fixed point acceleration parameter (that we shift
  // DELAY_CYCLE_SHIFT) fits into 32 bit.
  // Also 2 additional bits headroom because we need to shift it by 2 in the
  // division.
  const double start_accel_cycle_value = (1 << (DELAY_CYCLE_SHIFT + 2))
    * calcAccelerationCurveValueAt(0, acceleration);
  if (start_accel_cycle_value > 0xFFFFFFFF) {
    fprintf(stderr, "Too slow acceleration to deal with. If really needed, "
	    "reduce value of #define DELAY_CYCLE_SHIFT\n");
    return 0;
  }
  return 1;
}
#endif

static int beagleg_enqueue_internal(struct MotionQueue *backend,
                                    const struct bg_movement *param,
				    int defining_axis_steps) {
  struct MotionSegment new_element;
  new_element.direction_bits = 0;

  // The defining_axis_steps is the number of steps of the axis that requires
  // the most number of steps. All the others are a fraction of the steps.
  //
  // The top bits have LOOPS_PER_STEP states (2 is the minium, as we need two
  // cycles for a 0 1 transition. So in that case we have 31 bit fraction
  // and 1 bit that overflows and toggles for the steps we want to generate.
  const uint64_t max_fraction = 0xFFFFFFFF / LOOPS_PER_STEP;
  for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
    if (param->steps[i] < 0) {
      new_element.direction_bits |= (1 << i);
    }
    const uint64_t delta = abs(param->steps[i]);
    new_element.fractions[i] = delta * max_fraction / defining_axis_steps;
  }

  // TODO: clamp acceleration to be a minimum value.
  const int total_loops = LOOPS_PER_STEP * defining_axis_steps;
  // There are three cases: either we accelerate, travel or decelerate.
  if (param->v0 == param->v1) {
    // Travel
    new_element.loops_accel = new_element.loops_decel = 0;
    new_element.loops_travel = total_loops;
    const float travel_speed = clip_hardware_frequency_limit(param->v0);
    new_element.travel_delay_cycles = cycles_per_second()
      / (LOOPS_PER_STEP * travel_speed);
  } else if (param->v0 < param->v1) {
    // acclereate
    new_element.loops_travel = new_element.loops_decel = 0;
    new_element.loops_accel = total_loops;

    // v1 = v0 + a*t -> t = (v1 - v0)/a
    // s = a/2 * t^2 + v0 * t; subsitution t from above.
    // a = (v1^2-v0^2)/(2*s)
    float acceleration = (sq(param->v1) - sq(param->v0)) / (2.0 * defining_axis_steps);
    // If we accelerated from zero to our first speed, this is how many steps
    // we needed. We need to go this index into our taylor series.
    const int accel_loops_from_zero = LOOPS_PER_STEP *
      (sq(param->v0 - 0) / (2.0 * acceleration));
 
    new_element.accel_series_index = accel_loops_from_zero;
    new_element.hires_accel_cycles = (1 << DELAY_CYCLE_SHIFT)
      * calcAccelerationCurveValueAt(new_element.accel_series_index, acceleration);
  } else {  // v0 > v1
    // decelerate
    new_element.loops_travel = new_element.loops_accel = 0;
    new_element.loops_decel = total_loops;

    float acceleration = (sq(param->v0) - sq(param->v1)) / (2.0 * defining_axis_steps);
    // We are into the taylor sequence this value up and reduce from there.
    const int accel_loops_from_zero = LOOPS_PER_STEP *
      (sq(param->v0 - 0) / (2.0 * acceleration));

    new_element.accel_series_index = accel_loops_from_zero;
    new_element.hires_accel_cycles = (1 << DELAY_CYCLE_SHIFT)
      * calcAccelerationCurveValueAt(new_element.accel_series_index, acceleration);
  }
  
  new_element.aux = param->aux_bits;
  new_element.state = STATE_FILLED;
  backend->enqueue(&new_element);

  return 0;
}

static int beagleg_enqueue(void *ctx, const struct bg_movement *param,
                           FILE *err_stream) {
  struct MotionQueue *backend = (struct MotionQueue*)ctx;
  
  // TODO: this function should automatically split this into multiple segments
  // each with the maximum number of steps.
  int biggest_value = abs(param->steps[0]);
  for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
    if (abs(param->steps[i]) > biggest_value) {
      biggest_value = abs(param->steps[i]);
    }
  }
  if (biggest_value == 0) {
    fprintf(err_stream ? err_stream : stderr, "zero steps. Ignoring command.\n");
    return 1;
  }
  if (biggest_value > 65535 / LOOPS_PER_STEP) {
    // TODO: for now, we limit the number. This should be implemented by
    // cutting this in multiple pieces, each calling beagleg_enqueue_internal()
    fprintf(err_stream ? err_stream : stderr,
	    "At most %d steps, got %d. Ignoring command.\n",
	    65535 / LOOPS_PER_STEP, biggest_value);
    return 2;
  }
  return beagleg_enqueue_internal(backend, param, biggest_value);
}

static void beagleg_motor_enable(void *ctx, char on) {
  struct MotionQueue *backend = (struct MotionQueue*)ctx;
  backend->wait_queue_empty();
  backend->motor_enable(on);
}

static void beagleg_wait_queue_empty(void *ctx) {
  struct MotionQueue *backend = (struct MotionQueue*)ctx;
  backend->wait_queue_empty();
}

int beagleg_init_motor_ops(struct MotionQueue *backend,
                           struct MotorOperations *ops) {
  hardware_frequency_limit_ = 1e6;    // Don't go over 1 Mhz
  
  // Set up operations.
  ops->user_data = backend;
  ops->motor_enable = beagleg_motor_enable;
  ops->enqueue = beagleg_enqueue;
  ops->wait_queue_empty = beagleg_wait_queue_empty;
  
  return 0;
}
