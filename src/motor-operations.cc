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

#include "motor-operations.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <deque>

#include "common/logging.h"

#include "motor-interface-constants.h"
#include "motion-queue.h"
#include "hardware-mapping.h"

// We need two loops per motor step (edge up, edge down),
// So we need to multiply step-counts by 2
// This could be more, if we wanted to implement sub-step resolution with
// more than one bit output per step (probably only with hand-built drivers).
#define LOOPS_PER_STEP (1 << 1)

// If we do more than these number of steps, the fixed point fraction
// accumulate too much error.
#define MAX_STEPS_PER_SEGMENT (65535 / LOOPS_PER_STEP)

// TODO: don't store this singleton like, but keep in user_data of the MotorOperations
static float hardware_frequency_limit_ = 1e6;    // Don't go over 1 Mhz

static inline float sq(float x) { return x * x; }  // square a number
static inline double sqd(double x) { return x * x; }  // square a number
static inline int round2int(float x) { return (int) roundf(x); }

// Clip speed to maximum we can reach with hardware.
static float clip_hardware_frequency_limit(float v) {
  return v < hardware_frequency_limit_ ? v : hardware_frequency_limit_;
}

static float calcAccelerationCurveValueAt(int index, float acceleration) {
  // counter_freq * sqrt(2 / accleration)
  const float accel_factor = TIMER_FREQUENCY
    * (sqrtf(LOOPS_PER_STEP * 2.0f / acceleration)) / LOOPS_PER_STEP;
  // The approximation is pretty far off in the first step; adjust.
  const float c0 = (index == 0) ? accel_factor * 0.67605f : accel_factor;
  return c0 * (sqrtf(index + 1) - sqrtf(index));
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
  const float start_accel_cycle_value = (1 << (DELAY_CYCLE_SHIFT + 2))
    * calcAccelerationCurveValueAt(0, acceleration);
  if (start_accel_cycle_value > 0xFFFFFFFF) {
    Log_error("Too slow acceleration to deal with. If really needed, "
              "reduce value of #define DELAY_CYCLE_SHIFT\n");
    return 0;
  }
  return 1;
}
#endif

// Used to keep track of useful attributes of a motion segment's target move.
struct HistoryPositionInfo {
  HistoryPositionInfo () : position_steps(0), sign(1) {}
  int position_steps; // Absolute position at the end of the move.
  uint32_t fraction;  // 1/0xffffffff-th of the difference between the previous
                      // step and now.
  signed char sign;   // Sense of the move.
};

// Store the required informations needed to backtrack the absolute position.
struct MotionQueueMotorOperations::HistorySegment {
  HistoryPositionInfo pos_info[MOTION_MOTOR_COUNT];
  unsigned short aux_bits;
};

MotionQueueMotorOperations::
MotionQueueMotorOperations(HardwareMapping *hw, MotionQueue *backend)
  : hardware_mapping_(hw),
    backend_(backend),
    shadow_queue_(new std::deque<struct HistorySegment>()) {
  // Initialize the history queue.
  shadow_queue_->push_front({});
}

MotionQueueMotorOperations::~MotionQueueMotorOperations() {
  delete shadow_queue_;
}

void MotionQueueMotorOperations::EnqueueInternal(const LinearSegmentSteps &param,
                                                 int defining_axis_steps) {
  struct MotionSegment new_element = {};
  new_element.direction_bits = 0;

  // The new segment is based on the previous position.
  struct HistorySegment history_segment = shadow_queue_->front();

  // The defining_axis_steps is the number of steps of the axis that requires
  // the most number of steps. All the others are a fraction of the steps.
  //
  // The top bits have LOOPS_PER_STEP states (2 is the minium, as we need two
  // cycles for a 0 1 transition. So in that case we have 31 bit fraction
  // and 1 bit that overflows and toggles for the steps we want to generate.
  const uint64_t max_fraction = 0xFFFFFFFF / LOOPS_PER_STEP;
  for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
    bool flip = hardware_mapping_->IsMotorFlipped(i);
    if (param.steps[i] < 0) {
      if (!flip) new_element.direction_bits |= (1 << i);
      history_segment.pos_info[i].sign = -1;
    } else {
      if (flip) new_element.direction_bits |= (1 << i);
      history_segment.pos_info[i].sign = 1;
    }
    history_segment.pos_info[i].position_steps += param.steps[i];
    const uint64_t delta = abs(param.steps[i]);
    new_element.fractions[i] = delta * max_fraction / defining_axis_steps;
    history_segment.pos_info[i].fraction = new_element.fractions[i];
  }

  history_segment.aux_bits = param.aux_bits;
  shadow_queue_->push_front(history_segment);

  // TODO: clamp acceleration to be a minimum value.
  const int total_loops = LOOPS_PER_STEP * defining_axis_steps;
  // There are three cases: either we accelerate, travel or decelerate.
  if (param.v0 == param.v1) {
    // Travel
    new_element.loops_accel = new_element.loops_decel = 0;
    new_element.loops_travel = total_loops;
    const float travel_speed = clip_hardware_frequency_limit(param.v0);
    new_element.travel_delay_cycles = round2int(TIMER_FREQUENCY / (LOOPS_PER_STEP * travel_speed));
  } else if (param.v0 < param.v1) {
    // acclereate
    new_element.loops_travel = new_element.loops_decel = new_element.travel_delay_cycles = 0;
    new_element.loops_accel = total_loops;

    // v1 = v0 + a*t -> t = (v1 - v0)/a
    // s = a/2 * t^2 + v0 * t; subsitution t from above.
    // a = (v1^2-v0^2)/(2*s)
    float acceleration = (sq(param.v1) - sq(param.v0)) / (2.0f * defining_axis_steps);
    //fprintf(stderr, "M-OP HZ: defining=%d ; accel=%.2f\n", defining_axis_steps, acceleration);
    // If we accelerated from zero to our first speed, this is how many steps
    // we needed. We need to go this index into our taylor series.
    const int accel_loops_from_zero =
      round2int(LOOPS_PER_STEP * (sq(param.v0 - 0) / (2.0f * acceleration)));

    new_element.accel_series_index = accel_loops_from_zero;
    new_element.hires_accel_cycles =
      round2int((1 << DELAY_CYCLE_SHIFT) * calcAccelerationCurveValueAt(new_element.accel_series_index, acceleration));
  } else {  // v0 > v1
    // decelerate
    new_element.loops_travel = new_element.loops_accel = new_element.travel_delay_cycles = 0;
    new_element.loops_decel = total_loops;

    float acceleration = (sq(param.v0) - sq(param.v1)) / (2.0f * defining_axis_steps);
    //fprintf(stderr, "M-OP HZ: defining=%d ; decel=%.2f\n", defining_axis_steps, acceleration);
    // We are into the taylor sequence this value up and reduce from there.
    const int accel_loops_from_zero =
      round2int(LOOPS_PER_STEP * (sq(param.v0 - 0) / (2.0f * acceleration)));

    new_element.accel_series_index = accel_loops_from_zero;
    new_element.hires_accel_cycles =
      round2int((1 << DELAY_CYCLE_SHIFT) * calcAccelerationCurveValueAt(new_element.accel_series_index, acceleration));
  }

  new_element.aux = param.aux_bits;
  new_element.state = STATE_FILLED;
  backend_->MotorEnable(true);
  backend_->Enqueue(&new_element);
}

bool MotionQueueMotorOperations::GetPhysicalStatus(PhysicalStatus *status) {
  // Shrink the queue
  uint32_t loops;
  const int buffer_size = backend_->GetPendingElements(&loops);
  const int new_size = buffer_size > 0 ? buffer_size : 1;
  shadow_queue_->resize(new_size);

  // Get the last element
  const HistorySegment &hs = shadow_queue_->back();
  const uint64_t max_fraction = 0xFFFFFFFF / LOOPS_PER_STEP;

  // NOTE: Assuming MOTION_MOTOR_COUNT == BEAGLEG_NUM_MOTORS
  uint64_t steps;
  for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
    const HistoryPositionInfo pos_info = hs.pos_info[i];
    // Let's round-up the division, 5.1 is still a toggle, so it's 6 loops.
    steps = loops / LOOPS_PER_STEP;
    steps *= pos_info.fraction;
    steps += max_fraction - 1;
    steps /= max_fraction;
    status->pos_steps[i] = pos_info.position_steps - pos_info.sign * (int) steps;
  }
  status->aux_bits = hs.aux_bits;
  return true;
}

void MotionQueueMotorOperations::SetExternalPosition(int axis, int steps) {
  struct HistorySegment history_segment = shadow_queue_->front();
  if (steps < 0) {
    history_segment.pos_info[axis].sign = -1;
    history_segment.pos_info[axis].position_steps = -steps;
  } else {
    history_segment.pos_info[axis].sign = 1;
    history_segment.pos_info[axis].position_steps = steps;
  }
  shadow_queue_->push_front(history_segment);
  // Shrink the queue and remove the elements that we are not interested
  // in anymore.
  // TODO: We need to find a way to get the maximum number of elements
  // of the shadow queue (ie backend_->GetQueueStats()?)
  const int buffer_size = backend_->GetPendingElements(NULL);
  const int new_size = buffer_size > 0 ? buffer_size : 1;
  shadow_queue_->resize(new_size);
}

static int get_defining_axis_steps(const LinearSegmentSteps &param) {
  int defining_axis_steps = abs(param.steps[0]);
  for (int i = 1; i < BEAGLEG_NUM_MOTORS; ++i) {
    if (abs(param.steps[i]) > defining_axis_steps) {
      defining_axis_steps = abs(param.steps[i]);
    }
  }
  return defining_axis_steps;
}

void MotionQueueMotorOperations::Enqueue(const LinearSegmentSteps &param) {
  const int defining_axis_steps = get_defining_axis_steps(param);

  if (defining_axis_steps == 0) {
    // The new segment is based on the previous position.
    struct HistorySegment history_segment = shadow_queue_->front();

    // No move, but we still have to set the bits.
    struct MotionSegment empty_element = {};
    empty_element.aux = param.aux_bits;
    empty_element.state = STATE_FILLED;

    history_segment.aux_bits = param.aux_bits;
    shadow_queue_->push_front(history_segment);

    backend_->Enqueue(&empty_element);
  }
  else if (defining_axis_steps > MAX_STEPS_PER_SEGMENT) {
    // We have more steps that we can enqueue in one chunk, so let's cut
    // it in pieces.
    const double a = (sqd(param.v1) - sqd(param.v0))/(2.0*defining_axis_steps);
    const int divisions = (defining_axis_steps / MAX_STEPS_PER_SEGMENT) + 1;
    int64_t hires_steps_per_div[BEAGLEG_NUM_MOTORS];
    for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
      // (+1 to fix rounding trouble in the LSB)
      hires_steps_per_div[i] = ((int64_t)param.steps[i] << 32)/divisions + 1;
    }

    struct LinearSegmentSteps previous = {}, accumulator = {}, output;
    int64_t hires_step_accumulator[BEAGLEG_NUM_MOTORS] = {0};
    double previous_speed = param.v0;   // speed calculation in double

    output.aux_bits = param.aux_bits;  // use the original Aux bits for all segments
    for (int d = 0; d < divisions; ++d) {
      for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
        hires_step_accumulator[i] += hires_steps_per_div[i];
        accumulator.steps[i] = hires_step_accumulator[i] >> 32;
        output.steps[i] = accumulator.steps[i] - previous.steps[i];
      }
      const int division_steps = get_defining_axis_steps(output);
      // These squared values can get huge, lets not loose precision
      // here and do calculations in double (otherwise our results can
      // be a little bit off and fail to reach zero properly).
      const double v0squared = sqd(previous_speed);
      // v1 = v0 + a*t; t = (sqrt(v0^2 + 2 * a * steps) - v0) / a
      // -> v1 = sqrt(v0^ + 2 * a * steps)
      const double v1squared = v0squared + 2.0 * a * division_steps;
      // Rounding errors can make v1squared slightly negative...
      const double v1 = v1squared > 0.0 ? sqrt(v1squared) : 0;
      output.v0 = previous_speed;
      output.v1 = v1;
      EnqueueInternal(output, division_steps);
      previous = accumulator;
      previous_speed = v1;
    }
  } else {
    EnqueueInternal(param, defining_axis_steps);
  }
  // Shrink the queue and remove the elements that we are not interested
  // in anymore.
  // TODO: We need to find a way to get the maximum number of elements
  // of the shadow queue (ie backend_->GetQueueStats()?)
  const int buffer_size = backend_->GetPendingElements(NULL);
  const int new_size = buffer_size > 0 ? buffer_size : 1;
  shadow_queue_->resize(new_size);
}

void MotionQueueMotorOperations::MotorEnable(bool on) {
  backend_->WaitQueueEmpty();
  backend_->MotorEnable(on);
}

void MotionQueueMotorOperations::WaitQueueEmpty() {
  backend_->WaitQueueEmpty();
}
