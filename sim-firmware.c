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

/*
Gnuplot, showing speed and steps on the left axis, acceleration on
the right axis.

set grid       # easier to follow
set xtics 0.5
set yrange  [0:3000]  # Accleration is on different scale.
set ytics 1000
set y2range [-1500:1500]  # Accleration is on different scale.
set y2tics 1000
set ytics nomirror  # Don't intervene with y2tics
plot "/tmp/foo.x" using 1:2 title "speed" with lines, '' using 1:3 title "acceleration" with lines axes x1y2, '' using 1:4 title "steps" with lines

 */
#include "sim-firmware.h"

#include <stdint.h>
#include <strings.h>
#include <stdio.h>

#include "motion-queue.h"
#include "motor-interface-constants.h"

#define LOOPS_PER_STEP (1 << 1)

struct HardwareState {
  // Internal state
  uint32_t m[MOTION_MOTOR_COUNT];
};

static double sim_time;
static int sim_steps[MOTION_MOTOR_COUNT];  // we are only looking at the defining axis steps.
static double previous_velocity = 0;

static struct HardwareState state;

// This simulates what happens in the PRU. For testing purposes.
static void sim_enqueue(struct MotionSegment *segment) {
  if (segment->state == STATE_EXIT)
    return;
  // setting output direction according to segment->direction_bits;
  
  bzero(&state, sizeof(state));

  char is_first = 1;
  for (;;) {
    for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
      int before = (state.m[i] & 0x80000000) != 0;
      state.m[i] += segment->fractions[i];
      // Top bit is our step bit. Collect all of these and output to hardware.
      int after = (state.m[i] & 0x80000000) != 0;
      if (!before && after) {  // transition 0->1
        sim_steps[i] += ((1 << i) & segment->direction_bits) ? -1 : 1;
      }
    }

    sim_time += 160e-9;  // Updating the motor takes this time.
    
    uint32_t delay_loops = 0;
    uint32_t remainder = 0;
    if (segment->loops_accel > 0) {
      if (is_first) {
        fprintf(stderr, "Accel start: accel-series-idx=%5u, accel-cycles=%u\n",
                segment->accel_series_index, segment->hires_accel_cycles);
        is_first = 0;
      }
      if (segment->accel_series_index != 0) {
        const uint32_t divident = (segment->hires_accel_cycles << 1) + remainder;
        const uint32_t divisor = (segment->accel_series_index << 2) + 1;
        segment->hires_accel_cycles -= divident / divisor;
        remainder = divident % divisor;
      }
      ++segment->accel_series_index;
      --segment->loops_accel;
      delay_loops = segment->hires_accel_cycles >> DELAY_CYCLE_SHIFT;
      if (segment->loops_accel == 0) {
        fprintf(stderr, "Accel end  : accel-series-idx=%5u, accel-cycles=%u\n",
                segment->accel_series_index, segment->hires_accel_cycles);
      }
    }
    else if (segment->loops_travel > 0) {
      --segment->loops_travel;
      delay_loops = segment->travel_delay_cycles;
    }
    else if (segment->loops_decel > 0) {
      if (is_first) {
        fprintf(stderr, "Decel start: accel-series-idx=%5u, accel-cycles=%u\n",
                segment->accel_series_index, segment->hires_accel_cycles);
        is_first = 0;
      }
      const uint32_t divident = (segment->hires_accel_cycles << 1) + remainder;
      const uint32_t divisor = (segment->accel_series_index << 2) - 1;
      segment->hires_accel_cycles += divident / divisor;
      remainder = divident % divisor;
      --segment->accel_series_index;
      --segment->loops_decel;
      delay_loops = segment->hires_accel_cycles >> DELAY_CYCLE_SHIFT;
      if (segment->loops_decel == 0) {
        fprintf(stderr, "Accel end  : accel-series-idx=%5u, accel-cycles=%u\n",
                segment->accel_series_index, segment->hires_accel_cycles);
      }
    }
    else {
      break;  // done.
    }
    double wait_time = 1.0 * delay_loops / TIMER_FREQUENCY;
    double velocity = (1 / wait_time) / LOOPS_PER_STEP;  // in Hz.
    double acceleration = (velocity - previous_velocity) / wait_time;
    previous_velocity = velocity;
    sim_time += wait_time;
    // Total time; frequency=speed; steps walked for all motors.
    printf("%f %f %f ", sim_time, velocity, acceleration);
    for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
      printf("%d ", sim_steps[i]);
    }
    printf("\n");
  }
}

static void sim_wait_queue_empty() {}
static void sim_motor_enable(char on) {}
static void sim_shutdown(char do_flush) {}
void init_sim_motion_queue(struct MotionQueue *queue) {
  bzero(&state, sizeof(state));
  sim_time = 0;
  bzero(&sim_steps, sizeof(&sim_steps));
  queue->enqueue = &sim_enqueue;
  queue->wait_queue_empty = &sim_wait_queue_empty;
  queue->motor_enable = &sim_motor_enable;
  queue->shutdown = &sim_shutdown;
}
