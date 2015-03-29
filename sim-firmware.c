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
set ytics nomirror  # Don't intervene with y2tics
set y2tics
# X axis shows up at 7ths place
plot "/tmp/foo.data" using 1:3 title "acceleration: steps/s^2" with lines axes x1y2 lw 2, '' using 1:2 title "speed: steps/s" with lines lw 2, '' using 1:7 title "steps" with lines lw 2;
 */
#include "sim-firmware.h"

#include <math.h>
#include <stdint.h>
#include <strings.h>
#include <stdio.h>

#include "motion-queue.h"
#include "motor-interface-constants.h"

#define LOOPS_PER_STEP (1 << 1)

/*
 * Due to the timer accuracy, velocity is quantized (sometimes, adjacent steps have the
 * exact velocity followed by a step in velocity)
 * Calculating the acceleration requires some smoothing moving average over
 * a couple of measurements.
 * The minimum is 2 to look at two adjacent values.
 */
#define AVERAGE_RINBGUFFER_SIZE 5
static double avg_ringbuffer[AVERAGE_RINBGUFFER_SIZE];
static double avg_dt_sum = 0;
static uint32_t avg_pos = 0;

static void avg_reset() {
  avg_dt_sum = 0;
  avg_pos = 0;
  bzero(&avg_ringbuffer, AVERAGE_RINBGUFFER_SIZE * sizeof(double));
}
static double avg_get_acceleration() {
  if (avg_pos < 2)
    return 0;
  // We go back as far as the ringbuffer reaches. In the beginning, that is not far.
  int back = AVERAGE_RINBGUFFER_SIZE - 1;
  if (back >= avg_pos) {
    back = avg_pos - 1;
  }
  double dt0 = avg_ringbuffer[(avg_pos + AVERAGE_RINBGUFFER_SIZE - back) % AVERAGE_RINBGUFFER_SIZE];
  double dt1 = avg_ringbuffer[avg_pos % AVERAGE_RINBGUFFER_SIZE];
  if (dt0 <= 0 || dt1 <= 0)
    return 0;
  
  double v0 = (1 / dt0) / LOOPS_PER_STEP;
  double v1 = (1 / dt1) / LOOPS_PER_STEP;
  return (v1 - v0) / (avg_dt_sum - dt1);
}
static void avg_push_delta_time(double t) {
  int next_pos = (avg_pos + 1) % AVERAGE_RINBGUFFER_SIZE;
  avg_dt_sum -= avg_ringbuffer[next_pos];
  avg_ringbuffer[next_pos] = t;
  avg_dt_sum += t;
  avg_pos++;
}

struct HardwareState {
  // Internal state
  uint32_t m[MOTION_MOTOR_COUNT];
};

static double sim_time;
static int sim_steps[MOTION_MOTOR_COUNT];  // we are only looking at the defining axis steps.

static struct HardwareState state;

// This simulates what happens in the PRU. For testing purposes.
static void sim_enqueue(struct MotionSegment *segment) {
  if (segment->state == STATE_EXIT)
    return;
  // setting output direction according to segment->direction_bits;
  
  bzero(&state, sizeof(state));

#if JERK_EXPERIMENT
  uint32_t jerk_index = 1;
#endif
  char is_first = 1;
  uint32_t remainder = 0;
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
    
    // Higher resolution delay if we had fractional counts. Used to better calculate acceleration
    // for display purposes.
    double hires_delay = 0;

#if JERK_EXPERIMENT
    if (segment->jerk_start > 0) {
      if (is_first) {
        fprintf(stderr, "jerk start: jerk-timer-cycles=%.3f\n",
                segment->jerk_motion);
        is_first = 0;
      }
      // TODO: index 0 ?
      // TODO: is it 2* ?
      segment->jerk_motion -= 2*segment->jerk_motion / ((3 * jerk_index) + 1);
      --segment->jerk_start;
      ++jerk_index;
      delay_loops = segment->jerk_motion;
      hires_delay = segment->jerk_motion;
      if (hires_delay < 0) {
        fprintf(stderr, "Got less than 0 at index %d\n", jerk_index);
        segment->jerk_start = 0;
        hires_delay = delay_loops = 30000;
      }
      if (segment->jerk_start == 0) {
        fprintf(stderr, "jerk end  : jerk-timer-cycles=%.3f\n",
                segment->jerk_motion);
        is_first = 1;
      }
    }
    else
#endif
      if (segment->loops_accel > 0) {
      if (is_first) {
        fprintf(stderr, "Accel start: accel-series-idx=%5u, accel-timer-cycles=%.3f\n",
                segment->accel_series_index,
                1.0 * segment->hires_accel_cycles / (1<<DELAY_CYCLE_SHIFT));
        is_first = 0;
      }
      if (segment->accel_series_index != 0) {
        const uint32_t divident = (segment->hires_accel_cycles << 1) + remainder;
        const uint32_t divisor = (segment->accel_series_index << 2) + 1;
        segment->hires_accel_cycles -= (divident / divisor);
        remainder = divident % divisor;
      }
      ++segment->accel_series_index;
      --segment->loops_accel;
      delay_loops = segment->hires_accel_cycles >> DELAY_CYCLE_SHIFT;
      hires_delay = 1.0 * segment->hires_accel_cycles / (1<<DELAY_CYCLE_SHIFT);
      if (segment->loops_accel == 0) {
        fprintf(stderr, "Accel end  : accel-series-idx=%5u, accel-timer-cycles=%.3f\n",
                segment->accel_series_index,
                1.0 * segment->hires_accel_cycles / (1<<DELAY_CYCLE_SHIFT));
      }
    }
    else if (segment->loops_travel > 0) {
      --segment->loops_travel;
      delay_loops = segment->travel_delay_cycles;
      hires_delay = segment->travel_delay_cycles;
      if (is_first) {
        fprintf(stderr, "travel. timer-cycles=%u\n", delay_loops);
        is_first = 0;
      }
    }
    else if (segment->loops_decel > 0) {
      if (is_first) {
        fprintf(stderr, "Decel start: accel-series-idx=%5u, decel-timer-cycles=%.3f\n",
                segment->accel_series_index,
                1.0 * segment->hires_accel_cycles / (1<<DELAY_CYCLE_SHIFT));
        is_first = 0;
      }
      const uint32_t divident = (segment->hires_accel_cycles << 1) + remainder;
      const uint32_t divisor = (segment->accel_series_index << 2) - 1;
      segment->hires_accel_cycles += (divident / divisor);
      remainder = divident % divisor;
      --segment->accel_series_index;
      --segment->loops_decel;
      delay_loops = segment->hires_accel_cycles >> DELAY_CYCLE_SHIFT;
      hires_delay = 1.0 * segment->hires_accel_cycles / (1<<DELAY_CYCLE_SHIFT);
      if (segment->loops_decel == 0) {
        fprintf(stderr, "Decel end  : accel-series-idx=%5u, decel-timer-cycles=%.3f\n",
                segment->accel_series_index,
                1.0 * segment->hires_accel_cycles / (1<<DELAY_CYCLE_SHIFT));
      }
    }
    else {
      break;  // done.
    }
    double wait_time = 1.0 * delay_loops / TIMER_FREQUENCY;
    avg_push_delta_time(1.0 * hires_delay / TIMER_FREQUENCY);
    double acceleration = avg_get_acceleration();
    sim_time += wait_time;
    double velocity = (1 / wait_time) / LOOPS_PER_STEP;  // in Hz.
    // Total time; speed; acceleration; delay_loops. [steps walked for all motors].
    printf("%12.6f %12.4f %12.4f %10d      ", sim_time, velocity, acceleration, delay_loops);
    for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
      printf("%3d ", sim_steps[i]);
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
  avg_reset();
  queue->enqueue = &sim_enqueue;
  queue->wait_queue_empty = &sim_wait_queue_empty;
  queue->motor_enable = &sim_motor_enable;
  queue->shutdown = &sim_shutdown;

  // Total time; speed; acceleration; delay_loops. [steps walked for all motors].
  printf("#%11s %12s %12s %10s      ax0 ax1 ax2 ax3 ax4 ax5 ax6 ax7\n",
         "time", "speed", "accel", "timercnt");
}
