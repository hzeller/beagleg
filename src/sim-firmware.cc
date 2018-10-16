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

/*
Gnuplot X11 output does not do dashes by default, so this
should be in the ~/.Xdefaults

! gnuplot settings
gnuplot*dashed: on
gnuplot*borderDashes:   0
gnuplot*axisDashes:    16
gnuplot*line1Dashes:    0
gnuplot*line2Dashes:   42
gnuplot*line3Dashes:   13
gnuplot*line4Dashes:   44
gnuplot*line5Dashes:   15
gnuplot*line6Dashes: 4441
gnuplot*line7Dashes:   42
gnuplot*line8Dashes:   13
*/

/*
Gnuplot, showing speed and steps on the left axis, acceleration on
the right axis. Config: motor mapping motor_1: axis:x, motor_2:axis:y, motor_3:axis:z

set grid       # easier to follow
set ytics nomirror  # Don't intervene with y2tics
set y2tics

set ylabel "steps & steps/s (velocity)"
set y2label "steps/s^2 (acceleration)"

# euclid axis
set style line 1 linetype 1 linecolor rgb "black" lw 2  # velocity
set style line 2 linetype 2 linecolor rgb "black" lw 1  # accel

# first axis steps/speed/velocity (X)
set style line 10 linetype 5 linecolor rgb "red" lw 1  # steps
set style line 11 linetype 1 linecolor rgb "red" lw 2  # velocity
set style line 12 linetype 2 linecolor rgb "red" lw 1  # accel
# Y
set style line 20 linetype 5 linecolor rgb "blue" lw 1
set style line 21 linetype 1 linecolor rgb "blue" lw 2
set style line 22 linetype 2 linecolor rgb "blue" lw 1
# Z
set style line 30 linetype 5 linecolor rgb "green" lw 1
set style line 31 linetype 1 linecolor rgb "green" lw 2
set style line 32 linetype 2 linecolor rgb "green" lw 1

# Plotting X,Y axis
plot "/tmp/foo.data" \
        using 1:5 title "steps X" with lines ls 10, \
        '' using 1:6 title "velocity X" with lines ls 11,\
        '' using 1:7 title "accel X" axes x1y2 with lines ls 12, \
        '' using 1:8 title "steps Y" with lines ls 20, \
        '' using 1:9 title "velocity Y" with lines ls 21,\
        '' using 1:10 title "accel Y" axes x1y2 with lines ls 22

#.. as one-liner
plot "/tmp/foo.data" using 1:5 title "steps X" with lines ls 10, '' using 1:6 title "velocity X" with lines ls 11, '' using 1:7 title "accel X" axes x1y2 with lines ls 12, '' using 1:8 title "steps Y" with lines ls 20, '' using 1:9 title "velocity Y" with lines ls 21, '' using 1:10 title "accel Y" axes x1y2 with lines ls 22

# Euclid space velocity and acceleration.
plot "/tmp/foo.data" using 1:3 title "velocity Euclid" with lines ls 1, '' using 1:4 title "accel Euclid" axes x1y2 with lines ls 2

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

#define AVERAGE_RINBGUFFER_SIZE 10
class SimFirmwareQueue::Averager {
public:
  Averager() : dt_sum_(0), pos_(0) {
    bzero(&ringbuffer_, AVERAGE_RINBGUFFER_SIZE * sizeof(double));
  }

  double GetAcceleration() {
    if (pos_ < 2)
      return 0;
    // We go back as far as the ringbuffer reaches. In the beginning, that is not far.
    int back = AVERAGE_RINBGUFFER_SIZE - 1;
    if (back >= pos_) {
      back = pos_ - 1;
    }
    double dt0 = ringbuffer_[(pos_ + AVERAGE_RINBGUFFER_SIZE - back) % AVERAGE_RINBGUFFER_SIZE];
    double dt1 = ringbuffer_[pos_ % AVERAGE_RINBGUFFER_SIZE];
    if (dt0 <= 0 || dt1 <= 0)
      return 0;

    double v0 = (1 / dt0) / LOOPS_PER_STEP;
    double v1 = (1 / dt1) / LOOPS_PER_STEP;
    return (v1 - v0) / (dt_sum_ - dt1);
}

  void PushDeltaTime(double t) {
    const int next_pos = (pos_ + 1) % AVERAGE_RINBGUFFER_SIZE;
    dt_sum_ -= ringbuffer_[next_pos];
    ringbuffer_[next_pos] = t;
    dt_sum_ += t;
    pos_++;
  }

private:
  double ringbuffer_[AVERAGE_RINBGUFFER_SIZE];
  double dt_sum_;
  int pos_;
};


struct HardwareState {
  // Internal state
  uint32_t m[MOTION_MOTOR_COUNT];
};

static double sim_time;
static int sim_steps[MOTION_MOTOR_COUNT];  // we are only looking at the defining axis steps.

static struct HardwareState state;

// Default mapping of our motors to axis in typical test-setups.
// Should match Motor-Mapping in config file.
enum {
  X_MOTOR = 0,
  Y_MOTOR = 1,
  Z_MOTOR = 2
};

static double euclid(double x, double y, double z) {
  return sqrt(x*x + y*y + z*z);
}

// This simulates what happens in the PRU. For testing purposes.
bool SimFirmwareQueue::Enqueue(MotionSegment *segment) {
  if (segment->state == STATE_EXIT)
    return true;
  // setting output direction according to segment->direction_bits;

  bzero(&state, sizeof(state));

  // For convenience, this is the relative speed of each motor.
  double motor_speeds[MOTION_MOTOR_COUNT];
  const double div = 1.0 * 2147483647u;   // Simulates fixed point div
  for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
    motor_speeds[i] = segment->fractions[i] / div;
  }
  const double euklid_factor = euclid(motor_speeds[X_MOTOR],
                                      motor_speeds[Y_MOTOR],
                                      motor_speeds[Z_MOTOR]);

#if JERK_EXPERIMENT
  uint32_t jerk_index = 1;
#endif
  bool is_first = true;
  uint32_t remainder = 0;
  const char *msg = "";

  for (;;) {
    // Increment by motor fraction.
    for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
      int before = (state.m[i] & 0x80000000) != 0;
      state.m[i] += segment->fractions[i];
      // Top bit is our step bit. Collect all of these and output to hardware.
      int after = (state.m[i] & 0x80000000) != 0;
      if (!before && after) {  // transition 0->1
        sim_steps[i] += ((1 << i) & segment->direction_bits) ? -1 : 1;
      }
    }

    msg = "";
    sim_time += 160e-9;  // Updating the motor takes this time.

    uint32_t delay_loops = 0;

    // Higher resolution delay if we had fractional counts. Used to better calculate acceleration
    // for display purposes.
    double hires_delay = 0;

#if JERK_EXPERIMENT
    if (segment->jerk_start > 0) {
      if (is_first) {
        msg = "# jerk";
        fprintf(stderr, "jerk start: jerk-timer-cycles=%.3f\n",
                segment->jerk_motion);
        is_first = false;
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
        is_first = true;
      }
    }
    else
#endif
      if (segment->loops_accel > 0) {
      if (is_first) {
        msg = "# accel.";
        fprintf(stderr, "SIM: Accel start _/ : accel-series-idx=%5u, "
                "accel-timer-cycles=%10.3f (%d loops)\n",
                segment->accel_series_index,
                1.0 * segment->hires_accel_cycles / (1<<DELAY_CYCLE_SHIFT),
                segment->loops_accel);
        is_first = false;
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
        fprintf(stderr, "SIM: Accel end   /‾ : accel-series-idx=%5u, accel-timer-cycles=%10.3f\n",
                segment->accel_series_index,
                1.0 * segment->hires_accel_cycles / (1<<DELAY_CYCLE_SHIFT));
      }
    }
    else if (segment->loops_travel > 0) {
      delay_loops = segment->travel_delay_cycles;
      hires_delay = segment->travel_delay_cycles;
      if (is_first) {
        msg = "# travel.";
        fprintf(stderr, "SIM: travel      -- :                               timer-cycles=%6u     (%d loops)\n", delay_loops,
                segment->loops_travel);
        is_first = false;
      }
      --segment->loops_travel;
    }
    else if (segment->loops_decel > 0) {
      if (is_first) {
        msg = "# decel.";
        fprintf(stderr, "SIM: Decel start ‾\\ : accel-series-idx=%5u, "
                "decel-timer-cycles=%10.3f (%d loops)\n",
                segment->accel_series_index,
                1.0 * segment->hires_accel_cycles / (1<<DELAY_CYCLE_SHIFT),
                segment->loops_decel);
        is_first = false;
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
        fprintf(stderr, "SIM: Decel end   \\_ : accel-series-idx=%5u, decel-timer-cycles=%10.3f\n",
                segment->accel_series_index,
                1.0 * segment->hires_accel_cycles / (1<<DELAY_CYCLE_SHIFT));
      }
    }
    else {
      break;  // done.
    }
    double wait_time = 1.0 * delay_loops / TIMER_FREQUENCY;
    averager_->PushDeltaTime(1.0 * hires_delay / TIMER_FREQUENCY);
    double acceleration = averager_->GetAcceleration();
    sim_time += wait_time;
    double velocity = (1 / wait_time) / LOOPS_PER_STEP;  // in Hz.

    // Total time; speed; acceleration; delay_loops. [steps walked for all motors].
    fprintf(out_, "%12.8f %10d %12.4f %12.4f      ",
            sim_time, delay_loops,
            euklid_factor * velocity,
            euklid_factor * acceleration);
    for (int i = 0; i < relevant_motors_; ++i) {
      fprintf(out_, "%5d %10.4f %12.4f ", sim_steps[i],
              motor_speeds[i] * velocity,
              motor_speeds[i] * acceleration);
    }
    fprintf(out_, "%s\n", msg);
  }
  return true;
}

SimFirmwareQueue::SimFirmwareQueue(FILE *out, int relevant_motors)
  : out_(out),
    relevant_motors_(relevant_motors < MOTION_MOTOR_COUNT
                     ? relevant_motors
                     : MOTION_MOTOR_COUNT),
    averager_(new Averager()) {
  // Total time; speed; acceleration; delay_loops. [steps walked for all motors].
  printf("%12s %10s %12s %12s      ", "time", "timer-loop", "Euclid-speed", "Euclid-accel");
  for (int i = 0; i < relevant_motors_; ++i) {
    fprintf(out_, "%4s%d %9s%d %11s%d ",
            "s", i,
            "v", i,
            "a", i);
  }
  fprintf(out_, "\n");
}

SimFirmwareQueue::~SimFirmwareQueue() {
  delete averager_;
}
