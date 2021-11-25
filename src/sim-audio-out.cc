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

#include "sim-audio-out.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <strings.h>

#include "motion-queue.h"
#include "motor-interface-constants.h"

#define LOOPS_PER_STEP (1 << 1)

static constexpr unsigned char kWavHeader[] = {
  0x52, 0x49, 0x46, 0x46, 0xff, 0xff, 0xff, 0xff, 0x57, 0x41, 0x56,
  0x45, 0x66, 0x6D, 0x74, 0x20, 0x10, 0x00, 0x00, 0x00, 0x01, 0x00,
  0x02, 0x00, 0x44, 0xAC, 0x00, 0x00, 0x10, 0xB1, 0x02, 0x00, 0x04,
  0x00, 0x10, 0x00, 0x64, 0x61, 0x74, 0x61, 0xff, 0xff, 0xff, 0xff,
};

class SimFirmwareAudioQueue::AudioWriter {
 public:
  AudioWriter(FILE *out) : out_(out) {
    fwrite(kWavHeader, sizeof(kWavHeader), 1, out_);
  }

  ~AudioWriter() {
    fflush(out_);
    fseek(out_, 40, SEEK_SET);
    uint32_t data_size = 4 * current_sample_;
    fwrite(&data_size, sizeof(data_size), 1, out_);

    data_size += 36;
    fseek(out_, 4, SEEK_SET);
    fwrite(&data_size, sizeof(data_size), 1, out_);

    fclose(out_);
    fprintf(stderr, "WAV: wrote %u samples (%.2fs)\n", current_sample_,
            current_sample_ / 44100.0);
  }

  void Update(uint8_t channel_values, double until_time) {
    struct sample_t {
      int16_t left;
      int16_t right;
    } sample = {(int16_t)((channel_values & 0b01) ? 16000 : -16000),
                (int16_t)((channel_values & 0b10) ? 16000 : -16000)};
    const uint32_t target_sample = until_time * kSampleRate;
    for (/**/; current_sample_ < target_sample; ++current_sample_) {
      fwrite(&sample, sizeof(sample), 1, out_);
    }
  }

 private:
  static constexpr int kSampleRate = 44100;
  uint32_t current_sample_ = 0;
  FILE *const out_;
};

struct HardwareState {
  // Internal state
  uint32_t m[MOTION_MOTOR_COUNT];
};

static double sim_time = 0;
static struct HardwareState state;

// Default mapping of our motors to axis in typical test-setups.
// Should match Motor-Mapping in config file.
enum { X_MOTOR = 0, Y_MOTOR = 1, Z_MOTOR = 2 };

// This simulates what happens in the PRU. For testing purposes.
bool SimFirmwareAudioQueue::Enqueue(MotionSegment *segment) {
  if (segment->state == STATE_EXIT) return true;

  // For each segment, we start with a fresh motor state.
  bzero(&state, sizeof(state));

  uint32_t remainder = 0;

  for (;;) {
    uint8_t channel_value = 0x00;
    // Increment by motor fraction.
    for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
      state.m[i] += segment->fractions[i];
      channel_value |= ((state.m[i] & 0x80000000) != 0) << i;
    }

    sim_time += 160e-9;  // Updating the motor takes this time.

    uint32_t delay_loops = 0;

    if (segment->loops_accel > 0) {
      if (segment->accel_series_index != 0) {
        const uint32_t divident =
          (segment->hires_accel_cycles << 1) + remainder;
        const uint32_t divisor = (segment->accel_series_index << 2) + 1;
        segment->hires_accel_cycles -= (divident / divisor);
        remainder = divident % divisor;
      }
      ++segment->accel_series_index;
      --segment->loops_accel;
      delay_loops = segment->hires_accel_cycles >> DELAY_CYCLE_SHIFT;
    } else if (segment->loops_travel > 0) {
      delay_loops = segment->travel_delay_cycles;
      --segment->loops_travel;
    } else if (segment->loops_decel > 0) {
      const uint32_t divident = (segment->hires_accel_cycles << 1) + remainder;
      const uint32_t divisor = (segment->accel_series_index << 2) - 1;
      segment->hires_accel_cycles += (divident / divisor);
      remainder = divident % divisor;
      --segment->accel_series_index;
      --segment->loops_decel;
      delay_loops = segment->hires_accel_cycles >> DELAY_CYCLE_SHIFT;
    } else {
      break;  // done.
    }

    const double wait_time = 1.0 * delay_loops / TIMER_FREQUENCY;
    sim_time += wait_time;
    writer_->Update(channel_value, sim_time);
  }
  return true;
}

SimFirmwareAudioQueue::SimFirmwareAudioQueue(FILE *out)
    : writer_(new AudioWriter(out)) {}

SimFirmwareAudioQueue::~SimFirmwareAudioQueue() { delete writer_; }
