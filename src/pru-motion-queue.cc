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

// Implementation of the MotionQueue interfacing the PRU.
// We are using some shared memory between CPU and PRU to communicate.

#include "motion-queue.h"

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <strings.h>
#include <stdlib.h>

#include "generic-gpio.h"
#include "pwm-timer.h"
#include "logging.h"
#include "hardware-mapping.h"
#include "pru-hardware-interface.h"

//#define DEBUG_QUEUE

// The communication with the PRU. We memory map the static RAM in the PRU
// and write stuff into it from here. Mostly this is a ring-buffer with
// commands to execute, but also configuration data, such as what to do when
// an endswitch fires.
struct PRUCommunication {
  volatile struct MotionSegment ring_buffer[QUEUE_LEN];
  volatile struct QueueStatus status;
} __attribute__((packed));

#ifdef DEBUG_QUEUE
static void DumpMotionSegment(volatile const struct MotionSegment *e,
                              volatile struct PRUCommunication *pru_data) {
  if (e->state == STATE_EXIT) {
    Log_debug("enqueue[%02td]: EXIT", e - pru_data->ring_buffer);
  } else {
    MotionSegment copy = (MotionSegment&) *e;
    std::string line;
    line = StringPrintf("enqueue[%02td]: dir:0x%02x s:(%5d + %5d + %5d) = %5d ",
                        e - pru_data->ring_buffer, copy.direction_bits,
                        copy.loops_accel, copy.loops_travel, copy.loops_decel,
                        copy.loops_accel + copy.loops_travel + copy.loops_decel);

    if (copy.hires_accel_cycles > 0) {
      line += StringPrintf("accel : %5.0fHz (%d loops);",
                           TIMER_FREQUENCY /
                           (2.0*(copy.hires_accel_cycles >> DELAY_CYCLE_SHIFT)),
                           copy.hires_accel_cycles >> DELAY_CYCLE_SHIFT);
    }
    if (copy.travel_delay_cycles > 0) {
      line += StringPrintf("travel: %5.0fHz (%d loops);",
                           TIMER_FREQUENCY / (2.0*copy.travel_delay_cycles),
                           copy.travel_delay_cycles);
    }
#if 0
    // The fractional parts.
    for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
      if (copy.fractions[i] == 0) continue;  // not interesting.
      line += StringPrintf("f%d:0x%08x ", i, copy.fractions[i]);
    }
#endif
    Log_debug("%s", line.c_str());
  }
}
#endif

struct HistorySegment {
  uint32_t total_loops;
  uint32_t fractions[MOTION_MOTOR_COUNT];
  uint8_t direction_bits;
};

void PRUMotionQueue::RegisterHistorySegment(MotionSegment *element) {
  const unsigned int last_insert_position = (queue_pos_ - 1) % QUEUE_LEN;
  struct HistorySegment *new_slot = &shadow_queue_[last_insert_position];

  bzero(new_slot, sizeof(struct HistorySegment));

  new_slot->total_loops += element->loops_accel;
  new_slot->total_loops += element->loops_travel;
  new_slot->total_loops += element->loops_decel;

  new_slot->direction_bits = element->direction_bits;

  for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
    new_slot->fractions[i] = element->fractions[i];
  }
}

static void fractions_to_loops(const struct HistorySegment &segment,
                               int32_t *motors_loops) {
  const uint64_t max_fraction = 0xFFFFFFFF;
  for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
    const int sign = (segment.direction_bits >> i) & 1 ? -1 : 1;
    uint64_t loops = (uint64_t) segment.fractions[i] * (uint64_t) segment.total_loops;
    loops /= max_fraction;
    motors_loops[i] = sign * loops;
  }
}

void PRUMotionQueue::UpdateAbsolutePosition() {
  const struct QueueStatus status = *(struct QueueStatus*) &pru_data_->status;
  struct HistorySegment current = shadow_queue_[status.index];
  const int shadow_queue_delay
    = (status.index + QUEUE_LEN - back_shadow_queue_) % QUEUE_LEN;
  int32_t motors_loops[MOTION_MOTOR_COUNT];

  for (int i = 0; i < shadow_queue_delay; ++i) {
    fractions_to_loops(shadow_queue_[back_shadow_queue_], motors_loops);
    for (int j = 0; j < MOTION_MOTOR_COUNT; ++j) {
      absolute_pos_loops_[j] += motors_loops[j];
    }
    shadow_queue_[back_shadow_queue_].total_loops = 0;
    back_shadow_queue_ = (back_shadow_queue_ + 1) % QUEUE_LEN;
  }

  // Now update the current executing slot.
  if (current.total_loops) {
    uint64_t delta;
    fractions_to_loops(current, motors_loops);
    for (int i = 0; i < MOTION_MOTOR_COUNT; ++i) {
      absolute_pos_loops_[i] -= segment_partials_[i];

      delta = status.counter * abs(motors_loops[i]);
      delta /= current.total_loops;

      segment_partials_[i]
        = motors_loops[i] < 0 ? motors_loops[i] + delta
                              : motors_loops[i] - delta;

      absolute_pos_loops_[i] += segment_partials_[i];
    }
  }
}

void PRUMotionQueue::GetMotorsLoops(int32_t *absolute_pos_loops) {
  UpdateAbsolutePosition();
  memcpy(absolute_pos_loops, absolute_pos_loops_, sizeof(absolute_pos_loops_));
}

// Stop gap for compiler attempting to be overly clever when copying between
// host and PRU memory.
static void unaligned_memcpy(volatile void *dest, const void *src, size_t size) {
  volatile char *d = (volatile char*) dest;
  const char *s = (char*) src;
  const volatile char *end = d + size;
  while (d < end) {
    *d++ = *s++;
  }
}

void PRUMotionQueue::Enqueue(MotionSegment *element) {
  const uint8_t state_to_send = element->state;
  assert(state_to_send != STATE_EMPTY);  // forgot to set proper state ?
  // Initially, we copy everything with 'STATE_EMPTY', then flip the state
  // to avoid a race condition while copying.
  element->state = STATE_EMPTY;

  queue_pos_ %= QUEUE_LEN;
  while (pru_data_->ring_buffer[queue_pos_].state != STATE_EMPTY) {
    pru_interface_->WaitEvent();
  }

  volatile MotionSegment *queue_element = &pru_data_->ring_buffer[queue_pos_++];
  unaligned_memcpy(queue_element, element, sizeof(*queue_element));

  // Fully initialized. Tell busy-waiting PRU by flipping the state.
  queue_element->state = state_to_send;

  // Update the HistoryQueue before registering a new slot.
  UpdateAbsolutePosition();

  // Register the last inserted motion segment in the shadow queue.
  RegisterHistorySegment(element);
#ifdef DEBUG_QUEUE
  DumpMotionSegment(queue_element, pru_data_);
#endif
}

void PRUMotionQueue::WaitQueueEmpty() {
  const unsigned int last_insert_position = (queue_pos_ - 1) % QUEUE_LEN;
  while (pru_data_->ring_buffer[last_insert_position].state != STATE_EMPTY) {
    pru_interface_->WaitEvent();
  }
}

void PRUMotionQueue::MotorEnable(bool on) {
  hardware_mapping_->EnableMotors(on);
}

void PRUMotionQueue::Shutdown(bool flush_queue) {
  if (flush_queue) {
    struct MotionSegment end_element = {0};
    end_element.state = STATE_EXIT;
    Enqueue(&end_element);
    WaitQueueEmpty();
  }
  pru_interface_->Shutdown();
  MotorEnable(false);
}

PRUMotionQueue::~PRUMotionQueue() { delete shadow_queue_; }

PRUMotionQueue::PRUMotionQueue(HardwareMapping *hw, PruHardwareInterface *pru)
                               : hardware_mapping_(hw),
                                 pru_interface_(pru) {
  const bool success = Init();
  // For now, we just assert-fail here, if things fail.
  // Typically hardware-doomed event anyway.
  assert(success);

  // Initialize Shadow queue
  shadow_queue_ = new HistorySegment[QUEUE_LEN];
  bzero(absolute_pos_loops_, sizeof(absolute_pos_loops_));
  bzero(segment_partials_, sizeof(segment_partials_));
  back_shadow_queue_ = 0;
}

bool PRUMotionQueue::Init() {
  MotorEnable(false);  // motors off initially.
  if (!pru_interface_->Init())
    return false;

  if (!pru_interface_->AllocateSharedMem((void **) &pru_data_,
                                         sizeof(*pru_data_)))
    return false;

  for (int i = 0; i < QUEUE_LEN; ++i) {
    pru_data_->ring_buffer[i].state = STATE_EMPTY;
  }
  queue_pos_ = 0;

  return pru_interface_->StartExecution();
}
