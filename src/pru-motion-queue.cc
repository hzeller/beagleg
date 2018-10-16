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

#include "common/logging.h"

#include "generic-gpio.h"
#include "pwm-timer.h"
#include "hardware-mapping.h"
#include "pru-hardware-interface.h"

using internal::QueueStatus;

//#define DEBUG_QUEUE

// The communication with the PRU. We memory map the static RAM in the PRU
// and write stuff into it from here. Mostly this is a ring-buffer with
// commands to execute, but also configuration data, such as what to do when
// an endswitch fires.
struct PRUCommunication {
  volatile QueueStatus status;
  volatile MotionSegment ring_buffer[QUEUE_LEN];
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

// Calculate offset into ring-buffer.
static inline unsigned int RingbufferOffset(unsigned int current, int offset) {
  // Make sure to keep unsigned operations within reach in case QUEUE_LEN is
  // not a clean 2^n
  return (QUEUE_LEN + current + offset) % QUEUE_LEN;
}

void PRUMotionQueue::ClearPRUAbort(unsigned int idx) {
  volatile MotionSegment *e = &pru_data_->ring_buffer[idx];
  e->state = STATE_EMPTY;
}

int PRUMotionQueue::GetPendingElements(uint32_t *head_item_progress) {
  // Get data from the PRU
  const struct QueueStatus status = *(struct QueueStatus*) &pru_data_->status;
  const unsigned int last_insert_index = RingbufferOffset(queue_pos_, -1);
  if (head_item_progress) {
    *head_item_progress = status.counter;
  }

  if (pru_data_->ring_buffer[last_insert_index].state == STATE_EMPTY) {
    return 0;
  }

  unsigned int queue_len = RingbufferOffset(queue_pos_, -status.index);
  queue_len += queue_len ? 0 : QUEUE_LEN;
  return queue_len;
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

bool PRUMotionQueue::Enqueue(MotionSegment *element) {
  const uint8_t state_to_send = element->state;
  assert(state_to_send != STATE_EMPTY);  // forgot to set proper state ?
  // Initially, we copy everything with 'STATE_EMPTY', then flip the state
  // to avoid a race condition while copying.
  element->state = STATE_EMPTY;

  queue_pos_ %= QUEUE_LEN;
  while (pru_data_->ring_buffer[queue_pos_].state != STATE_EMPTY) {
    if (pru_data_->ring_buffer[queue_pos_].state == STATE_ABORT) {
      ClearPRUAbort(queue_pos_);
      return false;
    }
    pru_interface_->WaitEvent();
  }

  volatile MotionSegment *queue_element = &pru_data_->ring_buffer[queue_pos_++];
  unaligned_memcpy(queue_element, element, sizeof(*queue_element));

  // Fully initialized. Tell busy-waiting PRU by flipping the state.
  queue_element->state = state_to_send;

#ifdef DEBUG_QUEUE
  DumpMotionSegment(queue_element, pru_data_);
#endif
  return true;
}

void PRUMotionQueue::WaitQueueEmpty() {
  const unsigned int last_insert_index = RingbufferOffset(queue_pos_, -1);
  while (pru_data_->ring_buffer[last_insert_index].state != STATE_EMPTY) {
    if (pru_data_->ring_buffer[last_insert_index].state == STATE_ABORT) {
      break;
    }
    pru_interface_->WaitEvent();
  }
}

void PRUMotionQueue::MotorEnable(bool on) {
  hardware_mapping_->EnableMotors(on);
}

void PRUMotionQueue::Shutdown(bool flush_queue) {
  if (flush_queue) {
    struct MotionSegment end_element = {};
    end_element.state = STATE_EXIT;
    Enqueue(&end_element);
    WaitQueueEmpty();
  }
  pru_interface_->Shutdown();
  MotorEnable(false);
}

PRUMotionQueue::~PRUMotionQueue() {}

PRUMotionQueue::PRUMotionQueue(HardwareMapping *hw, PruHardwareInterface *pru)
  : hardware_mapping_(hw),
    pru_interface_(pru) {
  const bool success = Init();
  // For now, we just assert-fail here, if things fail.
  // Typically hardware-doomed event anyway.
  assert(success);
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
