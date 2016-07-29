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
};

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

// Stop gap for compiler attempting to be overly clever when copying between
// host and PRU memory.
static void unaligned_memcpy(void *dest, const void *src, size_t size) {
  char *d = (char*) dest;
  const char *s = (char*) src;
  const char *end = d + size;
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

  // Need to case volatile away, otherwise c++ complains.
  MotionSegment *queue_element =
    (MotionSegment*) &pru_data_->ring_buffer[queue_pos_++];

  unaligned_memcpy(queue_element, element, sizeof(*queue_element));

  // Fully initialized. Tell busy-waiting PRU by flipping the state.
  queue_element->state = state_to_send;
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
