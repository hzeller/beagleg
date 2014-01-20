/*
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

#include "motor-interface.h"

#include <assert.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <pruss_intc_mapping.h>
#include <prussdrv.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "motor-interface-constants.h"

// Generated PRU code from motor-interface-pru.p
#include "motor-interface-pru_bin.h"

#define DEBUG_QUEUE

#define MOTOR_COUNT 8

// Cycles we need to do other stuff in the update loop, thus take it out
// of the delay.
#define LOOP_OVERHEAD_CYCLES 11

struct QueueElement {
  // Queue header
  uint8_t state;
  uint8_t direction_bits;

  // TravelParameters (needs to match TravelParameters in motor-interface-pru.p)
  uint16_t accel_series_index;
  uint16_t steps_accel;    // Phase 1: steps spent in acceleration
  uint16_t steps_travel;   // Phase 2: steps spent in travel
  uint16_t steps_decel;    // Phase 3: steps spent in deceleration
  
  uint32_t hires_accel_cycles;  // initial delay cycles.
  uint32_t travel_cycles;       // travel cycles.

  uint32_t fractions[MOTOR_COUNT];  // fixed point fractions to add each step.
} __attribute__((packed));


#define PRU_NUM 0

static float acceleration_;
volatile struct QueueElement *shared_queue_;
static unsigned int queue_pos_;

static void init_queue(volatile struct QueueElement *elements) {
  bzero((void*) elements, QUEUE_LEN * sizeof(*elements));
  for (int i = 0; i < QUEUE_LEN; ++i) {
    elements[i].state = STATE_EMPTY;
  }
}

static volatile struct QueueElement *map_queue() {
  void *result;
  prussdrv_map_prumem(PRUSS0_PRU0_DATARAM, &result);
  shared_queue_ = (struct QueueElement*) result;
  queue_pos_ = 0;
  init_queue(shared_queue_);
  return shared_queue_;
}

static volatile struct QueueElement *next_queue_element() {
  queue_pos_ %= QUEUE_LEN;
  while (shared_queue_[queue_pos_].state != STATE_EMPTY) {
    prussdrv_pru_wait_event(PRU_EVTOUT_0);
    prussdrv_pru_clear_event(PRU0_ARM_INTERRUPT);
  }
  return &shared_queue_[queue_pos_++];
}

int beagleg_init(float acceleration_steps_s2) {
  acceleration_ = acceleration_steps_s2;
  unsigned int ret;
  tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
  prussdrv_init();

  /* Get the interrupt initialized */
  ret = prussdrv_open(PRU_EVTOUT_0);  // allow access.
  if (ret) {
    fprintf(stderr, "prussdrv_open() failed (%d)\n", ret);
    return ret;
  }
  prussdrv_pruintc_init(&pruss_intc_initdata);
  if (map_queue() == NULL) {
    fprintf(stderr, "Couldn't map memory\n");
    return 1;
  }

  // For some silly reason, the API does not take a const unsigned int*.
  prussdrv_pru_write_memory(PRUSS0_PRU0_IRAM, 0, (unsigned int*) PRUcode,
			    sizeof(PRUcode));
  prussdrv_pru_enable(0);

  return 0;
}

#ifdef DEBUG_QUEUE
static void DumpQueueElement(volatile const struct QueueElement *e) {
  if (e->state == STATE_EXIT) {
    fprintf(stderr, "enqueue[%02d]: EXIT\n", e - shared_queue_);
  } else {
    struct QueueElement copy = *e;
    fprintf(stderr, "enqueue[%02d]: dir:0x%02x steps:(%5d + %5d + %5d) = %5d "
	    "accel-delay: %d (%u raw); travel-delay: %d",
	    e - shared_queue_, copy.direction_bits,
	    copy.steps_accel, copy.steps_travel, copy.steps_decel,
	    copy.steps_accel + copy.steps_travel + copy.steps_decel,
	    copy.hires_accel_cycles >> DELAY_CYCLE_SHIFT,
	    copy.hires_accel_cycles, copy.travel_cycles);
#if 0
    for (int i = 0; i < MOTOR_COUNT; ++i) {
      fprintf(stderr, "f%d:0x%08x ", i, element->fractions[i]);
    }
#endif
  fprintf(stderr, "\n");
  }	    
}
#endif

static void enqueue_internal(struct QueueElement *element) {
  const uint8_t state_to_send = element->state;
  assert(state_to_send != STATE_EMPTY);  // forgot to set proper state ?
  // Initially, we copy everything with 'STATE_EMPTY', then flip the state
  // to avoid a race condition while copying.
  element->state = STATE_EMPTY; 
  volatile struct QueueElement *queue_element = next_queue_element();
  *queue_element = *element;

  // Fully initialized. Tell busy-waiting PRU by flipping the state.
  queue_element->state = state_to_send;
#ifdef DEBUG_QUEUE
  DumpQueueElement(queue_element);
#endif
}

static double cycles_per_second() { return 100e6; } // two cycles per loop.
#if 0
static int speed_2_delay(float steps_per_second) {
  // Roughly, we neexd 4 * cycle-time delay. At 200Mhz, we have 5ns cycles.
  // There is some overhead for each toplevel loop that we substract.
  const float kLoopTimeSeconds = 5e-9 * 4;
  float steps = (1/steps_per_second) / kLoopTimeSeconds;
  if (steps > 0x7fffffff) { return 0x7fffffff; }   // Cap veeery long period.
  return steps - LOOP_OVERHEAD_CYCLES;
}
#endif

int beagleg_enqueue(const struct bg_movement *param, FILE *err_stream) {
  struct QueueElement new_element;

  int biggest_value = abs(param->steps[0]);
  new_element.direction_bits = 0;
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    if (param->steps[i] < 0) {
      new_element.direction_bits |= (1 << i);
    }
    if (abs(param->steps[i]) > biggest_value) {
      biggest_value = abs(param->steps[i]);
    }
  }
  if (biggest_value == 0) {
    fprintf(err_stream ? err_stream : stderr, "zero steps. Ignoring command.\n");
    return 1;
  }
  if (biggest_value > 65535) {
    fprintf(err_stream ? err_stream : stderr,
	    "At most 65535 steps, got %d. Ignoring command.\n", biggest_value);
    return 2;
  }
  const uint64_t max_fraction = 0x7FFFFFFF;
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    const uint64_t delta = abs(param->steps[i]);
    new_element.fractions[i] = delta * max_fraction / biggest_value;
  }

  // Calculate speeds
  // First step while exerpimenting: assume start/endspeed always 0.
  // TODO: take these into account (requires acceleration planning)
  const int total_steps = biggest_value;

  // TODO(hzeller): we need two step-count per motor cycle (edge up, edge down),
  // So we need to multiply step-counts by 2 (currently, motors only move
  // half the distance).
  // We're constrained by 16 bit registers for the counters, so we can't easily
  // <<1. OTOH, we'd like to have 64k step-count, as it means a travel-distance
  // of comforatable > 400mm@160 steps/mm

  // Steps to reach requested speed at acceleration
  // v = a*t -> t = v/a
  // s = a/2 * t^2; subsitution t from above: s = v^2/(2*a)
  int steps_accel = (param->travel_speed*param->travel_speed
		     / (2.0 * acceleration_));
  if (acceleration_ <= 0) {
    // Acceleration set to 0 or negative: we assume 'infinite' acceleration.
    new_element.steps_accel = new_element.steps_decel = 0;
    new_element.steps_travel = total_steps;
  }
  else if (2 * steps_accel < total_steps) {
    new_element.steps_accel = steps_accel;
    new_element.steps_travel = total_steps - 2*steps_accel;
    new_element.steps_decel = steps_accel;
  }
  else {
    // We don't want deceleration have more steps than acceleration (the
    // iterative approximation will not be happy), so let's make sure to have
    // accel_steps >= decel_steps by using the fact that integer div essentially
    // does floor()
    new_element.steps_decel = total_steps / 2;
    new_element.steps_travel = 0;
    new_element.steps_accel = total_steps - new_element.steps_decel;
  }
  double accel_factor = cycles_per_second() * (sqrt(2.0 / acceleration_));

  new_element.accel_series_index = 0;   // zero speed start
  new_element.hires_accel_cycles = ((1 << DELAY_CYCLE_SHIFT)
				    * accel_factor * 0.67605);
  new_element.travel_cycles = cycles_per_second() / param->travel_speed;
  new_element.state = STATE_FILLED;

  enqueue_internal(&new_element);
  return 0;
}

void beagleg_wait_queue_empty(void) {
  const unsigned int last_insert_position = (queue_pos_ - 1) % QUEUE_LEN;
  while (shared_queue_[last_insert_position].state != STATE_EMPTY) {
    prussdrv_pru_wait_event(PRU_EVTOUT_0);
    prussdrv_pru_clear_event(PRU0_ARM_INTERRUPT);
  }
}

void beagleg_exit_nowait(void) {
  prussdrv_pru_disable(PRU_NUM);
  prussdrv_exit();
}

void beagleg_exit(void) {
  struct QueueElement end_element;
  bzero(&end_element, sizeof(end_element));
  end_element.state = STATE_EXIT;
  enqueue_internal(&end_element);
  beagleg_wait_queue_empty();
  beagleg_exit_nowait();
}
