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

// We need two loops per motor cycle (edge up, edge down),
// So we need to multiply step-counts by 2 (currently, motors only move
// half the distance).
#define SUB_STEPS (1 << 1)

struct QueueElement {
  // Queue header
  uint8_t state;
  uint8_t direction_bits;

  // TravelParameters (needs to match TravelParameters in motor-interface-pru.p)
  uint16_t accel_series_index;
  uint16_t loops_accel;    // Phase 1: loops spent in acceleration
  uint16_t loops_travel;   // Phase 2: lops spent in travel
  uint16_t loops_decel;    // Phase 3: loops spent in deceleration
  
  uint32_t hires_accel_cycles;  // initial delay cycles.
  uint32_t travel_cycles;       // travel cycles.

  uint32_t fractions[MOTOR_COUNT];  // fixed point fractions to add each step.
} __attribute__((packed));


#define PRU_NUM 0

static float acceleration_;
static float max_speed_;
volatile struct QueueElement *shared_queue_;
static unsigned int queue_pos_;

// delay loops per second.
static double cycles_per_second() { return 100e6; } // two cycles per loop.

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
  max_speed_ = 1e6;  // 1 MHz. Limit due to PRU-CPU speed.

  const double accel_factor = cycles_per_second()
    * (sqrt(SUB_STEPS * 2.0 / acceleration_));
  if ((1 << DELAY_CYCLE_SHIFT) * accel_factor * 0.67605 > 0xFFFFFFFF) {
    fprintf(stderr, "Too slow acceleration to deal with. If really needed, "
	    "reduce value of #define DELAY_CYCLE_SHIFT\n");
    return 1;
  }

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
    fprintf(stderr, "enqueue[%02td]: EXIT\n", e - shared_queue_);
  } else {
    struct QueueElement copy = *e;
    fprintf(stderr, "enqueue[%02td]: dir:0x%02x steps:(%5d + %5d + %5d) = %5d "
	    "accel-delay: %d (%u raw); travel-delay: %d",
	    e - shared_queue_, copy.direction_bits,
	    copy.loops_accel, copy.loops_travel, copy.loops_decel,
	    copy.loops_accel + copy.loops_travel + copy.loops_decel,
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

static void enqueue_element(struct QueueElement *element) {
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

// Clip speed to maximum we can reach.
static float clip_speed(float v) {
  return v < max_speed_ ? v : max_speed_;
}

static int beagleg_enqueue_internal(const struct bg_movement *param,
				    int defining_axis_steps) {
  struct QueueElement new_element;
  new_element.direction_bits = 0;

  // The defining_axis_steps is the number of steps of the axis that requires
  // the most number of steps. All the others are a fraction of the steps.
  //
  // The top bits represent SUB_STEPS (2 is the minium, as we need two cycles
  // for a 0 1 transition. So in that case we have 31 bit fraction and 1 bit
  // that overflows and toggles for the steps we want to generate.
  // It could be more if we wanted, for explicit sub-step addressing.
  const uint64_t max_fraction = 0xFFFFFFFF / SUB_STEPS;
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    if (param->steps[i] < 0) {
      new_element.direction_bits |= (1 << i);
    }
    const uint64_t delta = abs(param->steps[i]);
    new_element.fractions[i] = delta * max_fraction / defining_axis_steps;
  }

  const float travel_speed = clip_speed(param->travel_speed);

  // Calculate speeds
  // First step while exerpimenting: assume start/endspeed always 0.
  // TODO: take these into account (requires acceleration planning)
  const int total_loops = SUB_STEPS * defining_axis_steps;

  // Steps to reach requested speed at acceleration
  // v = a*t -> t = v/a
  // s = a/2 * t^2; subsitution t from above: s = v^2/(2*a)
  const int accel_loops = SUB_STEPS * (travel_speed*travel_speed
				       / (2.0 * acceleration_));

  if (acceleration_ <= 0) {
    // Acceleration set to 0 or negative: we assume 'infinite' acceleration.
    new_element.loops_accel = new_element.loops_decel = 0;
    new_element.loops_travel = total_loops;
  }
  else if (2 * accel_loops < total_loops) {
    new_element.loops_accel = accel_loops;
    new_element.loops_travel = total_loops - 2*accel_loops;
    new_element.loops_decel = accel_loops;
  }
  else {
    // We don't want deceleration have more steps than acceleration (the
    // iterative approximation will not be happy), so let's make sure to have
    // accel_steps >= decel_steps by using the fact that integer div essentially
    // does floor()
    new_element.loops_decel = total_loops / 2;
    new_element.loops_travel = 0;
    new_element.loops_accel = total_loops - new_element.loops_decel;
  }

  double accel_factor = cycles_per_second() * (sqrt(SUB_STEPS * 2.0 / acceleration_));

  new_element.accel_series_index = 0;   // zero speed start
  // TODO(hzeller): check that cycle shift does not mean overflow.
  new_element.hires_accel_cycles = ((1 << DELAY_CYCLE_SHIFT)
				    * accel_factor * 0.67605);
  new_element.travel_cycles = cycles_per_second() / param->travel_speed;
  new_element.state = STATE_FILLED;

  enqueue_element(&new_element);
  return 0;
}

int beagleg_enqueue(const struct bg_movement *param, FILE *err_stream) {
  // TODO: this function should automatically split this into multiple segments
  // each with the maximum number of steps.
  int biggest_value = abs(param->steps[0]);
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    if (abs(param->steps[i]) > biggest_value) {
      biggest_value = abs(param->steps[i]);
    }
  }
  if (biggest_value == 0) {
    fprintf(err_stream ? err_stream : stderr, "zero steps. Ignoring command.\n");
    return 1;
  }
  if (biggest_value > 65535 / SUB_STEPS) {
    // TODO: for now, we limit the number. This should be implemented by
    // cutting this in multiple pieces, each calling beagleg_enqueue_internal()
    fprintf(err_stream ? err_stream : stderr,
	    "At most %d steps, got %d. Ignoring command.\n", 65535 / SUB_STEPS,
	    biggest_value);
    return 2;
  }
  beagleg_enqueue_internal(param, biggest_value);
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
  enqueue_element(&end_element);
  beagleg_wait_queue_empty();
  beagleg_exit_nowait();
}
