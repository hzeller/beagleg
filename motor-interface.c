/*
 * (c) 2013, 1014 Henner Zeller <h.zeller@acm.org>
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

#include <errno.h>
#include <fcntl.h>
#include <pruss_intc_mapping.h>
#include <prussdrv.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "motor-interface-constants.h"

// Generated PRU code from motor-interface-pru.p
#include "motor-interface-pru_bin.h"

//#define DEBUG_QUEUE

#define MOTOR_COUNT 8

// Cycles we need to do other stuff in the update loop, thus take it out
// of the delay.
#define LOOP_OVERHEAD_CYCLES 11

struct QueueElement {
  uint8_t state;
  uint8_t direction_bits;
  uint16_t steps;          // Number of total steps
  uint16_t padding;        // align properly.
  uint32_t travel_delay;   // delay-loop count
  uint32_t fractions[MOTOR_COUNT];  // fixed point fractions to add each step.
} __attribute__((packed));


#define PRU_NUM 0

struct QueueElement *volatile shared_queue_;
unsigned int queue_pos_;

static void init_queue(struct QueueElement *elements) {
  memset(elements, 0x00, QUEUE_LEN * sizeof(*elements));
  for (int i = 0; i < QUEUE_LEN; ++i) {
    elements[i].state = STATE_EMPTY;
  }
}

static struct QueueElement *volatile map_queue() {
  void *result;
  prussdrv_map_prumem (PRUSS0_PRU0_DATARAM, &result);
  shared_queue_ = (struct QueueElement*) result;
  queue_pos_ = 0;
  init_queue(shared_queue_);
  return shared_queue_;
}

static struct QueueElement *volatile next_queue_element() {
  queue_pos_ %= QUEUE_LEN;
  while (shared_queue_[queue_pos_].state != STATE_EMPTY) {
    prussdrv_pru_wait_event (PRU_EVTOUT_0);
    prussdrv_pru_clear_event (PRU0_ARM_INTERRUPT);
  }
  return &shared_queue_[queue_pos_++];
}

int beagleg_init(void) {
  unsigned int ret;
  tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
  prussdrv_init ();

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
static void DumpQueueElement(const struct QueueElement *element) {
  fprintf(stderr, "enqueue[%d]: dir:0x%02x steps:%d delay:%d ",
	  element - shared_queue_,
	  element->direction_bits, element->steps, element->travel_delay);
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    fprintf(stderr, "f%d:0x%08x ", i, element->fractions[i]);
  }
  fprintf(stderr, "\n");
	    
}
#endif

static void enqueue_internal(struct QueueElement *element) {
  element->state = STATE_EMPTY;  // Don't set yet to avoid race.
  struct QueueElement *volatile queue_element = next_queue_element();
  memcpy(queue_element, element, sizeof(*element));
#ifdef DEBUG_QUEUE
  DumpQueueElement(queue_element);
#endif
  // Fully initialized. Tell PRU
  queue_element->state = STATE_FILLED;
}

static int speed_2_delay(float steps_per_second) {
  // Roughly, we neexd 4 * cycle-time delay. At 200Mhz, we have 5ns cycles.
  // There is some overhead for each toplevel loop that we substract.
  const float kLoopTimeSeconds = 5e-9 * 4;
  float steps = (1/steps_per_second) / kLoopTimeSeconds;
  if (steps > 0x7fffffff) { return 0x7fffffff; }   // Cap veeery long period.
  return steps - LOOP_OVERHEAD_CYCLES;
}

int beagleg_enqueue(const struct bg_movement *param, FILE *err_stream) {
  struct QueueElement new_element;
  int delay = speed_2_delay(param->travel_speed);
  if (delay <= 0) delay = 1;
  new_element.travel_delay = delay;
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
  new_element.steps = biggest_value;
  const uint64_t max_fraction = 0x7FFFFFFF;
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    const uint64_t delta = abs(param->steps[i]);
    new_element.fractions[i] = delta * max_fraction / biggest_value;
  }
  enqueue_internal(&new_element);
  return 0;
}

void beagleg_wait_queue_empty(void) {
  const unsigned int last_insert_position = (queue_pos_ - 1) % QUEUE_LEN;
  while (shared_queue_[last_insert_position].state == STATE_FILLED) {
    prussdrv_pru_wait_event (PRU_EVTOUT_0);
    prussdrv_pru_clear_event (PRU0_ARM_INTERRUPT);
  }
}

void beagleg_exit_nowait(void) {
  prussdrv_pru_disable (PRU_NUM);
  prussdrv_exit ();
}

void beagleg_exit(void) {
  struct QueueElement end_element;
  memset(&end_element, 0x00, sizeof(end_element));
  end_element.steps = 0;  // 0 steps: sentinel to exit.
  enqueue_internal(&end_element);
  beagleg_wait_queue_empty();
  beagleg_exit_nowait();
}
