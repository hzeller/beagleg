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

#include "shared-constants.h"

//#define DEBUG_QUEUE

#define MOTOR_COUNT 8

struct QueueElement {
  uint8_t state;
  uint8_t direction_bits;
  uint16_t steps;          // Number of total steps
  uint16_t travel_delay;   // delay-loop count
  uint32_t fractions[MOTOR_COUNT];  // fixed point fractions to add each step.
} __attribute__((packed));


#define PRU_NUM 0

struct QueueElement *volatile shared_queue_;
struct QueueElement *volatile shared_queue_it_;

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
  shared_queue_it_ = shared_queue_;
  init_queue(shared_queue_);
  return shared_queue_;
}

static struct QueueElement *volatile next_queue_element() {
  if (shared_queue_it_ - shared_queue_ >= QUEUE_LEN) {  // ringbuffer.
    shared_queue_it_ = shared_queue_;
  }
  while (shared_queue_it_->state != STATE_EMPTY) {
    prussdrv_pru_wait_event (PRU_EVTOUT_0);
    prussdrv_pru_clear_event (PRU0_ARM_INTERRUPT);
  }
  return shared_queue_it_++;
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
  // TODO: load in binary.
  prussdrv_exec_program(PRU_NUM, "./motor-control.bin");
  return 0;
}

#ifdef DEBUG_QUEUE
static void DumpQueueElement(const struct QueueElement *element) {
  fprintf(stderr, "enqueue: dir:0x%02x steps:%d speed:%d f0:0x%08x f7:0x%08x\n",
	  element->direction_bits, element->steps, element->travel_delay,
	  element->fractions[0], element->fractions[7]);
}
#endif

static void enqueue_internal(struct QueueElement *element) {
#ifdef DEBUG_QUEUE
  DumpQueueElement(element);
#endif
  element->state = STATE_EMPTY;  // Don't set yet to avoid race.
  struct QueueElement *volatile queue_element = next_queue_element();
  memcpy(queue_element, element, sizeof(*element));
  // Fully initialized. Tell PRU
  queue_element->state = STATE_FILLED;
}

static int speed_2_delay(float steps_per_second) {
  // Roughly, we neexd 4 * cycle-time delay. At 200Mhz, we have 5ns cycles.
  // There is some overhead for each toplevel loop, but we ignore that for now.
  const float kLoopTimeSeconds = 5e-9 * 4;
  return (1/steps_per_second) / kLoopTimeSeconds;
}

int beagleg_enqueue(const struct bg_movement *param) {
  int delay_steps = speed_2_delay(param->travel_speed);
  if (delay_steps > 65535) delay_steps = 65535;
  struct QueueElement new_element;
  new_element.travel_delay = delay_steps;
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
    fprintf(stderr, "zero steps. Bailing out");
    return 1;
  }
  if (biggest_value > 65535) {
    fprintf(stderr, "At most 65535 steps. Bailing out");
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
  // TODO
}

void beagleg_exit(void) {
  struct QueueElement end_element;
  end_element.steps = 0;
  enqueue_internal(&end_element);
  beagleg_wait_queue_empty();
  prussdrv_pru_disable (PRU_NUM);
  prussdrv_exit ();
}
