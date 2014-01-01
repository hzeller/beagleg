#include <assert.h>
#include <stdio.h>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>

#include "motor-interface.h"

int main(int argc, char *argv[]) {
  beagleg_init();

  if (argc != 4) {
    fprintf(stderr, "Usage: %s <steps-per-second> <steps1> <steps2>\n", argv[0]);
    return 1;
  }

  int steps_per_second = atoi(argv[1]);
  int steps1 = atoi(argv[2]);
  int steps2 = atoi(argv[3]);

  struct bg_movement movement;
  memset(&movement, 0x00, sizeof(movement));
  movement.travel_speed = steps_per_second;
  movement.steps[0] = steps1;
  movement.steps[7] = steps2;
  for (;;) {
    // Switch speed back and forth.
    movement.travel_speed = ((movement.steps[0] > 0)
			     ? steps_per_second
			     : 2 * steps_per_second);
    movement.steps[0] = -movement.steps[0];
    movement.steps[7] = -movement.steps[7];
    beagleg_enqueue(&movement);
  }

  beagleg_exit();

  return 0;
}

