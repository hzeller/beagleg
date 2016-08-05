
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <pruss_intc_mapping.h>
#include <prussdrv.h>
#include <stdio.h>
#include <strings.h>
#include <string.h>

#include "motor-interface-constants.h"
#include "motion-queue.h"

// Target PRU
#define PRU_NUM 0

#if PRU_NUM == 0
#  define PRU_DATARAM PRUSS0_PRU0_DATARAM
#elif PRU_NUM == 1
#  define PRU_DATARAM PRUSS0_PRU1_DATARAM
#endif

struct PRUCommunication {
  volatile struct MotionSegment ring_buffer[QUEUE_LEN];
  volatile uint32_t skip_frac;
} __attribute__((packed));

int main(int argc, char *argv[]) {
  volatile struct PRUCommunication *pru_data_;

  // Init pru memory
  prussdrv_init();
  /* Get the interrupt initialized */
  int ret = prussdrv_open(PRU_EVTOUT_0);  // allow access.
  if (ret) {
    printf("prussdrv_open() failed (%d) %s\n", ret, strerror(errno));
    return 1;
  }
  void *pru_mmap;
  // map the struct
  prussdrv_map_prumem(PRU_DATARAM, &pru_mmap);
  if (pru_mmap == NULL) {
    printf("Couldn't map PRU memory.\n");
    return 1;
  }

  pru_data_ = (struct PRUCommunication *) pru_mmap;

  const uint32_t max_frac = 0xFFFFFFFF;

  for (int i = 1; i < 100; ++i) {
    usleep(1e3);
    pru_data_->skip_frac = max_frac / i;
    printf("%u\n", max_frac / i);
  }

  printf("Stopping completely\n");
  pru_data_->skip_frac = 0;

  getchar();

  for (int i = 100; i >= 1; --i) {
    usleep(1e3);
    pru_data_->skip_frac = max_frac / i;
    printf("%u\n", max_frac / i);
  }

  printf("Resuming completely\n");
  pru_data_->skip_frac = max_frac;

  return 0;
}
