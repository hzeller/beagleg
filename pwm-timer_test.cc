/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2015 H Hartley Sweeten <hsweeten@visionengravers.com>
 * Test for PWM timers
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>

#include "pwm-timer.h"

static int usage(const char *progname) {
  fprintf(stderr, "Usage: %s <timer> <freq> [<dc>] [<delay>]\n", progname);
  fprintf(stderr, "Options:\n"
                  "  <timer>  : timer number to test (4, 5, 6, or 7) (Required)\n"
                  "  <freq>   : base frequency for the PWM (in hz)   (Required)\n"
                  "  <dc>     : duty cycle for the PWM (0 to 1)      (Optional, Default: 0.5 (50%%)\n"
                  "  <delay>  : seconds to delay before stopping PWM (Optional, Default: 5 seconds\n");
  fprintf(stderr, "Parameters must be in the order shown above.\n");
  fprintf(stderr, "A <dc> of 0 will cause the PWM to cycle from .1 to 1.0 \n");
  return 1;
}

int main(int argc, char *argv[]) {
  if (argc < 3) { return usage(argv[0]); }

  uint32_t gpio_def;
  switch (atoi(argv[1])) {
  case 4:  gpio_def = PIN_P8_7;  break;
  case 5:  gpio_def = PIN_P8_9;  break;
  case 6:  gpio_def = PIN_P8_10; break;
  case 7:  gpio_def = PIN_P8_8;  break;
  default: return usage(argv[0]);
  }
  int freq = atoi(argv[2]);
  float dc = 0.5;
  if (argc > 3) dc = atof(argv[3]);
  int delay = 5;
  if (argc > 4) delay = atoi(argv[4]);

  if (!pwm_timers_map()) return 1;

  pwm_timer_set_freq(gpio_def, freq);
  if (dc == 0) {
    pwm_timer_set_duty(gpio_def, 0.1);
    pwm_timer_start(gpio_def, 1);
    sleep(1);
    pwm_timer_set_duty(gpio_def, 0.2);
    sleep(1);
    pwm_timer_set_duty(gpio_def, 0.3);
    sleep(1);
    pwm_timer_set_duty(gpio_def, 0.4);
    sleep(1);
    pwm_timer_set_duty(gpio_def, 0.5);
    sleep(1);
    pwm_timer_set_duty(gpio_def, 0.6);
    sleep(1);
    pwm_timer_set_duty(gpio_def, 0.7);
    sleep(1);
    pwm_timer_set_duty(gpio_def, 0.8);
    sleep(1);
    pwm_timer_set_duty(gpio_def, 0.9);
    sleep(1);
    pwm_timer_set_duty(gpio_def, 1.0);
    sleep(1);
  } else {
    pwm_timer_set_duty(gpio_def, dc);
    pwm_timer_start(gpio_def, 1);
    sleep(delay);
  }
  pwm_timer_start(gpio_def, 0);

  pwm_timers_unmap();
  return 0;
}
