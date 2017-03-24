/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2015 H Hartley Sweeten <hsweeten@visionengravers.com>
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

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>

#include "common/logging.h"

#include "pwm-timer.h"

// Memory space mapped to the Clock Module registers
#define CM_BASE                 0x44e00000
#define CM_SIZE                 0x4000

// Clock Module Peripheral registers
#define CM_PER_TIMER7_CLKCTRL   (0x000 + 0x7c)
#define CM_PER_TIMER4_CLKCTRL   (0x000 + 0x88)
#define CM_PER_TIMER5_CLKCTRL   (0x000 + 0xec)
#define CM_PER_TIMER6_CLKCTRL   (0x000 + 0xf0)

#define IDLEST_MASK             (0x03 << 16)
#define MODULEMODE_ENABLE       (0x02 << 0)

// Clock Module PLL registers
#define CLKSEL_TIMER7_CLK       (0x500 + 0x04)
#define CLKSEL_TIMER4_CLK       (0x500 + 0x10)
#define CLKSEL_TIMER5_CLK       (0x500 + 0x18)
#define CLKSEL_TIMER6_CLK       (0x500 + 0x1c)

#define CLKSEL(x)               (((x) & 0x3) << 0)
#define CLKSEL_TCLKIN           CLKSEL(0)
#define CLKSEL_CLK_M_OSC        CLKSEL(1)
#define CLKSEL_CLK_32KHZ        CLKSEL(2)

// Memory space mapped to the Timer Module registers
#define TIMER4_BASE             0x48044000
#define TIMER5_BASE             0x48046000
#define TIMER6_BASE             0x48048000
#define TIMER7_BASE             0x4804a000

#define TIMER_MMAP_SIZE         0x1000

#define TCLR                    0x38        // Timer Control Register
#define TCLR_GPO_CFG            (1 << 14)
#define TCLR_CAPT_MODE          (1 << 13)
#define TCLR_PT                 (1 << 12)
#define TCLR_TRG(x)             (((x) & 0x3) << 10)
#define TCLR_TRG_NONE           TCLR_TRG(0)
#define TCLR_TRG_OVF            TCLR_TRG(1)
#define TCLR_TRG_OVF_MAT        TCLR_TRG(2)
#define TCLR_TCM(x)             (((x) & 0x3) << 8)
#define TCLR_TCM_NONE           TCLR_TCM(0)
#define TCLR_TCM_POS_EDGE       TCLR_TCM(1)
#define TCLR_TCM_NEG_EDGE       TCLR_TCM(2)
#define TCLR_TCM_BOTH_EDGE      TCLR_TCM(3)
#define TCLR_SCPWM              (1 << 7)
#define TCLR_CE                 (1 << 6)
#define TCLR_PRE                (1 << 5)
#define TCLR_PTV(x)             (((x) & 0x7) << 2)
#define TCLR_AR                 (1 << 1)
#define TCLR_ST                 (1 << 0)

#define TCRR                    0x3c        // Timer Counter Register
#define TLDR                    0x40        // Timer Load Register
#define TMAR                    0x4c        // Timer Match Register

#define TIMER_BASE_CLOCK        24000000    // CLK_M_OSC is 24 MHz on the BeagleBone Black
#define TIMER_OVERFLOW          0xffffffff

#define TIMER_DEFAULT_FREQ      16000       // default PWM frequency is 16 KHz

struct pwm_timer_data {
  volatile uint32_t *regs;
  int pwm_freq;
  uint32_t resolution;
  float duty_cycle;
  char pre;
  char ptv;
  char running;
};

struct pwm_timer_data timers[4] = { {}, {}, {}, {} };

static struct pwm_timer_data *pwm_timer_get_data(uint32_t gpio_def) {
  struct pwm_timer_data *timer = NULL;
  switch (gpio_def) {
  case PIN_P8_7:  timer = &timers[0]; break; // TIMER4
  case PIN_P8_9:  timer = &timers[1]; break; // TIMER5
  case PIN_P8_10: timer = &timers[2]; break; // TIMER6
  case PIN_P8_8:  timer = &timers[3]; break; // TIMER7
  case GPIO_NOT_MAPPED:  return NULL;  // Explicitly not mapped.
  default:
    Log_info("Unsupported PWM pin mapped. "
             "Only Timer PIN_P8_{7,8,9,10} supported");
    return NULL;               // unsupported pin
  }
  if (!timer->regs) return NULL;             // unmapped timer
  return timer;
}

void pwm_timer_start(uint32_t gpio_def, bool start) {
  struct pwm_timer_data *timer = pwm_timer_get_data(gpio_def);
  if (!timer) return;

  if (!start || !timer->pwm_freq || !timer->duty_cycle) {
    timer->regs[TCLR/4] = 0;                              // stop timer
    timer->running = 0;
  } else {
    timer->regs[TCLR/4] = (TCLR_ST |                       // start timer
                           TCLR_AR |                       // auto-reload
                           TCLR_PTV(timer->ptv)  |         // prescale value
                           ((timer->pre) ? TCLR_PRE : 0) | // enable prescale if necessary
                           TCLR_CE |                       // compare mode enabled
                           TCLR_TRG_OVF_MAT |              // trigger on overflow and match
                           TCLR_PT                         // PWM toggle mode
                           );
    timer->running = 1;
  }
}

void pwm_timer_set_duty(uint32_t gpio_def, float duty_cycle) {
  struct pwm_timer_data *timer = pwm_timer_get_data(gpio_def);
  if (!timer) return;

  if (!timer->pwm_freq || duty_cycle < 0.0 || duty_cycle > 1.0) return;

  if (duty_cycle == 0.0) {
    pwm_timer_start(gpio_def, 0);
  } else {
    uint32_t start = TIMER_OVERFLOW - timer->resolution;
    uint32_t dc = TIMER_OVERFLOW - ((uint32_t)(timer->resolution * (1.0 - duty_cycle)));

    // TLDR and TMAR must be at least 2 less than the overflow
    if (TIMER_OVERFLOW - start <= 2) start = TIMER_OVERFLOW - 2;
    if (TIMER_OVERFLOW - dc <= 2) dc = TIMER_OVERFLOW - 2;

    // TMAR must be at least 2 less than the TLDR
    if (dc - start <= 2) start = dc - 2;

    timer->regs[TCRR/4] = start;
    timer->regs[TLDR/4] = start;
    timer->regs[TMAR/4] = dc;
  }
  timer->duty_cycle = duty_cycle;
}

static void pwm_timer_calc_resolution(struct pwm_timer_data *timer, int pwm_freq) {
  float pwm_period = 1.0 / pwm_freq;
  uint64_t resolution = 0;
  int ratio;
  char pre;
  char ptv;

  // select the largest possible prescale ratio
  for (ratio = 1; ratio <= 256; ratio *= 2) {
    float clk_period = (1.0 / TIMER_BASE_CLOCK) * ratio;
    resolution = pwm_period / clk_period;
    if (resolution < TIMER_OVERFLOW) break;
  }
  switch (ratio) {
  default:
  case 1:   pre = 0; ptv = 0; break;
  case 2:   pre = 1; ptv = 0; break;
  case 4:   pre = 1; ptv = 1; break;
  case 8:   pre = 1; ptv = 2; break;
  case 16:  pre = 1; ptv = 3; break;
  case 32:  pre = 1; ptv = 4; break;
  case 64:  pre = 1; ptv = 5; break;
  case 128: pre = 1; ptv = 6; break;
  case 256: pre = 1; ptv = 7; break;
  }

  timer->pwm_freq = pwm_freq;
  timer->pre = pre;
  timer->ptv = ptv;
  timer->resolution = (uint32_t)resolution;
}

void pwm_timer_set_freq(uint32_t gpio_def, int pwm_freq) {
  struct pwm_timer_data *timer = pwm_timer_get_data(gpio_def);
  if (!timer) return;

  if (pwm_freq == 0) pwm_freq = TIMER_DEFAULT_FREQ;

  // FIXME: sanity check the PWM frequency
  if (pwm_freq > TIMER_BASE_CLOCK / 2) {
    timer->pwm_freq = 0;
    return;
  }
  pwm_timer_calc_resolution(timer, pwm_freq);
}

static void pwm_timers_ena_clk(volatile uint32_t *cm, uint32_t reg, int timer) {
  uint32_t val;

  val = cm[reg/4];
  if (val & IDLEST_MASK) {
    Log_debug("Enabling TIMER%d clock", timer);
    val |= MODULEMODE_ENABLE;
    cm[reg/4] = val;
    do {
      val = cm[reg/4];
    } while (val & IDLEST_MASK);
  }
}

static volatile uint32_t *map_port(int fd, size_t length, off_t offset) {
  return (volatile uint32_t*) mmap(0, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
}

static int pwm_timers_enable_clocks(int fd) {
  volatile uint32_t *cm;

  cm = map_port(fd, CM_SIZE, CM_BASE);
  if (cm == MAP_FAILED) { perror("mmap() CM"); return 0; }

  // Enable the timer clocks
  pwm_timers_ena_clk(cm, CM_PER_TIMER4_CLKCTRL, 4);
  pwm_timers_ena_clk(cm, CM_PER_TIMER5_CLKCTRL, 5);
  pwm_timers_ena_clk(cm, CM_PER_TIMER6_CLKCTRL, 6);
  pwm_timers_ena_clk(cm, CM_PER_TIMER7_CLKCTRL, 7);

  // Set all the timer input clocks to 24MHz
  cm[CLKSEL_TIMER4_CLK/4] = CLKSEL_CLK_M_OSC;
  cm[CLKSEL_TIMER5_CLK/4] = CLKSEL_CLK_M_OSC;
  cm[CLKSEL_TIMER6_CLK/4] = CLKSEL_CLK_M_OSC;
  cm[CLKSEL_TIMER7_CLK/4] = CLKSEL_CLK_M_OSC;

  munmap((void*)cm, CM_SIZE);
  return 1;
}

bool pwm_timers_map() {
  bool ret = false;
  int fd;

  memset(timers, 0x00, sizeof(*timers));

  fd = open("/dev/mem", O_RDWR);
  if (fd == -1) { perror("open()"); return ret; }

  if (!pwm_timers_enable_clocks(fd))    goto exit;

  timers[0].regs = map_port(fd, TIMER_MMAP_SIZE, TIMER4_BASE);
  if (timers[0].regs == MAP_FAILED) { perror("mmap() TIMER4"); goto exit; }
  timers[1].regs = map_port(fd, TIMER_MMAP_SIZE, TIMER5_BASE);
  if (timers[1].regs == MAP_FAILED) { perror("mmap() TIMER5"); goto exit; }
  timers[2].regs = map_port(fd, TIMER_MMAP_SIZE, TIMER6_BASE);
  if (timers[2].regs == MAP_FAILED) { perror("mmap() TIMER6"); goto exit; }
  timers[3].regs = map_port(fd, TIMER_MMAP_SIZE, TIMER7_BASE);
  if (timers[3].regs == MAP_FAILED) { perror("mmap() TIMER7"); goto exit; }

  ret = true;

 exit:
  close(fd);
  if (!ret)
    pwm_timers_unmap();
  return ret;
}

void pwm_timers_unmap() {
  int i;
  for (i = 0; i < 4; i++) {
    struct pwm_timer_data *timer = &timers[i];
    if (timer->regs) munmap((void*)timer->regs, TIMER_MMAP_SIZE);
    timer->regs = NULL;
  }
}
