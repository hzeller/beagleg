/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
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

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "motor-interface-constants.h"
#include "generic-gpio.h"

// Memory space mapped to the Clock Module registers
#define CM_BASE                 0x44e00000
#define CM_SIZE                 0x4000

// Clock Module Peripheral and Wakeup registers
#define CM_WKUP_GPIO0_CLKCTRL   (0x400 + 0x008)
#define CM_PER_GPIO1_CLKCTRL    (0x000 + 0x0ac)
#define CM_PER_GPIO2_CLKCTRL    (0x000 + 0x0b0)
#define CM_PER_GPIO3_CLKCTRL    (0x000 + 0x0b4)

#define IDLEST_MASK             (0x03 << 16)
#define MODULEMODE_ENABLE       (0x02 << 0)

#define GPIO_MMAP_SIZE 0x2000

#define MOTOR_OUT_BITS				\
  ((uint32_t) ( (1<<MOTOR_1_STEP_BIT)		\
		| (1<<MOTOR_2_STEP_BIT)		\
		| (1<<MOTOR_3_STEP_BIT)		\
		| (1<<MOTOR_4_STEP_BIT)		\
		| (1<<MOTOR_5_STEP_BIT)		\
		| (1<<MOTOR_6_STEP_BIT)		\
		| (1<<MOTOR_7_STEP_BIT)		\
		| (1<<MOTOR_8_STEP_BIT) ))

// Direction bits are a contiguous chunk, just a bit shifted.
#define DIRECTION_OUT_BITS ((uint32_t) (0xFF << DIRECTION_GPIO1_SHIFT))

// GPIO registers.
static volatile uint32_t *gpio_0 = NULL;
static volatile uint32_t *gpio_1 = NULL;
static volatile uint32_t *gpio_2 = NULL;
static volatile uint32_t *gpio_3 = NULL;

void set_motor_ena() {
  gpio_1[GPIO_DATAOUT/4] = (1 << MOTOR_ENABLE_GPIO1_BIT);
}

void clr_motor_ena() {
  gpio_1[GPIO_DATAOUT/4] = 0;
}

static volatile uint32_t *map_port(int fd, size_t length, off_t offset) {
  return mmap(0, length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
}

static void ena_gpio_clk(volatile uint32_t *cm, uint32_t reg, int bank) {
  uint32_t val;

  val = cm[reg/4];
  if (val & IDLEST_MASK) {
    fprintf(stderr, "Enabling GPIO-%d clock", bank);
    val |= MODULEMODE_ENABLE;
    cm[reg/4] = val;
    do {
      fprintf(stderr, ".");
      val = cm[reg/4];
    } while (val & IDLEST_MASK);
    fprintf(stderr, "\n");
  }
}

static int enable_gpio_clocks(int fd) {
  volatile uint32_t *cm;

  cm = map_port(fd, CM_SIZE, CM_BASE);
  if (cm == MAP_FAILED) { perror("mmap() CM"); return 0; }

  ena_gpio_clk(cm, CM_WKUP_GPIO0_CLKCTRL, 0);
  ena_gpio_clk(cm, CM_PER_GPIO1_CLKCTRL, 1);
  ena_gpio_clk(cm, CM_PER_GPIO2_CLKCTRL, 2);
  ena_gpio_clk(cm, CM_PER_GPIO3_CLKCTRL, 3);

  munmap((void*)cm, CM_SIZE);
  return 1;
}

int map_gpio() {
  int ret = 0;
  int fd;

  fd = open("/dev/mem", O_RDWR);
  if (fd == -1) { perror("open()"); return ret; }

  if (!enable_gpio_clocks(fd))	goto exit;

  gpio_0 = map_port(fd, GPIO_MMAP_SIZE, GPIO_0_BASE);
  if (gpio_0 == MAP_FAILED) { perror("mmap() GPIO-0"); goto exit; }
  gpio_1 = map_port(fd, GPIO_MMAP_SIZE, GPIO_1_BASE);
  if (gpio_1 == MAP_FAILED) { perror("mmap() GPIO-1"); goto exit; }
  gpio_2 = map_port(fd, GPIO_MMAP_SIZE, GPIO_2_BASE);
  if (gpio_2 == MAP_FAILED) { perror("mmap() GPIO-2"); goto exit; }
  gpio_3 = map_port(fd, GPIO_MMAP_SIZE, GPIO_3_BASE);
  if (gpio_3 == MAP_FAILED) { perror("mmap() GPIO-3"); goto exit; }

  // Prepare all the pins we need for output. All the other bits are inputs,
  // so the STOP switch bits are automatically prepared for input.
  gpio_0[GPIO_OE/4] = ~(MOTOR_OUT_BITS | (1 << AUX_1_BIT) | (1 << AUX_2_BIT));
  gpio_1[GPIO_OE/4] = ~(DIRECTION_OUT_BITS | (1 << MOTOR_ENABLE_GPIO1_BIT));

  ret = 1;

exit:
  close(fd);
  if (!ret)
    unmap_gpio();
  return ret;
}

void unmap_gpio() {
  if (gpio_0) { munmap((void*)gpio_0, GPIO_MMAP_SIZE); gpio_0 = NULL; }
  if (gpio_1) { munmap((void*)gpio_1, GPIO_MMAP_SIZE); gpio_1 = NULL; }
  if (gpio_2) { munmap((void*)gpio_2, GPIO_MMAP_SIZE); gpio_2 = NULL; }
  if (gpio_3) { munmap((void*)gpio_3, GPIO_MMAP_SIZE); gpio_3 = NULL; }
}
