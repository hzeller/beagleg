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

#ifndef __GENERIC_GPIO_H
#define __GENERIC_GPIO_H

#include <stdint.h>

#include "motor-interface-constants.h"

int get_gpio(uint32_t gpio_def);

void set_gpio(uint32_t gpio_def);
void clr_gpio(uint32_t gpio_def);

bool map_gpio();
void unmap_gpio();

#endif  // __GENERIC_GPIO_H
