/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
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

#ifndef __PWM_TIMER_H
#define __PWM_TIMER_H

#include <stdint.h>

#include "motor-interface-constants.h"

// Start/stop the PWM timer
void pwm_timer_start(uint32_t gpio_def, bool start);

// Set the PWM timer duty cycle
void pwm_timer_set_duty(uint32_t gpio_def, float duty_cycle);
void pwm_timer_set_freq(uint32_t gpio_def, int pwm_freq);

bool pwm_timers_map();
void pwm_timers_unmap();

#endif  // __PWM_TIMER_H
