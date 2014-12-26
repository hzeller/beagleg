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
#ifndef _BEAGLEG_MOTOR_INTERFACE_H_
#define _BEAGLEG_MOTOR_INTERFACE_H_
#include <stdio.h>

enum {
  BEAGLEG_NUM_MOTORS = 8
};

struct bg_movement {
  // Speed is steps/second of the axis with the highest number of steps, which
  // is the fastest axis. All other axes are scaled down accordingly.
  float start_speed;
  float travel_speed;
  float end_speed;

  float acceleration;       // steps/s^2 for fastest axis.

  unsigned char aux_bits;   // Aux-bits to switch.

  int steps[BEAGLEG_NUM_MOTORS]; // Steps for axis. Negative for reverse.
};

struct MotorControl {
  // Waits for the queue to be empty and Enables/disables motors according to the
  // given boolean value (Right now, motors cannot be individually addressed).
  void (*motor_enable)(char on);
  
  // Enqueue a coordinated move command.
  // If there is space in the ringbuffer, this function returns immediately,
  // otherwise it waits until a slot frees up.
  // Returns 0 on success, 1 if this is a no-op with no steps to move and 2 on
  // invalid parameters.
  // If "err_stream" is non-NULL, prints error message there.
  // Automatically enables motors if not already.
  int (*enqueue)(const struct bg_movement *param, FILE *err_stream);

  // Wait, until all elements in the ring-buffer are consumed.
  void (*wait_queue_empty)(void);
};

// Initialize beagleg pru motor control. Initializes operations in
// the given struct.
// This is essentially a singleton.
// Gets min value of acceleration expected to do range-checks.
//  Returns 0 on success, 1 on some error.
int beagleg_pru_init_motor_control(struct MotorControl *control);

// Shutdown motor control for good.
void beagleg_pru_exit();
void beagleg_pru_exit_nowait();

// Create a dummy motor control
void init_dummy_motor_control(struct MotorControl *control);

#endif  // _BEAGLEG_MOTOR_INTERFACE_H_
