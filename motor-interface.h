/*
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

#include <stdio.h>

struct bg_movement {
  // Speed is steps/second of the axis with the highest number of steps. All
  // other axes are scaled down accordingly.
  float start_speed;
  float travel_speed;
  float end_speed;

  int steps[8];   // number of steps for axis. Negative for 'backwards'.
};

// Initialize beagleg motor control. Initializes the acceleration in steps/s^2.
//  Returns 0 on success, 1 on some error.
int beagleg_init(float acceleration_steps_s2);
void beagleg_exit(void);  // shutdown motor control.
// shutdown motor control immediately, don't wait for current queue to empty.
void beagleg_exit_nowait(void);

// Enqueue a coordinated move command.
// If there is space in the ringbuffer, this function returns immediately,
// otherwise it waits until a slot frees up.
// Returns 0 on success, 1 if this is a no-op with no steps to move and 2 if
// number of steps per single command is exceeded (right now, 64k). If
// err_stream is non-NULL, prints error message there.
int beagleg_enqueue(const struct bg_movement *param, FILE *err_stream);

// Wait, until all elements in the ring-buffer are consumed.
void beagleg_wait_queue_empty(void);
