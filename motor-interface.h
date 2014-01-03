/*
 * (c) 2013, 1014 Henner Zeller <h.zeller@acm.org>
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

struct bg_movement {
  // Speed is steps/second of the axis with the highest number of steps. All
  // other axis are scaled accordingly.
  float start_speed;
  float travel_speed;
  float end_speed;

  int steps[8];   // number of steps for axis. Negative for 'backwards'
};

int beagleg_init(void); 
int beagleg_enqueue(const struct bg_movement *param);

void beagleg_wait_queue_empty(void);
void beagleg_exit(void);
