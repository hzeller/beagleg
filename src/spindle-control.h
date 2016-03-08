/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2016 H Hartley Sweeten <hsweeten@visionengravers.com>
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

#ifndef BEAGLEG_SPINDLE_CONTROL_
#define BEAGLEG_SPINDLE_CONTROL_

#include "string-util.h"

class HardwareMapping;
class ConfigParser;

class Spindle {
public:
  Spindle();
  ~Spindle() {}

   bool ConfigureFromFile(ConfigParser *parser);

   bool Init(HardwareMapping *hardware_mapping);

   // Turn spindle on clockwise (M3) or counterclockwise (M4) at speed (Sxx)
   void On(bool ccw, int rpm);
   // Turn spindle off (M5)
   void Off();

// FIXME: why can't this be private?
  class Impl;
  Impl *impl_;

private:
  class ConfigReader;

  std::string type_;
  std::string port_;

  int max_rpm_;
  int pwr_delay_ms_;
  int on_delay_ms_;
  int off_delay_ms_;
  bool allow_ccw_;
};

#endif  // BEAGLEG_SPINDLE_CONTROL_
