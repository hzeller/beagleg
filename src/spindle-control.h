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

#include "common/string-util.h"

class HardwareMapping;
class ConfigParser;

// Spindle configuration as read from config file.
struct SpindleConfig {
  SpindleConfig();
  bool ConfigureFromFile(ConfigParser *parser);

  std::string type;
  std::string port;
  // TODO: other config options e.g. needed for modbus spindles.

  int max_rpm;
  int pwr_delay_ms;
  int on_delay_ms;
  int off_delay_ms;
  bool allow_ccw;
};

class Spindle {
public:
  // Factory for a spindle given the configuration. Returns an instance
  // of a spindle if it can be created given the available hardware-mapping.
  // Otherwise, returns nullptr and might info-log why it couldn't create a
  // spindle (which might be fine for many set-ups, e.g. 3D printers).
  static Spindle *CreateFromConfig(const SpindleConfig &config,
                                   HardwareMapping *hardware_mapping);

  virtual ~Spindle() {}

   // Turn spindle on clockwise (M3) or counterclockwise (M4) at speed (Sxx)
  virtual void On(bool ccw, int rpm) = 0;

   // Turn spindle off (M5)
  virtual void Off() = 0;
};

#endif  // BEAGLEG_SPINDLE_CONTROL_
