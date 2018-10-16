/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
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

// Initializing the configuration.

#include "gcode-machine-control.h"

#include <stdlib.h>

#include "common/logging.h"
#include "common/string-util.h"

#include "config-parser.h"
#include "motor-operations.h"

// Default order in which axes should be homed.
static const char kHomeOrder[] = "ZXY";

MachineControlConfig::MachineControlConfig() {
  speed_factor = 1;
  acknowledge_lines = true;
  debug_print = false;
  synchronous = false;
  range_check = true;
  clamp_to_range = "";
  require_homing = true;
  enable_pause = false;
  home_order = kHomeOrder;
  threshold_angle = -1;
  speed_tune_angle = 0;
  auto_motor_disable_seconds = -1;
  auto_fan_disable_seconds = -1;
  auto_fan_pwm = 0;
}

namespace {
// Expermential. Work in progress.
class MachineControlConfigReader : public ConfigParser::Reader {
public:
  MachineControlConfigReader(MachineControlConfig *config)
    : config_(config) {}

  bool SeenSection(int line_no, const std::string &section_name) final {
    current_section_ = section_name;
    if (section_name == "general")
      return true;

    // See if this is a valid axis section.
    current_axis_ = gcodep_letter2axis(section_name[0]);
    if (current_axis_ != GCODE_NUM_AXES && section_name.substr(1) == "-axis") {
      return true;
    } else {
      current_axis_ = GCODE_NUM_AXES;
    }

    return false;
  }

  bool SeenNameValue(int line_no,
                     const std::string &name,
                     const std::string &value) final {
#define ACCEPT_VALUE(n, T, result) if (name != n) {} else return Parse##T(value, result)
#define ACCEPT_EXPR(n, result) if (name != n) {} else return ParseFloatExpr(value, result)

    if (current_section_ == "general") {
      ACCEPT_VALUE("home-order",     String, &config_->home_order);
      ACCEPT_VALUE("require-homing", Bool,   &config_->require_homing);
      ACCEPT_VALUE("range-check",    Bool,   &config_->range_check);
      ACCEPT_VALUE("clamp-to-range", String, &config_->clamp_to_range);
      ACCEPT_VALUE("synchronous",    Bool,   &config_->synchronous);
      ACCEPT_VALUE("enable-pause",   Bool,   &config_->enable_pause);
      ACCEPT_VALUE("auto-motor-disable-seconds",
                   Int,   &config_->auto_motor_disable_seconds);
      ACCEPT_VALUE("auto-fan-disable-seconds",
                   Int,  &config_->auto_fan_disable_seconds);
      ACCEPT_VALUE("auto-fan-pwm",   Int,    &config_->auto_fan_pwm);
      return false;
    }

    if (current_axis_ != GCODE_NUM_AXES) {
      ACCEPT_EXPR("steps-per-mm",     &config_->steps_per_mm[current_axis_]);
      ACCEPT_EXPR("steps-per-degree", &config_->steps_per_mm[current_axis_]);

      ACCEPT_EXPR("max-feedrate",     &config_->max_feedrate[current_axis_]);
      ACCEPT_EXPR("max-anglerate",    &config_->max_feedrate[current_axis_]);

      ACCEPT_EXPR("max-probe-feedrate", &config_->max_probe_feedrate[current_axis_]);

      ACCEPT_EXPR("max-acceleration", &config_->acceleration[current_axis_]);

      ACCEPT_EXPR("range",            &config_->move_range_mm[current_axis_]);

      if (name == "home-pos")
        return SetHomePos(line_no, current_axis_, value);
    }
    ReportError(line_no, StringPrintf("Unexpected configuration option '%s'",
                                      name.c_str()));
#undef ACCEPT_VALUE
#undef ACCEPT_EXPR
    return false;
  }

  void ReportError(int line_no, const std::string &msg) final {
    Log_error("Line %d: %s", line_no, msg.c_str());
  }

private:
  bool SetHomePos(int line_no, enum GCodeParserAxis axis,
                  const std::string &value) {
    if (config_->homing_trigger[axis] != HardwareMapping::TRIGGER_NONE) {
      ReportError(line_no, StringPrintf("home-pos[%c] already configured before ",
                                        gcodep_axis2letter(axis)));
      return false;
    }
    const std::string choice = ToLower(value);
    if (choice == "min") {
      config_->homing_trigger[axis] = HardwareMapping::TRIGGER_MIN;
    }
    else if (choice == "max") {
      config_->homing_trigger[axis] = HardwareMapping::TRIGGER_MAX;
      return true;
    }
    else {
      ReportError(line_no,
                  StringPrintf("home-pos[%c]: valid values are 'min' or 'max', but got '%s'",
                               gcodep_axis2letter(axis), value.c_str()));
      return false;
    }
    return true;
  }

  MachineControlConfig *const config_;
  enum GCodeParserAxis current_axis_;
  std::string current_section_;
};
}

bool MachineControlConfig::ConfigureFromFile(ConfigParser *parser) {
  MachineControlConfigReader reader(this);
  return parser->EmitConfigValues(&reader);
}
