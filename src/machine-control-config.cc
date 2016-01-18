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

#include "config-parser.h"
#include "logging.h"
#include "motor-operations.h"
#include "string-util.h"

// Default order in which axes should be homed.
static const char kHomeOrder[] = "ZXY";

MachineControlConfig::MachineControlConfig() {
  speed_factor = 1;
  acknowledge_lines = true;
  debug_print = false;
  synchronous = false;
  range_check = true;
  require_homing = true;
  home_order = kHomeOrder;
  threshold_angle = -1;
  auto_motor_disable_seconds = -1;
}

namespace {
// Expermential. Work in progress.
class MachineControlConfigReader : public ConfigParser::EventReceiver {
public:
  MachineControlConfigReader(MachineControlConfig *config)
    : config_(config) {}

  virtual bool SeenSection(int line_no, const std::string &section_name) {
    current_section_ = section_name;
    if (section_name == "general"
        || section_name == "motor-mapping"
        || section_name == "switch-mapping")
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

  virtual bool SeenNameValue(int line_no,
                             const std::string &name,
                             const std::string &value) {
#define ACCEPT_VALUE(n, T, result) if (name != n) {} else return Parse##T(value, result)
#define ACCEPT_EXPR(n, result) if (name != n) {} else return ParseFloatExpr(value, result)

    if (current_section_ == "general") {
      ACCEPT_VALUE("home-order",     String, &config_->home_order);
      ACCEPT_VALUE("require-homing", Bool,   &config_->require_homing);
      ACCEPT_VALUE("range-check",    Bool,   &config_->range_check);
      ACCEPT_VALUE("synchronous",    Bool,   &config_->synchronous);
      ACCEPT_VALUE("auto-motor-disable-seconds",
                   Int,   &config_->auto_motor_disable_seconds);
      return false;
    }

    if (current_section_ == "motor-mapping") {
      for (int i = 1; i <= BEAGLEG_NUM_MOTORS; ++i) {
        if (name == StringPrintf("motor_%d", i))
          return SetMotorAxis(line_no, i, value);
      }
      return false;
    }

    if (current_section_ == "switch-mapping") {
      for (int i = 1; i <= BEAGLEG_NUM_SWITCHES; ++i) {
        if (name == StringPrintf("switch_%d", i)) {
          return SetSwitchOptions(line_no, i, value);
        }
      }
      return false;
    }

    if (current_axis_ != GCODE_NUM_AXES) {
      ACCEPT_EXPR("steps-per-mm",     &config_->steps_per_mm[current_axis_]);
      ACCEPT_EXPR("steps-per-degree", &config_->steps_per_mm[current_axis_]);

      ACCEPT_EXPR("max-feedrate",     &config_->max_feedrate[current_axis_]);
      ACCEPT_EXPR("max-anglerate",    &config_->max_feedrate[current_axis_]);

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

  virtual void ReportError(int line_no, const std::string &msg) {
    Log_error("Line %d: %s", line_no, msg.c_str());
  }

private:
  // All the Accept() functions are done in the way that they always return
  // 'true' if the expected name is not matched, otherwise they return the
  // outcome of parsing the value. That way, they can be chained with &&
  bool ParseString(const std::string &value, std::string *result) {
    *result = value;
    return true;
  }

  bool ParseInt(const std::string &value, int *result) {
    char *end;
    *result = strtol(value.c_str(), &end, 10);
    return *end == '\0';
  }

  bool ParseBool(const std::string &value, bool *result) {
    if (value == "1" || value == "yes" || value == "true") {
      *result = true;
      return true;
    }
    if (value == "0" || value == "no" || value == "false") {
      *result = false;
      return true;
    }
    return false;
  }

  bool ParseFloatExpr(const std::string &value, float *result) {
    char *end;
    double eval = ParseDoubleExpression(value.c_str(), 1.0, &end);
    if (end == NULL || *end == '\0') {
      *result = eval;
      return true;
    }
    return false;
  }

  static double ParseDoubleExpression(const char *input, double fallback,
                                      char **end) {
    const char *full_expr = input;
    double value = strtod(input, end);
    if (*end == input) return fallback;
    for (;;) {
      while (isspace(**end)) ++*end;
      const char op = **end;
      if (op != '/' && op != '*') {
        return value;  // done. Not an operation.
      }
      ++*end;
      while (isspace(**end)) ++*end;
      input = *end;
      double operand;
      if (*input == '(') {
        operand = ParseDoubleExpression(input+1, 1.0, end);
        if (**end != ')') {
          fprintf(stderr, "Mismatching parenthesis in '%s'\n", full_expr);
          return fallback;
        } else {
          ++*end;
        }
      } else {
        operand = strtod(input, end);
      }
      if (*end == input) return fallback;
      if (op == '/')
        value /= operand;
      else if (op == '*')
        value *= operand;
    }
    return value;
  }

  bool SetHomePos(int line_no, enum GCodeParserAxis axis,
                  const std::string &value) {
    const std::string choice = ToLower(value);
    if (choice == "min") {
      if (config_->max_endstop_[axis].homing_use) {
        ReportError(line_no,
                    StringPrintf("home-pos[%c] Prior max configured as endstop.",
                                 gcodep_axis2letter(axis)));
        return false;
      }
      config_->min_endstop_[axis].homing_use = true;
      return true;
    } else if (choice == "max") {
      if (config_->min_endstop_[axis].homing_use) {
        ReportError(line_no,
                    StringPrintf("home-pos[%c] Prior min configured as endstop.",
                                 gcodep_axis2letter(axis)));
        return false;
      }
      config_->max_endstop_[axis].homing_use = true;
      return true;
    }
    ReportError(line_no,
                StringPrintf("home-pos[%c]: valid values are 'min' or 'max', but got '%s'",
                             gcodep_axis2letter(axis), value.c_str()));
    return false;
  }

  bool SetMotorAxis(int line_no, int motor_number, const std::string &in_val) {
    std::vector<StringPiece> options = SplitString(in_val, " \t,");
    for (size_t i = 0; i < options.size(); ++i) {
      const std::string option = ToLower(options[i]);
      if (HasPrefix(option, "axis:")) {
        std::string axis = option.substr(strlen("axis:"));
        if (axis.empty()) return false;
        bool is_negative = false;
        if (axis[0] == '-') {
          is_negative = true;
          axis = axis.substr(1);
        }
        if (axis.empty()) return false;
        const char axis_letter = axis[0];
        if (gcodep_letter2axis(axis_letter) == GCODE_NUM_AXES) {
          Log_error("Invalid axis letter '%c'", axis_letter);
          return false; // invalid axis.
        }

        // Now, we copy it to the somehwat old command-line friendly
        // configuration which encodes that as a string. We do that now for
        // compatibility, but later we should decommission this.
        if (config_->axis_mapping.length() < (size_t)motor_number) {
          config_->axis_mapping.resize(motor_number, '_');
        }
        if (config_->axis_mapping[motor_number-1] != '_') {
          Log_error("Line %d: Attempt to use motor twice: Motor %d already "
                    "mapped to axis %c", line_no, motor_number,
                    config_->axis_mapping[motor_number-1]);
          return false;
        }
        config_->axis_mapping[motor_number-1] = (is_negative
                                                 ? tolower(axis_letter)
                                                 : toupper(axis_letter));
      }
      // TODO: maybe option 'mirror'
      else {
        Log_error("Line %d: Unknown motor option '%s'", line_no, option.c_str());
        return false;
      }
    }
    return true;
  }

  bool SetSwitchOptions(int line_no, int switch_number,
                        const std::string &value) {
    std::vector<StringPiece> options = SplitString(value, " \t,");
    for (size_t i = 0; i < options.size(); ++i) {
      const std::string option = ToLower(options[i]);
      if (option.empty()) continue;
      if (option.length() == 5) {
        const GCodeParserAxis axis = gcodep_letter2axis(option[4]);
        if (axis == GCODE_NUM_AXES) {
          Log_error("Line %d: Expect min_<axis> or max_<axis>, e.g. min_x. Last character not a valid axis (got %s).", line_no, option.c_str());
          return false;
        }

        if (HasPrefix(option, "min_")) {
          config_->min_endstop_[axis].endstop_switch = switch_number;
        } else if (HasPrefix(option, "max_")) {
          config_->max_endstop_[axis].endstop_switch = switch_number;
        } else {
          Log_error("Line %d: Expect min_<axis> or max_<axis>, e.g. min_x. Got %s", line_no, option.c_str());
          return false;
        }
      }
      else if (option == "active:low") {
        config_->trigger_level_[switch_number - 1] = false;
      }
      else if (option == "active:high") {
        config_->trigger_level_[switch_number - 1] = true;
      } else {
        return false;
      }
    }

    return true;  // All went fine.
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
