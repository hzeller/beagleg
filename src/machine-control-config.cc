/* -*- mode: c++ c-basic-offset: 2; indent-tabs-mode: nil; -*-
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
#include "string-util.h"
#include "logging.h"

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
}

namespace {
// Expermential. Work in progress.
class MachineControlConfigReader : public ConfigParser::EventReceiver {
public:
  MachineControlConfigReader(MachineControlConfig *config) : config_(config){}

  virtual bool SeenSection(int line_no, const std::string &section_name) {
    in_general_ = (section_name == "general");
    in_motor_mapping_ = (section_name == "motor-mapping");
    if (in_general_ || in_motor_mapping_)
      return true;

    current_axis_ = gcodep_letter2axis(section_name[0]);
    if (current_axis_ != GCODE_NUM_AXES
        && section_name.substr(1) == "-axis") {
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

    if (in_general_) {
      ACCEPT_VALUE("home-order",     String, &config_->home_order);
      ACCEPT_VALUE("require-homing", Bool,   &config_->require_homing);
      ACCEPT_VALUE("range-check",    Bool,   &config_->range_check);
      ACCEPT_VALUE("synchronous",    Bool,   &config_->synchronous);
    }

    if (in_motor_mapping_) {
      if (name == "motor_1") return SetMotorAxis(1, value);
      if (name == "motor_2") return SetMotorAxis(2, value);
      if (name == "motor_3") return SetMotorAxis(3, value);
      if (name == "motor_4") return SetMotorAxis(4, value);
      if (name == "motor_5") return SetMotorAxis(5, value);
      if (name == "motor_6") return SetMotorAxis(6, value);
      if (name == "motor_7") return SetMotorAxis(7, value);
      if (name == "motor_8") return SetMotorAxis(8, value);
    }

    if (current_axis_ != GCODE_NUM_AXES) {
      ACCEPT_EXPR("steps-per-mm",     &config_->steps_per_mm[current_axis_]);
      ACCEPT_EXPR("steps-per-degree", &config_->steps_per_mm[current_axis_]);

      ACCEPT_EXPR("range",            &config_->move_range_mm[current_axis_]);

      ACCEPT_EXPR("max-feedrate",     &config_->max_feedrate[current_axis_]);
      ACCEPT_EXPR("max-anglerate",    &config_->max_feedrate[current_axis_]);

      ACCEPT_EXPR("max-acceleration", &config_->acceleration[current_axis_]);

      if (name == "home-pos") {
        const std::string choice = ToLower(value);
        if (choice == "min") {
          if (config_->max_endstop_[current_axis_].homing_use) {
            ReportError(line_no,
                        StringPrintf("home-pos[%c] Prior max configured as endstop.",
                                     gcodep_axis2letter(current_axis_)));
            return false;
          }
          config_->min_endstop_[current_axis_].homing_use = true;
          return true;
        } else if (choice == "max") {
          if (config_->min_endstop_[current_axis_].homing_use) {
            ReportError(line_no,
                        StringPrintf("home-pos[%c] Prior min configured as endstop.",
                                     gcodep_axis2letter(current_axis_)));
            return false;
          }
          config_->max_endstop_[current_axis_].homing_use = true;
          return true;
        } else {
          ReportError(line_no,
                      StringPrintf("home-pos[%c]: valid values are 'min' or 'max', but got '%s'",
                                   gcodep_axis2letter(current_axis_), value.c_str()));
          return false;
        }
      }
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

  bool SetMotorAxis(int motor_number, const std::string &value) {
    StringPiece axis(value);
    if (HasPrefix(ToLower(axis), "axis:")) {
      axis = axis.substr(strlen("axis:"));
    }
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
    // This is somehwat an alignment with the old, command-line based
    // configuration, that should change to be more config-file friendly.
    if (config_->axis_mapping.length() < (size_t)motor_number) {
      config_->axis_mapping.resize(motor_number, '_');
    }
    config_->axis_mapping[motor_number-1] = (is_negative
                                             ? tolower(axis_letter)
                                             : toupper(axis_letter));
    return true;
  }

  MachineControlConfig *const config_;
  enum GCodeParserAxis current_axis_;
  bool in_general_;
  bool in_motor_mapping_;
};
}

bool MachineControlConfig::InitializeFromFile(ConfigParser *parser) {
  MachineControlConfigReader reader(this);
  return parser->EmitConfigValues(&reader);
}
