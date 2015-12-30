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
      if (name == "motor_1") return SetMotorAxis(line_no, 1, value);
      if (name == "motor_2") return SetMotorAxis(line_no, 2, value);
      if (name == "motor_3") return SetMotorAxis(line_no, 3, value);
      if (name == "motor_4") return SetMotorAxis(line_no, 4, value);
      if (name == "motor_5") return SetMotorAxis(line_no, 5, value);
      if (name == "motor_6") return SetMotorAxis(line_no, 6, value);
      if (name == "motor_7") return SetMotorAxis(line_no, 7, value);
      if (name == "motor_8") return SetMotorAxis(line_no, 8, value);
    }

    if (current_axis_ != GCODE_NUM_AXES) {
      ACCEPT_EXPR("steps-per-mm",     &config_->steps_per_mm[current_axis_]);
      ACCEPT_EXPR("steps-per-degree", &config_->steps_per_mm[current_axis_]);

      ACCEPT_EXPR("range",            &config_->move_range_mm[current_axis_]);

      ACCEPT_EXPR("max-feedrate",     &config_->max_feedrate[current_axis_]);
      ACCEPT_EXPR("max-anglerate",    &config_->max_feedrate[current_axis_]);

      ACCEPT_EXPR("max-acceleration", &config_->acceleration[current_axis_]);

      if (name == "home-pos")
        return SetHomePos(line_no, current_axis_, value);

      if (name == "motor-connector")
        return SetMotorConnector(line_no, current_axis_, value);
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

  bool SetMotorConnector(int line_no, enum GCodeParserAxis axis,
                         const StringPiece &value) {
    // We can have multiple motors connected. Parameters separated with ';'
    std::vector<StringPiece> motor_params = SplitString(value, ";");
    for (size_t i = 0; i < motor_params.size(); ++i) {
      if (!SetSingleMotor(line_no, axis, motor_params[i]))
        return false;
    }
    return true;
  }
  
  bool SetSingleMotor(int line_no, enum GCodeParserAxis axis,
                      const StringPiece &parameters) {
    // We have various parameters that we have per motor.
    bool is_mirrored = false;
    int motor_number = -1;
    std::vector<StringPiece> sub_parts = SplitString(parameters, " \t");
    for (size_t i = 0; i < sub_parts.size(); ++i) {
      // Parse a config in the form 'motor:1 mirror'
      StringPiece value = TrimWhitespace(sub_parts[i]);
      if (HasPrefix(value, "motor:")) {
        const int m = ParseDecimal(value.substr(strlen("motor:")), -1);
        if (m < 0) {
          std::string v = value.ToString();
          Log_error("Line %d: Expected motor-number after 'motor:' (%s)",
                    line_no, v.c_str());
          return false;
        }

        if (motor_number < 0) {
          motor_number = m;
        } else {
          Log_error("Line %d: Multiple motors (%d and %d) in motor-connector: "
                    "did you forget ';'-separator ?", line_no,
                    motor_number, m);
          return false;
        }
      }
      else if (value == "mirror") {
        is_mirrored = true;
      } else if (!value.empty()) {
        Log_error("Line %d: Don't know how to deal with '%s'. Typo ?", line_no,
                  value.ToString().c_str());
        return false;
      }
      // microstepping ...
    }

    if (motor_number < 1 || motor_number > BEAGLEG_NUM_MOTORS) {
      Log_error("Line %d: There needs to be a motor:<num> field "
                "with num := 1..%d; but got %d",
                line_no, BEAGLEG_NUM_MOTORS, motor_number);
      return false;
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
    const char axis_letter = gcodep_axis2letter(axis);
    config_->axis_mapping[motor_number-1] = (is_mirrored
                                             ? tolower(axis_letter)
                                             : toupper(axis_letter));
    return true;
  }

  bool SetMotorAxis(int line_no, int motor_number, const std::string &value) {
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
