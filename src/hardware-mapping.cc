/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2016 Henner Zeller <h.zeller@acm.org>
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

#include "hardware-mapping.h"

#include <unistd.h>
#include <sys/types.h>

#include "config-parser.h"
#include "logging.h"
#include "generic-gpio.h"
#include "pwm-timer.h"
#include "motor-operations.h"  // LinearSegmentSteps
#include "string-util.h"

HardwareMapping::HardwareMapping() : is_initialized_(false) {
}

bool HardwareMapping::AddMotorMapping(LogicAxis axis, int motor,
                                      bool mirrored) {
  if (motor == 0) return true;  // allowed null-mapping
  if (motor < 1 || motor > NUM_MOTORS) {
    Log_error("Motor %d out of range [%d..%d]", motor, 1, NUM_MOTORS);
    return false;
  }
  if (driver_flip_[motor-1] != 0) {
    Log_error("Motor %d mapped more than once.", motor);
    return false;
  }
  axis_to_driver_[axis] |= 1 << (motor-1);
  driver_flip_[motor-1] = mirrored ? -1 : 1;
  return true;
}

bool HardwareMapping::AddAuxMapping(LogicOutput output, int aux) {
  if (aux == 0) return true;  // allowed null-mapping
  if (aux < 1 || aux > NUM_BOOL_OUTPUTS) {
    Log_error("Aux %d out of range [%d..%d]", aux, 1, NUM_BOOL_OUTPUTS);
    return false;
  }
  const uint32_t gpio_descriptor = get_aux_bit_gpio_descriptor(aux);
  if (gpio_descriptor == GPIO_NOT_MAPPED) {
    Log_info("Mapping '%s' to aux %d which has no hardware connector for %s"
             "; output will be ignored for this cape.",
             OutputToName(output), aux, CAPE_NAME);
    return true;  // This is not fatal, but no need to prepare these bits.
  }
  output_to_aux_bits_[output] |= 1 << (aux-1);
  return true;
}

bool HardwareMapping::AddPWMMapping(LogicOutput output, int pwm) {
  if (pwm == 0) return true;  // allowedd null-mapping
  if (pwm < 1 || pwm > NUM_PWM_OUTPUTS) {
    Log_error("PWM %d out of range [%d..%d]", pwm, 1, NUM_BOOL_OUTPUTS);
    return false;
  }
  if (output_to_pwm_gpio_[output] != GPIO_NOT_MAPPED) {
    // TODO: do we want to allow 1:n mapping, same output, multiple pins ?
    Log_error("Attempt to map '%s' which is already "
              "mapped to different pin", OutputToName(output));
    return false;
  }
  const uint32_t gpio_descriptor = get_pwm_gpio_descriptor(pwm);
  if (gpio_descriptor == GPIO_NOT_MAPPED) {
    Log_info("Mapping '%s' to pwm %d which has no hardware connector for %s"
             " ; output will be ignored for this cape.",
             OutputToName(output), pwm, CAPE_NAME);
  }
  output_to_pwm_gpio_[output] = gpio_descriptor;
  return true;
}

bool HardwareMapping::InitializeHardware() {
  assert(is_configured_);  // Call sequence error. Needs to be configured first.
  if (geteuid() != 0) {
    Log_error("Need to run as root to access GPIO pins.");
    return false;
  }
  if (!map_gpio()) {
    Log_error("Couldn't mmap() GPIO ranges.\n");
    return false;
  }
  if (!pwm_timers_map()) {
    Log_error("Couldn't mmap() TIMER ranges.\n");
    return false;
  }

  // The PWM_*_GPIO pins can produce PWM signals if they are mapped to one
  // of the TIMER pins and the dts set the pins to the correct mode (0x02).
  // If they are mapped to other pins, or the mode is wrong (0x07), they will
  // only work as GPIO outputs. Either way the pwm_timer_*() calls are safe.
  //
  // Make sure all the PWM timers are stopped and set the default base frequency.
  // (TODO: only initialize PWMs that are actually needed according to the
  // configuration).
  pwm_timer_start(PWM_1_GPIO, 0);
  pwm_timer_start(PWM_2_GPIO, 0);
  pwm_timer_start(PWM_3_GPIO, 0);
  pwm_timer_start(PWM_4_GPIO, 0);
  pwm_timer_set_freq(PWM_1_GPIO, 0);
  pwm_timer_set_freq(PWM_2_GPIO, 0);
  pwm_timer_set_freq(PWM_3_GPIO, 0);
  pwm_timer_set_freq(PWM_4_GPIO, 0);

  is_initialized_ = true;
  return true;
}

void HardwareMapping::UpdateAuxBitmap(LogicOutput type, bool is_on, AuxBitmap *flags) {
  if (is_on) {
    *flags |= output_to_aux_bits_[type];
  } else {
    *flags &= ~output_to_aux_bits_[type];
  }
}

void HardwareMapping::SetAuxOutput(AuxBitmap flags) {
  if (!is_initialized_) return;
  for (int i = 0; i < NUM_BOOL_OUTPUTS; ++i) {
    if (flags & (1 << i)) {
      set_gpio(get_aux_bit_gpio_descriptor(i + 1));
    } else {
      clr_gpio(get_aux_bit_gpio_descriptor(i + 1));
    }
  }
}

void HardwareMapping::SetPWMOutput(LogicOutput type, float value) {
  if (!is_initialized_) return;
  const uint32_t gpio = output_to_pwm_gpio_[type];
  if (value <= 0.0) {
    pwm_timer_start(gpio, false);
  } else {
    pwm_timer_set_duty(gpio, value);
    pwm_timer_start(gpio, true);
  }
}

std::string HardwareMapping::DebugMotorString(LogicAxis axis) {
  const MotorBitmap motormap_for_axis = axis_to_driver_[axis];
  std::string result;
  for (int motor = 0; motor < NUM_MOTORS; ++motor) {
    if (motormap_for_axis & (1 << motor)) {
      if (!result.empty()) result.append(", ");
      result.append(StringPrintf("%d", motor + 1));
    }
  }
  if (result.empty()) result.append("<none>");
  return result;
}

void HardwareMapping::AssignMotorSteps(LogicAxis axis, int steps,
                                       LinearSegmentSteps *out) {
  const MotorBitmap motormap_for_axis = axis_to_driver_[axis];
  for (int motor = 0; motor < NUM_MOTORS; ++motor) {
    if (motormap_for_axis & (1 << motor)) {
      out->steps[motor] = driver_flip_[motor] * steps;
    }
  }
}

HardwareMapping::AxisTrigger HardwareMapping::AvailableTrigger(LogicAxis axis) {
  if (!is_initialized_) return TRIGGER_NONE;
  return TRIGGER_NONE;   // TODO: implement me.
}

bool HardwareMapping::TestEndstop(LogicAxis axis, AxisTrigger expected_trigger) {
  if (!is_initialized_) return false;
  return false;  // TODO: implement me.
}

class HardwareMapping::ConfigReader : public ConfigParser::EventReceiver {
public:
  ConfigReader(HardwareMapping *config) : config_(config){}

  virtual bool SeenSection(int line_no, const std::string &section_name) {
    current_section_ = section_name;
    return (section_name == "aux-mapping" ||
            section_name == "pwm-mapping" ||
            section_name == "motor-mapping");
  }

  virtual bool SeenNameValue(int line_no,
                             const std::string &name,
                             const std::string &value) {

    if (current_section_ == "aux-mapping") {
      for (int i = 1; i <= NUM_BOOL_OUTPUTS; ++i) {
        if (name == StringPrintf("aux_%d", i)) {
          return SetAuxMapping(line_no, i, value);
        }
      }
    }

    if (current_section_ == "pwm-mapping") {
      for (int i = 1; i <= NUM_PWM_OUTPUTS; ++i) {
        if (name == StringPrintf("pwm_%d", i)) {
          return SetPwmMapping(line_no, i, value);
        }
      }
    }

    if (current_section_ == "motor-mapping") {
      for (int i = 1; i <= NUM_MOTORS; ++i) {
        if (name == StringPrintf("motor_%d", i))
          return SetMotorAxis(line_no, i, value);
      }
      return false;
    }

    return false;
  }

  virtual void ReportError(int line_no, const std::string &msg) {
    Log_error("Line %d: %s", line_no, msg.c_str());
  }

private:
  bool SetAuxMapping(int line_no, int aux_number, const std::string &value) {
    LogicOutput output;
    if (NameToOutput(value, &output)) {
      if (output == OUT_HOTEND || output == OUT_HEATEDBED) {
        ReportError(line_no, "It is a dangerous to connect hotends "
                    "or heated beds to a boolean output. Use a PWM output!");
        return false;
      }
      Log_debug("Aux %d -> %s", aux_number, value.c_str());
      return config_->AddAuxMapping(output, aux_number);
    } else {
      ReportError(line_no, StringPrintf("Unrecognized AUX output type: %s",
                                        value.c_str()));
      return false;
    }
  }

  bool SetPwmMapping(int line_no, int pwm_number, const std::string &value) {
    LogicOutput output;
    if (NameToOutput(value, &output)) {
      Log_debug("Pwm %d -> %s", pwm_number, value.c_str());
      return config_->AddPWMMapping(output, pwm_number);
    } else {
      ReportError(line_no, StringPrintf("Unrecognized PWM output type: %s",
                                        value.c_str()));
      return false;
    }
  }


  bool SetMotorAxis(int line_no, int motor_number, const std::string &in_val) {
    std::vector<StringPiece> options = SplitString(in_val, " \t,");
    for (size_t i = 0; i < options.size(); ++i) {
      const std::string option = ToLower(options[i]);
      if (HasPrefix(option, "axis:")) {
        std::string axis = option.substr(strlen("axis:"));
        if (axis.empty()) return false;
        bool mirrored = false;
        if (axis[0] == '-') {
          mirrored = true;
          axis = axis.substr(1);
        }
        if (axis.empty())
          return false;

        const char axis_letter = axis[0];
        LogicAxis logic_axis = gcodep_letter2axis(axis_letter);
        if (logic_axis == GCODE_NUM_AXES) {
          Log_error("Invalid axis letter '%c'", axis_letter);
          return false; // invalid axis.
        }
        return config_->AddMotorMapping(logic_axis, motor_number, mirrored);
      }
      // TODO: maybe option 'mirror' which would be equivalent to axis:-x
      else {
        Log_error("Line %d: Unknown motor option '%s'", line_no, option.c_str());
        return false;
      }
    }
    return true;
  }

  HardwareMapping *const config_;

  std::string current_section_;
};

bool HardwareMapping::ConfigureFromFile(ConfigParser *parser) {
  HardwareMapping::ConfigReader reader(this);
  if (parser->EmitConfigValues(&reader)) {
    is_configured_ = true;
    return true;
  }
  return false;
}

HardwareMapping::GPIODefinition
HardwareMapping::get_aux_bit_gpio_descriptor(int aux_num) {
  switch (aux_num) {
  case 1:  return AUX_1_GPIO;
  case 2:  return AUX_2_GPIO;
  case 3:  return AUX_3_GPIO;
  case 4:  return AUX_4_GPIO;
  case 5:  return AUX_5_GPIO;
  case 6:  return AUX_6_GPIO;
  case 7:  return AUX_7_GPIO;
  case 8:  return AUX_8_GPIO;
  case 9:  return AUX_9_GPIO;
  case 10: return AUX_10_GPIO;
  case 11: return AUX_11_GPIO;
  case 12: return AUX_12_GPIO;
  case 13: return AUX_13_GPIO;
  case 14: return AUX_14_GPIO;
  case 15: return AUX_15_GPIO;
  case 16: return AUX_16_GPIO;
  default: return GPIO_NOT_MAPPED;
  }
}

HardwareMapping::GPIODefinition
HardwareMapping::get_pwm_gpio_descriptor(int pwm_num) {
  switch (pwm_num) {
  case 1:  return PWM_1_GPIO;
  case 2:  return PWM_2_GPIO;
  case 3:  return PWM_3_GPIO;
  case 4:  return PWM_4_GPIO;
  default: return GPIO_NOT_MAPPED;
  }
}

const char *HardwareMapping::OutputToName(LogicOutput output) {
  switch (output) {
  case OUT_MIST:        return "mist";
  case OUT_FLOOD:       return "flood";
  case OUT_VACUUM:      return "vacuum";
  case OUT_SPINDLE:     return "spindle";
  case OUT_SPINDLE_SPEED:  return "spindle-speed";
  case OUT_SPINDLE_DIRECTION: return "spindle-dir";
  case OUT_COOLER:      return "cooler";
  case OUT_CASE_LIGHTS: return "case-lights";
  case OUT_FAN:         return "fan";
  case OUT_HOTEND:      return "hotend";
  case OUT_HEATEDBED:   return "heatedbed";

  case NUM_OUTPUTS: return "<invalid>";
    // no default case to have the compiler warn about new things.
  }
  return "<invalid>";
}

bool HardwareMapping::NameToOutput(StringPiece str, LogicOutput *result) {
  const std::string n = ToLower(str);
#define MAP_VAL(condition, val) if (condition)  do { *result = val; return true; } while(0)
  MAP_VAL(n == "mist",        OUT_MIST);
  MAP_VAL(n == "flood",       OUT_FLOOD);
  MAP_VAL(n == "vacuum",      OUT_VACUUM);
  MAP_VAL(n == "spindle" || n == "spindle-on", OUT_SPINDLE);
  MAP_VAL(n == "spindle-speed", OUT_SPINDLE_SPEED);
  MAP_VAL(n == "spindle-dir", OUT_SPINDLE_DIRECTION);
  MAP_VAL(n == "cooler",      OUT_COOLER);
  MAP_VAL(n == "case-lights", OUT_CASE_LIGHTS);
  MAP_VAL(n == "fan",         OUT_FAN);
  MAP_VAL(n == "hotend",      OUT_FAN);
  MAP_VAL(n == "heatedbed",   OUT_FAN);
#undef MAP_VAL
  return false;
}
