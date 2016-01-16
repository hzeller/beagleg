/* -*- mode: c++ c-basic-offset: 2; indent-tabs-mode: nil; -*-
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

HardwareMapping::HardwareMapping() : is_initialized_(false) {
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

void HardwareMapping::SetBooleanOutput(LogicOutput type, bool value) {
  if (!is_initialized_) return;
  const uint32_t gpio = output_to_bool_gpio_[type];
  if (gpio == GPIO_NOT_MAPPED) return;
  Log_debug("Set output for %d", type);
  // TODO: implement me.
}

void HardwareMapping::SetPWMOutput(LogicOutput type, float value) {
  if (!is_initialized_) return;
  const uint32_t gpio = output_to_pwm_gpio_[type];
  if (gpio == GPIO_NOT_MAPPED) {
    SetBooleanOutput(type, value > 0.0);  // Fallback in case this is just bool.
    return;
  }

  if (value <= 0.0) {
    pwm_timer_start(gpio, false);
  } else {
    pwm_timer_set_duty(gpio, value);
    pwm_timer_start(gpio, true);
  }
}

HardwareMapping::AxisTrigger HardwareMapping::EndstopTrigger(LogicAxis axis) {
  return TRIGGER_NONE;   // TODO: implement me.
}

bool HardwareMapping::TestEndstop(LogicAxis axis, AxisTrigger expected_trigger) {
  return true;  // TODO: implement me.
}

bool HardwareMapping::NameToOutput(StringPiece str, LogicOutput *result) {
  const std::string n = ToLower(str);
#define MAP_VAL(condition, val) if (condition)  do { *result = val; return true; } while(0)
  MAP_VAL(n == "mist",        OUT_MIST);
  MAP_VAL(n == "flood",       OUT_FLOOD);
  MAP_VAL(n == "vacuum",      OUT_VACCUM);
  MAP_VAL(n == "spindle" || n == "spindle-on", OUT_SPINDLE);
  MAP_VAL(n == "spindle-dir", OUT_SPINDLE_DIRECTION);
  MAP_VAL(n == "cooler",      OUT_COOLER);
  MAP_VAL(n == "case-lights", OUT_CASE_LIGHTS);
  MAP_VAL(n == "fan",         OUT_FAN);
  MAP_VAL(n == "hotend",      OUT_FAN);
  MAP_VAL(n == "heatedbed",   OUT_FAN);
#undef MAP_VAL
  return false;
}

static uint32_t get_aux_bit_gpio_descriptor(int pin) {
  switch (pin) {
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

static uint32_t get_pwm_gpio_descriptor(int pwm_num) {
  switch (pwm_num) {
  case 1:  return PWM_1_GPIO;
  case 2:  return PWM_2_GPIO;
  case 3:  return PWM_3_GPIO;
  case 4:  return PWM_4_GPIO;
  default: return GPIO_NOT_MAPPED;
  }
}

class HardwareMapping::ConfigReader : public ConfigParser::EventReceiver {
public:
  ConfigReader(HardwareMapping *config) : config_(config){}

  virtual bool SeenSection(int line_no, const std::string &section_name) {
    current_section_ = section_name;
    return section_name == "aux-mapping" || section_name == "pwm-mapping";
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

    return false;
  }

  virtual void ReportError(int line_no, const std::string &msg) {
    Log_error("Line %d: %s", line_no, msg.c_str());
  }

private:
  bool SetAuxMapping(int line_no, int aux_number, const std::string &value) {
    const uint32_t gpio_descriptor = get_aux_bit_gpio_descriptor(aux_number);
    LogicOutput output;
    if (NameToOutput(value, &output)) {
      if (config_->output_to_bool_gpio_[output] != GPIO_NOT_MAPPED ||
          config_->output_to_pwm_gpio_[output] != GPIO_NOT_MAPPED) {
        // TODO: do we want to allow 1:n mapping, same output, multiple pins ?
        ReportError(line_no,
                    StringPrintf("Attempt to map '%s' which is already "
                                 "mapped to different pin.", value.c_str()));
        return false;
      }

      if (gpio_descriptor == GPIO_NOT_MAPPED) {
        Log_info("Mapping '%s' to aux %d which has no hardware connector for %s"
                 "; output will be ignored for this cape.",
                 value.c_str(), aux_number, CAPE_NAME);
        // continue, this is not fatal.
      } else {
        Log_debug("PWM %d -> %s", aux_number, value.c_str());
      }

      config_->output_to_bool_gpio_[output] = gpio_descriptor;
      switch (output) {
      case OUT_HOTEND:   // Fish out bad ideas
      case OUT_HEATEDBED:
        ReportError(line_no, "It is a dangours idea to connect hotends "
                    "or heated beds to a boolean output. Use a PWM output!");
        return false;

      default:
        return true;
      }
    } else {
      ReportError(line_no, StringPrintf("Unrecognized AUX output type: %s",
                                        value.c_str()));
      return false;
    }
  }

  bool SetPwmMapping(int line_no, int aux_number, const std::string &value) {
    const uint32_t gpio_descriptor = get_pwm_gpio_descriptor(aux_number);
    LogicOutput output;
    if (NameToOutput(value, &output)) {
      if (config_->output_to_bool_gpio_[output] != GPIO_NOT_MAPPED ||
          config_->output_to_pwm_gpio_[output] != GPIO_NOT_MAPPED) {
        // TODO: do we want to allow 1:n mapping, same output, multiple pins ?
        ReportError(line_no,
                    StringPrintf("Attempt to map '%s' which is already "
                                 "mapped to different pin", value.c_str()));
        return false;
      }
      if (gpio_descriptor == GPIO_NOT_MAPPED) {
        Log_info("Mapping '%s' to pwm %d which has no hardware connector for %s"
                 " ; output will be ignored for this cape.",
                 value.c_str(), aux_number, CAPE_NAME);
        // continue, this is not fatal.
      } else {
        Log_debug("Aux %d -> %s", aux_number, value.c_str());
      }

      config_->output_to_pwm_gpio_[output] = gpio_descriptor;
      return true;
    } else {
      ReportError(line_no, StringPrintf("Unrecognized PWM output type: %s",
                                        value.c_str()));
      return false;
    }
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
