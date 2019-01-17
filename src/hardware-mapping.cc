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

#include "common/logging.h"
#include "common/string-util.h"

#include "config-parser.h"
#include "generic-gpio.h"
#include "pwm-timer.h"
#include "motor-operations.h"  // LinearSegmentSteps

HardwareMapping::HardwareMapping()
  : estop_input_(0), pause_input_(0), start_input_(0), probe_input_(0),
    estop_state_(true), motors_enabled_(false), aux_bits_(0),
    is_hardware_initialized_(false) {
}

HardwareMapping::~HardwareMapping() {
  if (is_hardware_initialized_) {
    ResetHardware();
    unmap_gpio();
    pwm_timers_unmap();
  }
}

bool HardwareMapping::AddAuxMapping(NamedOutput output, int aux) {
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
    Log_debug("(no #define AUX_%d_GPIO in hardware/%s/beagleg-pin-mapping.h)",
              aux, CAPE_NAME);
    return true;  // This is not fatal, but no need to prepare these bits.
  }
  output_to_aux_bits_[output] |= 1 << (aux-1);
  return true;
}
bool HardwareMapping::HasAuxMapping(NamedOutput output) const {
  return output_to_aux_bits_[output] != 0;
}

bool HardwareMapping::AddPWMMapping(NamedOutput output, int pwm) {
  if (pwm == 0) return true;  // allowedd null-mapping
  if (pwm < 1 || pwm > NUM_PWM_OUTPUTS) {
    Log_error("PWM %d out of range [%d..%d]", pwm, 1, NUM_BOOL_OUTPUTS);
    return false;
  }
  if (HasPWMMapping(output)) {
    // TODO: do we want to allow 1:n mapping, same output, multiple pins ?
    Log_error("Attempt to map '%s' which is already "
              "mapped to different pin", OutputToName(output));
    return false;
  }
  const uint32_t gpio_descriptor = get_pwm_gpio_descriptor(pwm);
  if (gpio_descriptor == GPIO_NOT_MAPPED) {
    Log_info("Mapping '%s' to pwm %d which has no hardware connector for %s"
             "; output will be ignored for this cape.",
             OutputToName(output), pwm, CAPE_NAME);
    Log_debug("(no #define PWM_%d_GPIO in hardware/%s/beagleg-pin-mapping.h)",
              pwm, CAPE_NAME);
  }
  output_to_pwm_gpio_[output] = gpio_descriptor;
  return true;
}
bool HardwareMapping::HasPWMMapping(NamedOutput output) const {
  return output_to_pwm_gpio_[output] != GPIO_NOT_MAPPED;
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

int HardwareMapping::GetFirstFreeMotor() {
  for (int motor = 0; motor < NUM_MOTORS; ++motor) {
    if (driver_flip_[motor] == 0)
      return motor + 1;
  }
  return 0;
}

bool HardwareMapping::IsMotorFlipped(int motor) {
  return driver_flip_[motor] == -1;
}

bool HardwareMapping::InitializeHardware() {
  if (geteuid() != 0) {
    Log_error("Need to run as root to access GPIO pins.");
    return false;
  }
  if (!map_gpio()) {
    Log_error("Couldn't mmap() GPIO ranges.\n");
    return false;
  }

#ifdef _DISABLE_PWM_TIMERS
  Log_info("PWM timers are disabled.\n");
#else
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
#endif

  is_hardware_initialized_ = true;
  ResetHardware();

  // Do some sanity check. If logical min/max are re-using switch channels,
  // then we can have the electrical situation that both look triggered at
  // startup, which is of course an impossible mechanical state.
  // Since we don't know in which direction to move out of the way, we have to
  // bail for safety.
  for (GCodeParserAxis axis : AllAxes()) {
    if (TestAxisSwitch(axis, TRIGGER_MIN) && TestAxisSwitch(axis, TRIGGER_MAX)) {
      Log_error("Error: Both min and max switch for axis %c are triggered at "
                "the same time. No way to safely proceed. Bailing out",
                gcodep_axis2letter(axis));
      return false;
    }
  }
  return true;
}

void HardwareMapping::ResetHardware() {
  if (!is_hardware_initialized_) return;
  aux_bits_ = 0;
  SetAuxOutputs();
  EnableMotors(false);
  for (int i = 0; i < NUM_PWM_OUTPUTS; ++i) {
    pwm_timer_start(get_pwm_gpio_descriptor(i+1), false);
  }
}

void HardwareMapping::EnableMotors(bool on) {
  if (!is_hardware_initialized_) return;
  if (on && InSoftEStop()) return;
  // Right now, we just have this hardcoded, but if 'enable' should
  // ever be configurable via config file and not given by the hardware
  // mapping include, we can do that here.
  if (on ^ MOTOR_ENABLE_IS_ACTIVE_HIGH) clr_gpio(MOTOR_ENABLE_GPIO);
  else set_gpio(MOTOR_ENABLE_GPIO);
  motors_enabled_ = on;
}

HardwareMapping::AuxBitmap HardwareMapping::GetAuxBits() {
  return aux_bits_;
}

int HardwareMapping::GetAuxBit(int pin) {
  return (aux_bits_ >> (pin - 1)) & 1;
}

void HardwareMapping::UpdateAuxBits(int pin, bool is_on) {
  if (is_on) aux_bits_ |= (1 << (pin - 1));
  else       aux_bits_ &= ~(1 << (pin - 1));
}

void HardwareMapping::UpdateAuxBitmap(NamedOutput type, bool is_on) {
  if (is_on) aux_bits_ |= output_to_aux_bits_[type];
  else       aux_bits_ &= ~output_to_aux_bits_[type];

  if (type == NamedOutput::ESTOP) estop_state_ = is_on;  // Estop: special attention.
}

void HardwareMapping::SetAuxOutputs() {
  if (!is_hardware_initialized_) return;
  for (int i = 0; i < NUM_BOOL_OUTPUTS; ++i) {
    if (aux_bits_ & (1 << i)) set_gpio(get_aux_bit_gpio_descriptor(i + 1));
    else                      clr_gpio(get_aux_bit_gpio_descriptor(i + 1));
  }
}

void HardwareMapping::AuxOutputsOff() {
  aux_bits_ = 0;
  SetAuxOutputs();
}

bool HardwareMapping::InSoftEStop() {
  return estop_state_;
}

bool HardwareMapping::MotorsEnabled() {
  return motors_enabled_;
}

void HardwareMapping::SetPWMOutput(NamedOutput type, float value) {
#ifdef _DISABLE_PWM_TIMERS
  return;
#else
  if (!is_hardware_initialized_) return;
  const uint32_t gpio = output_to_pwm_gpio_[type];
  if (value <= 0.0) {
    pwm_timer_start(gpio, false);
  } else {
    pwm_timer_set_duty(gpio, value);
    pwm_timer_start(gpio, true);
  }
#endif
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
      out->steps[motor] = steps;
    }
  }
}

int HardwareMapping::GetAxisSteps(LogicAxis axis, const PhysicalStatus &status) {
  uint8_t mask = axis_to_driver_[axis];
  int m = 0;
  while (((mask & 0x01) != 0x01) && m < NUM_MOTORS) {
    // while the first bit is not 1, iterate.
    mask = mask >> 1;
    m++;
  }

  // Maybe this axis is not mapped to any motor.
  if (m == NUM_MOTORS) return 0;
  return status.pos_steps[m];
}

HardwareMapping::AxisTrigger HardwareMapping::AvailableAxisSwitch(LogicAxis axis) {
  if (!is_hardware_initialized_) {
    // Pretend we have all the switches.
    return (AxisTrigger) (TRIGGER_MIN|TRIGGER_MAX);
  }
  int result = 0;
  if (axis_to_min_endstop_[axis] != 0) result |= TRIGGER_MIN;
  if (axis_to_max_endstop_[axis] != 0) result |= TRIGGER_MAX;
  return (AxisTrigger) result;  // Safe to cast: all within range.
}

bool HardwareMapping::TestSwitch(const int switch_number, bool def_result) {
  if (!is_hardware_initialized_) return def_result;
  GPIODefinition gpio_def = get_endstop_gpio_descriptor(switch_number);
  if (gpio_def == GPIO_NOT_MAPPED) return def_result;
  bool state = get_gpio(gpio_def);
  int debounce = 0;
  for (;;) {
    usleep(10);
    bool new_state = get_gpio(gpio_def);
    if (new_state == state) {
      debounce++;
      if (debounce == 2) break;
    } else {
      state = new_state;
      debounce = 0;
    }
  }
  return (state == trigger_level_[switch_number-1]);
}

bool HardwareMapping::TestAxisSwitch(LogicAxis axis, AxisTrigger requested_trigger) {
  bool result = false;
  if (requested_trigger & TRIGGER_MIN)
    result |= TestSwitch(axis_to_min_endstop_[axis], false);
  if (requested_trigger & TRIGGER_MAX)
    result |= TestSwitch(axis_to_max_endstop_[axis], false);
  return result;
}

bool HardwareMapping::TestEStopSwitch() {
  return TestSwitch(estop_input_, false);
}

bool HardwareMapping::TestPauseSwitch() {
  return TestSwitch(pause_input_, false);
}

bool HardwareMapping::TestStartSwitch() {
  return TestSwitch(start_input_, true);
}

bool HardwareMapping::TestProbeSwitch() {
  return TestSwitch(probe_input_, true);
}

class HardwareMapping::ConfigReader : public ConfigParser::Reader {
public:
  ConfigReader(HardwareMapping *config) : config_(config){}

  bool SeenSection(int line_no, const std::string &section_name) final {
    current_section_ = section_name;
    return (section_name == "aux-mapping" ||
            section_name == "pwm-mapping" ||
            section_name == "switch-mapping" ||
            section_name == "motor-mapping");
  }

  bool SeenNameValue(int line_no,
                     const std::string &name,
                     const std::string &value) final {
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

    if (current_section_ == "switch-mapping") {
      for (int i = 1; i <= NUM_SWITCHES; ++i) {
        if (name == StringPrintf("switch_%d", i)) {
          return SetSwitchOptions(line_no, i, value);
        }
      }
      return false;
    }

    return false;
  }

  void ReportError(int line_no, const std::string &msg) final {
    Log_error("Line %d: %s", line_no, msg.c_str());
  }

private:
  bool SetAuxMapping(int line_no, int aux_number, const std::string &value) {
    NamedOutput output;
    if (NameToOutput(value, &output)) {
      if (output == NamedOutput::HOTEND || output == NamedOutput::HEATEDBED) {
        ReportError(line_no, "It is a dangerous to connect hotends "
                    "or heated beds to a boolean output. Use a PWM output!");
        return false;
      }
      if (aux_number == 16 && output != NamedOutput::LED) {
        ReportError(line_no, "Aux 16 can only be mapped to an led!");
        return false;
      }
      if (output == NamedOutput::LED && aux_number != 16) {
        ReportError(line_no, "An led can only be mapped to Aux 16!");
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
    NamedOutput output;
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

  bool SetSwitchOptions(int line_no, int switch_number,
                        const std::string &value) {
    std::vector<StringPiece> options = SplitString(value, " \t,");
    for (size_t i = 0; i < options.size(); ++i) {
      const std::string option = ToLower(options[i]);
      if (option.empty()) continue;
      if (option == "e-stop") {
        config_->estop_input_ = switch_number;
      }
      else if (option == "pause") {
        config_->pause_input_ = switch_number;
      }
      else if (option == "start") {
        config_->start_input_ = switch_number;
      }
      else if (option == "probe") {
        config_->probe_input_ = switch_number;
      }
      else if (option.length() == 5) {
        const GCodeParserAxis axis = gcodep_letter2axis(option[4]);
        if (axis == GCODE_NUM_AXES) {
          Log_error("Line %d: Expect min_<axis> or max_<axis>, e.g. min_x. "
                    "Last character not a valid axis (got %s).",
                    line_no, option.c_str());
          return false;
        }

        if (get_endstop_gpio_descriptor(switch_number) == GPIO_NOT_MAPPED) {
          Log_error("Switch %d has no hardware connector on %s cape. "
                    "Possibly dangerous to ignore, bailing out.",
                    switch_number, CAPE_NAME);
          // It is somewhat dangerous to assume that an endswitch is working
          // when it fact it is not. Bail out.
          return false;
        }
        if (HasPrefix(option, "min_")) {
          config_->axis_to_min_endstop_[axis] = switch_number;
        } else if (HasPrefix(option, "max_")) {
          config_->axis_to_max_endstop_[axis] = switch_number;
        } else {
          Log_error("Line %d: Expect min_<axis> or max_<axis>, e.g. min_x. "
                    "Got %s", line_no, option.c_str());
          return false;
        }
      }
      else if (option == "active:low") {
        config_->trigger_level_[switch_number - 1] = false;
      }
      else if (option == "active:high") {
        config_->trigger_level_[switch_number - 1] = true;
      }
      else {
        Log_error("Line %d: Invalid option '%s' for switch %d.", line_no,
                  option.c_str(), switch_number);
        return false;
      }
    }

    return true;  // All went fine.
  }

  HardwareMapping *const config_;

  std::string current_section_;
};

bool HardwareMapping::ConfigureFromFile(ConfigParser *parser) {
  HardwareMapping::ConfigReader reader(this);
  return parser->EmitConfigValues(&reader);
}

const char *HardwareMapping::OutputToName(NamedOutput output) {
  switch (output) {
  case NamedOutput::MIST:        return "mist";
  case NamedOutput::FLOOD:       return "flood";
  case NamedOutput::VACUUM:      return "vacuum";
  case NamedOutput::SPINDLE:     return "spindle";
  case NamedOutput::SPINDLE_SPEED:  return "spindle-speed";
  case NamedOutput::SPINDLE_DIRECTION: return "spindle-dir";
  case NamedOutput::COOLER:      return "cooler";
  case NamedOutput::CASE_LIGHTS: return "case-lights";
  case NamedOutput::FAN:         return "fan";
  case NamedOutput::HOTEND:      return "hotend";
  case NamedOutput::HEATEDBED:   return "heatedbed";
  case NamedOutput::POINTER:     return "pointer";
  case NamedOutput::LED:         return "led";
  case NamedOutput::ATX_POWER:   return "atx-power";
  case NamedOutput::ESTOP:       return "estop";

  case NamedOutput::NUM_OUTPUTS: return "<invalid>";
    // no default case to have the compiler warn about new things.
  }
  return "<invalid>";
}

bool HardwareMapping::NameToOutput(StringPiece str, NamedOutput *result) {
  const std::string n = ToLower(str);
#define MAP_VAL(condition, val) if (condition)  do { *result = val; return true; } while(0)
  MAP_VAL(n == "mist",        NamedOutput::MIST);
  MAP_VAL(n == "flood",       NamedOutput::FLOOD);
  MAP_VAL(n == "vacuum",      NamedOutput::VACUUM);
  MAP_VAL(n == "spindle" || n == "spindle-on", NamedOutput::SPINDLE);
  MAP_VAL(n == "spindle-speed" || n == "spindle-pwm",
          NamedOutput::SPINDLE_SPEED);
  MAP_VAL(n == "spindle-dir", NamedOutput::SPINDLE_DIRECTION);
  MAP_VAL(n == "cooler",      NamedOutput::COOLER);
  MAP_VAL(n == "case-lights", NamedOutput::CASE_LIGHTS);
  MAP_VAL(n == "fan",         NamedOutput::FAN);
  MAP_VAL(n == "hotend",      NamedOutput::HOTEND);
  MAP_VAL(n == "heatedbed",   NamedOutput::HEATEDBED);
  MAP_VAL(n == "pointer",     NamedOutput::POINTER);
  MAP_VAL(n == "led",         NamedOutput::LED);
  MAP_VAL(n == "atx-power",   NamedOutput::ATX_POWER);
  MAP_VAL(n == "estop",       NamedOutput::ESTOP);
#undef MAP_VAL
  return false;
}

// Mapping of numbered IO pins to GPIO definition. The *_GPIO macros
// are defined in the cape specific header files.
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

HardwareMapping::GPIODefinition
HardwareMapping::get_endstop_gpio_descriptor(int switch_num) {
  switch (switch_num) {
  case 1:  return IN_1_GPIO;
  case 2:  return IN_2_GPIO;
  case 3:  return IN_3_GPIO;
  case 4:  return IN_4_GPIO;
  case 5:  return IN_5_GPIO;
  case 6:  return IN_6_GPIO;
  case 7:  return IN_7_GPIO;
  case 8:  return IN_8_GPIO;
  case 9:  return IN_9_GPIO;
  default: return GPIO_NOT_MAPPED;
  }
}
