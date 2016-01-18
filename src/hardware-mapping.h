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

#ifndef BEAGLEG_HARDWARE_MAPPING_
#define BEAGLEG_HARDWARE_MAPPING_

#include <stdint.h>

#include "gcode-parser.h"  // For GCodeParserAxis
#include "string-util.h"

class ConfigParser;

// The hardware mapping class maps symbolic names of functionality,
// e.g. 'Motor for Axis X' or 'Output for Mist' or 'PWM for heater' to
// an actual hardware output (or none).
//
// The mapping happens in two levels:
//
// 1) On the low level, there is the hardware
//    that provides a number of outputs for motors, aux-pins or PWM-controlled
//    pins as well as input switches. They are tied to a particular GPIO pin on
//    the processor. This might map MOTOR_2_STEP_GPIO to PIN_P9_19 on a
//    Beaglebone Black for instance. On the cape provided by the user, there
//    might be a screw terminal 'Motor 2 step'. So this level provides numbered
//    connectors Motor 1..8, switch 1..8 or PWM 1..4 and maps it to even lower
//    level GPIOs. These mappings are provided in form of #defines in the
//    hardware/<CAPENAME>/beagleg-pin-mapping.h and used in this class and in
//    PRU.
//
// 2) The next level maps logic names to the numbered connectors. For instance
//    Axis A might map to 'motor 2', 'min-x-home' to 'switch 4',
//    'Spindle speed' to PWM 3 or 'cooling fan' to Aux 1. This mapping is chosen
//    by the user of the CAPE by filling the [motor-mapping], [switch-mapping],
//    [aux-mapping] or [pwm-mapping].
//
// This class uses the pin mappings provided in the cape specific include file
// and the logic mappings provided by the user in the configuration file to
// present logic accesses to these values.
class HardwareMapping {
public:
  // Some anonymous sets of constexpr, here represented as enum as we don't
  // have constexpr in this version of c++ yet.
  enum {
    NUM_SWITCHES      =  8,
    NUM_BOOL_OUTPUTS  = 16,
    NUM_PWM_OUTPUTS   =  4,
    NUM_MOTORS        =  8,
  };

  HardwareMapping();

  // Pick the relevant mapping parameter from the configuration file.
  bool ConfigureFromFile(ConfigParser *parser);

  // Initialize the hardware (needs to be configured first).
  // If this function is never called, all outputs are simulated.
  bool InitializeHardware();

  // A register containing all the necessary bits.
  typedef uint16_t AuxFlags;

  typedef GCodeParserAxis LogicAxis;  // This is provided by the gcode parser.

  enum AxisTrigger {
    TRIGGER_NONE = 0x00,   // None of the Axis is triggering
    TRIGGER_MIN  = 0x01,   // Min position of axis is triggering
    TRIGGER_MAX  = 0x02,   // Max position of axis is triggering
    TRIGGER_ANY  = 0x03,   // Any of the axis is triggering
  };

  enum LogicOutput {
    OUT_MIST,
    OUT_FLOOD,
    OUT_VACUUM,
    OUT_SPINDLE,
    OUT_SPINDLE_SPEED,
    OUT_SPINDLE_DIRECTION,
    OUT_COOLER,
    OUT_CASE_LIGHTS,
    OUT_FAN,
    OUT_HOTEND,
    OUT_HEATEDBED,

    NUM_OUTPUTS   // last.
  };

  // Set logic output value to on/off for the particular logic output
  // in the given flag-set. Only updates the flags, does not set the
  // output.
  void UpdateAuxFlags(LogicOutput type, bool value, AuxFlags *flags);

  // Set the output according to the flags immediately (unbuffered).
  // There are some cases in which this is necessary, but usually
  // the values are set synchronously with the motor movements to avoid timing
  // problems due to the buffer.
  void SetAuxOutput(AuxFlags flags);

  // Set PWM value for given output immediately.
  void SetPWMOutput(LogicOutput type, float value);

  // Returns which endstop trigger are available. Possible values are
  // TRIGGER_NONE, if there are no end-stops configured, TRIGGER_MIN/MAX
  // if either min or max are available as endstop trigger or TRIGGER_ANY,
  // if both ends are triggering.
  AxisTrigger AvailableTrigger(LogicAxis axis);

  // Returns true if endstop for given axis has been reached.
  // "expected_trigger" can be any of MIN/MAX to test for that
  // particular end-trigger or ANY if we don't care which end is affected.
  // If an expected trigger is asked that is not returned in AvailableTrigger(),
  // this will always return false.
  bool TestEndstop(LogicAxis axis, AxisTrigger expected_trigger);

  // other switches: emergency off etc.
  // inputs: analog inputs needed.

private:
  class ConfigReader;

  // A GPIO Definition contains the relevant information to address a particular
  // hardware pin.
  typedef uint32_t GPIODefinition;

  // Converts the human readable name of an output to the enumeration if possible.
  static bool NameToOutput(StringPiece str, LogicOutput *result);

  // return GPIO definition for aux number. Values range from 1..NUM_BOOL_OUTPUTS
  static GPIODefinition get_aux_bit_gpio_descriptor(int aux_number);

  // Get GPIO definition for pwm number. Value range from 1..NUM_PWM_OUTPUTS
  static GPIODefinition get_pwm_gpio_descriptor(int pwm_number);

  FixedArray<AuxFlags, NUM_OUTPUTS> output_to_aux_bits_;
  FixedArray<GPIODefinition, NUM_OUTPUTS> output_to_pwm_gpio_;

  bool is_configured_;
  bool is_initialized_;
};

#endif  // BEAGLEG_HARDWARE_MAPPING_
