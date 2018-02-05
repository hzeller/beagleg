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

#include "gcode-parser/gcode-parser.h"  // For GCodeParserAxis
#include "common/string-util.h"

class ConfigParser;
struct LinearSegmentSteps;

// The hardware connected to the BeagleBone typically has a number of universal
// motor, PWM and Aux outputs and inputs. The user wants to tie a universal
// connector to a particular logic function.
//
// Let's say the board has a set of numbered outputs for motors (motor 1..8),
// some for PWM etc. This class allows to tie them to logical functions within
// BeagleG (motor 1 shall be the Z axis), configurable by the user.
//
// This class maps symbolic names of functionality,
// e.g. 'Motor for Axis X' or 'Output for Mist' or 'PWM for heater' to
// an actual hardware output (or none).
//
// Internall, the mapping happens in two levels:
//
// 1) On the low level, there is the cape that provides a number of outputs
//    for motors, aux-pins or PWM-controlled
//    pins as well as input switches. They are tied to a particular GPIO pin on
//    the processor. This might map MOTOR_2_STEP_GPIO to PIN_P9_19 on a
//    Beaglebone Black for instance. On the cape provided by the user, there
//    might be a screw terminal 'Motor 2 step'. So this level provides numbered
//    connectors Motor 1..8, switch 1..8 or PWM 1..4 and maps it to even lower
//    level GPIOs. These low-level mappings are provided in form of #defines in
//    the hardware/<CAPENAME>/beagleg-pin-mapping.h and used in this class and
//    in the PRU.
//
// 2) The next level maps logic names to the numbered connectors. For instance
//    Axis A might map to 'motor 2', 'min-x-home' to 'switch 4',
//    'Spindle speed' to PWM 3 or 'cooling fan' to Aux 1. This mapping is chosen
//    by the user of the CAPE by filling the [motor-mapping], [switch-mapping],
//    [aux-mapping] or [pwm-mapping] sections of the configuration file.
//
// This class uses the pin mappings provided in the cape specific include file
// and the logic mappings provided by the user in the configuration file to
// present logic accesses to these values.
//
// An object of this class can be used to just simulate hardare access if
// InitializeHardware() is not called.
class HardwareMapping {
public:
  // Some anonymous sets of constexpr, here represented as enum as we don't
  // have constexpr in this version of c++ yet.
  enum {
    NUM_SWITCHES      =  9,
    NUM_BOOL_OUTPUTS  = 16,
    NUM_PWM_OUTPUTS   =  4,
    NUM_MOTORS        =  8
  };

  // A register containing all the necessary bits.
  typedef uint16_t AuxBitmap;
  typedef uint8_t  MotorBitmap;

  typedef GCodeParserAxis LogicAxis;  // This is provided by the gcode parser.

  enum AxisTrigger {
    TRIGGER_NONE = 0x00,   // None of the Axis is triggering
    TRIGGER_MIN  = 0x01,   // Min position of axis is triggering
    TRIGGER_MAX  = 0x02,   // Max position of axis is triggering
    TRIGGER_ANY  = 0x03    // Any of the axis is triggering
  };

  enum LogicOutput {
    OUT_MIST,              // M7 = on; M9 = off
    OUT_FLOOD,             // M8 = on; M9 = off
    OUT_VACUUM,            // M10 = on; M11 = off
    OUT_SPINDLE,           // M3/M4 = on; M5 = off
    OUT_SPINDLE_SPEED,
    OUT_SPINDLE_DIRECTION, // M4 = on; M3/M5 = off
    OUT_COOLER,            // M245 = on; M246 = off
    OUT_CASE_LIGHTS,       // M355 S1 = on; M355 S0 = off
    OUT_FAN,               // M106 Sn (set pwm or on if n > 0); M107 = off
    OUT_HOTEND,
    OUT_HEATEDBED,
    OUT_POINTER,           // M64 Px = on; M65 Px = off
    OUT_LED,               // toggles on/off with M42 while waiting for start switch
    OUT_ATX_POWER,         // M80 = on; M81 = off
    OUT_ESTOP,             // M0 = on; M999 = off

    NUM_OUTPUTS   // last.
  };

  HardwareMapping();
  ~HardwareMapping();

  /*
   * various configuration options. Direct setters for programmatic
   * access. Configuration file option for standard BeagleG config.
   */

  // Pick the relevant mapping parameter from the configuration file.
  bool ConfigureFromFile(ConfigParser *parser);

  // Connect logic output to aux pin in the range [1..NUM_BOOL_OUTPUTS]
  // A value of 0 for 'aux' is accepted, but does not connect it to anything.
  bool AddAuxMapping(LogicOutput output, int aux);

  // Connect logic output to pwm pin in the range [1..NUM_PWM_OUTPUTS]
  // A value of 0 for 'pwm' is accpeted, but does not connect it to anything.
  bool AddPWMMapping(LogicOutput output, int pwm);

  // Add a motor mapping: connect the logic axis to given motor.
  // Motor is in the range [1..NUM_MOTORS]. If 'mirrored' is true,
  // motor turns the opposite direction.
  // A value of 0 for 'motor' is accepted but does not connect it to anything.
  bool AddMotorMapping(LogicAxis axis, int motor, bool mirrored);

  // Determine if we have a motor configured for given axis.
  bool HasMotorFor(LogicAxis axis) { return axis_to_driver_[axis] != 0; }

  // Return first motor free to map. Returns value in the range
  // [1..NUM_MOTORS] if there is a free motor, 0 otherwise.
  // Useful for auto-configuration.
  int GetFirstFreeMotor();

  /*
   * If not used as a simulated machine, InitializeHardware() is needed.
   */

  // Initialize the hardware (needs to be configured first).
  // If this function is never called, all outputs are simulated.
  bool InitializeHardware();

  // This returns if we are in hardware simulation mode.
  bool IsHardwareSimulated() { return !is_hardware_initialized_; }

  // -- Boolean and PWM outputs.

  // Enable motors.
  void EnableMotors(bool on);

  // Get the logic state of all the aux_bits_.
  AuxBitmap GetAuxBits();

  // Get the logic state of a given pin.
  int GetAuxBit(int pin);

  // Set logic output value to on/off for a given pin.
  // Only updates the aux_bits_ does not set the output (can be done
  // with SetAuxOutput()).
  void UpdateAuxBits(int pin, bool is_on);

  // Set logic output value to on/off for the particular logic output.
  // Only updates the aux_bits_ does not set the output (can be done
  // with SetAuxOutput()).
  void UpdateAuxBitmap(LogicOutput type, bool value);

  // Set the output according to the aux_bits_ immediately (unbuffered).
  // There are some cases in which this is necessary, but usually
  // the values are set synchronously with the motor movements to avoid timing
  // problems due to the buffer.
  void SetAuxOutputs();

  // Set PWM value for given output immediately.
  void SetPWMOutput(LogicOutput type, float value);

  // -- Motor outputs

  // Given the logic axis and number of steps, assign these steps to the mapped
  // motors in the LinearSegmentSteps
  void AssignMotorSteps(LogicAxis axis, int steps, LinearSegmentSteps *out);

  // -- Switch access

  // Returns which endstop trigger are available. Possible values are
  // TRIGGER_NONE, if there are no end-stops configured, TRIGGER_MIN/MAX
  // if either min or max are available as endstop trigger or TRIGGER_ANY,
  // if both ends are triggering.
  AxisTrigger AvailableAxisSwitch(LogicAxis axis);

  // Returns true if endstop for given axis has been reached.
  // "expected_trigger" can be any of MIN/MAX to test for that
  // particular end-trigger or ANY if we don't care which end is affected.
  // If an requested trigger is asked that is not returned in AvailableAxisSwitch(),
  // this will always return false.
  bool TestAxisSwitch(LogicAxis axis, AxisTrigger requested_trigger);

  // Returns true if the E-Stop input is active.
  bool TestEStopSwitch();

  // Returns true if the pause input is active.
  bool TestPauseSwitch();

  // Returns true if the start input is active (or it's not available)
  bool TestStartSwitch();

  // Returns true if the probe input is active (or it's not available)
  bool TestProbeSwitch();

  bool HasProbeSwitch(LogicAxis axis) const {
    if (axis == AXIS_Z) return probe_input_ != 0;
    return false;
  }

  // other input switches.
  // inputs: analog inputs needed.

  // Return a string with a comma separated list of motors attached to given
  // logic axis.
  std::string DebugMotorString(LogicAxis axis);

private:
  class ConfigReader;

  // A GPIO Definition contains the relevant information to address a particular
  // hardware pin.
  typedef uint32_t GPIODefinition;

  // Converts the human readable name of an output to the enumeration if possible.
  static bool NameToOutput(StringPiece str, LogicOutput *result);
  static const char *OutputToName(LogicOutput output);

  // Return GPIO definition for various types of out/input. Count starts with 1.
  static GPIODefinition get_aux_bit_gpio_descriptor(int aux_number);
  static GPIODefinition get_pwm_gpio_descriptor(int pwm_number);
  static GPIODefinition get_endstop_gpio_descriptor(int switch_num);

  // Test current state of given switch number; if not configured, return
  // default result.
  bool TestSwitch(const int switch_number, bool default_result);

  void ResetHardware();  // Initialize to a safe state.

  // Mapping of logical outputs to hardware outputs.
  FixedArray<AuxBitmap, NUM_OUTPUTS> output_to_aux_bits_;
  FixedArray<GPIODefinition, NUM_OUTPUTS> output_to_pwm_gpio_;

    // "axis_to_driver": Which axis is mapped to which physical output drivers.
  // This allows to have a logical axis (e.g. X, Y, Z) output to any physical
  // or a set of multiple drivers (mirroring).
  // Bitmap of drivers output should go.
  FixedArray<MotorBitmap, GCODE_NUM_AXES> axis_to_driver_;
  FixedArray<int, NUM_MOTORS> driver_flip_;  // 1 or -1 for for individual driver

  FixedArray<int, GCODE_NUM_AXES> axis_to_min_endstop_;
  FixedArray<int, GCODE_NUM_AXES> axis_to_max_endstop_;
  FixedArray<bool, NUM_SWITCHES> trigger_level_;
  int estop_input_;
  int pause_input_;
  int start_input_;
  int probe_input_;

  AuxBitmap aux_bits_;       // Set via M42 or various other settings.

  bool is_hardware_initialized_;
};

#endif  // BEAGLEG_HARDWARE_MAPPING_
