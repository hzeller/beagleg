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

// TODO: this is motion planner and 'other stuff printers do' in one. Separate.
// TODO: this is somewhat work-in-progress.
// TODO: after the transition from C to C++, there are still some C-isms in here.

#include "gcode-machine-control.h"

#include <assert.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>

#include "container.h"
#include "gcode-parser.h"
#include "generic-gpio.h"
#include "hardware-mapping.h"
#include "spindle-control.h"
#include "logging.h"
#include "motor-operations.h"
#include "pwm-timer.h"
#include "string-util.h"

// In case we get a zero feedrate, send this frequency to motors instead.
#define ZERO_FEEDRATE_OVERRIDE_HZ 5

#define VERSION_STRING "PROTOCOL_VERSION:0.1 FIRMWARE_NAME:BeagleG "    \
  "CAPE:" CAPE_NAME " FIRMWARE_URL:http%3A//github.com/hzeller/beagleg"

// The target position vector is essentially a position in the
// GCODE_NUM_AXES-dimensional space.
//
// An AxisTarget has a position vector, in absolute machine coordinates, and a
// speed when arriving at that position.
//
// The speed is initially the aimed goal; if it cannot be reached, the
// AxisTarget will be modified to contain the actually reachable value. That is
// used in planning along the path.
struct AxisTarget {
  int position_steps[GCODE_NUM_AXES];  // Absolute position at end of segment. In steps.

  // Derived values
  int delta_steps[GCODE_NUM_AXES];     // Difference to previous position.
  enum GCodeParserAxis defining_axis;  // index into defining axis.
  float speed;                         // (desired) speed in steps/s on defining axis.
  float angle;
  unsigned short aux_bits;             // Auxillary bits in this segment; set with M42
};

// The three levels of homing confidence. If we ever switch off
// power to the motors after homing, we can't be sure.
enum HomingState {
  HOMING_STATE_NEVER_HOMED,
  HOMING_STATE_HOMED_BUT_MOTORS_UNPOWERED,
  HOMING_STATE_HOMED,
};

// The GCode control implementation. Essentially we are a state machine
// driven by the events we get from the gcode parsing.
// We implement the event receiver interface directly.
class GCodeMachineControl::Impl : public GCodeParser::EventReceiver {
public:
  // Create Impl. It is not fully initialized yet, call Init()
  Impl(const MachineControlConfig &config,
       MotorOperations *motor_ops,
       HardwareMapping *hardware_mapping,
       Spindle *spindle,
       FILE *msg_stream);

  // Initialize. Only if this succeeds, we have a properly initialized
  // object.
  bool Init();

  ~Impl() {}

  const MachineControlConfig &config() const { return cfg_; }
  void set_msg_stream(FILE *msg) { msg_stream_ = msg; }

  // -- GCodeParser::Events interface implementation --
  virtual void gcode_start();          // Start program. Use for initialization.
  virtual void gcode_finished(bool end_of_stream);  // End of program or stream.

  virtual void inform_origin_offset(const AxesRegister &origin);

  virtual void gcode_command_done(char letter, float val);
  virtual void input_idle(bool is_first);
  virtual void wait_for_start();
  virtual void go_home(AxisBitmap_t axis_bitmap);
  virtual bool probe_axis(float feed_mm_p_sec, enum GCodeParserAxis axis,
                          float *probed_position);
  virtual void set_speed_factor(float factor);    // M220 feedrate factor 0..1
  virtual void set_fanspeed(float value);         // M106, M107: speed 0...255
  virtual void set_temperature(float degrees_c);  // M104, M109: Set temp. in Celsius.
  virtual void wait_temperature();                // M109, M116: Wait for temp. reached.
  virtual void dwell(float time_ms);              // G4: dwell for milliseconds.
  virtual void motors_enable(bool enable);        // M17,M84,M18: Switch on/off motors
  virtual bool coordinated_move(float feed_mm_p_sec, const AxesRegister &target);
  virtual bool rapid_move(float feed_mm_p_sec, const AxesRegister &target);
  virtual const char *unprocessed(char letter, float value, const char *);

private:
  bool check_for_pause();
  void issue_motor_move_if_possible();
  void machine_move(float feedrate, const AxesRegister &axes);
  bool test_homing_status_ok();
  bool test_within_machine_limits(const AxesRegister &axes);
  void bring_path_to_halt();
  void get_endstop_status();
  void get_current_position();
  const char *aux_bit_commands(char letter, float value, const char *);
  const char *special_commands(char letter, float value, const char *);
  void assign_steps_to_motors(struct LinearSegmentSteps *command,
                              enum GCodeParserAxis axis,
                              int steps);
  void move_machine_steps(const struct AxisTarget *last_pos,
                          struct AxisTarget *target_pos,
                          const struct AxisTarget *upcoming);
  int move_to_endstop(enum GCodeParserAxis axis,
                      float feedrate, int backoff,
                      HardwareMapping::AxisTrigger trigger);
  void home_axis(enum GCodeParserAxis axis);
  void set_output_flags(HardwareMapping::LogicOutput out, bool is_on);

  // Print to msg_stream.
  void mprintf(const char *format, ...);

private:
  const struct MachineControlConfig cfg_;
  MotorOperations *const motor_ops_;
  HardwareMapping *const hardware_mapping_;
  Spindle *const spindle_;
  FILE *msg_stream_;

  // Derived configuration
  float g0_feedrate_mm_per_sec_;         // Highest of all axes; used for G0
                                         // (will be trimmed if needed)
  // Pre-calculated per axis limits in steps, steps/s, steps/s^2
  // All arrays are indexed by axis.
  AxesRegister max_axis_speed_;   // max travel speed hz
  AxesRegister max_axis_accel_;   // acceleration hz/s
  float highest_accel_;           // hightest accel of all axes.

  // Current machine configuration
  AxesRegister coordinate_display_origin_; // parser tells us
  float current_feedrate_mm_per_sec_;    // Set via Fxxx and remembered
  float prog_speed_factor_;              // Speed factor set by program (M220)
  HardwareMapping::AuxBitmap last_aux_bits_;  // last enqueued aux bits.
  time_t next_auto_disable_motor_;
  time_t next_auto_disable_fan_;
  bool pause_enabled_;                  // Enabled via M120, disabled via M121

  // Next buffered positions. Written by incoming gcode, read by outgoing
  // motor movements.
  RingDeque<AxisTarget, 4> planning_buffer_;

  enum HomingState homing_state_;
};

static inline int round2int(float x) { return (int) roundf(x); }

GCodeMachineControl::Impl::Impl(const MachineControlConfig &config,
                                MotorOperations *motor_ops,
                                HardwareMapping *hardware_mapping,
                                Spindle *spindle,
                                FILE *msg_stream)
  : cfg_(config),
    motor_ops_(motor_ops),
    hardware_mapping_(hardware_mapping),
    spindle_(spindle),
    msg_stream_(msg_stream),
    g0_feedrate_mm_per_sec_(-1),
    highest_accel_(-1),
    current_feedrate_mm_per_sec_(-1),
    prog_speed_factor_(1),
    homing_state_(HOMING_STATE_NEVER_HOMED) {
    pause_enabled_ = cfg_.enable_pause;
    next_auto_disable_motor_ = -1;
    next_auto_disable_fan_ = -1;
}

// The actual initialization. Can fail, hence we make it a separate method.
bool GCodeMachineControl::Impl::Init() {
  // Always keep the steps_per_mm positive, but extract direction for
  // final assignment to motor.
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (cfg_.steps_per_mm[i] < 0) {
      Log_error("Negative number of steps per unit for Axis %c",
                gcodep_axis2letter((GCodeParserAxis)i));
      return false;
    }
    // Permissible hard override. We want to keep cfg_ const for most of the
    // places, so writing here in Init() is permissible.
    const_cast<MachineControlConfig&>(cfg_).steps_per_mm[i]
                                           = fabsf(cfg_.steps_per_mm[i]);
    if (cfg_.max_feedrate[i] < 0) {
      Log_error("Invalid negative feedrate %.1f for axis %c\n",
              cfg_.max_feedrate[i], gcodep_axis2letter((GCodeParserAxis)i));
      return false;
    }
    if (cfg_.acceleration[i] < 0) {
      Log_error("Invalid negative acceleration %.1f for axis %c\n",
              cfg_.acceleration[i], gcodep_axis2letter((GCodeParserAxis)i));
      return false;
    }
  }

  current_feedrate_mm_per_sec_ = cfg_.max_feedrate[AXIS_X] / 10;
  float lowest_accel = cfg_.max_feedrate[AXIS_X] * cfg_.steps_per_mm[AXIS_X];
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (cfg_.max_feedrate[i] > g0_feedrate_mm_per_sec_) {
      g0_feedrate_mm_per_sec_ = cfg_.max_feedrate[i];
    }
    max_axis_speed_[i] = cfg_.max_feedrate[i] * cfg_.steps_per_mm[i];
    const float accel = cfg_.acceleration[i] * cfg_.steps_per_mm[i];
    max_axis_accel_[i] = accel;
    if (accel > highest_accel_)
      highest_accel_ = accel;
    if (accel < lowest_accel)
      lowest_accel = accel;
  }
  prog_speed_factor_ = 1.0f;

  int error_count = 0;

  // Check if things are plausible: we only allow one home endstop per axis.
  for (int a = 0; a < GCODE_NUM_AXES; ++a) {
    GCodeParserAxis axis = (GCodeParserAxis) a;
    const HardwareMapping::AxisTrigger homing_trigger = cfg_.homing_trigger[axis];
    if (homing_trigger == HardwareMapping::TRIGGER_NONE) {
      continue;
    }
    if (homing_trigger == HardwareMapping::TRIGGER_ANY) {
      Log_error("Error: There can only be one home-origin for axis %c, "
                "but both min/max are set for homing.\n",
                gcodep_axis2letter(axis));
      ++error_count;
      continue;
    }
    if ((hardware_mapping_->AvailableAxisSwitch(axis) & homing_trigger) == 0) {
      Log_error("Error: There is no switch associated in [switch-mapping] "
                "for the home origin of axis %c", gcodep_axis2letter(axis));
      ++error_count;
    }
  }

  // Now let's see what motors are mapped to any useful output.
  Log_debug("-- Config --\n");
  for (int ii = 0; ii < GCODE_NUM_AXES; ++ii) {
    const GCodeParserAxis axis = (GCodeParserAxis) ii;
    if (cfg_.steps_per_mm[axis] > 0 && !hardware_mapping_->HasMotorFor(axis)) {
      // User didn't give a motor mapping for this one. Auto map it.
      const int new_motor = hardware_mapping_->GetFirstFreeMotor();
      if (new_motor == 0) {
        Log_error("Not enough motor connectors: no free motor for axis %c",
                  gcodep_axis2letter(axis));
        ++error_count;
      }
      hardware_mapping_->AddMotorMapping(axis, new_motor, false);
    }
    if (!hardware_mapping_->HasMotorFor(axis))
      continue;
    const bool is_error = (cfg_.steps_per_mm[axis] <= 0
                           || cfg_.max_feedrate[axis] <= 0);
    // Some generic useful output
    std::string line;
    const char *unit = is_rotational_axis(axis) ? "° " : "mm";
    line = StringPrintf("%c axis: Motor %-4s|%5.1f%s/s, %7.1f%s/s^2, %9.4f steps/%s ",
                        gcodep_axis2letter(axis),
                        hardware_mapping_->DebugMotorString(axis).c_str(),
                        cfg_.max_feedrate[axis], unit,
                        cfg_.acceleration[axis], unit,
                        cfg_.steps_per_mm[axis], unit);
    if (cfg_.move_range_mm[axis] > 0) {
      line += StringPrintf("[ range %5.1f%s ] ", cfg_.move_range_mm[axis], unit);
    } else {
      line += "[ unknown range ] ";
    }
    if ((cfg_.homing_trigger[axis] & HardwareMapping::TRIGGER_MIN) != 0) {
      line += StringPrintf("|<-HOME@min; ");
    }
    else if ((cfg_.homing_trigger[axis] & HardwareMapping::TRIGGER_MAX) != 0) {
      line += StringPrintf("HOME@max->|; ");
    }

    if (!cfg_.range_check)
      line += "Limit checks disabled!";

    Log_debug("%s", line.c_str());

    if (is_error) {
      Log_error("ERROR: Motor %s configured for axis '%c', but invalid "
                "feedrate or steps/%s.",
                hardware_mapping_->DebugMotorString(axis).c_str(),
                gcodep_axis2letter(axis), unit);
      ++error_count;
    }
  }

  if (error_count)
    return false;

  // Initial machine position. We assume the homed position here, which is
  // wherever the endswitch is for each axis.
  struct AxisTarget *init_axis = planning_buffer_.append();
  bzero(init_axis, sizeof(*init_axis));
  for (int axis = 0; axis < GCODE_NUM_AXES; ++axis) {
    HardwareMapping::AxisTrigger trigger = cfg_.homing_trigger[axis];
    if (trigger == 0) {
      init_axis->position_steps[axis] = 0;
    }
    else {
      const float home_pos = trigger == HardwareMapping::TRIGGER_MIN
        ? 0 : cfg_.move_range_mm[axis];
      init_axis->position_steps[axis] = round2int(home_pos * cfg_.steps_per_mm[axis]);
    }
  }

  return true;
}

// machine-printf. Only prints if there is a msg-stream.
void GCodeMachineControl::Impl::mprintf(const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  if (msg_stream_) vfprintf(msg_stream_, format, ap);
  va_end(ap);
}

// Dummy implementations of callbacks not yet handled.
void GCodeMachineControl::Impl::set_temperature(float f) {
  mprintf("// BeagleG: set_temperature(%.1f) not implemented.\n", f);
}
void GCodeMachineControl::Impl::wait_temperature() {
  mprintf("// BeagleG: wait_temperature() not implemented.\n");
}
void GCodeMachineControl::Impl::motors_enable(bool b) {
  bring_path_to_halt();
  motor_ops_->MotorEnable(b);
  if (!b && homing_state_ == HOMING_STATE_HOMED) {
    homing_state_ = HOMING_STATE_HOMED_BUT_MOTORS_UNPOWERED;
  }
}
void GCodeMachineControl::Impl::gcode_command_done(char l, float v) {
  if (cfg_.acknowledge_lines) mprintf("ok\n");
}
void GCodeMachineControl::Impl::inform_origin_offset(const AxesRegister &o) {
  coordinate_display_origin_ = o;
}

void GCodeMachineControl::Impl::set_fanspeed(float speed) {
  if (speed < 0.0 || speed > 255.0) return;
  float duty_cycle = speed / 255.0;
  // The fan can be mapped to an aux and/or pwm signal
  set_output_flags(HardwareMapping::OUT_FAN, duty_cycle > 0.0);
  hardware_mapping_->SetPWMOutput(HardwareMapping::OUT_FAN, duty_cycle);
}

void GCodeMachineControl::Impl::wait_for_start() {
  int flash_usec = 100 * 1000;
  while (!hardware_mapping_->TestStartSwitch()) {
    set_output_flags(HardwareMapping::OUT_LED, true);
    hardware_mapping_->SetAuxOutputs();
    usleep(flash_usec);
    set_output_flags(HardwareMapping::OUT_LED, false);
    hardware_mapping_->SetAuxOutputs();
    usleep(flash_usec);
  }
}

bool GCodeMachineControl::Impl::check_for_pause() {
  return hardware_mapping_->TestPauseSwitch();
}

const char *GCodeMachineControl::Impl::unprocessed(char letter, float value,
                                                   const char *remaining) {
  return special_commands(letter, value, remaining);
}

const char *GCodeMachineControl::Impl::special_commands(char letter, float value,
                                                        const char *remaining) {
  // Only M commands are handled here
  if (letter != 'M') return remaining;

  const int code = (int)value;
  switch (code) {
  case 0:
  case 999:
    set_output_flags(HardwareMapping::OUT_ESTOP, code == 0);
    hardware_mapping_->SetAuxOutputs();
    break;
  case 3: case 4: case 5:                   // aux pin spindle control
  case 7: case 8: case 9: case 10: case 11: // aux pin mist/flood/vacuum control
  case 42:                                  // aux pin state query
  case 62: case 63: case 64: case 65:       // aux pin set
  case 245: case 246:                       // aux pin cooler control
  case 355:                                 // aux pin case lights control
    remaining = aux_bit_commands(letter, value, remaining);
    break;
  case 80:
  case 81:
    set_output_flags(HardwareMapping::OUT_ATX_POWER, code == 80);
    hardware_mapping_->SetAuxOutputs();
    break;
  case 105: mprintf("T-300\n"); break;  // no temp yet.
  case 114: get_current_position(); break;
  case 115: mprintf("%s\n", VERSION_STRING); break;
  case 117:
    mprintf("// Msg: %s\n", remaining); // TODO: different output ?
    remaining = NULL;  // consume the full line.
    break;
  case 119: get_endstop_status(); break;
  case 120: pause_enabled_ = true; break;
  case 121: pause_enabled_ = false; break;
  default:
    mprintf("// BeagleG: didn't understand ('%c', %d, '%s')\n",
            letter, code, remaining);
    remaining = NULL;  // In this case, let's just discard remainig block.
    break;
  }
  return remaining;
}

const char *GCodeMachineControl::Impl::aux_bit_commands(char letter, float value,
                                                        const char *remaining) {
  const int m_code = (int)value;
  const char* after_pair;
  int spindle_rpm = -1;

  switch (m_code) {
  case 3: case 4:
    for (;;) {
      after_pair = GCodeParser::ParsePair(remaining, &letter, &value, msg_stream_);
      if (after_pair == NULL) break;
      else if (letter == 'S') spindle_rpm = round2int(value);
      else break;
      remaining = after_pair;
    }
    if (spindle_rpm >= 0) spindle_->On(m_code == 4, spindle_rpm);
    break;
  case 5:
    spindle_->Off();
    break;
  case 7: set_output_flags(HardwareMapping::OUT_MIST, true); break;
  case 8: set_output_flags(HardwareMapping::OUT_FLOOD, true); break;
  case 9:
    set_output_flags(HardwareMapping::OUT_MIST, false);
    set_output_flags(HardwareMapping::OUT_FLOOD, false);
    break;
  case 10: set_output_flags(HardwareMapping::OUT_VACUUM, true); break;
  case 11: set_output_flags(HardwareMapping::OUT_VACUUM, false); break;
  case 42:
  case 62: case 63: case 64: case 65: {
    int bit_value = -1;
    int pin = -1;
    for (;;) {
      after_pair = GCodeParser::ParsePair(remaining, &letter, &value, msg_stream_);
      if (after_pair == NULL) break;
      if (letter == 'P') pin = round2int(value);
      else if (letter == 'S' && m_code == 42) bit_value = round2int(value);
      else break;
      remaining = after_pair;
    }

    if (m_code == 62 || m_code == 64)
      bit_value = 1;
    else if (m_code == 63 || m_code == 65)
      bit_value = 0;

    if (pin > 0 && pin <= HardwareMapping::NUM_BOOL_OUTPUTS) {
      if (bit_value >= 0 && bit_value <= 1) {
        hardware_mapping_->UpdateAuxBits(pin, bit_value == 1);

        if (m_code == 64 || m_code == 65) {    // update the AUX pin immediately
          hardware_mapping_->SetAuxOutputs();
        }
      }
      else if (m_code == 42 && msg_stream_) {  // Just read operation.
        mprintf("%d\n", hardware_mapping_->GetAuxBit(pin));
      }
    }
  }
    break;
  case 245: set_output_flags(HardwareMapping::OUT_COOLER, true); break;
  case 246: set_output_flags(HardwareMapping::OUT_COOLER, false); break;
  case 355: {
    int on_value = 0;
    for (;;) {
      after_pair = GCodeParser::ParsePair(remaining, &letter, &value, msg_stream_);
      if (after_pair == NULL) break;
      if (letter == 'S') on_value = round2int(value);
      else break;
      remaining = after_pair;
    }
    set_output_flags(HardwareMapping::OUT_CASE_LIGHTS, on_value > 0);
  }
    break;
  }
  return remaining;
}

void GCodeMachineControl::Impl::set_output_flags(HardwareMapping::LogicOutput out,
                                                 bool is_on) {
  hardware_mapping_->UpdateAuxBitmap(out, is_on);
}

void GCodeMachineControl::Impl::get_current_position() {
  if (planning_buffer_.size() > 0) {
    const struct AxisTarget *current = planning_buffer_[0];
    const int *mpos = current->position_steps;
    const float x = 1.0f * mpos[AXIS_X] / cfg_.steps_per_mm[AXIS_X];
    const float y = 1.0f * mpos[AXIS_Y] / cfg_.steps_per_mm[AXIS_Y];
    const float z = 1.0f * mpos[AXIS_Z] / cfg_.steps_per_mm[AXIS_Z];
    const float e = 1.0f * mpos[AXIS_E] / cfg_.steps_per_mm[AXIS_E];
    const AxesRegister &origin = coordinate_display_origin_;
    mprintf("X:%.3f Y:%.3f Z:%.3f E:%.3f",
            x - origin[AXIS_X], y - origin[AXIS_Y], z - origin[AXIS_Z],
            e - origin[AXIS_E]);
    mprintf(" [ABS. MACHINE CUBE X:%.3f Y:%.3f Z:%.3f]", x, y, z);
    switch (homing_state_) {
    case HOMING_STATE_NEVER_HOMED:
      mprintf(" (Unsure: machine never homed!)\n");
      break;
    case HOMING_STATE_HOMED_BUT_MOTORS_UNPOWERED:
      mprintf(" (Lower confidence: motor power off at least once after homing)\n");
      break;
    case HOMING_STATE_HOMED:
      mprintf(" (confident: machine was homed)\n");
      break;
    }
  } else {
    mprintf("// no current pos\n");
  }
}

void GCodeMachineControl::Impl::get_endstop_status() {
  bool any_endstops_found = false;
  for (int ai = 0; ai < GCODE_NUM_AXES; ++ai) {
    const  GCodeParserAxis axis = (GCodeParserAxis) ai;
    HardwareMapping::AxisTrigger triggers = hardware_mapping_->AvailableAxisSwitch(axis);
    if ((triggers & HardwareMapping::TRIGGER_MIN) != 0) {
      mprintf("%c_min:%s:%s ",
              tolower(gcodep_axis2letter(axis)),
              cfg_.homing_trigger[axis] == HardwareMapping::TRIGGER_MIN
              ? "home" : "limit",
              hardware_mapping_->TestAxisSwitch(axis, HardwareMapping::TRIGGER_MIN)
              ? "TRIGGERED" : "open");
      any_endstops_found = true;
    }
    if ((triggers & HardwareMapping::TRIGGER_MAX) != 0) {
      mprintf("%c_max:%s:%s ",
              tolower(gcodep_axis2letter(axis)),
              cfg_.homing_trigger[axis] == HardwareMapping::TRIGGER_MAX
              ? "home" : "limit",
              hardware_mapping_->TestAxisSwitch(axis, HardwareMapping::TRIGGER_MAX)
              ? "TRIGGERED" : "open");
      any_endstops_found = true;
    }
  }
  if (any_endstops_found) {
    mprintf("\n");
  } else {
    mprintf("// This machine has no endstops configured.\n");
  }
}

void GCodeMachineControl::Impl::gcode_start() {
  if (cfg_.auto_fan_pwm > 0)
    set_fanspeed(cfg_.auto_fan_pwm);
}

void GCodeMachineControl::Impl::gcode_finished(bool end_of_stream) {
  bring_path_to_halt();
  spindle_->Off();
  if (end_of_stream && cfg_.auto_motor_disable_seconds > 0)
    motors_enable(false);
}

static float euclid_distance(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

// Number of steps to accelerate or decelerate (negative "a") from speed
// v0 to speed v1. Modifies v1 if we can't reach the speed with the allocated
// number of steps.
static float steps_for_speed_change(float a, float v0, float *v1, int max_steps) {
  // s = v0 * t + a/2 * t^2
  // v1 = v0 + a*t
  const float t = (*v1 - v0) / a;
  // TODO:
  if (t < 0) Log_error("Error condition: t=%.1f INSUFFICIENT LOOKAHEAD\n", t);
  float steps = a/2 * t*t + v0 * t;
  if (steps <= max_steps) return steps;
  // Ok, we would need more steps than we have available. We correct the speed to what
  // we actually can reach.
  *v1 = sqrtf(v0*v0 + 2 * a * max_steps);
  return max_steps;
}

// Example with speed: given the axis with the highest (relative) out of bounds speed
// what's the speed of the defining_axis? All the axes should be scaled of this maximum offset.
// offset = limit[i] / value[i]
static float clamp_to_limits(const float value,
                             const AxesRegister &max_axis_value,
                             const int *axis_steps,
                             enum GCodeParserAxis defining_axis) {
  float max_offset = 1, offset;
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    offset = axis_steps[i] > 0 ? fabs(max_axis_value[i] * axis_steps[defining_axis] / (value * axis_steps[i])) : 1;
    if (offset < max_offset) max_offset = offset;
  }
  return value * max_offset;
}

// Given that we want to travel "s" steps, start with speed "v0",
// accelerate peak speed v1 and slow down to "v2" with acceleration "a",
// what is v1 ?
static float get_peak_speed(float s, float v0, float v2, float a) {
  return sqrtf(v2*v2 + v0*v0 + 2 * a * s) / sqrtf(2);
}

// Speed relative to defining axis
static float get_speed_factor_for_axis(const struct AxisTarget *t,
                                       enum GCodeParserAxis request_axis) {
  if (t->delta_steps[t->defining_axis] == 0) return 0.0f;
  return 1.0f * t->delta_steps[request_axis] / t->delta_steps[t->defining_axis];
}

// Get the speed for a particular axis. Depending on the direction, this can
// be positive or negative.
static float get_speed_for_axis(const struct AxisTarget *target,
                                enum GCodeParserAxis request_axis) {
  return target->speed * get_speed_factor_for_axis(target, request_axis);
}

static char within_acceptable_range(float new_val, float old_val, float fraction) {
  const float max_diff = fraction * old_val;
  if (new_val < old_val - max_diff) return 0;
  if (new_val > old_val + max_diff) return 0;
  return 1;
}

// Determine the fraction of the speed that "from" should decelerate
// to at the end of its travel.
// The way trapezoidal moves work, be still have to decelerate to zero in
// most times, which is inconvenient. TODO(hzeller): speed matching is not
// cutting it :)
static float determine_joining_speed(const struct AxisTarget *from,
                                     const struct AxisTarget *to,
                                     const float threshold,
                                     const float angle) {
  // Our goal is to figure out what our from defining speed should
  // be at the end of the move.
  char is_first = 1;
  float from_defining_speed = from->speed;
  for (int ai = 0; ai < GCODE_NUM_AXES; ++ai) {
    const GCodeParserAxis axis = (GCodeParserAxis) ai;
    const int from_delta = from->delta_steps[axis];
    const int to_delta = to->delta_steps[axis];

    // Quick integer decisions
    if (angle < threshold) continue;
    if (from_delta == 0 && to_delta == 0) continue;   // uninteresting: no move.
    if (from_delta == 0 || to_delta == 0) return 0.0f; // accel from/to zero
    if ((from_delta < 0 && to_delta > 0) || (from_delta > 0 && to_delta < 0))
      return 0.0f;  // turing around

    float to_speed = get_speed_for_axis(to, axis);
    // What would this speed translated to our defining axis be ?
    float speed_conversion = 1.0f * from->delta_steps[from->defining_axis] / from->delta_steps[axis];
    float goal = to_speed * speed_conversion;
    if (goal < 0.0f) return 0.0f;
    if (is_first || within_acceptable_range(goal, from_defining_speed, 1e-5)) {
      if (goal < from_defining_speed) from_defining_speed = goal;
      is_first = 0;
    } else {
      return 0.0f;  // Too far off.
    }
  }
  return from_defining_speed;
}

// Assign steps to all the motors responsible for given axis.
void GCodeMachineControl::Impl::assign_steps_to_motors(struct LinearSegmentSteps *command,
                                                       enum GCodeParserAxis axis,
                                                       int steps) {
  hardware_mapping_->AssignMotorSteps(axis, steps, command);
}

// Returns true, if all results in zero movement
static uint8_t substract_steps(struct LinearSegmentSteps *value,
                               const struct LinearSegmentSteps &substract) {
  uint8_t has_nonzero = 0;
  for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
    value->steps[i] -= substract.steps[i];
    has_nonzero |= (value->steps[i] != 0);
  }
  return has_nonzero;
}

// Move the given number of machine steps for each axis.
//
// This will be up to three segments: accelerating from last_pos speed to
// target speed, regular travel, and decelerating to the speed that the
// next segment is never forced to decelerate, but stays at speed or accelerate.
//
// The segments are sent to the motor operations backend.
//
// Since we calculate the deceleration, this modifies the speed of target_pos
// to reflect what the last speed was at the end of the move.
void GCodeMachineControl::Impl::move_machine_steps(const struct AxisTarget *last_pos,
                                                   struct AxisTarget *target_pos,
                                                   const struct AxisTarget *upcoming) {
  if (target_pos->delta_steps[target_pos->defining_axis] == 0) {
    if (last_aux_bits_ != target_pos->aux_bits) {
      // Special treatment: bits changed since last time, let's push them through.
      struct LinearSegmentSteps bit_set_command = {0};
      bit_set_command.aux_bits = target_pos->aux_bits;
      motor_ops_->Enqueue(bit_set_command, msg_stream_);
      last_aux_bits_ = target_pos->aux_bits;
    }
    return;
  }
  struct LinearSegmentSteps accel_command = {0};
  struct LinearSegmentSteps move_command = {0};
  struct LinearSegmentSteps decel_command = {0};

  assert(target_pos->speed > 0);  // Speed is always a positive scalar.

  // Aux bits are set synchronously with what we need.
  move_command.aux_bits = target_pos->aux_bits;
  const enum GCodeParserAxis defining_axis = target_pos->defining_axis;

  // Common settings.
  memcpy(&accel_command, &move_command, sizeof(accel_command));
  memcpy(&decel_command, &move_command, sizeof(decel_command));

  move_command.v0 = target_pos->speed;
  move_command.v1 = target_pos->speed;

  // Let's see what our defining axis had as speed in the previous segment. The
  // last segment might have had a different defining axis, so we calculate
  // what the the fraction of the speed that our _current_ defining axis had.
  const float last_speed = fabsf(get_speed_for_axis(last_pos, defining_axis));

  // We need to arrive at a speed that the upcoming move does not have
  // to decelerate further (after all, it has a fixed feed-rate it should not
  // go over).
  float next_speed = determine_joining_speed(target_pos, upcoming,
                                             cfg_.threshold_angle,
                                             fabsf(last_pos->angle - target_pos->angle));

  const int *axis_steps = target_pos->delta_steps;  // shortcut.
  const int abs_defining_axis_steps = abs(axis_steps[defining_axis]);
  static float a =
    clamp_to_limits(max_axis_accel_[defining_axis],
                    max_axis_accel_,
                    axis_steps,
                    defining_axis);
  const float peak_speed = get_peak_speed(abs_defining_axis_steps,
                                          last_speed, next_speed, a);
  assert(peak_speed > 0);

  // TODO: if we only have < 5 steps or so, we should not even consider
  // accelerating or decelerating, but just do one speed.

  if (peak_speed < target_pos->speed) {
    target_pos->speed = peak_speed;  // Don't manage to accelerate to desired v
  }

  const float accel_fraction =
    (last_speed < target_pos->speed)
    ? steps_for_speed_change(a, last_speed, &target_pos->speed,
                             abs_defining_axis_steps) / abs_defining_axis_steps
    : 0;

  // We only decelerate if the upcoming speed is _slower_
  float dummy_next_speed = next_speed;  // Don't care to modify; we don't have
  const float decel_fraction =
    (next_speed < target_pos->speed)
    ? steps_for_speed_change(-a, target_pos->speed, &dummy_next_speed,
                             abs_defining_axis_steps) / abs_defining_axis_steps
    : 0;

  assert(accel_fraction + decel_fraction <= 1.0 + 1e-4);

#if 1
  // fudging: if we have tiny acceleration segments, don't do these at all
  // but only do speed; otherwise we have a lot of rattling due to many little
  // segments of acceleration/deceleration (e.g. for G2/G3).
  // This is not optimal. Ideally, we would actually calculate in terms of
  // jerk and optimize to stay within that constraint.
  const int accel_decel_steps
    = (accel_fraction + decel_fraction) * abs_defining_axis_steps;
  const float accel_decel_mm
    = (accel_decel_steps / cfg_.steps_per_mm[defining_axis]);
  const char do_accel = (accel_decel_mm > 2 || accel_decel_steps > 16);
#else
  const char do_accel = 1;
#endif

  char has_accel = 0;
  char has_move = 0;
  char has_decel = 0;

  if (do_accel && accel_fraction * abs_defining_axis_steps > 0) {
    has_accel = 1;
    accel_command.v0 = last_speed;           // Last speed of defining axis
    accel_command.v1 = target_pos->speed;    // New speed of defining axis

    // Now map axis steps to actual motor driver
    for (int i = 0; i < GCODE_NUM_AXES; ++i) {
      const int accel_steps = round2int(accel_fraction * axis_steps[i]);
      assign_steps_to_motors(&accel_command, (GCodeParserAxis)i,
                             accel_steps);
    }
  }

  if (do_accel && decel_fraction * abs_defining_axis_steps > 0) {
    has_decel = 1;
    decel_command.v0 = target_pos->speed;
    decel_command.v1 = next_speed;
    target_pos->speed = next_speed;

    // Now map axis steps to actual motor driver
    for (int i = 0; i < GCODE_NUM_AXES; ++i) {
      const int decel_steps = round2int(decel_fraction * axis_steps[i]);
      assign_steps_to_motors(&decel_command, (GCodeParserAxis)i,
                             decel_steps);
    }
  }

  // Move is everything that hasn't been covered in speed changes.
  // So we start with all steps and substract steps done in acceleration and
  // deceleration.
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    assign_steps_to_motors(&move_command, (GCodeParserAxis)i,
                           axis_steps[i]);
  }
  substract_steps(&move_command, accel_command);
  has_move = substract_steps(&move_command, decel_command);

  if (cfg_.synchronous) {
    motor_ops_->WaitQueueEmpty();
  }
  if (has_accel) motor_ops_->Enqueue(accel_command, msg_stream_);
  if (has_move) motor_ops_->Enqueue(move_command, msg_stream_);
  if (has_decel) motor_ops_->Enqueue(decel_command, msg_stream_);

  last_aux_bits_ = target_pos->aux_bits;
}

// If we have enough data in the queue, issue motor move.
void GCodeMachineControl::Impl::issue_motor_move_if_possible() {
  if (planning_buffer_.size() >= 3) {
    move_machine_steps(planning_buffer_[0],  // Current established position.
                       planning_buffer_[1],  // Position we want to move to.
                       planning_buffer_[2]); // Next upcoming.
    planning_buffer_.pop_front();
  }
}

void GCodeMachineControl::Impl::machine_move(float feedrate,
                                             const AxesRegister &axis) {
  // We always have a previous position.
  struct AxisTarget *previous = planning_buffer_.back();
  struct AxisTarget *new_pos = planning_buffer_.append();
  int max_steps = -1;
  enum GCodeParserAxis defining_axis = AXIS_X;
  enum GCodeParserAxis euclidean_defining_axis = AXIS_X;

  // Real world -> machine coordinates. Here, we are rounding to the next full
  // step, but we never accumulate the error, as we always use the absolute
  // position as reference.
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    new_pos->position_steps[i] = round2int(axis[i] * cfg_.steps_per_mm[i]);
    new_pos->delta_steps[i] = new_pos->position_steps[i] - previous->position_steps[i];

    // The defining axis is the one that has to travel the most steps. It defines
    // the frequency to go.
    // All the other axes are doing a fraction of the defining axis.
    if (abs(new_pos->delta_steps[i]) > max_steps) {
      max_steps = abs(new_pos->delta_steps[i]);
      defining_axis = (enum GCodeParserAxis) i;
      if(i <= AXIS_Z) euclidean_defining_axis = (enum GCodeParserAxis) i;
    }
  }
  new_pos->aux_bits = hardware_mapping_->GetAuxBits();
  new_pos->defining_axis = defining_axis;
  new_pos->angle = previous->angle + 180.0f; // default angle to force a speed change

  // Now let's calculate the travel speed in steps/s on the defining axis.
  if (max_steps > 0) {
    float travel_speed = feedrate * cfg_.steps_per_mm[euclidean_defining_axis];

    // If we're in the euclidian space, choose the step-frequency according to
    // the relative feedrate of the defining axis.
    // (A straight 200mm/s should be the same as a diagnoal 200mm/s)
    // (If we don't have motion in XYZ, the feedrate should represent the mm/min of the extruder?
    // should we allow the user to set the tool speed only with the S command?)
    if (new_pos->delta_steps[euclidean_defining_axis] != 0) {
      // We need to calculate the feedrate in real-world coordinates as each
      // axis can have a different amount of steps/mm
      // TODO(hzeller): avoid this back calculation.

      // DSTEPS: avoid division by zero if there is no config defined for axis.
#define DSTEPS(ax) (new_pos->delta_steps[ax]) ? new_pos->delta_steps[ax] / cfg_.steps_per_mm[ax] : 0
      const float dx = DSTEPS(AXIS_X);
      const float dy = DSTEPS(AXIS_Y);
      const float dz = DSTEPS(AXIS_Z);
#undef DSTEPS
      const float total_xyz_len_mm = euclid_distance(dx, dy, dz);
      const float steps_per_mm = cfg_.steps_per_mm[euclidean_defining_axis];
      const float defining_axis_len_mm = new_pos->delta_steps[euclidean_defining_axis] / steps_per_mm;
      const float euclid_fraction = fabsf(defining_axis_len_mm) / total_xyz_len_mm;
      travel_speed *= euclid_fraction * max_steps / fabsf(new_pos->delta_steps[euclidean_defining_axis]);

      // If this is a truish XY vector, calculate the angle of the vector
      if (fabsf(dz) < 0.01)
        new_pos->angle = (atan2f(dy, dx) / 3.14159265359) * 180.0f;
    } else {
      travel_speed = feedrate * cfg_.steps_per_mm[defining_axis];
    }
    new_pos->speed = clamp_to_limits(travel_speed,
                                     max_axis_speed_,
                                     new_pos->delta_steps,
                                     defining_axis);
  } else {
    new_pos->speed = 0;
  }

  issue_motor_move_if_possible();
}

void GCodeMachineControl::Impl::bring_path_to_halt() {
  // Enqueue a new position that is the same position as the last
  // one seen, but zero speed. That will allow the previous segment to
  // slow down. Enqueue.
  struct AxisTarget *previous = planning_buffer_.back();
  struct AxisTarget *new_pos = planning_buffer_.append();
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    new_pos->position_steps[i] = previous->position_steps[i];
    new_pos->delta_steps[i] = 0;
  }
  new_pos->defining_axis = AXIS_X;
  new_pos->speed = 0;
  new_pos->aux_bits = hardware_mapping_->GetAuxBits();
  new_pos->angle = 0;
  issue_motor_move_if_possible();
}

bool GCodeMachineControl::Impl::test_homing_status_ok() {
  if (!cfg_.require_homing)
    return true;
  if (homing_state_ > HOMING_STATE_NEVER_HOMED)
    return true;
  mprintf("// ERROR: please home machine first (G28).\n");
  return false;
}

bool GCodeMachineControl::Impl::test_within_machine_limits(const AxesRegister &axes) {
  if (!cfg_.range_check)
    return true;

  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    // Min range ...
    if (axes[i] < 0) {
      // Machine cube must be in positive range.
      if (coordinate_display_origin_[i] != 0) {
        mprintf("// ERROR outside machine limit: Axis %c < min allowed "
                "%+.1fmm in current coordinate system. Ignoring move!\n",
                gcodep_axis2letter((GCodeParserAxis)i),
                -coordinate_display_origin_[i]);
      } else {
        // No relative G92 or similar set. Display in simpler form.
        mprintf("// ERROR outside machine limit: Axis %c < 0. "
                "Ignoring move!\n", gcodep_axis2letter((GCodeParserAxis)i));
      }
      return false;
    }

    // Max range ..
    if (cfg_.move_range_mm[i] <= 0)
      continue;  // max range not configured.
    const float max_limit = cfg_.move_range_mm[i];
    if (axes[i] > max_limit) {
      // Machine cube must be within machine limits if defined.
      if (coordinate_display_origin_[i] != 0) {
        mprintf("// ERROR outside machine limit: Axis %c > max allowed %+.1fmm "
                "in current coordinate system (=%.1fmm machine absolute). "
                "Ignoring move!\n",
                gcodep_axis2letter((GCodeParserAxis)i),
                max_limit - coordinate_display_origin_[i], max_limit);
      } else {
        // No relative G92 or similar set. Display in simpler form.
        mprintf("// ERROR outside machine limit: Axis %c > %.1fmm. "
                "Ignoring move!\n", gcodep_axis2letter((GCodeParserAxis)i),
                max_limit);
      }
      return false;
    }
  }
  return true;
}

bool GCodeMachineControl::Impl::coordinated_move(float feed,
                                                 const AxesRegister &axis) {
  if (!test_homing_status_ok())
    return false;
  if (!test_within_machine_limits(axis))
    return false;
  if (feed > 0) {
    current_feedrate_mm_per_sec_ = cfg_.speed_factor * feed;
  }
  float feedrate = prog_speed_factor_ * current_feedrate_mm_per_sec_;
  machine_move(feedrate, axis);
  return true;
}

bool GCodeMachineControl::Impl::rapid_move(float feed,
                                           const AxesRegister &axis) {
  if (!test_homing_status_ok())
    return false;
  if (!test_within_machine_limits(axis))
    return false;
  float rapid_feed = g0_feedrate_mm_per_sec_;
  const float given = cfg_.speed_factor * prog_speed_factor_ * feed;
  machine_move(given > 0 ? given : rapid_feed, axis);
  return true;
}

void GCodeMachineControl::Impl::dwell(float value) {
  bring_path_to_halt();
  motor_ops_->WaitQueueEmpty();
  usleep((int) (value * 1000));

  if (pause_enabled_ && check_for_pause()) {
    Log_debug("Pause input detected, waiting for Start");
    wait_for_start();
  }
}

void GCodeMachineControl::Impl::input_idle(bool is_first) {
  bring_path_to_halt();
  if (cfg_.auto_motor_disable_seconds > 0) {
    if (is_first) {
      next_auto_disable_motor_ = time(NULL) + cfg_.auto_motor_disable_seconds;
      next_auto_disable_fan_ = -1;
    } else if (next_auto_disable_motor_ != -1 &&
               time(NULL) >= next_auto_disable_motor_) {
      motors_enable(false);
      next_auto_disable_motor_ = -1;
      if (cfg_.auto_fan_disable_seconds > 0) {
        next_auto_disable_fan_ = time(NULL) + cfg_.auto_fan_disable_seconds;
      }
    }
  }
  if (next_auto_disable_fan_ != -1 && time(NULL) >= next_auto_disable_fan_) {
    set_fanspeed(0);
    next_auto_disable_fan_ = -1;
  }
}

void GCodeMachineControl::Impl::set_speed_factor(float value) {
  if (value < 0) {
    value = 1.0f + value;   // M220 S-10 interpreted as: 90%
  }
  if (value < 0.005) {
    mprintf("// M220: Not accepting speed factors < 0.5%% (got %.1f%%)\n",
            100.0f * value);
    return;
  }
  prog_speed_factor_ = value;
}

// Moves to endstop and returns how many steps it moved in the process.
int GCodeMachineControl::Impl::move_to_endstop(enum GCodeParserAxis axis,
                                               float feedrate, int backoff,
                                               HardwareMapping::AxisTrigger trigger) {
  int total_movement = 0;
  struct LinearSegmentSteps move_command = {0};
  const float steps_per_mm = cfg_.steps_per_mm[axis];
  float target_speed = feedrate * steps_per_mm;
  if (target_speed > max_axis_speed_[axis]) {
    target_speed = max_axis_speed_[axis];
  }

  move_command.aux_bits = hardware_mapping_->GetAuxBits();

  move_command.v0 = 0;
  move_command.v1 = target_speed;

  const int dir = trigger == HardwareMapping::TRIGGER_MIN ? -1 : 1;
  // move axis until endstop is hit
  int segment_move_steps = 0.5 * steps_per_mm * dir;
  assign_steps_to_motors(&move_command, axis, segment_move_steps);
  while (!hardware_mapping_->TestAxisSwitch(axis, trigger)) {
    motor_ops_->Enqueue(move_command, msg_stream_);
    motor_ops_->WaitQueueEmpty();
    total_movement += segment_move_steps;
    // TODO: possibly acceleration over multiple segments.
    move_command.v0 = move_command.v1;
  }

  if (backoff) {
    // move axis off endstop
    segment_move_steps = 0.1 * steps_per_mm * -dir;
    assign_steps_to_motors(&move_command, axis, segment_move_steps);
    while (hardware_mapping_->TestAxisSwitch(axis, trigger)) {
      motor_ops_->Enqueue(move_command, msg_stream_);
      motor_ops_->WaitQueueEmpty();
      total_movement += segment_move_steps;
    }
  }

  return total_movement;
}

void GCodeMachineControl::Impl::home_axis(enum GCodeParserAxis axis) {
  const HardwareMapping::AxisTrigger trigger = cfg_.homing_trigger[axis];
  if (trigger == HardwareMapping::TRIGGER_NONE)
    return;   // no homing defined
  struct AxisTarget *last = planning_buffer_.back();
  move_to_endstop(axis, 15, 1, trigger);

  const float home_pos = trigger == HardwareMapping::TRIGGER_MIN ?
    0 : cfg_.move_range_mm[axis];
  last->position_steps[axis] = round2int(home_pos * cfg_.steps_per_mm[axis]);
}

void GCodeMachineControl::Impl::go_home(AxisBitmap_t axes_bitmap) {
  bring_path_to_halt();
  for (std::string::const_iterator o = cfg_.home_order.begin();
       o != cfg_.home_order.end() ; ++o) {
    const enum GCodeParserAxis axis = gcodep_letter2axis(*o);
    if (axis == GCODE_NUM_AXES || !(axes_bitmap & (1 << axis)))
      continue;
    home_axis(axis);
  }
  homing_state_ = HOMING_STATE_HOMED;
}

bool GCodeMachineControl::Impl::probe_axis(float feedrate,
                                           enum GCodeParserAxis axis,
                                           float *probe_result) {
  if (!test_homing_status_ok())
    return false;

  bring_path_to_halt();

  // -- somewhat hackish

  // We try to find the axis that is _not_ used for homing.
  // this is not yet 100% the way it should be. We should actually
  // define the probe-'endstops' somewhat differently.
  // For now, we just do the simple thing
  HardwareMapping::AxisTrigger home_trigger = cfg_.homing_trigger[axis];
  HardwareMapping::AxisTrigger probe_trigger;
  if (home_trigger == HardwareMapping::TRIGGER_MIN) {
    probe_trigger = HardwareMapping::TRIGGER_MAX;
  } else {
    probe_trigger = HardwareMapping::TRIGGER_MIN;
  }
  if ((hardware_mapping_->AvailableAxisSwitch(axis) & probe_trigger) == 0) {
    mprintf("// BeagleG: No probe - axis %c does not have a travel endstop\n",
            gcodep_axis2letter(axis));
    return false;
  }

  struct AxisTarget *last = planning_buffer_.back();
  if (feedrate <= 0) feedrate = 20;
  // TODO: if the probe fails to trigger, there is no mechanism to stop
  // it right now...
  int total_steps = move_to_endstop(axis, feedrate, 0, probe_trigger);
  last->position_steps[axis] += total_steps;
  *probe_result = 1.0f * last->position_steps[axis] / cfg_.steps_per_mm[axis];
  return true;
}

GCodeMachineControl::GCodeMachineControl(Impl *impl) : impl_(impl) {
}
GCodeMachineControl::~GCodeMachineControl() {
  delete impl_;
}

GCodeMachineControl* GCodeMachineControl::Create(const MachineControlConfig &config,
                                                 MotorOperations *motor_ops,
                                                 HardwareMapping *hardware_mapping,
                                                 Spindle *spindle,
                                                 FILE *msg_stream) {
  Impl *result = new Impl(config, motor_ops, hardware_mapping, spindle, msg_stream);
  if (!result->Init()) {
    delete result;
    return NULL;
  }
  return new GCodeMachineControl(result);
}

void GCodeMachineControl::GetHomePos(AxesRegister *home_pos) {
  home_pos->zero();
  for (int axis = 0; axis < GCODE_NUM_AXES; ++axis) {
    HardwareMapping::AxisTrigger trigger = impl_->config().homing_trigger[axis];
    (*home_pos)[axis] = (trigger & HardwareMapping::TRIGGER_MAX)
      ? impl_->config().move_range_mm[axis] : 0;
  }
}

GCodeParser::EventReceiver *GCodeMachineControl::ParseEventReceiver() {
  return impl_;
}

void GCodeMachineControl::SetMsgOut(FILE *msg_stream) {
  impl_->set_msg_stream(msg_stream);
}
