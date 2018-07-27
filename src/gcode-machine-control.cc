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

#include "common/container.h"
#include "common/logging.h"
#include "common/string-util.h"
#include "gcode-parser/gcode-parser.h"

#include "adc.h"
#include "generic-gpio.h"
#include "hardware-mapping.h"
#include "motor-operations.h"
#include "planner.h"
#include "pwm-timer.h"
#include "spindle-control.h"

// In case we get a zero feedrate, send this frequency to motors instead.
#define ZERO_FEEDRATE_OVERRIDE_HZ 5

#define VERSION_STRING "PROTOCOL_VERSION:0.1 FIRMWARE_NAME:BeagleG "    \
  "CAPE:" CAPE_NAME " FIRMWARE_URL:http%3A//github.com/hzeller/beagleg"

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

  ~Impl() {
    delete planner_;
  }

  const MachineControlConfig &config() const { return cfg_; }
  void set_msg_stream(FILE *msg) { msg_stream_ = msg; }
  EStopState GetEStopStatus();
  HomingState GetHomeStatus();
  bool GetMotorsEnabled();
  void GetCurrentPosition(AxesRegister *pos);

  // -- GCodeParser::Events interface implementation --
  void gcode_start(GCodeParser *parser) final;
  void gcode_finished(bool end_of_stream) final;  // End of program or stream.

  void inform_origin_offset(const AxesRegister &origin, const char *n) final;

  void gcode_command_done(char letter, float val) final;
  void input_idle(bool is_first) final;
  void wait_for_start() final;
  void go_home(AxisBitmap_t axis_bitmap) final;
  bool probe_axis(float feed_mm_p_sec, enum GCodeParserAxis axis,
                          float *probed_position) final;
  void set_speed_factor(float factor) final;    // M220 feedrate factor 0..1
  void set_fanspeed(float value) final;         // M106, M107: speed 0...255
  void set_temperature(float degrees_c) final;  // M104, M109: Set temp. in Celsius.
  void wait_temperature() final;                // M109, M116: Wait for temp. reached.
  void dwell(float time_ms) final;              // G4: dwell for milliseconds.
  void motors_enable(bool enable) final;        // M17,M84,M18: Switch on/off motors
  void clamp_to_range(AxisBitmap_t affected, AxesRegister *axes) final;
  bool coordinated_move(float feed_mm_p_sec, const AxesRegister &target) final;
  bool rapid_move(float feed_mm_p_sec, const AxesRegister &target) final;
  const char *unprocessed(char letter, float value, const char *) final;

private:
  bool in_estop();
  void set_estop(bool hard);
  bool clear_estop();
  bool check_for_estop();
  bool check_for_pause();
  void issue_motor_move_if_possible();
  bool test_homing_status_ok();
  bool test_within_machine_limits(const AxesRegister &axes);
  void mprint_endstop_status();
  void mprint_current_position();
  const char *aux_bit_commands(char letter, float value, const char *);
  const char *special_commands(char letter, float value, const char *);
  float acceleration_for_move(const int *axis_steps,
                              enum GCodeParserAxis defining_axis);
  int move_to_endstop(enum GCodeParserAxis axis,
                      float feedrate, HardwareMapping::AxisTrigger trigger);
  int move_to_probe(enum GCodeParserAxis axis, float feedrate, const int dir,
                    int max_steps);
  void home_axis(enum GCodeParserAxis axis);
  void set_output_flags(HardwareMapping::NamedOutput out, bool is_on);
  void handle_M105();
  // Parse GCode spindle M3/M4 block.
  const char *set_spindle_on(bool is_ccw, const char *);
  void set_spindle_off();

  // Print to msg_stream.
  void mprintf(const char *format, ...);

private:
  const struct MachineControlConfig cfg_;
  MotorOperations *const motor_ops_;
  HardwareMapping *const hardware_mapping_;
  Spindle *const spindle_;

  Planner *planner_ = nullptr;
  FILE *msg_stream_ = nullptr;
  GCodeParser *parser_ = nullptr;

  // Derived configuration
  float g0_feedrate_mm_per_sec_;         // Highest of all axes; used for G0
                                         // (will be trimmed if needed)
  AxisBitmap_t axis_clamped_;            // All axes that are clamped to range
  // Current machine configuration
  AxesRegister coordinate_display_origin_; // parser tells us
  std::string coordinate_display_origin_name_;
  float current_feedrate_mm_per_sec_;    // Set via Fxxx and remembered
  float prog_speed_factor_;              // Speed factor set by program (M220)
  time_t next_auto_disable_motor_;
  time_t next_auto_disable_fan_;
  bool pause_enabled_;                  // Enabled via M120, disabled via M121

  GCodeMachineControl::HomingState homing_state_;
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
    parser_(NULL),
    g0_feedrate_mm_per_sec_(-1),
    current_feedrate_mm_per_sec_(-1),
    prog_speed_factor_(1),
    homing_state_(GCodeMachineControl::HomingState::NEVER_HOMED) {
    pause_enabled_ = cfg_.enable_pause;
    next_auto_disable_motor_ = -1;
    next_auto_disable_fan_ = -1;
}

// The actual initialization. Can fail, hence we make it a separate method.
bool GCodeMachineControl::Impl::Init() {
  // Always keep the steps_per_mm positive, but extract direction for
  // final assignment to motor.
  for (const GCodeParserAxis axis : AllAxes()) {
    if (cfg_.steps_per_mm[axis] < 0) {
      Log_error("Negative number of steps per unit for Axis %c",
                gcodep_axis2letter(axis));
      return false;
    }
    // Permissible hard override. We want to keep cfg_ const for most of the
    // places, so writing here in Init() is permissible.
    const_cast<MachineControlConfig&>(cfg_).steps_per_mm[axis]
                                           = fabsf(cfg_.steps_per_mm[axis]);
    if (cfg_.max_feedrate[axis] < 0) {
      Log_error("Invalid negative feedrate %.1f for axis %c\n",
                cfg_.max_feedrate[axis], gcodep_axis2letter(axis));
      return false;
    }
    if (cfg_.acceleration[axis] < 0) {
      Log_error("Invalid negative acceleration %.1f for axis %c\n",
                cfg_.acceleration[axis], gcodep_axis2letter(axis));
      return false;
    }
  }

  for (const GCodeParserAxis axis : AllAxes()) {
    if (cfg_.max_feedrate[axis] > g0_feedrate_mm_per_sec_) {
      g0_feedrate_mm_per_sec_ = cfg_.max_feedrate[axis];
    }
  }
  prog_speed_factor_ = 1.0f;

  int error_count = 0;

  // Check if things are plausible: we only allow one home endstop per axis.
  for (const GCodeParserAxis axis : AllAxes()) {
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

  axis_clamped_ = 0;
  for (char c : cfg_.clamp_to_range) {
    const GCodeParserAxis clamped_axis = gcodep_letter2axis(c);
    if (clamped_axis == GCODE_NUM_AXES) {
      Log_error("clamp-to-range: Invalid clamping axis %c", c);
      ++error_count;
      continue;
    }

    // Technically, we can do all axes, but in practice, it doesn't make sense
    // without risking undesirable work-results.
    // So for now, only do that on Z which can make sense in 2.5D applications.
    if (clamped_axis != AXIS_Z) {
      Log_error("clamp-to-range: Only Z-axis clamp allowed (not %c)", c);
      ++error_count;
      continue;
    }

    axis_clamped_ |= (1 << clamped_axis);
  }

  // Now let's see what motors are mapped to any useful output.
  Log_debug("-- Config --\n");
  for (const GCodeParserAxis axis : AllAxes()) {
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
      if (hardware_mapping_->HasProbeSwitch(axis))
        line += "PROBE@max->|; ";
    }
    else if ((cfg_.homing_trigger[axis] & HardwareMapping::TRIGGER_MAX) != 0) {
      if (hardware_mapping_->HasProbeSwitch(axis))
        line += "|<-PROBE@min; ";
      line += StringPrintf("HOME@max->|; ");
    }

    if (axis_clamped_ & (1 << axis))
      line += " [clamped to range]";

    if (!cfg_.range_check)
      line += " Limit checks disabled!";

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

  planner_ = new Planner(&cfg_, hardware_mapping_, motor_ops_);
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
  planner_->BringPathToHalt();
  motor_ops_->MotorEnable(b);
  if (!b && homing_state_ == GCodeMachineControl::HomingState::HOMED) {
    homing_state_ = GCodeMachineControl::HomingState::HOMED_BUT_MOTORS_UNPOWERED;
  }
}
void GCodeMachineControl::Impl::gcode_command_done(char l, float v) {
  if (cfg_.acknowledge_lines) mprintf("ok\n");
}
void GCodeMachineControl::Impl::inform_origin_offset(const AxesRegister &o,
                                                     const char *named) {
  coordinate_display_origin_ = o;
  coordinate_display_origin_name_ = named;
}

void GCodeMachineControl::Impl::set_fanspeed(float speed) {
  if (speed < 0.0 || speed > 255.0) return;
  float duty_cycle = speed / 255.0;
  // The fan can be mapped to an aux and/or pwm signal
  set_output_flags(HardwareMapping::NamedOutput::FAN, duty_cycle > 0.0);
  hardware_mapping_->SetPWMOutput(HardwareMapping::NamedOutput::FAN, duty_cycle);
}

// number of checks to ensure the pause switch is inactive
#define PAUSE_ACTIVE_DETECT	2

void GCodeMachineControl::Impl::wait_for_start() {
  // Interlock: can't start while wait is still active.
  if (pause_enabled_ && check_for_pause()) {
    mprintf("// BeagleG: pause switch active\n");
    int pause_active = PAUSE_ACTIVE_DETECT;
    while (pause_active) {
      usleep(1000);
      // TODO: we should probably have de-bouncing logic rather in the
      // hardware mapping. E.g. something like TestPauseSwitch(2000).
      if (check_for_pause())
	pause_active = PAUSE_ACTIVE_DETECT;
      else
	pause_active--;
    }
    mprintf("// BeagleG: pause switch cleared\n");
  }
  if (!hardware_mapping_->TestStartSwitch()) {
    mprintf("// BeagleG: waiting for start switch\n");
    const int flash_usec = 100 * 1000;
    while (!hardware_mapping_->TestStartSwitch()) {
      set_output_flags(HardwareMapping::NamedOutput::LED, true);
      hardware_mapping_->SetAuxOutputs();
      usleep(flash_usec);
      set_output_flags(HardwareMapping::NamedOutput::LED, false);
      hardware_mapping_->SetAuxOutputs();
      usleep(flash_usec);
    }
  }
}

bool GCodeMachineControl::Impl::in_estop() {
  return hardware_mapping_->InSoftEStop();
}

void GCodeMachineControl::Impl::set_estop(bool hard) {
  hardware_mapping_->AuxOutputsOff();
  set_output_flags(HardwareMapping::NamedOutput::ESTOP, true);
  motors_enable(false);
  homing_state_ = GCodeMachineControl::HomingState::NEVER_HOMED;
  mprintf("// Beagleg: %s E-Stop.\n", hard ? "Hard" : "Soft");
}

bool GCodeMachineControl::Impl::clear_estop() {
  if (in_estop()) {
    if (hardware_mapping_->TestEStopSwitch()) {
      mprintf("// Beagleg: still in Hard E-Stop.\n");
      return false;
    }
    set_output_flags(HardwareMapping::NamedOutput::ESTOP, false);
    hardware_mapping_->SetAuxOutputs();
    mprintf("// Beagleg: Soft E-Stop cleared.\n");
  }
  return true;
}

bool GCodeMachineControl::Impl::check_for_estop() {
  // TODO(Hartley): in case of (estop() == true) should we just return just
  // that? Because right now, we only return if we see the EStop switch the
  // first time.
  if (!in_estop() && hardware_mapping_->TestEStopSwitch()) {
    Log_info("E-Stop input detected, forcing Software E-Stop.");
    set_estop(true);
    return true;
  }
  return false;
}

bool GCodeMachineControl::Impl::check_for_pause() {
  return hardware_mapping_->TestPauseSwitch();
}

void GCodeMachineControl::Impl::handle_M105() {
  mprintf("// ");
  for (int chan = 0; chan < 8; chan++) {
    int raw = arc_read_raw(chan);
    mprintf("RAW%d:%d ", chan, raw);
  }
  mprintf("\n");
  mprintf("T-300\n");
}

// TODO(hzeller): the M3/M4 block should be dealt with in the gcode parser.
const char *GCodeMachineControl::Impl::set_spindle_on(bool is_ccw,
                                                      const char *remaining) {
  if (!spindle_) return remaining;
  int spindle_rpm = -1;
  const char* after_pair;
  char letter;
  float value;

  // Ensure that the PRU queue is flushed before turning on the spindle.
  planner_->BringPathToHalt();
  for (;;) {
    after_pair = parser_->ParsePair(remaining, &letter, &value, msg_stream_);
    if (after_pair == NULL) break;
    else if (letter == 'S') spindle_rpm = round2int(value);
    else break;
    remaining = after_pair;
  }
  if (spindle_rpm >= 0) spindle_->On(is_ccw, spindle_rpm);
  return remaining;
}

void GCodeMachineControl::Impl::set_spindle_off() {
  if (!spindle_) return;
  // Ensure that the PRU queue is flushed before turning off the spindle.
  planner_->BringPathToHalt();
  spindle_->Off();
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
  case 0: set_estop(false); break;
  case 999: clear_estop(); break;
  case 3: case 4: case 5:                   // aux pin spindle control
  case 7: case 8: case 9: case 10: case 11: // aux pin mist/flood/vacuum control
  case 42:                                  // aux pin state query
  case 62: case 63: case 64: case 65:       // aux pin set
  case 245: case 246:                       // aux pin cooler control
  case 355:                                 // aux pin case lights control
    remaining = aux_bit_commands(letter, value, remaining);
    break;
  case 400:
    dwell(0);
    break;
  case 80:
  case 81:
    set_output_flags(HardwareMapping::NamedOutput::ATX_POWER, code == 80);
    hardware_mapping_->SetAuxOutputs();
    break;
  case 105: handle_M105(); break;
  case 114: mprint_current_position(); break;
  case 115: mprintf("%s\n", VERSION_STRING); break;
  case 117:
    mprintf("// Msg: %s\n", remaining); // TODO: different output ?
    remaining = NULL;  // consume the full line.
    break;
  case 119: mprint_endstop_status(); break;
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

const char *GCodeMachineControl::Impl::aux_bit_commands(
  char letter, float value, const char *remaining) {
  const int m_code = (int)value;
  const char* after_pair;

  switch (m_code) {
  case 3: remaining = set_spindle_on(false, remaining); break;  // CW
  case 4: remaining = set_spindle_on(true, remaining); break;   // CCW
  case 5: set_spindle_off(); break;
  case 7: set_output_flags(HardwareMapping::NamedOutput::MIST, true); break;
  case 8: set_output_flags(HardwareMapping::NamedOutput::FLOOD, true); break;
  case 9:
    set_output_flags(HardwareMapping::NamedOutput::MIST, false);
    set_output_flags(HardwareMapping::NamedOutput::FLOOD, false);
    break;
  case 10: set_output_flags(HardwareMapping::NamedOutput::VACUUM, true); break;
  case 11: set_output_flags(HardwareMapping::NamedOutput::VACUUM, false); break;
  case 42:
  case 62: case 63: case 64: case 65: {
    int bit_value = -1;
    int pin = -1;
    for (;;) {
      after_pair = parser_->ParsePair(remaining, &letter, &value, msg_stream_);
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
  case 245:
    set_output_flags(HardwareMapping::NamedOutput::COOLER, true);
    break;
  case 246:
    set_output_flags(HardwareMapping::NamedOutput::COOLER, false);
    break;
  case 355: {
    int on_value = 0;
    for (;;) {
      after_pair = parser_->ParsePair(remaining, &letter, &value, msg_stream_);
      if (after_pair == NULL) break;
      if (letter == 'S') on_value = round2int(value);
      else break;
      remaining = after_pair;
    }
    set_output_flags(HardwareMapping::NamedOutput::CASE_LIGHTS, on_value > 0);
  }
    break;
  }
  return remaining;
}

void GCodeMachineControl::Impl::set_output_flags(
  HardwareMapping::NamedOutput out, bool is_on) {
  hardware_mapping_->UpdateAuxBitmap(out, is_on);
}

GCodeMachineControl::EStopState GCodeMachineControl::Impl::GetEStopStatus() {
  if (in_estop()) {
    if (hardware_mapping_->TestEStopSwitch())
      return GCodeMachineControl::EStopState::HARD;
    return GCodeMachineControl::EStopState::SOFT;
  }
  return GCodeMachineControl::EStopState::NONE;
}

GCodeMachineControl::HomingState GCodeMachineControl::Impl::GetHomeStatus() {
  return homing_state_;
}

bool GCodeMachineControl::Impl::GetMotorsEnabled() {
  return hardware_mapping_->MotorsEnabled();
}

void GCodeMachineControl::Impl::GetCurrentPosition(AxesRegister *pos) {
  planner_->GetCurrentPosition(pos);
  for (const GCodeParserAxis axis : AllAxes()) {
    (*pos)[axis] -= coordinate_display_origin_[axis];
  }
}

void GCodeMachineControl::Impl::mprint_current_position() {
  AxesRegister current_pos;
  planner_->GetCurrentPosition(&current_pos);
  const AxesRegister &origin = coordinate_display_origin_;
  mprintf(
#if M114_DEBUG
    "X:%.6f Y:%.6f Z:%.6f E:%.6f",
#else
    "X:%.3f Y:%.3f Z:%.3f E:%.3f",
#endif
          current_pos[AXIS_X] - origin[AXIS_X],
          current_pos[AXIS_Y] - origin[AXIS_Y],
          current_pos[AXIS_Z] - origin[AXIS_Z],
          current_pos[AXIS_E] - origin[AXIS_E]);
  mprintf(
#if M114_DEBUG
    " [ABS. MACHINE CUBE X:%.6f Y:%.6f Z:%.6f]",
#else
    " [ABS. MACHINE CUBE X:%.3f Y:%.3f Z:%.3f]",
#endif
          current_pos[AXIS_X], current_pos[AXIS_Y], current_pos[AXIS_Z]);

  // Coordinate system. Always printed, even if empty to have a predictable
  // format.
  mprintf(" [%s]", coordinate_display_origin_name_.c_str());

  switch (homing_state_) {
  case GCodeMachineControl::HomingState::NEVER_HOMED:
    mprintf(" (Unsure: machine never homed!)\n");
    break;
  case GCodeMachineControl::HomingState::HOMED_BUT_MOTORS_UNPOWERED:
    mprintf(" (Lower confidence: motor power off at least once after homing)\n");
    break;
  case GCodeMachineControl::HomingState::HOMED:
    mprintf(" (confident: machine was homed)\n");
    break;
  }
}

void GCodeMachineControl::Impl::mprint_endstop_status() {
  bool any_endstops_found = false;
  for (const GCodeParserAxis axis : AllAxes()) {
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

void GCodeMachineControl::Impl::gcode_start(GCodeParser *parser) {
  parser_ = parser;
  if (cfg_.auto_fan_pwm > 0)
    set_fanspeed(cfg_.auto_fan_pwm);
}

void GCodeMachineControl::Impl::gcode_finished(bool end_of_stream) {
  planner_->BringPathToHalt();
  set_spindle_off();
  if (end_of_stream && cfg_.auto_motor_disable_seconds > 0)
    motors_enable(false);
}

bool GCodeMachineControl::Impl::test_homing_status_ok() {
  if (!cfg_.require_homing)
    return true;
  if (homing_state_ > GCodeMachineControl::HomingState::NEVER_HOMED)
    return true;
  mprintf("// ERROR: please home machine first (G28).\n");
  return false;
}

bool GCodeMachineControl::Impl::test_within_machine_limits(const AxesRegister &axes) {
  if (!cfg_.range_check)
    return true;

  for (const GCodeParserAxis i : AllAxes()) {
    // Min range ...
    if (axes[i] < 0) {
      // Machine cube must be in positive range.
      if (coordinate_display_origin_[i] != 0) {
        mprintf("// ERROR outside machine limit: Axis %c < min allowed "
                "%+.1fmm in current coordinate system. Ignoring move!\n",
                gcodep_axis2letter(i), -coordinate_display_origin_[i]);
      } else {
        // No relative G92 or similar set. Display in simpler form.
        mprintf("// ERROR outside machine limit: Axis %c < 0. "
                "Ignoring move!\n", gcodep_axis2letter(i));
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
        // We have a different origin, so display coordinate relative to that.
        mprintf("// ERROR outside machine limit: Axis %c %.1fmm > max allowed %+.1fmm "
                "in current coordinate system (=%.1fmm machine absolute). "
                "Ignoring move!\n",
                gcodep_axis2letter(i),
                axes[i] - coordinate_display_origin_[i],
                max_limit - coordinate_display_origin_[i], max_limit);
      } else {
        // No relative G92 or similar set. Display in simpler form.
        mprintf("// ERROR outside machine limit: Axis %c = %.1f > %.1fmm. "
                "Ignoring move!\n", gcodep_axis2letter(i),
                axes[i], max_limit);
      }
      return false;
    }
  }
  return true;
}

void GCodeMachineControl::Impl::clamp_to_range(AxisBitmap_t affected,
                                               AxesRegister *mutable_axes) {
  // Due diligence: only clamp if all affected axes are actually configured.
  // Otherwise we might end up in undesirable positions with partial clamps.
  // (Right now, only Z-Axis is allowed in fact, see Init()).
  if ((affected & axis_clamped_) != affected)
    return;

  AxesRegister &axes = *mutable_axes;
  for (const GCodeParserAxis i : AllAxes()) {
    if ((axis_clamped_ & (1 << i)) == 0)
      continue;
    if (axes[i] < 0) {
      mprintf("// WARNING clamping Axis %c move %.1f to %.1fmm\n",
              gcodep_axis2letter(i), axes[i], 0);
      axes[i] = 0;
    }
    const float max_limit = cfg_.move_range_mm[i];
    if (max_limit <= 0) continue;
    if (axes[i] > max_limit) {
      mprintf("// WARNING clamping Axis %c move %.1f to %.1fmm\n",
              gcodep_axis2letter(i), axes[i], max_limit);
      axes[i] = max_limit;
    }
  }
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
  if (current_feedrate_mm_per_sec_ <= 0) {
    mprintf("// Error: No feedrate set yet.\n");
    return false;
  }

  float feedrate = prog_speed_factor_ * current_feedrate_mm_per_sec_;
  planner_->Enqueue(axis, feedrate);
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
  if (given > 0 && current_feedrate_mm_per_sec_ <= 0) {
    current_feedrate_mm_per_sec_ = given;  // At least something for G1.
  }
  planner_->Enqueue(axis, given > 0 ? given : rapid_feed);
  return true;
}

void GCodeMachineControl::Impl::dwell(float value) {
  planner_->BringPathToHalt();
  motor_ops_->WaitQueueEmpty();
  if (hardware_mapping_->IsHardwareSimulated()) {
    if (value > 999.0) {
      // Let some interactive user know that they can't expect dwell time here.
      mprintf("// FYI: hardware simulated. All dwelling is immediate.\n", value);
    }
  } else {
    // TODO: this needs to wait in the event multiplexer.
    // Since we might need pretty high precision (see rpt2pnp), this can't
    // just be quantized to 50ms.
    usleep((int) (value * 1000));
  }

  if (!check_for_estop()) {
    if (pause_enabled_ && check_for_pause()) {
      Log_debug("Pause input detected, waiting for Start");
      wait_for_start();
    }
  }
}

void GCodeMachineControl::Impl::input_idle(bool is_first) {
  planner_->BringPathToHalt();
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
  check_for_estop();
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
                                               float feedrate,
                                               HardwareMapping::AxisTrigger trigger) {
  if (hardware_mapping_->IsHardwareSimulated())
    return 0;  // There are no switches to trigger, so pretend we stopped.

  const float kHomingMM = 0.5;                    // TODO: make configurable?
  const float kBackoffMM = kHomingMM / 10.0;      // TODO: make configurable?

  int total_movement = 0;
  const int dir = trigger == HardwareMapping::TRIGGER_MIN ? -1 : 1;
  float v0 = 0;
  float v1 = feedrate;
  while (!hardware_mapping_->TestAxisSwitch(axis, trigger)) {
    if (hardware_mapping_->TestEStopSwitch()) return 0;
    total_movement += planner_->DirectDrive(axis, dir * kHomingMM, v0, v1);
    v0 = v1;  // TODO: possibly acceleration over multiple segments.
  }

  // Go back until switch is not triggered anymore.
  while (hardware_mapping_->TestAxisSwitch(axis, trigger)) {
    if (hardware_mapping_->TestEStopSwitch()) return 0;
    total_movement += planner_->DirectDrive(axis, -dir * kBackoffMM, v0, v1);
  }

  return total_movement;
}

int GCodeMachineControl::Impl::move_to_probe(enum GCodeParserAxis axis,
                                             float feedrate, const int dir,
                                             int max_steps) {
  if (hardware_mapping_->IsHardwareSimulated())
    return 0;  // There are no switches to trigger, so pretend we stopped.

  const float kProbeMM = 0.05;                   // TODO: make configurable?

  int total_movement = 0;
  float v0 = 0;
  float v1 = feedrate;
  while (!hardware_mapping_->TestProbeSwitch()) {
    total_movement += planner_->DirectDrive(axis, dir * kProbeMM, v0, v1);
    v0 = v1;  // TODO: possibly acceleration over multiple segments.
    if (abs(total_movement) > max_steps) {
      mprintf("// G30: max probe reached\n");
      return total_movement;
    }
  }

  return total_movement;
}

// TODO(hzeller): Should planner provide homing features ?
void GCodeMachineControl::Impl::home_axis(enum GCodeParserAxis axis) {
  const HardwareMapping::AxisTrigger trigger = cfg_.homing_trigger[axis];
  if (trigger == HardwareMapping::TRIGGER_NONE)
    return;  // TODO: warn that there is no swich ? Should we pretend go back?
  const float home_pos = ((trigger == HardwareMapping::TRIGGER_MAX)
                          ? cfg_.move_range_mm[axis]
                          : 0.0f);
  const float kHomingSpeed = 15; // mm/sec  (make configurable ?)

  planner_->BringPathToHalt();
  move_to_endstop(axis, kHomingSpeed, trigger);
  planner_->SetExternalPosition(axis, home_pos);
}

void GCodeMachineControl::Impl::go_home(AxisBitmap_t axes_bitmap) {
  planner_->BringPathToHalt();
  if (!clear_estop()) return;
  for (const char axis_letter : cfg_.home_order) {
    const enum GCodeParserAxis axis = gcodep_letter2axis(axis_letter);
    if (axis == GCODE_NUM_AXES || !(axes_bitmap & (1 << axis)))
      continue;
    home_axis(axis);
  }
  if (check_for_estop()) return;
  homing_state_ = GCodeMachineControl::HomingState::HOMED;
}

bool GCodeMachineControl::Impl::probe_axis(float feedrate,
                                           enum GCodeParserAxis axis,
                                           float *probe_result) {
  if (!test_homing_status_ok())
    return false;

  planner_->BringPathToHalt();

  if (!hardware_mapping_->HasProbeSwitch(axis)) {
    mprintf("// BeagleG: No probe - axis %c does not have a probe switch\n",
            gcodep_axis2letter(axis));
    return false;
  }

  // -- somewhat hackish

  AxesRegister machine_pos;
  planner_->GetCurrentPosition(&machine_pos);

  // The probe endstop should be in the direction that is _not_ used for homing.
  HardwareMapping::AxisTrigger home_trigger = cfg_.homing_trigger[axis];
  const int dir = home_trigger == HardwareMapping::TRIGGER_MIN ? 1 : -1;

  if (feedrate <= 0) feedrate = 20;
  int max_steps = abs(cfg_.move_range_mm[axis] * cfg_.steps_per_mm[axis]);
  int total_steps = move_to_probe(axis, feedrate, dir, max_steps);
  float distance_moved = total_steps / cfg_.steps_per_mm[axis];

  const float new_pos = machine_pos[axis] + distance_moved;
  planner_->SetExternalPosition(axis, new_pos);
  *probe_result = new_pos;
  return true;
}

GCodeMachineControl::GCodeMachineControl(Impl *impl) : impl_(impl) {
}
GCodeMachineControl::~GCodeMachineControl() {
  delete impl_;
}

GCodeMachineControl* GCodeMachineControl::Create(
  const MachineControlConfig &config,
  MotorOperations *motor_ops,
  HardwareMapping *hardware_mapping,
  Spindle *spindle,
  FILE *msg_stream)
{
  Impl *result = new Impl(config, motor_ops,
                          hardware_mapping, spindle, msg_stream);
  if (!result->Init()) {
    delete result;
    return NULL;
  }
  return new GCodeMachineControl(result);
}

void GCodeMachineControl::GetHomePos(AxesRegister *home_pos) {
  home_pos->zero();
  for (const GCodeParserAxis axis : AllAxes()) {
    HardwareMapping::AxisTrigger trigger = impl_->config().homing_trigger[axis];
    (*home_pos)[axis] = (trigger & HardwareMapping::TRIGGER_MAX)
      ? impl_->config().move_range_mm[axis] : 0;
  }
}

GCodeMachineControl::EStopState GCodeMachineControl::GetEStopStatus() {
  return impl_->GetEStopStatus();
}

GCodeMachineControl::HomingState GCodeMachineControl::GetHomeStatus() {
  return impl_->GetHomeStatus();
}

bool GCodeMachineControl::GetMotorsEnabled() {
  return impl_->GetMotorsEnabled();
}

void GCodeMachineControl::GetCurrentPosition(AxesRegister *pos) {
  impl_->GetCurrentPosition(pos);
}

GCodeParser::EventReceiver *GCodeMachineControl::ParseEventReceiver() {
  return impl_;
}

void GCodeMachineControl::SetMsgOut(FILE *msg_stream) {
  impl_->set_msg_stream(msg_stream);
}
