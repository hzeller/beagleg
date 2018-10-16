/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2016 H Hartley Sweeten <hsweeten@visionengravers.com>
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

// At this point, we only have two spindle implementations, for now just kept
// in one file.

#include "spindle-control.h"

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <errno.h>

#include <memory>

#include "common/logging.h"
#include "common/string-util.h"

#include "config-parser.h"
#include "hardware-mapping.h"

// Default spindle type
static const char kType[] = "simple-pwm";

// Default virtual comm port used by the Pololu Simple Motor Controller
static const char kPort[] = "/dev/ttyACM0";

// Default max rpm
static const int kMaxRPM = 3000;

// Default simple-pwm spindle ramping constants; 10% ramp change every 10ms
// TODO: make configurable. Lasers can be ramped up and down in no time :)
static const float kRampEpsilon = 0.1;
static const int kRampDelayMs = 10;

static void sleep_ms(int ms) {
  // TODO(hzeller): This needs to hook up with the file multiplexer
  if (ms <= 0) return;
  struct timespec req;
  struct timespec rem;
  req.tv_sec = ms / 1000;
  req.tv_nsec = (ms % 1000) * (1000 * 1000);
  if (nanosleep(&req, &rem) == -1) {
    if (errno == EINTR) {
      Log_error("sleep_ms: nanosleep() was interrupted (%ld.%ld sec remaining)",
                rem.tv_sec, rem.tv_nsec);
    } else {
      Log_error("sleep_ns: nanosleep() failed: %s", strerror(errno));
    }
  }
}

SpindleConfig::SpindleConfig() {
  type = kType;
  port = kPort;
  max_rpm = kMaxRPM;
  pwr_delay_ms = 0;
  on_delay_ms = 0;
  off_delay_ms = 0;
  allow_ccw = false;
}

namespace {
class SpindleConfigReader : public ConfigParser::Reader {
public:
  SpindleConfigReader(SpindleConfig *config) : config_(config) {}

  bool SeenSection(int line_no, const std::string &section_name) final {
    return (section_name == "spindle");  // The only section we deal with.
  }

  bool SeenNameValue(int line_no,
                     const std::string &name, const std::string &value) final {
#define ACCEPT_VALUE(n, T, result) if (name != n) {} else return Parse##T(value, result)
    ACCEPT_VALUE("type",           String, &config_->type);
    ACCEPT_VALUE("port",           String, &config_->port);
    ACCEPT_VALUE("max-rpm",        Int,    &config_->max_rpm);
    ACCEPT_VALUE("max-accel",      Int,    &config_->max_accel);
    ACCEPT_VALUE("max-decel",      Int,    &config_->max_decel);
    ACCEPT_VALUE("pwr-delay-msec", Int,    &config_->pwr_delay_ms);
    ACCEPT_VALUE("on-delay-msec",  Int,    &config_->on_delay_ms);
    ACCEPT_VALUE("off-delay-msec", Int,    &config_->off_delay_ms);
    ACCEPT_VALUE("allow-ccw",      Bool,   &config_->allow_ccw);
#undef ACCEPT_VALUE

    ReportError(line_no, StringPrintf("Unexpected configuration option "
                                      "'%s' in spindle section", name.c_str()));
    return false;
  }

private:
  SpindleConfig *const config_;
};


// Convenience base-class for all our spindles.
class BaseSpindle : public Spindle {
public:
  BaseSpindle(const SpindleConfig &config, HardwareMapping *hardware_mapping)
    : config_(config), hardware_mapping_(hardware_mapping) {}

  virtual bool Init() { return true; }
  virtual void On(bool ccw, int rpm) = 0;
  virtual void Off() = 0;

  // Right now, this sets the output immediately, but we might want to do this
  // along with the move if this is a laser.
  void set_output_synchronous(HardwareMapping::NamedOutput out, bool is_on) {
    hardware_mapping_->UpdateAuxBitmap(out, is_on);
    hardware_mapping_->SetAuxOutputs();
  }

protected:
  const SpindleConfig config_;
  HardwareMapping *const hardware_mapping_;

  bool is_off_ = true;
  bool is_ccw_ = false;
  float duty_cycle_ = 0;
};

class PWMSpindle : public BaseSpindle {
public:
  PWMSpindle(const SpindleConfig &config, HardwareMapping *hardware_mapping)
    : BaseSpindle(config, hardware_mapping) {
    Log_debug("PWMSpindle");
  }

  // Litmus test before constructing.
  static bool CheckRequiredHardware(const SpindleConfig &config,
                                    const HardwareMapping *hw) {
    bool success = true;
    if (!hw->HasAuxMapping(HardwareMapping::NamedOutput::SPINDLE)
        && !hw->HasPWMMapping(HardwareMapping::NamedOutput::SPINDLE_SPEED)) {
      // We want to have at least one of these.
      Log_info("No 'spindle' Aux pin configured to turn on spindle or "
               "'spindle-speed' PWM output to control its speed.");
      success = false;
    }
    if (config.allow_ccw &&
        !hw->HasAuxMapping(HardwareMapping::NamedOutput::SPINDLE_DIRECTION)) {
      Log_info("allow-ccw set, but no spindle-direction aux bit configured");
      success = false;
    }
    return success;
  }

  void On(bool ccw, int rpm) final {
    if (!config_.allow_ccw && ccw) {
      Log_debug("Spindle: ccw rotation is not allowed");
      return;
    }

    // Turn on spindle power if necessary.
    if (is_off_) {
      set_output_synchronous(HardwareMapping::NamedOutput::SPINDLE, true);
      sleep_ms(config_.pwr_delay_ms);
    }

    // direction change? ramp down spindle
    if (ccw != is_ccw_) ramp_down();

    // set the spindle direction
    set_output_synchronous(HardwareMapping::NamedOutput::SPINDLE_DIRECTION, ccw);
    is_ccw_ = ccw;

    // ramp the spindle to the target speed
    const float target = std::min((float)rpm / config_.max_rpm, 1.0f);
    const float epsilon = (duty_cycle_ < target) ? kRampEpsilon : -kRampEpsilon;
    while (duty_cycle_ != target) {
      duty_cycle_ += epsilon;
      if ((epsilon < 0 && duty_cycle_ < target) ||
          (epsilon > 0 && duty_cycle_ > target))
        duty_cycle_ = target;
      hardware_mapping_->SetPWMOutput(HardwareMapping::NamedOutput::SPINDLE_SPEED,
                                      duty_cycle_);
      sleep_ms(kRampDelayMs);
    }

    // optionally delay before continuing
    if (is_off_) {
      sleep_ms(config_.on_delay_ms);
      is_off_ = false;
    }

    Log_debug("PWMSpindle: on %s at %d RPM (duty_cycle: %f)",
              ccw ? "ccw" : "cw",
              (int)(config_.max_rpm * duty_cycle_), duty_cycle_);
  }

  void Off() final {
    ramp_down();
    sleep_ms(config_.off_delay_ms);
    set_output_synchronous(HardwareMapping::NamedOutput::SPINDLE, false);
    is_off_ = true;
    Log_debug("PWMSpindle: off");
  }

private:
  void ramp_down() {
    while (duty_cycle_ > 0) {
      if (duty_cycle_ >= kRampEpsilon)
        duty_cycle_ -= kRampEpsilon;
      else
        duty_cycle_ = 0;
      hardware_mapping_->SetPWMOutput(HardwareMapping::NamedOutput::SPINDLE_SPEED,
                                      duty_cycle_);
      sleep_ms(kRampDelayMs);
    }
  }
};

class PololuSMCSpindle : public BaseSpindle {
private:
  enum {
    // Command codes
    CMD_EXIT_SAFE_START                   = 0x83,
    CMD_MOTOR_FORWARD                     = 0x85,
    CMD_MOTOR_REVERSE                     = 0x86,
    CMD_MOTOR_FORWARD_7BIT                = 0x89,
    CMD_MOTOR_REVERSE_7BIT                = 0x8a,
    CMD_MOTOR_BREAK                       = 0x92,
    CMD_GET_VARIABLE                      = 0xa1,
    CMD_SET_MOTOR_LIMIT                   = 0xa2,
    CMD_GET_FIRMWARE_VER                  = 0xc2,
    CMD_STOP_MOTOR                        = 0xe0,

    // Variable Ids and bit definitions
    ERROR_STATUS                          = 0,
    ERROR_STATUS_SAFE_START_VIOLATION     = (1 << 0),
    ERROR_STATUS_REQUIRED_CHANNEL_INVALID = (1 << 1),
    ERROR_STATUS_SERIAL_ERROR             = (1 << 2),
    ERROR_STATUS_COMMAND_TIMEOUT          = (1 << 3),
    ERROR_STATUS_LIMIT_KILL_SWITCH        = (1 << 4),
    ERROR_STATUS_LOW_VIN                  = (1 << 5),
    ERROR_STATUS_HIGH_VIN                 = (1 << 6),
    ERROR_STATUS_OVER_TEMPERATURE         = (1 << 7),
    ERROR_STATUS_MOTOR_DRIVER_ERROR       = (1 << 8),
    ERROR_STATUS_ERR_LINE_HIGH            = (1 << 9),

    ERRORS_OCCURED                        = 1,

    SERIAL_ERRORS_OCCURED                 = 2,
    SERIAL_ERRORS_OCCURED_FRAME           = (1 << 1),
    SERIAL_ERRORS_OCCURED_NOISE           = (1 << 2),
    SERIAL_ERRORS_OCCURED_RX_OVERRUN      = (1 << 3),
    SERIAL_ERRORS_OCCURED_FORMAT          = (1 << 4),
    SERIAL_ERRORS_OCCURED_CRC             = (1 << 5),

    LIMIT_STATUS                          = 3,
    LIMIT_STATUS_ERROR_SAFE_START         = (1 << 0),
    LIMIT_STATUS_TEMPERATURE              = (1 << 1),
    LIMIT_STATUS_MAX_SPEED                = (1 << 2),
    LIMIT_STATUS_START_SPEED              = (1 << 3),
    LIMIT_STATUS_TARGET_SPEED             = (1 << 4),
    LIMIT_STATUS_RC1                      = (1 << 5),
    LIMIT_STATUS_RC2                      = (1 << 6),
    LIMIT_STATUS_AN1                      = (1 << 7),
    LIMIT_STATUS_AN2                      = (1 << 8),
    LIMIT_STATUS_USB                      = (1 << 9),

    RC1_UNLIMITED_RAW_VALUE               = 4,
    RC1_RAW_VALUE                         = 5,
    RC1_SCALED_VALUE                      = 6,
    RC2_UNLIMITED_RAW_VALUE               = 8,
    RC2_RAW_VALUE                         = 9,
    RC2_SCALED_VALUE                      = 10,

    AN1_UNLIMITED_RAW_VALUE               = 12,
    AN1_RAW_VALUE                         = 13,
    AN1_SCALED_VALUE                      = 14,
    AN2_UNLIMITED_RAW_VALUE               = 16,
    AN2_RAW_VALUE                         = 17,
    AN2_SCALED_VALUE                      = 18,

    TARGET_SPEED                          = 20,
    SPEED                                 = 21,
    BRAKE_AMOUNT                          = 22,
    INPUT_VOLTAGE                         = 23,
    TEMPERATURE                           = 24,
    RC_PERIOD                             = 26,
    BAUD_RATE_REGISTER                    = 27,
    SYSTEM_TIME_LOW                       = 28,
    SYSTEM_TIME_HIGH                      = 29,

    MAX_SPEED_FORWARD                     = 30,
    MAX_ACCELERATION_FORWARD              = 31,
    MAX_DECELERATION_FORWARD              = 32,
    BRAKE_DURATION_FORWARD                = 33,
    MAX_SPEED_REVERSE                     = 36,
    MAX_ACCELERATION_REVERSE              = 37,
    MAX_DECELERATION_REVERSE              = 38,
    BRAKE_DURATION_REVERSE                = 39,

    RESET_FLAGS                           = 127,
    RESET_FLAGS_RST                       = 0x04,
    RESET_FLAGS_VIN                       = 0x0c,
    RESET_FLAGS_SOFTWARE                  = 0x14,
    RESET_FLAGS_WATCHDOG                  = 0x24,

    // Set Motor Limit IDs
    SYMMERIC_LIMIT_MAX_SPEED              = 0,
    SYMMERIC_LIMIT_MAX_ACCEL              = 1,
    SYMMERIC_LIMIT_MAX_DECEL              = 2,
    SYMMERIC_LIMIT_BRAKE_DURATION         = 3,
    FORWARD_LIMIT_MAX_SPEED               = 4,
    FORWARD_LIMIT_MAX_ACCEL               = 5,
    FORWARD_LIMIT_MAX_DECEL               = 6,
    FORWARD_LIMIT_BRAKE_DURATION          = 7,
    REVERSE_LIMIT_MAX_SPEED               = 8,
    REVERSE_LIMIT_MAX_ACCEL               = 9,
    REVERSE_LIMIT_MAX_DECEL               = 10,
    REVERSE_LIMIT_BRAKE_DURATION          = 11,

    MAX_SPEED                             = 3200
  };

public:
  PololuSMCSpindle(const SpindleConfig &config,
                   HardwareMapping *hardware_mapping)
    : BaseSpindle(config, hardware_mapping) {
    fd_ = open(config.port.c_str(), O_RDWR | O_NOCTTY);
    Log_debug("PololuSMCSpindle: port %s  (fd = %d)", config.port.c_str(), fd_);
  }

  ~PololuSMCSpindle() {
    if (fd_) close(fd_);
  }

  // Litmus test before constructing.
  static bool CheckRequiredHardware(const SpindleConfig &config,
                                    const HardwareMapping *hw) {
    bool success = true;
    if (!hw->HasAuxMapping(HardwareMapping::NamedOutput::SPINDLE)) {
      Log_info("No 'spindle' Aux pin configured to turn on spindle.");
      success = false;
    }
    if (config.allow_ccw &&
        !hw->HasAuxMapping(HardwareMapping::NamedOutput::SPINDLE_DIRECTION)) {
      Log_info("allow-ccw set, but no spindle-direction aux bit configured");
      success = false;
    }
    return success;
  }

  bool Init() final {
    if (fd_ == -1) return false;

    struct termios options;
    tcgetattr(fd_, &options);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_oflag &= ~(ONLCR | OCRNL);
    tcsetattr(fd_, TCSANOW, &options);

    int id, major, minor;
    get_version(&id, &major, &minor);
    Log_debug("PololuSMCSpindle: initialized  ProductID:0x%04x  Firmware:%d.%d\n",
              id, major, minor);

    set_motor_limit(SYMMERIC_LIMIT_MAX_ACCEL, config_.max_accel);
    set_motor_limit(SYMMERIC_LIMIT_MAX_DECEL, config_.max_decel);
    check_limits(true);
    check_limits(false);

    Off();

    return true;
  }

  void On(bool ccw, int rpm) final {
    if (fd_ == -1) return;

    if (!config_.allow_ccw && ccw) {
      Log_debug("Spindle: ccw rotation is not allowed");
      return;
    }

    if (is_off_) {
      set_output_synchronous(HardwareMapping::NamedOutput::SPINDLE, true);
      sleep_ms(config_.pwr_delay_ms);
      exit_safe_start();
    }

    // scale the desired RPM to the MAX_SPEED of the SMC
    int speed = std::min(rpm * MAX_SPEED / config_.max_rpm, (int)MAX_SPEED);
    set_target_speed(ccw, speed);

    if (is_off_) {
      // wait for the SMC to fully accelerate the motor
      sleep_ms(config_.on_delay_ms);
      signed short actual = (signed short)get_variable(SPEED);
      while (actual < speed) {
        sleep_ms(250);
        actual = (signed short)get_variable(SPEED);
      }
      is_off_ = false;
    }

    float duty_cycle = std::min((float)rpm / config_.max_rpm, 1.0f);
    Log_debug("PololuSMCSpindle: on %s at %d RPM (speed: %d)",
              ccw ? "ccw" : "cw", (int)(config_.max_rpm * duty_cycle), speed);
  }

  void Off() {
    if (fd_ == -1) return;

    const unsigned char command = CMD_STOP_MOTOR;
    send(&command, 1);

    // wait for the SMC to fully decelerate the motor
    sleep_ms(config_.off_delay_ms);
    signed short speed = (signed short)get_variable(SPEED);
    while (speed) {
      sleep_ms(250);
      speed = (signed short)get_variable(SPEED);
    };

    set_output_synchronous(HardwareMapping::NamedOutput::SPINDLE, false);
    is_off_ = true;
    Log_debug("PololuSMCSpindle: off");
  }

private:
  void exit_safe_start() {
    check_errors(ERRORS_OCCURED);
    const unsigned char command = CMD_EXIT_SAFE_START;
    send(&command, 1);
  }

  void get_version(int *id, int *major, int *minor) {
    const unsigned char command = CMD_GET_FIRMWARE_VER;
    send(&command, 1);
    unsigned char response[4];
    receive(response, 4);

    if (id) *id = response[0] + 256 * response[1];
    if (minor) *minor = response[2];
    if (major) *major = response[3];
  }

  void set_target_speed(bool ccw, int speed) {
    unsigned char command[3];
    command[0] = (ccw) ? CMD_MOTOR_REVERSE : CMD_MOTOR_FORWARD;
    command[1] = speed & 0x1f;
    command[2] = (speed >> 5) & 0x7f;
    send(command, sizeof(command));
  }

  void set_motor_limit(int id, int val) {
    if (val == 0) return;
    const char *name;
    switch (id) {
    case SYMMERIC_LIMIT_MAX_SPEED:
    case FORWARD_LIMIT_MAX_SPEED:
    case REVERSE_LIMIT_MAX_SPEED:
      name = "Max Speed";
      break;

    case SYMMERIC_LIMIT_MAX_ACCEL:
    case FORWARD_LIMIT_MAX_ACCEL:
    case REVERSE_LIMIT_MAX_ACCEL:
      name = "Max Accel";
      break;

    case SYMMERIC_LIMIT_MAX_DECEL:
    case FORWARD_LIMIT_MAX_DECEL:
    case REVERSE_LIMIT_MAX_DECEL:
      name = "Max Decel";
      break;

    case SYMMERIC_LIMIT_BRAKE_DURATION:
    case FORWARD_LIMIT_BRAKE_DURATION:
    case REVERSE_LIMIT_BRAKE_DURATION:
      name = "Brake Duration";
      break;

    default:
      Log_error("Invalid motor limit id (%d)\n", id);
      return;
    }

    unsigned char command[4];
    command[0] = CMD_SET_MOTOR_LIMIT;
    command[1] = id;
    command[2] = val & 0x7f;
    command[3] = (val >> 7) & 0x7f;
    send(command, sizeof(command));

    unsigned char response[1];
    receive(response, 1);

    switch (response[0]) {
    case 0:
      Log_debug("  %s limit set to %d\n", name, val);
      break;
    case 1:
      Log_debug("  Unable to set %s forward limit to %d because of Hard Motor Limit settings\n",
                name, val);
      break;
    case 2:
      Log_debug("  Unable to set %s reverse limit to %d because of Hard Motor Limit settings\n",
                name, val);
      break;
    case 3:
      Log_debug("  Unable to set %s forward and reverse limits %d because of Hard Motor Limit settings\n",
                name, val);
      break;
    }
  }

  // Reads a variable from the SMC and returns it as a number between 0 and 65535.
  // The 'id' must be one of Variable Ids listed in the class definition.
  // For variables that are actually signed, additional processing is required.
  int get_variable(unsigned char id) {
    unsigned char command[2] = { CMD_GET_VARIABLE, id };
    send(command, sizeof(command));
    unsigned char response[2];
    receive(response, sizeof(response));

    return response[0] + 256 * response[1];
  }

  // Read the status flag register Error Status or Errors Occurred.
  // Note, reading the Errors Occurred register clears all the bits.
  void check_errors(unsigned char id) {
    int errors = get_variable(id);
    if (errors) {
      Log_debug("  Error%s: 0x%04x\n",
                id == ERROR_STATUS ? " status" : "s occurred", errors);
      if (errors & ERROR_STATUS_SAFE_START_VIOLATION)
        Log_debug("    Safe-Start Violation\n");
      if (errors & ERROR_STATUS_REQUIRED_CHANNEL_INVALID)
        Log_debug("    Required Channel Invalid\n");
      if (errors & ERROR_STATUS_SERIAL_ERROR)
        Log_debug("    Serial Error\n");
      if (errors & ERROR_STATUS_COMMAND_TIMEOUT)
        Log_debug("    Command Timeout\n");
      if (errors & ERROR_STATUS_LIMIT_KILL_SWITCH)
        Log_debug("    Limit/Kill Switch\n");
      if (errors & ERROR_STATUS_LOW_VIN)
        Log_debug("    Low VIN\n");
      if (errors & ERROR_STATUS_HIGH_VIN)
        Log_debug("    High VIN\n");
      if (errors & ERROR_STATUS_OVER_TEMPERATURE)
        Log_debug("    Over Temperature\n");
      if (errors & ERROR_STATUS_MOTOR_DRIVER_ERROR)
        Log_debug("    Motor Driver Error\n");
      if (errors & ERROR_STATUS_ERR_LINE_HIGH)
        Log_debug("    ERR Line High\n");
    }
  }

  void check_limits(bool forward) {
    int speed, accel, decel, brake;
    if (forward) {
      speed = get_variable(MAX_SPEED_FORWARD);
      accel = get_variable(MAX_ACCELERATION_FORWARD);
      decel = get_variable(MAX_DECELERATION_FORWARD);
      brake = get_variable(BRAKE_DURATION_FORWARD);
    } else {
      speed = get_variable(MAX_SPEED_REVERSE);
      accel = get_variable(MAX_ACCELERATION_REVERSE);
      decel = get_variable(MAX_DECELERATION_REVERSE);
      brake = get_variable(BRAKE_DURATION_REVERSE);
    }
    Log_debug("  %s max speed: %d max accel: %d max decel: %d brake duration: %d\n",
              forward ? "Forward" : "Reverse", speed, accel, decel, brake);
  }

  void send(const void *buf, int count) {
    int sent = write(fd_, buf, count);
    if (sent == -1)
      perror("PololuSMCSpindle: send() error");
    else if (sent != count)
      Log_debug("PololuSMCSpindle: send() short write %d of %d bytes", sent, count);
  }

  void receive(void *buf, int count) {
    int got = read(fd_, buf, count);
    if (got == -1)
      perror("PololuSMCSpindle: receive() error");
    else if (got != count)
      Log_debug("PololuSMCSpindle: receive() short read %d of %d bytes", got, count);
  }

  int fd_;
};
}  // anonymous namespace

bool SpindleConfig::ConfigureFromFile(ConfigParser *parser) {
  SpindleConfigReader reader(this);
  return parser->EmitConfigValues(&reader);
}

Spindle *Spindle::CreateFromConfig(const SpindleConfig &config,
                                   HardwareMapping *hardware_mapping) {
  std::unique_ptr<BaseSpindle> spindle;
  if (config.type == "simple-pwm" &&
      PWMSpindle::CheckRequiredHardware(config, hardware_mapping)) {
    spindle.reset(new PWMSpindle(config, hardware_mapping));
  }
  else if (config.type == "pololu-smc" &&
           PololuSMCSpindle::CheckRequiredHardware(config, hardware_mapping)) {
    spindle.reset(new PololuSMCSpindle(config, hardware_mapping));
  }
  else {
    Log_info("No spindle.");
    // TODO: return a dummy spindle to avoid testing for nullptr everywhere?
    return nullptr;
  }

  if (!spindle->Init())
    return nullptr;

  Log_debug("  max_rpm      : %d", config.max_rpm);
  Log_debug("  max_accel    : %d", config.max_accel);
  Log_debug("  max_decel    : %d", config.max_decel);
  Log_debug("  pwr_delay_ms : %d", config.pwr_delay_ms);
  Log_debug("  on_delay_ms  : %d", config.on_delay_ms);
  Log_debug("  off_delay_ms : %d", config.off_delay_ms);
  Log_debug("  allow_ccw    : %s", config.allow_ccw ? "yes" : "no");
  return spindle.release();
}
