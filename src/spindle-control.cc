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

#include "spindle-control.h"

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <errno.h>

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
static const float kRampEpsilon = 0.1;
static const int kRampDelayMs = 10;

static void sleep_ms(int ms) {
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

Spindle::Spindle() {
  impl_ = NULL;
  type_ = kType;
  port_ = kPort;
  max_rpm_ = kMaxRPM;
  pwr_delay_ms_ = 0;
  on_delay_ms_ = 0;
  off_delay_ms_ = 0;
  allow_ccw_ = false;
}

class Spindle::ConfigReader : public ConfigParser::Reader {
public:
  ConfigReader(Spindle *config) : config_(config) {}

  virtual bool SeenSection(int line_no, const std::string &section_name) {
    current_section_ = section_name;
    return (section_name == "spindle");
  }

  virtual bool SeenNameValue(int line_no,
                             const std::string &name,
                             const std::string &value) {
#define ACCEPT_VALUE(n, T, result) if (name != n) {} else return Parse##T(value, result)
    if (current_section_ == "spindle") {
      ACCEPT_VALUE("type",           String, &config_->type_);
      ACCEPT_VALUE("port",           String, &config_->port_);
      ACCEPT_VALUE("max-rpm",        Int,    &config_->max_rpm_);
      ACCEPT_VALUE("pwr-delay-msec", Int,    &config_->pwr_delay_ms_);
      ACCEPT_VALUE("on-delay-msec",  Int,    &config_->on_delay_ms_);
      ACCEPT_VALUE("off-delay-msec", Int,    &config_->off_delay_ms_);
      ACCEPT_VALUE("allow-ccw",      Bool,   &config_->allow_ccw_);

      return false;
    }
    ReportError(line_no, StringPrintf("Unexpected configuration option '%s'",
                                      name.c_str()));

#undef ACCEPT_VALUE
    return false;
  }

  virtual void ReportError(int line_no, const std::string &msg) {
    Log_error("Line %d: %s", line_no, msg.c_str());
  }

private:
  Spindle *const config_;

  std::string current_section_;
};

bool Spindle::ConfigureFromFile(ConfigParser *parser) {
  Spindle::ConfigReader reader(this);
  return parser->EmitConfigValues(&reader);
}

class Spindle::Impl {
public:
  Impl(HardwareMapping *hardware_mapping, int max_rpm,
       int pwr_delay_ms, int on_delay_ms, int off_delay_ms)
    : hardware_mapping_(hardware_mapping), max_rpm_(max_rpm),
      pwr_delay_ms_(pwr_delay_ms),
      on_delay_ms_(on_delay_ms),
      off_delay_ms_(off_delay_ms) {
    is_off_ = true;
    is_ccw_ = false;
    duty_cycle_ = 0;
  }

  virtual ~Impl() {}

  virtual bool Init() { return true; }

  virtual void On(bool ccw, int rpm) = 0;
  virtual void Off() = 0;

  void set_output_flags(HardwareMapping::LogicOutput out, bool is_on) {
    hardware_mapping_->UpdateAuxBitmap(out, is_on);
  }

protected:
  HardwareMapping *hardware_mapping_;

  int max_rpm_;
  int pwr_delay_ms_;
  int on_delay_ms_;
  int off_delay_ms_;

  bool is_off_;
  bool is_ccw_;
  float duty_cycle_;
};

class PWMSpindle : public Spindle::Impl {
public:
  PWMSpindle(HardwareMapping *hardware_mapping, int max_rpm,
             int pwr_delay_ms, int on_delay_ms, int off_delay_ms)
    : Impl(hardware_mapping, max_rpm, pwr_delay_ms, on_delay_ms, off_delay_ms) {
    Log_debug("PWMSpindle: constructed");
    Log_debug("  max_rpm      : %d", max_rpm);
    Log_debug("  pwr_delay_ms : %d", pwr_delay_ms);
    Log_debug("  on_delay_ms  : %d", on_delay_ms);
    Log_debug("  off_delay_ms : %d", off_delay_ms);
  }

  void On(bool ccw, int rpm) {
    // turn on spindle power if necessary
    if (is_off_) {
      set_output_flags(HardwareMapping::OUT_SPINDLE, true);
      if (pwr_delay_ms_) sleep_ms(pwr_delay_ms_);
    }

    // direction change? ramp down spindle
    if (ccw != is_ccw_) ramp_down();

    // set the spindle direction
    set_output_flags(HardwareMapping::OUT_SPINDLE_DIRECTION, ccw);
    is_ccw_ = ccw;

    // ramp the spindle to the target speed
    float target = std::min((float)rpm / max_rpm_, 1.0f);
    float epsilon = (duty_cycle_ < target) ? kRampEpsilon : -kRampEpsilon;
    while (duty_cycle_ != target) {
      duty_cycle_ += epsilon;
      if ((epsilon < 0 && duty_cycle_ < target) ||
          (epsilon > 0 && duty_cycle_ > target))
        duty_cycle_ = target;
      hardware_mapping_->SetPWMOutput(HardwareMapping::OUT_SPINDLE_SPEED,
                                      duty_cycle_);
      sleep_ms(kRampDelayMs);
    }

    // optionally delay before continuing
    if (is_off_) {
      if (on_delay_ms_) sleep_ms(on_delay_ms_);
      is_off_ = false;
    }

    Log_debug("PWMSpindle: on %s at %d RPM (duty_cycle: %f)",
              ccw ? "ccw" : "cw", (int)(max_rpm_ * duty_cycle_), duty_cycle_);
  }

  void Off() {
    ramp_down();
    if (off_delay_ms_) sleep_ms(off_delay_ms_);
    set_output_flags(HardwareMapping::OUT_SPINDLE, false);
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
      hardware_mapping_->SetPWMOutput(HardwareMapping::OUT_SPINDLE_SPEED,
                                      duty_cycle_);
      sleep_ms(kRampDelayMs);
    }
  }
};

class PololuSMCSpindle : public Spindle::Impl {
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

    MAX_SPEED                             = 3200
  };

public:
  PololuSMCSpindle(HardwareMapping *hardware_mapping, const char *port,
                   int max_rpm, int pwr_delay_ms, int on_delay_ms,
                   int off_delay_ms)
    : Impl(hardware_mapping, max_rpm, pwr_delay_ms, on_delay_ms, off_delay_ms) {
    fd_ = open(port, O_RDWR | O_NOCTTY);
    Log_debug("PololuSMCSpindle: constructed");
    Log_debug("  port         : %s  (fd = %d)", port, fd_);
    Log_debug("  max_rpm      : %d", max_rpm);
    Log_debug("  pwr_delay_ms : %d", pwr_delay_ms);
    Log_debug("  on_delay_ms  : %d", on_delay_ms);
    Log_debug("  off_delay_ms : %d", off_delay_ms);
  }

  ~PololuSMCSpindle() {
    if (fd_) close(fd_);
  }

  bool Init() {
    if (fd_ == -1) return false;

    struct termios options;
    tcgetattr(fd_, &options);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    options.c_oflag &= ~(ONLCR | OCRNL);
    tcsetattr(fd_, TCSANOW, &options);

    const unsigned char command = CMD_GET_FIRMWARE_VER;
    send(&command, 1);
    unsigned char response[4];
    receive(response, 4);
    Log_debug("PololuSMCSpindle: initialized  ProductID:0x%04x  Firmware:%d.%d\n",
           response[0] + 256 * response[1], response[3], response[2]);

    Off();

    return true;
  }

  void On(bool ccw, int rpm) {
    if (fd_ == -1) return;

    if (is_off_) {
      set_output_flags(HardwareMapping::OUT_SPINDLE, true);
      if (pwr_delay_ms_) sleep_ms(pwr_delay_ms_);
      exit_safe_start();
    }

    // scale the desired RPM to the MAX_SPEED of the SMC
    int speed = std::min(rpm * MAX_SPEED / max_rpm_, (int)MAX_SPEED);

    unsigned char command[3];
    command[0] = (ccw) ? CMD_MOTOR_REVERSE : CMD_MOTOR_FORWARD;
    command[1] = speed & 0x1f;
    command[2] = (speed >> 5) & 0x7f;
    send(command, sizeof(command));

    if (is_off_) {
      if (on_delay_ms_) sleep_ms(on_delay_ms_);
      is_off_ = false;
    }

    float duty_cycle = std::min((float)rpm / max_rpm_, 1.0f);
    Log_debug("PololuSMCSpindle: on %s at %d RPM (speed: %d)",
              ccw ? "ccw" : "cw", (int)(max_rpm_ * duty_cycle), speed);
  }

  void Off() {
    if (fd_ == -1) return;

    const unsigned char command = CMD_STOP_MOTOR;
    send(&command, 1);

    if (off_delay_ms_) sleep_ms(off_delay_ms_);
    set_output_flags(HardwareMapping::OUT_SPINDLE, false);
    is_off_ = true;
    Log_debug("PololuSMCSpindle: off");
  }

private:
  void exit_safe_start() {
    const unsigned char command = CMD_EXIT_SAFE_START;
    send(&command, 1);
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

bool Spindle::Init(HardwareMapping *hardware_mapping) {
  std::string line;
  if (type_ == "simple-pwm") {
    impl_ = new PWMSpindle(hardware_mapping, max_rpm_,
                           pwr_delay_ms_, on_delay_ms_, off_delay_ms_);
  } else if (type_ == "pololu-smc") {
    impl_ = new PololuSMCSpindle(hardware_mapping, port_.c_str(), max_rpm_,
                                 pwr_delay_ms_, on_delay_ms_, off_delay_ms_);
  } else {
    return false;
  }
  Log_debug("  allow_ccw : %s", allow_ccw_ ? "yes" : "no");
  return impl_->Init();
}

void Spindle::On(bool ccw, int rpm) {
    if (!allow_ccw_ && ccw) {
      Log_debug("Spindle: ccw rotation is not allowed");
      return;
    }

  if (impl_) impl_->On(ccw, rpm);
}

void Spindle::Off() {
  if (impl_) impl_->Off();
}
