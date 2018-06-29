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
#include <arpa/inet.h>
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <grp.h>
#include <math.h>
#include <netinet/in.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cmath>
#include <memory>

#include "common/fd-mux.h"
#include "common/logging.h"
#include "common/string-util.h"
#include "config-parser.h"
#include "gcode-machine-control.h"
#include "gcode-parser/gcode-parser.h"
#include "gcode-parser/gcode-streamer.h"
#include "hardware-mapping.h"
#include "motion-queue.h"
#include "motor-operations.h"
#include "pru-hardware-interface.h"
#include "sim-firmware.h"
#include "spindle-control.h"

static int usage(const char *prog, const char *msg) {
  if (msg) {
    fprintf(stderr, "\033[1m\033[31m%s\033[0m\n\n", msg);
  }
  fprintf(stderr, "Usage: %s [options] [<gcode-filename>]\n"
          "Options:\n", prog);
  fprintf(stderr,
          "  -c, --config <config-file> : Configuration file. (Required)\n"
          "  -p, --port <port>          : Listen on this TCP port for GCode.\n"
          "  -b, --bind-addr <bind-ip>  : Bind to this IP (Default: 0.0.0.0).\n"
          "  -l, --logfile <logfile>    : Logfile to use. If empty, messages go to syslog (Default: /dev/stderr).\n"
          "      --param <paramfile>    : Parameter file to use.\n"
          "  -d, --daemon               : Run as daemon.\n"
          "      --priv <uid>[:<gid>]   : After opening GPIO: drop privileges to this (default: daemon:daemon)\n"
          "      --help                 : Display this help text and exit.\n"
          "\nMostly for testing and debugging:\n"
          "  -f <factor>                : Feedrate speed factor (Default 1.0).\n"
          "  -n                         : Dryrun; don't send to motors, no GPIO or PRU needed (Default: off).\n"
          // -N dry-run with simulation output; mostly for development, so not mentioned here.
          "  -P                         : Verbose: Show some more debug output (Default: off).\n"
          "  -S                         : Synchronous: don't queue (Default: off).\n"
          "      --allow-m111           : Allow changing the debug level with M111 (Default: off).\n"
          "\nSegment acceleration tuning:\n"
          "     --threshold-angle       : Specifies the threshold angle used for segment acceleration (Default: 10 degrees).\n"
          "     --speed-tune-angle      : Specifies the angle used for proportional speed-tuning. (Default: 60 degrees)\n\n"
          "                               The --threshold-angle + --speed-tune-angle must be less than 90 degrees.\n"
          "\nConfiguration file overrides:\n"
          "     --homing-required       : Require homing before any moves (require-homing = yes).\n"
          "     --nohoming-required     : (Opposite of above^): Don't require homing before any moves (require-homing = no).\n"
          "     --norange-check         : Disable machine limit checks. (range-check = no).\n");
  return 1;
}

static int fyi_option_gone(const char *prog) {
  fprintf(stderr,
          "Options for machine settings have been removed in favor of a configuration file.\n"
          "Provide it with -c <config-file>.\n"
          "See https://github.com/hzeller/beagleg/blob/master/sample.config\n\n");
  return usage(prog, NULL);
}

// Call with <uid>:<gid> string.
static bool drop_privileges(StringPiece privs) {
  std::vector<StringPiece> pair = SplitString(privs, ":");
  if (pair.size() < 1 || pair.size() > 2) {
    Log_error("Drop privileges: Require colon separated <uid>[:<gid>]");
    return false;
  }

  std::string name;
  if (pair.size() == 2) {
    name = pair[1].ToString();
    struct group *g = getgrnam(name.c_str());
    if (g == NULL) {
      Log_error("Drop privileges: Couldn't look up group '%s'.", name.c_str());
      return false;
    }
    if (setresgid(g->gr_gid, g->gr_gid, g->gr_gid) != 0) {
      Log_error("Couldn't drop group privs to '%s' (gid %d): %s",
                name.c_str(), g->gr_gid, strerror(errno));
      return false;
    }
  }

  name = pair[0].ToString();
  struct passwd *p = getpwnam(name.c_str());
  if (p == NULL) {
    Log_error("Drop privileges: Couldn't look up user '%s'.", name.c_str());
    return false;
  }
  if (setresuid(p->pw_uid, p->pw_uid, p->pw_uid) != 0) {
    Log_error("Couldn't drop user privs to '%s' (uid %d): %s",
              name.c_str(), p->pw_uid, strerror(errno));
    return false;
  }

  return true;
}

// Reads the given "gcode_filename" with GCode and operates machine with it.
static void send_file_to_machine(GCodeMachineControl *machine,
                                 GCodeStreamer *streamer,
                                 const char *gcode_filename) {
  machine->SetMsgOut(stderr);
  int fd = open(gcode_filename, O_RDONLY);
  streamer->ConnectStream(fd, stderr);
}

// Open server. Return file-descriptor or -1 if listen fails.
// Bind to "bind_addr" (can be NULL, then it is 0.0.0.0) and "port".
static int open_server(const char *bind_addr, int port) {
  if (port < 0 || port > 65535) {
    Log_error("Invalid port %d\n", port);
    return -1;
  }
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) {
    Log_error("creating socket: %s", strerror(errno));
    return -1;
  }

  struct sockaddr_in serv_addr = {};
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind_addr && !inet_pton(AF_INET, bind_addr, &serv_addr.sin_addr.s_addr)) {
    Log_error("Invalid bind IP address %s\n", bind_addr);
    return -1;
  }
  serv_addr.sin_port = htons(port);
  int on = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  if (bind(s, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    Log_error("Trouble binding to %s:%d: %s",
              bind_addr ? bind_addr : "0.0.0.0", port,
              strerror(errno));
    return -1;
  }
  return s;
}

// Accept connections and receive GCode.
// Only one connection can be active at a time.
// Socket must already be opened by open_server(). "bind_addr" and "port"
// are just FYI information for nicer log-messages.
static void run_gcode_server(int listen_socket, FDMultiplexer *event_server,
                             GCodeMachineControl *machine,
                             GCodeStreamer *streamer,
                             const char *bind_addr, int port) {
  if (listen(listen_socket, 2) < 0) {
    Log_error("listen(fd=%d) failed: %s", listen_socket, strerror(errno));
    return;
  }

  Log_info("Ready to accept GCode-connections on %s:%d",
           bind_addr ? bind_addr : "0.0.0.0", port);

  event_server->RunOnReadable(listen_socket,
                              [listen_socket,machine,streamer]() {
    struct sockaddr_in client;
    socklen_t socklen = sizeof(client);
    int connection = accept(listen_socket, (struct sockaddr*) &client, &socklen);
    if (connection < 0) {
      Log_error("accept(): %s", strerror(errno));
      return true;
    }

    // We need to set the fd to non blocking in order to avoid
    // blocking reads caused by spurious situations in Linux.
    // http://man7.org/linux/man-pages/man2/select.2.html#BUGS
    const int flags = fcntl(connection, F_GETFL, 0);
    if (flags < 0) {
      Log_error("fcntl(): %s", strerror(errno));
      return true;
    }
    if (fcntl(connection, F_SETFL, flags | O_NONBLOCK) < 0) {
      Log_error("fcntl(): %s", strerror(errno));
      return true;
    }

    if (streamer->IsStreaming()) {
      // For now, only one. Though we could have multiple.
      dprintf(connection, "// Sorry, can only handle one connection at a time."
              "There is only one machine after all.\n");
      close(connection);
      return true;
    }

    char ip_buffer[INET_ADDRSTRLEN];
    const char *print_ip = inet_ntop(AF_INET, &client.sin_addr,
                                     ip_buffer, sizeof(ip_buffer));
    Log_info("Accepting new connection from %s\n", print_ip);

    FILE *msg_stream = fdopen(connection, "w");
    machine->SetMsgOut(msg_stream);
    streamer->ConnectStream(connection, msg_stream);
    return true;
  });
}

// THIS IS A SAMPLE ONLY at this point. We need to come up with a proper
// definition first what we want from a status server.
// At this point: whenever it receives the character 'p' it prints the
// position as json.
static void run_status_server(const char *bind_addr, int port,
                              FDMultiplexer *event_server,
                              GCodeMachineControl *machine) {
  const int listen_socket = open_server(bind_addr, port);
  if (listen_socket < 0) return;
  if (listen(listen_socket, 2) < 0) {
    Log_error("listen(fd=%d) failed: %s", listen_socket, strerror(errno));
    return;
  }

  Log_info("Starting experimental status server on port %d", port);

  event_server->RunOnReadable(
    listen_socket, [listen_socket, machine, event_server]() {
      struct sockaddr_in client;
      socklen_t socklen = sizeof(client);
      int conn = accept(listen_socket, (struct sockaddr*) &client, &socklen);
      if (conn < 0) {
        Log_error("accept(): %s", strerror(errno));
        return true;
      }

      event_server->RunOnReadable(conn, [conn, machine]() {
          char query;
          if (read(conn, &query, 1) <= 0) {
            close(conn);
            return false;
          }
          if (query == 'p') {
            AxesRegister pos;
            machine->GetCurrentPosition(&pos);
            // JSON {"x_axis":fval, "y_axis":fval, "z-axis":fval, "note":"experimental"}
            dprintf(conn, "{\"x_axis\":%.3f, \"y_axis\":%.3f, "
                    "\"z_axis\":%.3f, \"note\":\"experimental\"}\n",
                    pos[AXIS_X], pos[AXIS_Y], pos[AXIS_Z]);
          }
          if (query == 's') {
            GCodeMachineControl::EStopState estop_status = machine->GetEStopStatus();
	    GCodeMachineControl::HomingState home_status = machine->GetHomeStatus();
            // JSON {"estop":"status", "homed":"status", "motors":bool}
            dprintf(conn, "{\"estop\":\"%s\", \"homed\":\"%s\", \"motors\":%s}\n",
                    estop_status == GCodeMachineControl::EStopState::NONE ? "none" :
                    estop_status == GCodeMachineControl::EStopState::SOFT ? "soft" :
                    estop_status == GCodeMachineControl::EStopState::HARD ? "hard" : "unknown",
                    home_status == GCodeMachineControl::HomingState::NEVER_HOMED ? "no" :
                    home_status == GCodeMachineControl::HomingState::HOMED_BUT_MOTORS_UNPOWERED ? "maybe" :
                    home_status == GCodeMachineControl::HomingState::HOMED ? "yes" : "unknown",
		    machine->GetMotorsEnabled() ? "true" : "false");
          }
          return true;
        });
      return true;
    });
}

// Create an absolute filename from a path, without the file not needed
// to exist (so works where realpath() doesn't)
static std::string MakeAbsoluteFile(const char *in) {
  if (!in || in[0] == '\0') return "";
  if (in[0] == '/') return in;
  char buf[1024];
  return std::string(getcwd(buf, sizeof(buf))) + "/" + in;
}

int main(int argc, char *argv[]) {
  MachineControlConfig config;
  bool dry_run = false;
  bool simulation_output = false;
  const char *logfile = NULL;
  std::string paramfile;
  const char *config_file = NULL;
  bool as_daemon = false;
  const char *privs = "daemon:daemon";

  // Less common options don't have a short option.
  enum LongOptionsOnly {
    OPT_HELP = 1000,
    OPT_SET_THRESHOLD_ANGLE,
    OPT_SET_SPEED_TUNE_ANGLE,
    OPT_REQUIRE_HOMING,
    OPT_DONT_REQUIRE_HOMING,
    OPT_DISABLE_RANGE_CHECK,
    OPT_LOOP,
    OPT_PRIVS,
    OPT_ENABLE_M111,
    OPT_PARAM_FILE,
    OPT_STATUS_SERVER
  };

  static struct option long_options[] = {
    { "help",               no_argument,       NULL, OPT_HELP },

    // Required options
    { "config",             required_argument, NULL, 'c'},

    // Configuration file overrides
    { "homing-required",    no_argument,       NULL, OPT_REQUIRE_HOMING },
    { "nohoming-required",  no_argument,       NULL, OPT_DONT_REQUIRE_HOMING },
    { "norange-check",      no_argument,       NULL, OPT_DISABLE_RANGE_CHECK },

    // Optional
    { "port",               required_argument, NULL, 'p'},
    { "bind-addr",          required_argument, NULL, 'b'},
    { "loop",               optional_argument, NULL, OPT_LOOP },
    { "logfile",            required_argument, NULL, 'l'},
    { "param",              required_argument, NULL, OPT_PARAM_FILE },
    { "daemon",             no_argument,       NULL, 'd'},
    { "priv",               required_argument, NULL, OPT_PRIVS },
    { "allow-m111",         no_argument,       NULL, OPT_ENABLE_M111 },
    { "status-server",      required_argument, NULL, OPT_STATUS_SERVER },

    // possibly deprecated soon.
    { "threshold-angle",    required_argument, NULL, OPT_SET_THRESHOLD_ANGLE },
    { "speed-tune-angle",   required_argument, NULL, OPT_SET_SPEED_TUNE_ANGLE },

    { 0,                    0,                 0,    0  },
  };

  int listen_port = -1;
  int status_server_port = -1;
  char *bind_addr = NULL;
  bool require_homing = false;
  bool dont_require_homing = false;
  bool disable_range_check = false;
  bool allow_m111 = false;
  config.threshold_angle = 10;
  config.speed_tune_angle = 60;
  int opt;
  while ((opt = getopt_long(argc, argv, "p:b:SPnNf:l:dc:",
                            long_options, NULL)) != -1) {
    switch (opt) {
    case 'f':
      config.speed_factor = (float)atof(optarg);
      if (config.speed_factor <= 0)
        return usage(argv[0], "Speedfactor cannot be <= 0");
      break;
    case OPT_SET_THRESHOLD_ANGLE:
      config.threshold_angle = (float)atof(optarg);
      break;
    case OPT_SET_SPEED_TUNE_ANGLE:
      config.speed_tune_angle = (float)atof(optarg);
      break;
    case OPT_REQUIRE_HOMING:
      require_homing = true;
      break;
    case OPT_DONT_REQUIRE_HOMING:
      dont_require_homing = true;
      break;
    case OPT_DISABLE_RANGE_CHECK:
      disable_range_check = true;
      break;
    case 'n':
      dry_run = true;
      break;
    case 'N':
      dry_run = true;
      simulation_output = true;
      break;
    case 'P':
      config.debug_print = true;
      break;
    case 'S':
      config.synchronous = true;
      break;
    case OPT_LOOP:
      Log_error("--loop has been removed. Did you use it ? "
                "Let beagleg-dev@googlegroups.com know");
      break;
    case 'p':
      listen_port = atoi(optarg);
      break;
    case OPT_STATUS_SERVER:
      status_server_port = atoi(optarg);
      break;
    case 'b':
      bind_addr = strdup(optarg);
      break;
    case 'l':
      logfile = strdup(optarg);
      break;
    case OPT_PARAM_FILE:
      paramfile = MakeAbsoluteFile(optarg);
      break;
    case 'd':
      as_daemon = true;
      break;
    case 'c':
      config_file = realpath(optarg, NULL); // realpath() -> abs path if exists
      if (!config_file)
        config_file = strdup(optarg);  // Not existing. Report issue later.
      break;
    case OPT_PRIVS:
      privs = strdup(optarg);
      break;
    case OPT_ENABLE_M111:
      allow_m111 = true;
      break;
    case OPT_HELP:
      return usage(argv[0], NULL);
    default:
      // Deprecated, or unknown, option
      return fyi_option_gone(argv[0]);
    }
  }

  if (config.threshold_angle + config.speed_tune_angle >= 90) {
    return usage(argv[0], "--threshold-angle + --speed-tune-angle must be < 90 degrees.");
  }

  if (require_homing && dont_require_homing) {
    return usage(argv[0], "Choose one: --homing-required or --nohoming-required.");
  }

  const bool has_filename = (optind < argc);
  if (! (has_filename ^ (listen_port > 0))) {
    return usage(argv[0], "Choose one: <gcode-filename> or --port <port>.");
  }

  // As daemon, we use whatever the user chose as logfile
  // (including nothing->syslog). Interactive, nothing means stderr.
  Log_init(as_daemon ? logfile : (logfile == NULL ? "/dev/stderr" : logfile));
  Log_info("BeagleG " BEAGLEG_VERSION " startup; "
           CAPE_NAME " hardware interface.");

  if (config.threshold_angle > 0) {
    const double deg2rad = M_PI / 180.0;
    const double min_speed_adj = std::cos(config.speed_tune_angle * deg2rad);
    const double max_speed_adj = std::cos((config.threshold_angle + config.speed_tune_angle) * deg2rad);
    Log_debug("Speed-tuning from %.2f to %.2f for angles +/-%.2f degrees (speed-tune @ %.2f degrees)\n",
              min_speed_adj, max_speed_adj, config.threshold_angle, config.speed_tune_angle);
  }

  // If reading from file: don't print 'ok' for every line.
  config.acknowledge_lines = !has_filename;

  if (!config_file) {
    Log_error("Expected config file -c <config>");
    return 1;
  } else {
    Log_info("Reading config %s", config_file);
  }

  ConfigParser config_parser;
  if (!config_parser.SetContentFromFile(config_file)) {
    Log_error("Exiting. Cannot read config file '%s'", config_file);
    return 1;
  }
  if (!config.ConfigureFromFile(&config_parser)) {
    Log_error("Exiting. Parse error in configuration file '%s'", config_file);
    return 1;
  }

  HardwareMapping hardware_mapping;
  if (!hardware_mapping.ConfigureFromFile(&config_parser)) {
    Log_error("Exiting. Couldn't initialize hardware mapping (%s)", config_file);
    return 1;
  }

  SpindleConfig spindle_config;
  if (!spindle_config.ConfigureFromFile(&config_parser)) {
    Log_error("Exiting. Errors in spindle configuration in (%s)", config_file);
    return 1;
  }

  std::unique_ptr<Spindle> spindle(
    Spindle::CreateFromConfig(spindle_config, &hardware_mapping));

  // ... other configurations that read from that file.

  // Handle command line configuration overrides.
  if (require_homing)      config.require_homing = true;
  if (dont_require_homing) config.require_homing = false;
  if (disable_range_check) config.range_check = false;

  if (as_daemon && daemon(0, 0) != 0) {
    Log_error("Can't become daemon: %s", strerror(errno));
  }

  FDMultiplexer event_server;
  // Open socket early, so that we
  //  (a) can bail out early before messing with GPIO/PRU settings if
  //      someone is alrady listening (starting as daemon twice?).
  //  (b) open socket while we have not dropped privileges yet.
  int listen_socket = -1;
  if (!has_filename) {
    listen_socket = open_server(bind_addr, listen_port);
    if (listen_socket < 0) {
      Log_error("Exiting. Couldn't bind to socket to listen.");
      return 1;
    }
  }

  // The backend for our stepmotor control. We either talk to the PRU or
  // just ignore them on dummy.
  MotionQueue *motion_backend;
  PruHardwareInterface *pru_hw_interface = NULL;
  if (dry_run) {
    // The backend
    if (simulation_output) {
      motion_backend = new SimFirmwareQueue(stdout, 3); // TODO: derive from cfg
    } else {
      motion_backend = new DummyMotionQueue();
    }
  } else {
    if (!hardware_mapping.InitializeHardware()) {
      Log_error("Exiting. (Just testing ? "
                "Use the dryrun option -n to not write to GPIO).");
      return 1;
    }
    pru_hw_interface = new UioPrussInterface();
    motion_backend = new PRUMotionQueue(&hardware_mapping, pru_hw_interface);
  }

  // Listen port bound, GPIO initialized. Ready to drop privileges.
  if (geteuid() == 0 && strlen(privs) > 0) {
    if (drop_privileges(privs)) {
      Log_info("Dropped privileges to '%s'", privs);
    } else {
      Log_error("Exiting. Could not drop privileges to %s", privs);
      return 1;
    }
  }
  Log_info("BeagleG running with PID %d", getpid());

  MotionQueueMotorOperations motor_operations(&hardware_mapping, motion_backend);

  GCodeMachineControl *machine_control
    = GCodeMachineControl::Create(config, &motor_operations,
                                  &hardware_mapping, spindle.get(),
                                  stderr);
  if (machine_control == NULL) {
    Log_error("Exiting. Cannot initialize machine control.");
    return 1;
  }
  GCodeParser::Config parser_cfg(paramfile);
  parser_cfg.allow_m111 = allow_m111;
  GCodeParser::Config::ParamMap parameters;
  parser_cfg.parameters = &parameters;
  parser_cfg.LoadParams();

  machine_control->GetHomePos(&parser_cfg.machine_origin);
  GCodeParser *parser = new GCodeParser(parser_cfg,
                                        machine_control->ParseEventReceiver());
  GCodeStreamer *streamer =
    new GCodeStreamer(&event_server, parser,
                      machine_control->ParseEventReceiver());
  int ret = 0;
  if (has_filename) {
    const char *filename = argv[optind];
    send_file_to_machine(machine_control, streamer, filename);
  } else {
    run_gcode_server(listen_socket, &event_server, machine_control,
                     streamer,  bind_addr, listen_port);
  }

  if (status_server_port > 0 && !has_filename) {
    run_status_server(bind_addr, status_server_port,
                      &event_server, machine_control);
  }

  event_server.Loop();  // Run service until Ctrl-C or all sockets closed.
  Log_info("Exiting.");

  delete streamer;
  delete parser;
  delete machine_control;

  const bool caught_signal = (ret == 1);
  if (caught_signal) {
    Log_info("Caught signal: immediate exit. "
             "Skipping potential remaining queue.");
  }
  motion_backend->Shutdown(!caught_signal);

  delete motion_backend;
  delete pru_hw_interface;

  free(bind_addr);

  parser_cfg.SaveParams();

  Log_info("Shutdown.");
  return ret;
}
