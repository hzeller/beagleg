/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
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
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/types.h>
#include <unistd.h>

#include "gcode-machine-control.h"
#include "gcode-parser.h"
#include "motor-operations.h"
#include "motion-queue.h"

#include "sim-firmware.h"

static int usage(const char *prog, const char *msg) {
  if (msg) {
    fprintf(stderr, "%s\n\n", msg);
  }
  fprintf(stderr, "Usage: %s [options] [<gcode-filename>]\n"
	  "Options:\n"
	  "  --steps-mm <axis-steps>   : steps/mm, comma separated[*] (Default 160,160,160,40,0, ...).\n"
	  "                                (negative for reverse)\n"
	  "  --max-feedrate <rate> (-m): Max. feedrate per axis (mm/s), comma separated[*] (Default: 200,200,90,10,0, ...).\n"
	  "  --accel <accel>       (-a): Acceleration per axis (mm/s^2), comma separated[*] (Default 4000,4000,1000,10000,0, ...).\n"
<<<<<<< HEAD
#if 0   // not yet implemented
	  "  --home-pos <0/1/2>,*      : Home positions of axes, comma separated\n"
	  "                                0 = none, 1 = origin; 2 = end-of-range (Default: 1,1,1,0,...).\n"
	  "  --range <range-mm>    (-r): Comma separated range of of axes in mm (0..range[axis]). Only\n"
	  "                               values > 0 are actively clipped. (Default: 100,100,100,-1,-1, ...)\n"
#endif
	  "  --axis-mapping            : Axis letter mapped to which motor connector (=string pos)\n"
	  "                              Use letter or '_' for empty slot.\n"
          "                              You can use the same letter multiple times for mirroring.\n"
	  "                              Use lowercase to reverse. (Default: 'XYZEA')\n"
#if 0     // Commented out for now as it is just confusing. Only needed for
          // different hardware.
          "  --channel-layout          : Driver channel (0..7) mapped to which motor connector (=string pos)\n"
          "                              This depends on the harware mapping of the cape (Default for BUMPS: '23140').\n"
#endif
=======
          "  --channel-layout          : Driver channel (0..7) mapped to which motor connector (=string pos)\n"
          "                                This depends on the harware mapping of the cape (Default for BUMPS: '23140').\n"
	  "  --axis-mapping            : Axis letter mapped to which motor connector (=string pos)\n"
	  "                                Use letter or '_' for empty slot.\n"
          "                                You can use the same letter multiple times for mirroring.\n"
	  "                                Use lowercase to reverse. (Default: 'XYZEA')\n"
	  "  --range <range-mm>    (-r): Comma separated range of of axes in mm (0..range[axis]). Only\n"
	  "                                values > 0 are actively clipped. (Default: 100,100,100,-1,-1, ...)\n"
	  "  --min-endswitch           : Axis letter mapped to which endstop connector for negative travel (=string pos)\n"
	  "                                Use letter or '_' for unused endstop.\n"
	  "                                Use uppercase if endstop is used for homimg, lowercase if used for travel limit.\n"
	  "  --max-endswitch           : Axis letter mapped to which endstop connector for positive travel (=string pos)\n"
	  "                                Use letter or '_' for unused endstop.\n"
	  "                                Use uppercase if endstop is used for homimg, lowercase if used for travel limit.\n"
          "  --home-order              : Order to home axes, all axes involved with homing should be listed (Default: ZXY)\n"
	  "  --endswitch-polarity      : 'Hit' polarity for each endstop connector (=string pos).\n"
	  "                                Use '1' or '+' for logic high trigger.\n"
	  "                                Use '0' or '-' for logic low trigger.\n"
	  "                                Use '_' for unused endstops.\n"
>>>>>>> multi-hardware-support
	  "  --port <port>         (-p): Listen on this TCP port for GCode.\n"
	  "  --bind-addr <bind-ip> (-b): Bind to this IP (Default: 0.0.0.0).\n"
	  "  -f <factor>               : Print speed factor (Default 1.0).\n"
	  "  -n                        : Dryrun; don't send to motors (Default: off).\n"
          // -N dry-run with simulation output; mostly for development, so not mentioned here.
	  "  -P                        : Verbose: Print motor commands (Default: off).\n"
	  "  -S                        : Synchronous: don't queue (Default: off).\n"
	  "  --loop[=count]            : Loop file number of times (no value: forever)\n",
	  prog);
  fprintf(stderr, "[*] All comma separated axis numerical values are in the sequence X,Y,Z,E,A,B,C,U,V,W\n");
  fprintf(stderr, "(the actual mapping to a connector happens with --axis-mapping,\n");
  fprintf(stderr, "the default values map the channels left to right on the Bumps-board as X,Y,Z,E,A)\n");
  fprintf(stderr, "You can either specify --port <port> to listen for commands or give a GCode-filename\n");
  fprintf(stderr, "All numbers can be given as multiplicative expression\n"
          "which makes microstepping and unit conversions more readable\n"
          "e.g. --steps-mm '16*200/(25.4/4),8*200/4'\n");
  return 1;
}

// Reads the given "gcode_filename" with GCode and operates machine with it.
// If "loop_count" is >= 0, repeats this number after the first execution.
static int send_file_to_machine(struct MachineControlConfig *config,
                                struct MotorOperations *motor_ops,
                                const char *gcode_filename, int loop_count) {
  int ret;
  while (loop_count < 0 || loop_count-- > 0) {
    int fd = open(gcode_filename, O_RDONLY);
    GCodeMachineControl_t *machine_control
      = gcode_machine_control_new(config, motor_ops, stderr);
    if (machine_control == NULL)
      return 1;
    ret = gcodep_parse_stream(fd,
                              gcode_machine_control_event_receiver(machine_control),
                              stderr);
    gcode_machine_control_delete(machine_control);
    if (ret != 0)
      break;
  }
  return ret;
}

// Run TCP server on "bind_addr" (can be NULL, then it is 0.0.0.0) and "port".
// Interprets GCode coming from a connection. Only one connection at a
// time can be active.
static int run_server(struct MachineControlConfig *config,
                      struct MotorOperations *motor_ops,
                      const char *bind_addr, int port) {
  if (port > 65535) {
    fprintf(stderr, "Invalid port %d\n", port);
    return 1;
  }
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) {
    perror("creating socket");
    return 1;
  }

  struct sockaddr_in serv_addr = {0};
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind_addr && !inet_pton(AF_INET, bind_addr, &serv_addr.sin_addr.s_addr)) {
    fprintf(stderr, "Invalid bind IP address %s\n", bind_addr);
    return 1;
  }
  serv_addr.sin_port = htons(port);
  int on = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  if (bind(s, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
    perror("trouble binding");
    return 1;
  }

  signal(SIGPIPE, SIG_IGN);  // Pesky clients, closing connections...

  listen(s, 2);
  printf("Listening on %s:%d\n", bind_addr ? bind_addr : "0.0.0.0", port);

  int process_result;
  do {
    struct sockaddr_in client;
    socklen_t socklen = sizeof(client);
    int connection = accept(s, (struct sockaddr*) &client, &socklen);
    if (connection < 0) {
      perror("accept");
      return 1;
    }
    char ip_buffer[INET_ADDRSTRLEN];
    const char *print_ip = inet_ntop(AF_INET, &client.sin_addr,
				     ip_buffer, sizeof(ip_buffer));
    printf("Accepting new connection from %s\n", print_ip);
    FILE *msg_stream = fdopen(connection, "w");
    GCodeMachineControl_t *machine_control
      = gcode_machine_control_new(config, motor_ops, msg_stream);
    if (machine_control == NULL)
      return 1;
    process_result
      = gcodep_parse_stream(connection,
                            gcode_machine_control_event_receiver(machine_control),
                            msg_stream);
    gcode_machine_control_delete(machine_control);
    fclose(msg_stream);
    printf("Connection to %s closed.\n", print_ip);
  } while (process_result == 0);

  close(s);
  fprintf(stderr, "Last gcode_machine_control_from_stream() == %d. Exiting\n",
	  process_result);
  return process_result;
}

// Parse a double value and allow simple multiplicative operations.
// So "13.524" or "3200/6.35" or "3200/25.4*4" would be examples for valid input.
// Returns the parsed value and in "end" the end of parse position.
static double parse_double_optional_fraction(const char *input, double fallback,
                                             char **end) {
  const char *full_expr = input;
  double value = strtod(input, end);
  if (*end == input) return fallback;
  for (;;) {
    while (isspace(**end)) ++*end;
    const char op = **end;
    if (op != '/' && op != '*') {
      return value;  // done. Not an operation.
    }
    ++*end;
    while (isspace(**end)) ++*end;
    input = *end;
    double operand;
    if (*input == '(') {
      operand = parse_double_optional_fraction(input+1, 1.0, end);
      if (**end != ')') {
        fprintf(stderr, "Mismatching parenthesis in '%s'\n", full_expr);
        return fallback;
      } else {
        ++*end;
      }
    } else {
      operand = strtod(input, end);
    }
    if (*end == input) return fallback;
    if (op == '/')
      value /= operand;
    else if (op == '*')
      value *= operand;
  }
  return value;
}

// Parse comma (or other character) separated array of up to "count" float
// numbers and fill into result[]. Returns number of elements parsed on success.
static int parse_float_array(const char *input, float result[], int count) {
  const char *full = input;
  for (int i = 0; i < count; ++i) {
    char *end;
    result[i] = (float) parse_double_optional_fraction(input, 0, &end);
    if (end == input) return 0;  // parse error.
    while (isspace(*end)) ++end;
    if (*end == '\0') return i + 1;
    if (*end != ',') {
      fprintf(stderr, "Expected comma separation\n%s\n%*s^\n",
              full, (int)(end - full), " ");
      return 0;
    }
    input = end + 1;
  }
  return count;
}

int main(int argc, char *argv[]) {
  struct MachineControlConfig config;
  gcode_machine_control_default_config(&config);

  char dry_run = 0;
  char simulation_output = 0;

  // Less common options don't have a short option.
  enum LongOptionsOnly {
    OPT_SET_STEPS_MM = 1000,
    OPT_SET_HOME_POS,
    OPT_SET_HOME_ORDER,
    OPT_SET_CHANNEL_LAYOUT,
    OPT_SET_MOTOR_MAPPING,
    OPT_SET_MIN_ENDSWITCH,
    OPT_SET_MAX_ENDSWITCH,
    OPT_SET_ENDSWITCH_POLARITY,
    OPT_LOOP,
  };

  static struct option long_options[] = {
    { "max-feedrate",       required_argument, NULL, 'm'},
    { "accel",              required_argument, NULL, 'a'},
    { "range",              required_argument, NULL, 'r' },
    { "steps-mm",           required_argument, NULL, OPT_SET_STEPS_MM },
    { "home-order",         required_argument, NULL, OPT_SET_HOME_ORDER },
    { "channel-layout",     required_argument, NULL, OPT_SET_CHANNEL_LAYOUT },
    { "axis-mapping",       required_argument, NULL, OPT_SET_MOTOR_MAPPING },
    { "min-endswitch",      required_argument, NULL, OPT_SET_MIN_ENDSWITCH },
    { "max-endswitch",      required_argument, NULL, OPT_SET_MAX_ENDSWITCH },
    { "endswitch-polarity", required_argument, NULL, OPT_SET_ENDSWITCH_POLARITY },
    { "port",               required_argument, NULL, 'p'},
    { "bind-addr",          required_argument, NULL, 'b'},
    { "loop",               optional_argument, NULL, OPT_LOOP },
    { 0,                    0,                 0,    0  },
  };

  int listen_port = -1;
  int file_loop_count = 1;
  char *bind_addr = NULL;
  int opt;
  int parse_count;
  while ((opt = getopt_long(argc, argv, "m:a:p:b:r:SPnNf:",
			    long_options, NULL)) != -1) {
    switch (opt) {
    case 'f':
      config.speed_factor = (float)atof(optarg);
      if (config.speed_factor <= 0)
	return usage(argv[0], "Speedfactor cannot be <= 0");
      break;
    case 'm':
      parse_count = parse_float_array(optarg, config.max_feedrate,
                                      GCODE_NUM_AXES);
      if (!parse_count) return usage(argv[0], "max-feedrate missing.");
      break;
    case 'a':
      parse_count = parse_float_array(optarg, config.acceleration,
                                      GCODE_NUM_AXES);
      if (!parse_count) return usage(argv[0], "Acceleration missing.");
      // Negative or 0 means: 'infinite'.
      break;
    case OPT_SET_STEPS_MM:
      if (!parse_float_array(optarg, config.steps_per_mm, GCODE_NUM_AXES))
	return usage(argv[0], "steps/mm failed to parse.");
      break;
    case OPT_SET_CHANNEL_LAYOUT:
      config.channel_layout = strdup(optarg);
      break;
    case OPT_SET_MOTOR_MAPPING:
      config.axis_mapping = strdup(optarg);
      break;
    case OPT_SET_MIN_ENDSWITCH:
      config.min_endswitch = strdup(optarg);
      break;
    case OPT_SET_MAX_ENDSWITCH:
      config.max_endswitch = strdup(optarg);
      break;
    case OPT_SET_ENDSWITCH_POLARITY:
      config.endswitch_polarity = strdup(optarg);
      break;
    case OPT_SET_HOME_ORDER:
      config.home_order = strdup(optarg);
      break;
    case 'r':
      if (!parse_float_array(optarg, config.move_range_mm, GCODE_NUM_AXES))
	return usage(argv[0], "Failed to parse ranges.");
      break;
    case 'n':
      dry_run = 1;
      break;
    case 'N':
      dry_run = 1;
      simulation_output = 1;
      break;
    case 'P':
      config.debug_print = 1;
      break;
    case 'S':
      config.synchronous = 1;
      break;
    case OPT_LOOP:
      file_loop_count = (optarg) ? atoi(optarg) : -1;
      break;
    case 'p':
      listen_port = atoi(optarg);
      break;
    case 'b':
      bind_addr = strdup(optarg);
      break;
    default:
      return usage(argv[0], "Unknown flag");
    }
  }

  const char has_filename = (optind < argc);
  if (! (has_filename ^ (listen_port > 0))) {
    return usage(argv[0], "Choose one: <gcode-filename> or --port <port>.");
  }
  if (!has_filename && file_loop_count != 1) {
    return usage(argv[0], "--loop only makes sense with a filename.");
  }

  // The backend for our stepmotor control. We either talk to the PRU or
  // just ignore them on dummy.
  struct MotionQueue motion_backend;
  if (dry_run) {
    if (simulation_output) {
      init_sim_motion_queue(&motion_backend);
    } else {
      init_dummy_motion_queue(&motion_backend);
    }
  } else {
    if (geteuid() != 0) {
      // TODO: running as root is generally not a good idea. Setup permissions
      // to just access these GPIOs.
      fprintf(stderr, "Need to run as root to access GPIO pins. "
	      "(use the dryrun option -n to not write to GPIO)\n");
      return 1;
    }
    init_pru_motion_queue(&motion_backend);
  }

  struct MotorOperations motor_operations;
  beagleg_init_motor_ops(&motion_backend, &motor_operations);

  int ret = 0;
  if (has_filename) {
    const char *filename = argv[optind];
    ret = send_file_to_machine(&config, &motor_operations,
                               filename, file_loop_count);
  } else {
    ret = run_server(&config, &motor_operations, bind_addr, listen_port);
  }

  const char caught_signal = (ret == 2);
  if (caught_signal) {
    fprintf(stderr, "Immediate exit. Skipping potential remaining queue.\n");
  }
  motion_backend.shutdown(!caught_signal);

  free(bind_addr);
  return ret;
}
