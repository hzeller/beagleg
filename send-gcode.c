/*
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

#include "gcode-parser.h"
#include "motor-interface.h"
#include "determine-print-stats.h"
#include "gcode-machine-control.h"

// Some default settings.
static const float kDefaultMaxFeedrate = 200; // mm/s
static const float kDefaultAcceleration = 4000; // mm/s^2
static const float kStepsPerMM[] = { 160, 160, 160, 40,  0,  0,  0,  0 };
static const float kHomePos[]    = {   0,   0,   0, -1, -1, -1, -1, -1 };
static const float kMoveRange[]  = { 100, 100, 100, -1, -1, -1, -1, -1 };
static const float kFilamentDiameter = 1.7;  // mm

static void print_file_stats(const char *filename,
			     struct MachineControlConfig *config) {
  struct BeagleGPrintStats result;
  if (determine_print_stats(open(filename, O_RDONLY),
			    config->max_feedrate, config->speed_factor,
			    &result) == 0) {
    const float filament_volume
      = kFilamentDiameter*kFilamentDiameter/4 * M_PI * result.filament_len;
    printf("----------------------------------------------\n");
    printf("Print time: %.3f seconds; height: %.1fmm; max feedrate: %.1fmm/s; "
	   "filament length: %.1fmm (volume %.2fcm^3).\n",
	   result.total_time_seconds, result.last_z, result.max_G1_feedrate,
	   result.filament_len, filament_volume / 1000);
    printf("----------------------------------------------\n");
  }
}

static int usage(const char *prog, const char *msg) {
  if (msg) {
    fprintf(stderr, "%s\n\n", msg);
  }
  fprintf(stderr, "Usage: %s [options] [<gcode-filename>]\n"
	  "Options:\n"
	  "  --max-feedrate <rate> (-m): Max. feedrate (Default %.1fmm/s).\n"
	  "  --accel <accel>       (-a): "
	  "Acceleration/Deceleration (Default %.1fmm/s^2).\n"
	  "  --port <port>         (-p): Listen on this TCP port.\n"
	  "  --bind-addr <bind-ip> (-b): Bind to this IP (Default: 0.0.0.0)\n"
	  "  --steps-mm <axis-steps>   : steps/mm, comma separated. "
	  "(Default 160,160,160,40)\n"
#if 0   // not yet implemented
	  "  --home-pos <pos-mm>       : Home positions of axes. Only values "
	  ">= 0\n"
	  "                               participate in homing. "
	  "(Default: 0,0,0,-1,-1,...)\n"
	  "  --range <range-mm>    (-r): Range of of axes in mm (0..range[axis])"
	  ". Only\n"
	  "                               values > 0 are actively clipped. "
	  "(Default: 100,100,100,-1,-1,...)\n"
#endif
	  "  -f <factor>               : Print speed factor (Default 1.0).\n"
	  "  -n                        : Dryrun; don't send to motors "
	  "(Default: off).\n"
	  "  -P                        : Verbose: Print motor commands "
	  "(Default: off).\n"
	  "  -S                        : Synchronous: don't queue "
	  "(Default: off).\n"
	  "  -R                        : Repeat file forever.\n",
	  prog, kDefaultMaxFeedrate, kDefaultAcceleration);
  fprintf(stderr, "You can either specify -l <port> to listen for commands "
	  "or give a filename\n");
  return 1;
}

static int send_file_to_printer(const char *filename, char do_loop) {
  do {
    int fd = open(filename, O_RDONLY);
    if (gcode_machine_control_from_stream(fd, STDERR_FILENO) != 0)
      return 1;
  } while (do_loop);
  return 0;
}

static int run_server(const char *bind_addr, int port) {
  if (port > 65535) {
    fprintf(stderr, "Invalid port %d\n", port);
    return 1;
  }
  int s = socket(AF_INET, SOCK_STREAM, 0);
  if (s < 0) {
    perror("creating socket");
    return 1;
  }

  struct sockaddr_in serv_addr;
  bzero(&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind_addr) {
    if (!inet_pton(AF_INET, bind_addr, &serv_addr.sin_addr.s_addr)) {
      fprintf(stderr, "Invalid bind IP address %s\n", bind_addr);
      return 1;
    }
  } else {
    bind_addr = "0.0.0.0";
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
  printf("Listening on %s:%d\n", bind_addr, port);

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
    process_result = gcode_machine_control_from_stream(connection, connection);
    printf("Connection to %s closed.\n", print_ip);
  } while (process_result == 0);

  close(s);
  fprintf(stderr, "Last gcode_machine_control_from_stream() == %d. Exiting\n",
	  process_result);
  return 0;
}

static int parse_float_array(const char *input, float result[], int count) {
  for (int i = 0; i < count; ++i) {
    char *end;
    result[i] = strtod(input, &end);
    if (end == input) return 0;  // parse error.
    if (*end == '\0') return 1;
    input = end + 1;
  }
  return 1;
}

int main(int argc, char *argv[]) {
  struct MachineControlConfig config;
  // Per axis X, Y, Z, E (Z and E: need to look up)
  memcpy(config.axis_steps_per_mm, kStepsPerMM,
	 sizeof(config.axis_steps_per_mm));
  memcpy(config.home_position, kHomePos,
	 sizeof(config.home_position));
  config.max_feedrate = kDefaultMaxFeedrate;
  config.speed_factor = 1;
  config.acceleration = kDefaultAcceleration;
  config.dry_run = 0;
  config.debug_print = 0;
  config.synchronous = 0;

  // Less common options don't have a short option.
  enum LongOptionsOnly {
    SET_STEPS_MM = 1000,
    SET_HOME_POS,
  };

  static struct option long_options[] = {
    { "max-feedrate",  required_argument, NULL, 'm'},
    { "accel",         required_argument, NULL, 'a'},
    { "steps-mm",      required_argument, NULL, SET_STEPS_MM },
    { "port",          required_argument, NULL, 'p'},
    { "home-pos",      required_argument, NULL, SET_HOME_POS },
    { "range",         required_argument, NULL, 'r' },
    { "bind-addr",     required_argument, NULL, 'b'},
    { 0,               0,                 0,    0  },
  };

  int listen_port = -1;
  char do_file_repeat = 0;
  char *bind_addr = NULL;
  int opt;
  while ((opt = getopt_long(argc, argv, "m:a:p:b:r:SPRnf:",
			    long_options, NULL)) != -1) {
    switch (opt) {
    case 'f':
      config.speed_factor = atof(optarg);
      if (config.speed_factor <= 0)
	return usage(argv[0], "Speedfactor cannot be <= 0");
      break;
    case 'm':
      config.max_feedrate = atof(optarg);
      if (config.max_feedrate <= 0)
	return usage(argv[0], "Feedrate cannot be <= 0");
      break;
    case 'a':
      config.acceleration = atof(optarg);
      // Negative or 0 means: 'infinite'.
      break;
    case SET_STEPS_MM:
      if (!parse_float_array(optarg, config.axis_steps_per_mm, 8))
	return usage(argv[0], "steps/mm failed to parse.");
      break;
    case SET_HOME_POS:
      if (!parse_float_array(optarg, config.home_position, 8))
	return usage(argv[0], "Failed to parse home pos.");
      break;
    case 'r':
      if (!parse_float_array(optarg, config.move_range, 8))
	return usage(argv[0], "Failed to parse ranges.");
      break;
    case 'n':
      config.dry_run = 1;
      break;
    case 'P':
      config.debug_print = 1;
      break;
    case 'S':
      config.synchronous = 1;
      break;
    case 'R':
      do_file_repeat = 1;
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
    return usage(argv[0], "Choose one: <gcode-filename> or -l <port>.");
  }
  if (!has_filename && do_file_repeat) {
    return usage(argv[0], "-R (repeat) only makes sense with a filename.");
  }

  if (gcode_machine_control_init(&config) != 0) {
    return 1;
  }

  int ret = 0;
  if (has_filename) {
    const char *filename = argv[optind];
    print_file_stats(filename, &config);
    ret = send_file_to_printer(filename, do_file_repeat);
  } else {
    ret = run_server(bind_addr, listen_port);
  }

  gcode_machine_control_exit();
  free(bind_addr);
  return ret;
}
