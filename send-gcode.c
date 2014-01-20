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
static const int kDefaultMaxFeedrate = 200; // mm/s
static const int kDefaultAcceleration = 4000; // mm/s^2
static const int kStepsPerMM[] = { 160, 160, 160, 40, 0, 0, 0, 0 };
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
	  "  -f <factor> : Print speed factor (Default 1.0).\n"
	  "  -m <rate>   : Max. feedrate (Default %dmm/s).\n"
	  "  -a <accel>  : Acceleration/Deceleration (Default %dmm/s^2).\n"
	  "  -l <port>   : Listen on this TCP port.\n"
	  "  -b <bind-ip>: Bind to this IP (Default: 0.0.0.0)\n"
	  "  -n          : Dryrun; don't send to motors (Default: off).\n"
	  "  -P          : Verbose: Print motor commands (Default: off).\n"
	  "  -S          : Synchronous: don't queue (Default: off).\n"
	  "  -R          : Repeat file forever.\n",
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

int main(int argc, char *argv[]) {
  struct MachineControlConfig config;
  // Per axis X, Y, Z, E (Z and E: need to look up)
  memcpy(config.axis_steps_per_mm, kStepsPerMM,
	 sizeof(config.axis_steps_per_mm));
  config.max_feedrate = kDefaultMaxFeedrate;
  config.speed_factor = 1;
  config.acceleration_steps_s2 = kStepsPerMM[0] * kDefaultAcceleration;
  config.dry_run = 0;
  config.debug_print = 0;
  config.synchronous = 0;

  int listen_port = -1;
  char do_file_repeat = 0;
  char *bind_addr = NULL;
  int opt;
  while ((opt = getopt(argc, argv, "SPRnl:f:m:a:b:")) != -1) {
    switch (opt) {
    case 'f':
      config.speed_factor = atof(optarg);
      if (config.speed_factor <= 0)
	return usage(argv[0], "Speedfactor cannot be <= 0");
      break;
    case 'm':
      config.max_feedrate = atoi(optarg);
      if (config.max_feedrate <= 0)
	return usage(argv[0], "Feedrate cannot be <= 0");
      break;
    case 'a':
      config.acceleration_steps_s2 = kStepsPerMM[0] * atoi(optarg);
      // Negative or 0 means: 'infinite'.
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
    case 'l':
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
