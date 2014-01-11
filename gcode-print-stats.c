/*
 * (c) 2013, 1014 Henner Zeller <h.zeller@acm.org>
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

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "determine-print-stats.h"

int usage(const char *prog) {
  fprintf(stderr, "Usage: %s <gcode-file> [<gcode-file> ..]\n", prog);
  return 1;
}

static void print_file_stats(const char *filename, int indentation,
			     float speed_factor, float max_feedrate) {
  struct BeagleGPrintStats result;
  int fd = strcmp(filename, "-") == 0 ? STDIN_FILENO : open(filename, O_RDONLY);
  if (determine_print_stats(fd,
			    max_feedrate, speed_factor, &result) == 0) {
    // Filament length looks a bit high, is this input or extruded ?
    printf("%-*s\t%9.3fs\t%6.1fmm\t%7.1fmm", indentation, filename,
	   result.total_time_seconds, result.last_z, result.filament_len);
    if (result.highest_capped_feedrate > 0) {
      printf("Max feedrate requested %.1f mm/s, but capped to %.1f mm/s ",
	     result.highest_capped_feedrate, max_feedrate);
    }
    printf("\n");
  } else {
    printf("#%s not-processed\n", filename);
  }
}

int main(int argc, char *argv[]) {
  if (argc < 2) return usage(argv[0]);
  int max_feedrate = 200;  // mm/s
  int factor = 1.0;        // print speed factor.
  int longest_filename = strlen(argv[1]);
  for (int i = 2; i < argc; ++i) {
    int len = strlen(argv[i]);
    if (len > longest_filename) longest_filename = len;
  }
  // Print table header
  printf("%-*s\t%10s\t%8s\t%9s\n", longest_filename,
	 "#--filename", "time", "height", "filament");
  for (int i = 1; i < argc; ++i) {
    print_file_stats(argv[i], longest_filename, factor, max_feedrate);
  }
  return 0;
}
