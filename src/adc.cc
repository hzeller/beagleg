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

/*
 * $ echo PyBBIO-ADC > $SLOTS
 * $ cat in_voltage?_raw
 */

#include "adc.h"

#include <stdio.h>
#include <stdlib.h>

#include "common/logging.h"

static const char kSysNode[] = "/sys/bus/iio/devices/iio:device0";

int arc_read_raw(int chan) {
  if (chan < 0 || chan > 7) {
    Log_error("arc_read_raw: invalid channel %d", chan);
    return -1;
  }

  char node[256];
  snprintf(node, sizeof(node), "%s/in_voltage%d_raw", kSysNode, chan);
  FILE *fp = fopen(node, "r");
  if (!fp) {
    Log_error("arc_read_raw: unable to open %s", node);
    return -1;
  }

  char buf[32];
  if (!fgets(buf, sizeof(buf), fp)) {
    Log_error("arc_read_raw: unable to read %s", node);
    sprintf(buf, "-1");
  }
  fclose(fp);

  return atoi(buf);
}
