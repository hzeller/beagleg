/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2015 Henner Zeller <h.zeller@acm.org>
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

#ifndef BEAGLEG_LOGGING_H
#define BEAGLEG_LOGGING_H

#include <string>

// With filename given, logs debug, info and error to that file.
// If filename is NULL, info and errors are logged to syslog.
void Log_init(const char *filename);

// Define this with empty, if you're not using gcc.
#define PRINTF_FMT_CHECK(fmt_pos, args_pos)             \
  __attribute__ ((format (printf, fmt_pos, args_pos)))

void Log_debug(const char *format, ...) PRINTF_FMT_CHECK(1, 2);
void Log_info(const char *format, ...) PRINTF_FMT_CHECK(1, 2);
void Log_error(const char *format, ...) PRINTF_FMT_CHECK(1, 2);

#undef PRINTF_FMT_CHECK

#endif /* BEAGLEG_LOGGING_H */
