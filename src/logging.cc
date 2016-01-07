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
#include "logging.h"

#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

static int log_fd = 2;  // Allow logging before Log_init().

static const char *const kInfoHighlight  = "\033[1mINFO  ";
static const char *const kDebugHighlight = "\033[1m\033[34mDEBUG ";
static const char *const kErrorHighlight = "\033[1m\033[31mERROR ";
static const char *const kTermReset      = "\033[0m";

static const char *debug_markup_start_ = "DEBUG ";
static const char *info_markup_start_  = "INFO  ";
static const char *error_markup_start_ = "ERROR ";
static const char *markup_end_ = "";

void Log_init(const char *filename) {
  if (filename == NULL || strlen(filename) == 0) {
    openlog(NULL, LOG_PID|LOG_CONS, LOG_DAEMON);
    log_fd = -1;
  } else {
    log_fd = open(filename, O_CREAT|O_APPEND|O_WRONLY, 0644);
    if (log_fd < 0) {
      perror("Cannot open logfile");
      openlog(NULL, LOG_PID|LOG_CONS, LOG_DAEMON); // fallback.
      return;
    }
    bool enable_color = isatty(log_fd);
    if (enable_color) {
      info_markup_start_ = kInfoHighlight;
      debug_markup_start_ = kDebugHighlight;
      error_markup_start_ = kErrorHighlight;
      markup_end_ = kTermReset;
    }
  }
}

static void Log_internal(int fd, const char *markup_start,
                         const char *format, va_list ap) {
  struct timeval now;
  gettimeofday(&now, NULL);
  struct tm time_breakdown;
  localtime_r(&now.tv_sec, &time_breakdown);
  char fmt_buf[128];
  strftime(fmt_buf, sizeof(fmt_buf), "%F %T", &time_breakdown);
  struct iovec parts[3];
  parts[0].iov_len = asprintf((char**) &parts[0].iov_base,
                              "%s[%s.%06ld]%s ",
                              markup_start, fmt_buf, now.tv_usec,
                              markup_end_);
  parts[1].iov_len = vasprintf((char**) &parts[1].iov_base, format, ap);
  parts[2].iov_base = (void*) "\n";
  parts[2].iov_len = 1;
  int already_newline
    = (parts[1].iov_len > 0 &&
       ((const char*)parts[1].iov_base)[parts[1].iov_len-1] == '\n');
  if (writev(fd, parts, already_newline ? 2 : 3) < 0) {
    // Logging trouble. Ignore.
  }

  free(parts[0].iov_base);
  free(parts[1].iov_base);
}

void Log_debug(const char *format, ...) {
  if (log_fd < 0) return;
  va_list ap;
  va_start(ap, format);
  Log_internal(log_fd, debug_markup_start_, format, ap);
  va_end(ap);
}

void Log_info(const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  if (log_fd < 0) {
    vsyslog(LOG_INFO, format, ap);
  } else {
    Log_internal(log_fd, info_markup_start_, format, ap);
  }
  va_end(ap);
}

void Log_error(const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  if (log_fd < 0) {
    vsyslog(LOG_ERR, format, ap);
  } else {
    Log_internal(log_fd, error_markup_start_, format, ap);
  }
  va_end(ap);
}
