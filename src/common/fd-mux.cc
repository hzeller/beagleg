/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2018 Henner Zeller <h.zeller@acm.org>
 *          Leonardo Romor <leonardo.romor@gmail.com>
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

#include "fd-mux.h"
#include "logging.h"

#include <unistd.h>
#include <signal.h>
#include <vector>
#include <algorithm>
#include <sys/select.h>
#include <errno.h>
#include <string.h>

static volatile sig_atomic_t caught_signal = 0;

static void receive_signal(int signo) {
  static char msg[] = "Caught signal. Shutting down ASAP.\n";
  if (!caught_signal) {
    write(STDERR_FILENO, msg, sizeof(msg));
  }
  caught_signal = 1;
}

static void arm_signal_handler() {
  caught_signal = 0;
  struct sigaction sa = {};
  sa.sa_handler = receive_signal;
  sa.sa_flags = SA_RESETHAND;  // oneshot, no restart
  sigaction(SIGTERM, &sa, NULL);  // Regular kill
  sigaction(SIGINT, &sa, NULL);   // Ctrl-C

  // Other, internal problems that should never happen, but
  // can trigger multiple times before we gain back control
  // to shut down as cleanly as possible. These are not one-shot.
  sa.sa_flags = 0;
  sigaction(SIGSEGV, &sa, NULL);
  sigaction(SIGBUS, &sa, NULL);
  sigaction(SIGFPE, &sa, NULL);
}

static void disarm_signal_handler() {
  signal(SIGTERM, SIG_DFL);  // Regular kill
  signal(SIGINT, SIG_DFL);   // Ctrl-C
  signal(SIGSEGV, SIG_DFL);
  signal(SIGBUS, SIG_DFL);
  signal(SIGFPE, SIG_DFL);
}

bool FDMultiplexer::RunOnReadable(int fd, const Handler &handler) {
  return r_handlers_.insert({ fd, handler }).second;
}

bool FDMultiplexer::RunOnWritable(int fd, const Handler &handler) {
  return w_handlers_.insert({ fd, handler }).second;
}

void FDMultiplexer::RunOnIdle(const Handler &handler) {
  t_handlers_.push_back(handler);
}

bool FDMultiplexer::SingleCycle(unsigned int timeout_ms) {
  fd_set read_fds;
  fd_set write_fds;

  struct timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;

  int maxfd = -1;
  FD_ZERO(&read_fds);
  FD_ZERO(&write_fds);

  // Readers
  for (const auto &it : r_handlers_) {
    if (it.first >= maxfd) maxfd = it.first + 1;
    FD_SET(it.first, &read_fds);
  }

  // Writers
  for (const auto &it : w_handlers_) {
    if (it.first >= maxfd) maxfd = it.first + 1;
    FD_SET(it.first, &write_fds);
  }

  if (maxfd < 0) {
    // file descriptors only can be registred from within handlers
    // or before running the Loop(). If no filedesctiptors are left,
    // there is no chance for any to re-appear, so we can exit.
    fprintf(stderr, "No filedescriptor registered. Exiting loop()");
    return false;
  }

  int fds_ready = select(maxfd, &read_fds, &write_fds, nullptr, &timeout);
  if (fds_ready < 0) {
    if (!caught_signal)
      perror("select() failed");
    return false;
  }

  if (fds_ready == 0) {
    // Timeout situation.
    for (int i = t_handlers_.size() - 1; i >= 0; --i) {
      if (!t_handlers_[i]()) {
        t_handlers_[i] = t_handlers_.back();
        t_handlers_.pop_back();
      }
    }
    return true;
  }

  // Handle reads
  for (const auto &it : r_handlers_) {
    if (FD_ISSET(it.first, &read_fds)) {
      const bool retrigger = it.second();
      if (!retrigger)
        to_delete_r_.push_back(it.first);
      if (--fds_ready == 0)
        break;
    }
  }
  for (int i : to_delete_r_) {
    r_handlers_.erase(i);
  }

  // Handle writes
  for (const auto &it : w_handlers_) {
    if (FD_ISSET(it.first, &write_fds)) {
      const bool retrigger = it.second();
      if (!retrigger)
        to_delete_w_.push_back(it.first);
      if (--fds_ready == 0)
        break;
    }
  }
  for (int i : to_delete_w_) {
    w_handlers_.erase(i);
  }

  to_delete_r_.clear();
  to_delete_w_.clear();

  return true;
}

int FDMultiplexer::Loop() {
  const unsigned timeout = timeout_ms_;
  arm_signal_handler();
  while (SingleCycle(timeout)) {}
  disarm_signal_handler();

  if (caught_signal)
    return 1;
  return 0;
}
