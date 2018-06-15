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

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#include <algorithm>

#include "logging.h"

static volatile sig_atomic_t caught_signal = 0;

static void receive_signal(int signo) {
  static const char msg[] = "Caught signal. Shutting down ASAP.\n";
  if (!caught_signal) {
    write(STDERR_FILENO, msg, sizeof(msg));
  }
  caught_signal = 1;
}

static void arm_signal_handler() {
  caught_signal = 0;
  struct sigaction sa = {};
  sa.sa_handler = receive_signal;

  // We might get multiple signals on shutdown, so not using SA_RESETHAND
  sa.sa_flags = 0;
  sigaction(SIGTERM, &sa, NULL);  // Regular kill
  sigaction(SIGINT, &sa, NULL);   // Ctrl-C

  // Other, internal problems that should never happen, but
  // can trigger multiple times before we gain back control
  // to shut down as cleanly as possible.
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
  return read_handlers_.insert({ fd, handler }).second;
}

bool FDMultiplexer::RunOnWritable(int fd, const Handler &handler) {
  return write_handlers_.insert({ fd, handler }).second;
}

void FDMultiplexer::RunOnIdle(const Handler &handler) {
  idle_handlers_.push_back(handler);
}

void FDMultiplexer::CallHandlers(fd_set *to_call_fd_set, int *available_fds,
                                 HandlerMap *handlers) {
  for (auto it = handlers->begin(); *available_fds && it != handlers->end(); ) {
    bool keep_handler = true;
    if (FD_ISSET(it->first, to_call_fd_set)) {
      --*available_fds;
      keep_handler = it->second();
    }
    it = keep_handler ? std::next(it) : handlers->erase(it);
  }
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
  for (const auto &it : read_handlers_) {
    maxfd = std::max(maxfd, it.first);
    FD_SET(it.first, &read_fds);
  }

  // Writers
  for (const auto &it : write_handlers_) {
    maxfd = std::max(maxfd, it.first);
    FD_SET(it.first, &write_fds);
  }

  if (maxfd < 0) {
    // file descriptors only can be registred from within handlers
    // or before running the Loop(). So if no filedesctiptors are left,
    // there is no chance for any to re-appear, so we can exit.
    Log_info("Exiting loop() after last file descriptor is gone.");
    return false;
  }

  int fds_ready = select(maxfd + 1, &read_fds, &write_fds, nullptr, &timeout);
  if (fds_ready < 0) {
    if (!caught_signal)
      perror("select() failed");
    return false;
  }

  if (fds_ready == 0) {             // No FDs ready: timeout situation.
    for (auto it = idle_handlers_.begin(); it != idle_handlers_.end(); /**/) {
      const bool keep_handler = (*it)();
      it = keep_handler ? std::next(it) : idle_handlers_.erase(it);
    }
    return true;
  }

  CallHandlers(&read_fds, &fds_ready, &read_handlers_);
  CallHandlers(&write_fds, &fds_ready, &write_handlers_);

  return true;
}

int FDMultiplexer::Loop() {
  const unsigned timeout = idle_ms_;

  arm_signal_handler();
  while (SingleCycle(timeout) && !caught_signal) {
    /**/
  }
  disarm_signal_handler();

  return caught_signal ? 1 : 0;
}
