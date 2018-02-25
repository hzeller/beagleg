// -*- mode: c++; c-basic-offset: 4; indent-tabs-mode: nil; -*-

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

void FDMultiplexer::ScheduleDelete(int fd) {
  if (r_handlers_.find(fd) != r_handlers_.end()) {
    to_delete_r_.push_back(fd);
  } else if (w_handlers_.find(fd) != w_handlers_.end()) {
    to_delete_w_.push_back(fd);
  }
}

int FDMultiplexer::Loop() {
  fd_set read_fds;
  fd_set write_fds;
  struct timeval timeout;

  arm_signal_handler();
  while (!caught_signal) {
    int maxfd = -1;
    FD_ZERO(&read_fds);
    FD_ZERO(&write_fds);
    timeout.tv_sec = 0;
    timeout.tv_usec = 5e4;

    // Readers
    for (const auto &it : r_handlers_) {
      if (it.first >= maxfd) maxfd = it.first+1;
      FD_SET(it.first, &read_fds);
    }

    // Writers
    for (const auto &it : w_handlers_) {
      if (it.first >= maxfd) maxfd = it.first+1;
      FD_SET(it.first, &write_fds);
    }

    if (maxfd < 0) {
      // file descriptors only can be registred from within handlers
      // or before running the Loop(). If no filedesctiptors are left,
      // there is no chance for any to re-appear, so we can exit.
      fprintf(stderr, "No filedescriptor registered. Exiting loop()");
      break;
    }

    int fds_ready = select(maxfd, &read_fds, &write_fds, nullptr, &timeout);
    if (fds_ready < 0) {
      if (!caught_signal)
        perror("select() failed");
      break;
    }

    if (fds_ready == 0) {
      // Timeout situation. We are not registering timeout handlers
      // currently.
      continue;
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
  }
  disarm_signal_handler();

  if (caught_signal)
    return 2;
  return 0;
}
