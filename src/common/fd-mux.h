/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
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

#ifndef FD_MUX_H_
#define FD_MUX_H_

#include <map>
#include <functional>
#include <list>

#include <sys/select.h>

// This needs a better name.
class FDMultiplexer {
public:
  FDMultiplexer(unsigned idle_ms = 50) : idle_ms_(idle_ms) {}

  // Handlers for events from this multiplexer.
  // Returns true if we want to continue to be called in the future or false
  // if we wish to be taken out of the multiplexer.
  typedef std::function<bool()> Handler;

  // These can only be set before Loop() is called or from a
  // running handler itself.
  // Returns false if that filedescriptor is already registered.
  bool RunOnReadable(int fd, const Handler &handler);
  bool RunOnWritable(int fd, const Handler &handler);

  // Handler called regularly every idle_ms in case there's nothing to do.
  void RunOnIdle(const Handler &handler);

  // Run the main loop. Blocks while there is still a filedescriptor
  // registered (return 0) or until a signal is triggered (return 1).
  int Loop();

protected:
  // Run a single cycle resulting in exactly one call of a handler function.
  // This means that one of these happened:
  //   (1) The next file descriptor became ready and its Handler is called
  //   (2) We encountered a timeout and the idle-Handler has been called.
  //   (3) Signal received or select() issue. Returns false in this case.
  //
  // This is broken out to make it simple to test steps in unit tests.
  bool SingleCycle(unsigned timeout_ms);

private:
  typedef std::map<int, Handler> HandlerMap;

  // Call all the handlers available in fd_set, removing the ones that
  // return 'false'. "available_fds" is an optimizatin that tells CallHandlers
  // how many fds there are in the first place.
  void CallHandlers(fd_set *to_call_fd_set, int *available_fds,
                    HandlerMap *handlers);

  const unsigned idle_ms_;
  HandlerMap read_handlers_;
  HandlerMap write_handlers_;
  std::list<Handler> idle_handlers_;
};

#endif // FD_MUX_H_
