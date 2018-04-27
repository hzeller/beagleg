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
#include <vector>

// This needs a better name.
class FDMultiplexer {
public:
  FDMultiplexer(unsigned timeout_ms = 50) : timeout_ms_(timeout_ms) {}

  // Handlers for events from this multiplexer. Returns true if we want to continue
  // to be called in the future or false if we wish to be taken out of the multiplexer.
  typedef std::function<bool()> Handler;

  // These can only be set before Loop() is called or from a
  // running handler itself.
  // Returns false if that filedescriptor is already registered.
  bool RunOnReadable(int fd, const Handler &handler);
  bool RunOnWritable(int fd, const Handler &handler);

  // Handlers run in case there's nothing to do.
  void RunOnIdle(const Handler &handler);

  // Run the main loop. Blocks while there is still a filedescriptor
  // registered (return 0) or until a signal is triggered (return 1).
  int Loop();

protected:
  // Run a single cycle resulting in exactly one call of a handler function.
  // This means either the next file descriptor became ready and its Handler is called or
  // we encountered a timeout and the idle-Handler has been called.
  // This is broken out to make it simple to test steps in unit tests.
  bool SingleCycle(unsigned timeout_ms);

private:
  const unsigned timeout_ms_;
  std::map<int, Handler> read_handlers_;
  std::map<int, Handler> write_handlers_;
  std::vector<Handler> idle_handlers_;
  std::vector<int> to_delete_read_;
  std::vector<int> to_delete_write_;
};

#endif // FD_MUX_H_
