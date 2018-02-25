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

  // Handler for connections. Returns '1' if we want to continue reading
  // or '0' if we wish to be taken out of the multiplexer.
  // '-1' Can be used to exit the multiplexer with an error, for example
  // if one of our tasklets has an error.
  typedef std::function<int()> Handler;

  // These can only be set before Loop() is called or from a
  // running handler itself.
  // Returns false if that filedescriptor is already registered.
  bool RunOnReadable(int fd, const Handler &handler);
  bool RunOnWritable(int fd, const Handler &handler);
  void ScheduleDelete(int fd);

  // Run the main loop. Blocks while there is still a filedescriptor
  // registered (return 0) or until a signal is triggered (return 2).
  int Loop();

private:
  std::map<int, Handler> r_handlers_;
  std::map<int, Handler> w_handlers_;
  std::vector<int> to_delete_r_;
  std::vector<int> to_delete_w_;
};

#endif // FD_MUX_H_
