/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
* (c) 2018 Leonardo Romor <leonardo.romor@gmail.com>
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
#ifndef FD_GCODE_STREAMER_H_
#define FD_GCODE_STREAMER_H_

#include "common/fd-mux.h"
#include "common/linebuf-reader.h"
#include "gcode-parser/gcode-parser.h"

class GCodeStreamer {
public:
  // GCodeStreamer needs to outlive FDMultiplexer.
  GCodeStreamer(FDMultiplexer *event_server, GCodeParser *parser,
                GCodeParser::EventReceiver *parse_events);

  // Reads GCode lines from "fd" and feeds them to the GCodeParser.
  // Error messages are sent to "err_stream" if non-NULL.
  // Reads until EOF.
  // The input file descriptor is closed.
  bool ConnectStream(int fd, FILE *msg_stream);

  // Returns true if we are already connected to a stream.
  bool IsStreaming() { return connection_fd_ >= 0; }

private:
  void CloseStream();

  FDMultiplexer *const event_server_;
  GCodeParser *const parser_;
  GCodeParser::EventReceiver *const parse_events_;

  LinebufReader reader_;
  bool is_processing_;

  FILE *msg_stream_;
  int connection_fd_;
  int lines_processed_;

  bool ReadData();
  bool Timeout();
};

#endif // FD_TASK_H_
