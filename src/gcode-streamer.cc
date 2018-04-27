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
#include <fcntl.h>

#include "common/fd-mux.h"
#include "common/logging.h"
#include "gcode-parser/gcode-parser.h"

#include "gcode-streamer.h"

GCodeStreamer::GCodeStreamer(FDMultiplexer *event_server, GCodeParser *parser,
                             GCodeParser::EventReceiver *parse_events)
  : event_server_(event_server), parser_(parser), parse_events_(parse_events),
    reader_(), is_streaming_(false), is_processing_(false), connection_fd_(-1) {
  // Let's start the input idle tasklet
  event_server_->RunOnIdle([this](){
    return Timeout();
  });
}

bool GCodeStreamer::ConnectStream(int fd, FILE *msg_stream) {
  if (is_streaming_) {
    CloseStream();
    return false;
  }
  is_streaming_ = true;

  msg_stream_ = msg_stream;
  if (msg_stream_) {
    // Output needs to be unbuffered, otherwise they'll never make it.
    setvbuf(msg_stream_, NULL, _IONBF, 0);
  }
  connection_fd_ = fd;

  event_server_->RunOnReadable(connection_fd_, [this](){
    return ReadData();
  });
  return true;
}

void GCodeStreamer::CloseStream() {
  if (msg_stream_) {
    fflush(msg_stream_);
  }
  close(connection_fd_);
}

// New data to be fed into the linebuffer
bool GCodeStreamer::ReadData() {
  // Update buffer
  if (reader_.Update(connection_fd_) == 0) {
    Log_info("Reached EOF.");

    // Parse any potentially remaining gcode from previous connections.
    const char *line = reader_.IncompleteLine();
    if (line) {
      parser_->ParseLine(line, msg_stream_);
    }

    // always call gcode_finished() to disable motors at end of stream
    parse_events_->gcode_finished(true);
    CloseStream();
    is_processing_ = false;
    is_streaming_ = false;
    return false;
  }

  is_processing_ = true;
  const char *line;
  while ((line = reader_.ReadLine())) {
    // NOTE:(important)
    // This should return true or false in case the line was movement or not
    // and only if is, reset the timer.
    parser_->ParseLine(line, msg_stream_);
  }

  // Loop again
  return true;
}

// No more line data within x seconds
bool GCodeStreamer::Timeout() {
  parse_events_->input_idle(is_processing_);
  is_processing_ = false;
  return true;
}
