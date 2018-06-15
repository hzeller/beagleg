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
#include "gcode-streamer.h"

#include <fcntl.h>

#include "common/logging.h"

GCodeStreamer::GCodeStreamer(FDMultiplexer *event_server, GCodeParser *parser,
                             GCodeParser::EventReceiver *parse_events)
  : event_server_(event_server), parser_(parser), parse_events_(parse_events),
    is_processing_(false), connection_fd_(-1), lines_processed_(0) {
  // Let's start the input idle tasklet
  // TODO: the lifetime implications are a bit problematic as we need to
  // outlive the Loop() of the event server.
  event_server_->RunOnIdle([this](){
    return Timeout();
  });
}

bool GCodeStreamer::ConnectStream(int fd, FILE *msg_stream) {
  if (connection_fd_ >= 0) {
    return false;  // Alrady connected.
  }

  // Stuff written to msg-stream needs to be unbuffered, otherwise
  // they'll not make it in timely response to the input.
  if (msg_stream) setvbuf(msg_stream, NULL, _IONBF, 0);

  msg_stream_ = msg_stream;
  connection_fd_ = fd;
  lines_processed_ = 0;

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
  connection_fd_ = -1;
  Log_info("Processed %d GCode blocks.", lines_processed_);
}

// New data to be fed into the linebuffer
bool GCodeStreamer::ReadData() {
  // Update buffer
  if (reader_.Update(connection_fd_) == 0) {
    Log_info("Reached EOF.");

    // Parse any potentially remaining gcode from previous connections.
    const char *line = reader_.IncompleteLine();
    if (line) {
      parser_->ParseBlock(line, msg_stream_);
    }

    // always call gcode_finished() to disable motors at end of stream
    parse_events_->gcode_finished(true);
    CloseStream();
    is_processing_ = false;
    return false;  // We're done processing, remove us from fd-mux
  }

  is_processing_ = true;
  const char *line;
  while ((line = reader_.ReadLine())) {
    // NOTE:(important)
    // This should return true or false in case the line was movement or not
    // and only if is, reset the timer.
    parser_->ParseBlock(line, msg_stream_);
    ++lines_processed_;
  }

  // Loop again
  return true;
}

// We didn't receive a line within x milliseconds.
bool GCodeStreamer::Timeout() {
  parse_events_->input_idle(is_processing_);
  is_processing_ = false;
  return true;
}
