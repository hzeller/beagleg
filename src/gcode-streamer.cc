
#include <sys/socket.h>
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
  event_server_->RunOnTimeout([this](){
    return Timeout();
  });
}

bool GCodeStreamer::ConnectStream(int fd, FILE *msg_stream) {

  bool ret = true;

  if (is_streaming_) {
    ret = false;
    goto exit;
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

  // Cleanup any potentially remaining gcode from previous connections.
  reader_.Flush();

exit:
  if (!ret) {
    CloseStream();
  }
  return ret;
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
    // Let's test if the connection is still active
    char buffer;
    if (recv(connection_fd_, &buffer,
             sizeof(buffer), MSG_PEEK | MSG_DONTWAIT) == 0) {
      Log_info("Client disconnected.");
    } else {
      Log_info("Reached EOF.");
    }

    // always call gcode_finished() to disable motors at end of stream
    parse_events_->gcode_finished(true);
    CloseStream();
    is_processing_ = false;
    is_streaming_ = false;
    return false;
  }

  // Let's first check that it has at least one line.
  const char *line = reader_.ReadLine();
  if (line == NULL)
    return true;

  is_processing_ = true;
  do {
    // NOTE:(important)
    // This should return true or false in case the line was movimentation or not
    // and only if is, reset the timer.
    parser_->ParseLine(line, msg_stream_);
    line = reader_.ReadLine();
  } while (line != NULL);

  // Loop again
  return true;
}

// No more line data within x seconds
bool GCodeStreamer::Timeout() {
  parse_events_->input_idle(is_processing_);
  is_processing_ = false;
  return true;
}
