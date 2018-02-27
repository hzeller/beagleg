
#include <sys/timerfd.h>
#include <sys/socket.h>

#include "common/fd-mux.h"
#include "common/logging.h"
#include "gcode-parser/gcode-parser.h"

#include "gcode-streamer.h"

GCodeStreamer::GCodeStreamer(FDMultiplexer *event_server, GCodeParser *parser,
                             GCodeParser::EventReceiver *parse_events, float timeout_ms)
  : event_server_(event_server), parser_(parser), parse_events_(parse_events),
    reader_(), is_streaming_(false), connection_fd_(-1), timer_fd_(-1) {
  timeout_ = {0};
  timeout_.it_value.tv_nsec = timeout_ms * 1e6;
}

bool GCodeStreamer::ParseStream(int fd, FILE *msg_stream) {

  bool ret = true;
  struct itimerspec disarm_value = {0};

  if (is_streaming_) {
    ret = false;
    goto exit;
  }

  msg_stream_ = msg_stream;
  if (msg_stream_) {
    // Output needs to be unbuffered, otherwise they'll never make it.
    setvbuf(msg_stream_, NULL, _IONBF, 0);
  }

  // Create and disarm the timer
  timer_fd_ = timerfd_create(CLOCK_REALTIME, 0);
  if (timer_fd_ < 0) {
    perror("timerfd_create()");
    ret = false;
    goto exit;
  }

  if (timerfd_settime(timer_fd_, 0, &disarm_value, NULL) < 0) {
    perror("timerfd_settime()");
    ret = false;
    goto exit;
  }

  connection_fd_ = fd;

  event_server_->RunOnReadable(connection_fd_, [this](){
    return ReadData();
  });

  event_server_->RunOnReadable(timer_fd_, [this](){
    return Timeout();
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

  // always call gcode_finished() to disable motors at end of stream
  parse_events_->gcode_finished(true);

  event_server_->ScheduleDelete(connection_fd_);
  event_server_->ScheduleDelete(timer_fd_);

  close(connection_fd_);
  close(timer_fd_);

  is_streaming_ = false;
}

// New data to be fed into the linebuffer
int GCodeStreamer::ReadData() {

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
    CloseStream();
    return 0;
  }

  // Let's first check that it has at least one line.
  const char *line = reader_.ReadLine();
  if (line == NULL)
    return 1; // Return without resetting the timer.

  do {
    // NOTE:(important)
    // This should return true or false in case the line was movimentation or not
    // and only if is, reset the timer.
    parser_->ParseLine(line, msg_stream_);
    line = reader_.ReadLine();
  } while (line != NULL);

  // No more lines, arm the idle timer
  if (timerfd_settime(timer_fd_, 0, &timeout_, NULL) == -1) {
    perror("timerfd_settime()");
    return -1;
  }

  // Loop again
  return 1;
}

// No more line data within x seconds
int GCodeStreamer::Timeout() {
  read(timer_fd_, NULL, sizeof(uint64_t));

  struct itimerspec timer_value;
  timerfd_gettime(timer_fd_, &timer_value);

  // Timer was expired, but in the same cycle that we parsed new data.
  if (timer_value.it_value.tv_nsec != 0 &&
      timer_value.it_value.tv_sec != 0)
    return 1;

  parse_events_->input_idle(true);
  return 1;
}
