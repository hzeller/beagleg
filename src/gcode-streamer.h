
#ifndef FD_GCODE_STREAMER_H_
#define FD_GCODE_STREAMER_H_

#include "gcode-parser/gcode-parser.h"
#include "common/linebuf-reader.h"

class GCodeStreamer {

public:
  GCodeStreamer(FDMultiplexer *event_server, GCodeParser *parser,
                GCodeParser::EventReceiver *parse_events, float timeout_ms = 10);

  bool ParseStream(int fd, FILE *msg_stream);

  bool IsStreaming() { return is_streaming_; }
  ~GCodeStreamer() {}

  void CloseStream();

private:


  FDMultiplexer *event_server_;
  GCodeParser *parser_;
  GCodeParser::EventReceiver *parse_events_;
  LinebufReader reader_;
  bool is_streaming_;
  struct itimerspec timeout_;

  FILE *msg_stream_;
  int connection_fd_;
  int timer_fd_;

  int ReadData();
  int Timeout();
};

#endif // FD_TASK_H_
