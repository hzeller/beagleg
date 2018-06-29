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
#include "motion-queue.h"

#include <stdio.h>
#include <string.h>

#include <memory>
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "common/fd-mux.h"
#include "common/logging.h"
#include "gcode-parser/gcode-parser.h"
#include "gcode-streamer.h"

class MockStream {
public:

  MockStream() : fd_{-1, -1} {
    if (pipe(fd_) < 0) {
      perror("pipe()");
      exit(1);
    }
  }
  ~MockStream() {}

  int GetReceiverFiledescriptor() { return fd_[RECEIVING_FD]; }
  int SendData(const char *data) {
    // Feed data in the pipe
    return write(fd_[SENDING_FD], data, strlen(data));
  }

  void CloseSender() { close(fd_[SENDING_FD]); }

private:
  enum { RECEIVING_FD, SENDING_FD };
  int fd_[2];
};

class StreamTester : public GCodeParser::EventReceiver {
public:
  StreamTester()
    : parser_(new GCodeParser(GCodeParser::Config(), this)),
      streamer_(new GCodeStreamer(&event_server_, parser_.get(), this)),
      stream_mock_(NULL) {}

  bool OpenStream() {
    assert(stream_mock_ == NULL);
    stream_mock_ = new MockStream();
    return streamer_->ConnectStream(stream_mock_->GetReceiverFiledescriptor(), NULL);
  }

  void CloseStream() {
    stream_mock_->CloseSender();
    delete stream_mock_;
    stream_mock_ = NULL;
  }

  void SendString(const char *line) {
    stream_mock_->SendData(line);
  }

  void Cycle() {
    event_server_.SingleCycle(0);
  }

  MOCK_METHOD1(gcode_start, void(GCodeParser *parser));
  MOCK_METHOD1(gcode_finished, void(bool end_of_stream));
  MOCK_METHOD1(input_idle, void(bool is_first));
  MOCK_METHOD2(coordinated_move,
               bool(float feed_mm_p_sec, const AxesRegister &absolute_pos));

  void go_home(AxisBitmap_t axis_bitmap) override {}
  void set_speed_factor(float factor) override {}
  void set_fanspeed(float value) override {}
  void set_temperature(float degrees_c) override {}
  void wait_temperature() override {}
  void dwell(float time_ms) override {}
  void motors_enable(bool enable) override {}
  bool rapid_move(float feed_mm_p_sec,
                  const AxesRegister &axes) override { return true;}
  const char *unprocessed(char letter, float value,
                          const char *rest_of_line) override { return NULL; }

private:
  class MockFDMultiplexer : public FDMultiplexer {
  public:
    friend class StreamTester;
  };

  MockFDMultiplexer event_server_;
  std::unique_ptr<GCodeParser> parser_;
  std::unique_ptr<GCodeStreamer> streamer_;
  MockStream *stream_mock_;
};

using namespace ::testing;

// The new line is the "send" character. If we disconnect and the linebuffer
// has not completed the last line, we should parse it anyway.
TEST(Streaming, two_consecutive_connections) {
  StreamTester tester;

  // First connection
  EXPECT_CALL(tester, gcode_start(_)).Times(1);
  EXPECT_CALL(tester, coordinated_move(FloatEq(1000.0 / 60), _)).Times(1);
  EXPECT_CALL(tester, gcode_finished(_)).Times(1);
  tester.OpenStream();
  tester.SendString("G1X200F1000"); // Write incomplete move command
  tester.CloseStream();
  tester.Cycle(); // Loop to read
  tester.Cycle(); // Loop to close

  // Second connection
  EXPECT_CALL(tester, gcode_start(_)).Times(0);
  EXPECT_CALL(tester, coordinated_move(FloatEq(1000.0 / 60), _)).Times(1);
  EXPECT_CALL(tester, gcode_finished(_)).Times(1);
  tester.OpenStream(); // Re open the stream
  tester.SendString("G1X200F1000\n"); // Receive a complete string
  tester.CloseStream();
  tester.Cycle(); // Loop to read
  tester.Cycle(); // Loop to close
}

// Generic check that during stream, all the necessary callbacks are called in
// the correct order.
TEST(Streaming, basic_stream) {
  StreamTester tester;
  {
    InSequence s;
    EXPECT_CALL(tester, gcode_start(_)).Times(1);
    EXPECT_CALL(tester, coordinated_move(FloatEq(1000.0 / 60), _)).Times(2);
    EXPECT_CALL(tester, input_idle(_)).Times(0);
    EXPECT_CALL(tester, gcode_finished(_)).Times(0);
  }
  tester.OpenStream();
  tester.SendString("G1X200F1000\nG1X200F1000\n");
  tester.Cycle(); // First loop reads the lines

  EXPECT_CALL(tester, coordinated_move(FloatEq(1000.0 / 60), _)).Times(1);
  EXPECT_CALL(tester, input_idle(_)).Times(0);
  tester.SendString("G1X200F1000\n");
  tester.Cycle(); // Loop and read another line

  EXPECT_CALL(tester, input_idle(true)).Times(1);
  tester.SendString("G1X200F1000");   // Send incomplete line
  tester.Cycle(); // First loop reads, but since it's not a line,
                  // we are not parsing anything
  tester.Cycle(); // Timeout, input idle

  EXPECT_CALL(tester, input_idle(false)).Times(1);
  tester.Cycle(); // Second idle timeout

  EXPECT_CALL(tester, coordinated_move(FloatEq(1000.0 / 60), _)).Times(1);
  EXPECT_CALL(tester, gcode_finished(_)).Times(1);
  tester.CloseStream();
  tester.Cycle(); // Wait the stream to close
}

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
