/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Test for pru motion queue.
 *
 * We simulate the pru hardware and test if the absolute position is correctly
 * retrieved.
 *
 */
#include "motion-queue.h"

#include <stdio.h>
#include <string.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "common/fd-mux.h"
#include "common/logging.h"
#include "gcode-parser/gcode-parser.h"
#include "gcode-streamer.h"

class MockStream {
public:

  MockStream() {}
  ~MockStream() { ClientDisconnect(); }

  int ClientConnect() {
    ClientDisconnect();
    if (pipe(fd_) < 0) {
      perror("pipe()");
      exit(1);
    }
    return fd_[0];
  }

  int NewData(const char *data) {
    // Feed data in the pipe
    return write(fd_[1], data, strlen(data));
  }

  void ClientDisconnect() { close(fd_[1]); }

private:
  int fd_[2];
};

class StreamTester : public GCodeParser::EventReceiver {
public:
  StreamTester() {
    GCodeParser::Config config;
    parser_ = new GCodeParser(config, this, false);
    streamer_ = new GCodeStreamer(&event_server_, parser_, this);
    stream_mock_ = NULL;
  }

  ~StreamTester() {
    delete parser_;
    delete streamer_;
    delete stream_mock_;
  }

  bool OpenStream() {
    assert(stream_mock_ == NULL);
    stream_mock_ = new MockStream();
    return streamer_->ConnectStream(stream_mock_->ClientConnect(), NULL);
  }

  void CloseStream() {
    stream_mock_->ClientDisconnect();
    stream_mock_ = NULL;
  }
  void ReceiveString(const char *line) {
    stream_mock_->NewData(line);
  }
  void Cycle() {
    event_server_.Cycle(0);
  }

  MOCK_METHOD1(gcode_start, void(GCodeParser *parser));
  MOCK_METHOD1(gcode_finished, void(bool end_of_stream));
  MOCK_METHOD1(input_idle, void(bool is_first));
  MOCK_METHOD2(coordinated_move,
               bool(float feed_mm_p_sec, const AxesRegister &absolute_pos));

  virtual void go_home(AxisBitmap_t axis_bitmap) {}
  virtual void set_speed_factor(float factor) {}
  virtual void set_fanspeed(float value) {}
  virtual void set_temperature(float degrees_c) {}
  virtual void wait_temperature() {}
  virtual void dwell(float time_ms) {}
  virtual void motors_enable(bool enable) {}
  virtual bool rapid_move(float feed_mm_p_sec,
                          const AxesRegister &axes) { return true;}
  virtual const char *unprocessed(char letter, float value,
                                  const char *rest_of_line) { return NULL; }

private:

  class MockFDMultiplexer : public FDMultiplexer {
  public:
    friend class StreamTester;
  };

  MockFDMultiplexer event_server_;
  GCodeParser *parser_;
  GCodeStreamer *streamer_;
  MockStream *stream_mock_;
};

using namespace ::testing;

// The new line is the "send" character. If we disconnect and the linebuffer
// has not completed the last line, we should drop it
// NOTE: Broken
TEST(Streaming, no_newline_no_parsing) {
  StreamTester tester;

  // Connect
  // Write incomplete move command
  // Close the stream
  {
    EXPECT_CALL(tester, gcode_start(_)).Times(0);
    EXPECT_CALL(tester, coordinated_move(_, _)).Times(0);
  }
  tester.OpenStream();
  tester.ReceiveString("G1X200F1000");
  tester.CloseStream();
  tester.Cycle();

  {
    EXPECT_CALL(tester, gcode_finished(_)).Times(1);
  }

  tester.Cycle();
  {
    EXPECT_CALL(tester, gcode_start(_)).Times(1);
    EXPECT_CALL(tester, coordinated_move(FloatEq(1000.0 / 60), _)).Times(1);
  }
  tester.OpenStream();
  tester.ReceiveString("G1X200F1000\n");
  tester.Cycle();
  tester.CloseStream();
}

TEST(Streaming, big_line) {
  // Stream a line bigger than the LinebufReader buffer.
  // Loop
  // Assert that the line is truncated.
}

// Generic check that during stream, all the necessary callbacks are called in
// the correct order.
TEST(Streaming, basic_stream) {
  StreamTester tester;

  // Connect
  // Write one line
  // Loop, without reaching the idle timeout
  // ASSERT that the idle timeout has not been called
  // ASSERT ParseLine has been called two times
  {
    InSequence s;
    EXPECT_CALL(tester, gcode_start(_)).Times(1);
    EXPECT_CALL(tester, coordinated_move(FloatEq(1000.0 / 60), _)).Times(2);
    EXPECT_CALL(tester, input_idle(_)).Times(0);
    EXPECT_CALL(tester, gcode_finished(_)).Times(0);
  }
  tester.OpenStream();
  tester.ReceiveString("G1X200F1000\nG1X200F1000\n");
  tester.Cycle();

  // One cycle, no input idle
  {
    EXPECT_CALL(tester, coordinated_move(FloatEq(1000.0 / 60), _)).Times(1);
    EXPECT_CALL(tester, input_idle(_)).Times(0);
  }
  tester.ReceiveString("G1X200F1000\n");
  tester.Cycle();

  // One cycle without new gcode lines
  {
    EXPECT_CALL(tester, input_idle(true)).Times(1);
  }
  tester.ReceiveString("G1X200F1000");
  tester.Cycle();
  tester.Cycle();

  // Second idle timeout
  {
    EXPECT_CALL(tester, input_idle(false)).Times(1);
  }
  tester.Cycle();

  // Close the stream
  // ASSERT gcode_finished is called
  {
    EXPECT_CALL(tester, gcode_finished(_)).Times(1);
  }
  tester.CloseStream();
  tester.Cycle();
}

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
