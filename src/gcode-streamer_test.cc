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

#include "common/logging.h"

class StreamMock {
public:
  StreamMock() {
    if (pipe(fd_) < 0) {
      perror("pipe()");
      exit(EXIT_FAILURE);
    }
  }

  int GetFd() { return fd_[0]; }

  int NewData(const char *data) {
    // Feed data in the pipe
    return write(fd_[1], data, strlen(data));
  }

  void ClientDisconnect() { close(fd_[0]); close(fd_[1]); }

private:
  int fd_[2];
};


// The new line is the "send" character. If we disconnect and the linebuffer
// has not completed the last line, we should drop it
TEST(Streaming, no_newline_no_parsing) {
  // Write some lines and fill the remaining buffer of LinebufReader  of G1 X100 F1000
  // Close the stream
  // Open a new stream and write M65 P1
  // ASSERT the new gcode is not G1 X100 F1000 M65 P1
}

// TODO: Still need to be implemented the code that should correctly answer to
// this test
TEST(Streaming, big_line) {
  // Stream a line bigger than the LinebufReader buffer.
  // Loop
  // Assert that the line is truncated.
}

// Check that during stream, all the necessary callbacks are called in
// the correct order.
TEST(Streaming, basic_stream) {
  // Write one line
  // Loop, without reaching the idle timeout
  // ASSERT that the idle timeout has not been called
  // ASSERT ParseLine has been called two times
  // Write another line before the idle timeout.
  // Loop
  // ASSERT ParseLine has been called once
  // Let the idle timeout pass
  // ASSERT input_idle has been called
  // Close the stream
  // ASSERT gcode_finished is called
}

// Let's test that if both the timer and the connection fd are ready
// if the connection fd callback is called first, the timer should not trigger.
TEST(Streaming, timer_parse_order) {
  // Execute the ReadData task
  // Execute the Timeout task
  // ASSERT input_idle has not been called
}

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
