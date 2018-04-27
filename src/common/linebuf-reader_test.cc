/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2016 Henner Zeller <h.zeller@acm.org>
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

#include "linebuf-reader.h"

#include <string>
#include <memory>
#include <gtest/gtest.h>

// Raw lines that we use as samples.
constexpr int kSampleLineCount = 4;
const char *kSampleLines[kSampleLineCount + 1] = {
  "This is a short line",
  "This is another, longer line",
  "",   // Emtpy line
  "",
  NULL
};

class InputStreamSimulator {
public:
  InputStreamSimulator(size_t chunk_size, const char *endline)
    : chunk_size_(chunk_size), longest_line_len_(0), read_pos_(0) {
    for (const char *line : kSampleLines) {
      if (line == NULL) break;
      buffer_.append(line).append(endline);
      if (strlen(line) > longest_line_len_)
        longest_line_len_ = strlen(line);
    }
  }

  // Read from this inputStream Simulator.
  // This has the signature required by LinebufReader::ReadFun.
  ssize_t Read(char *buf, size_t size) {
    size_t read_size = std::min(size, chunk_size_);
    read_size = std::min(read_size, buffer_.length() - read_pos_);
    memcpy(buf, buffer_.data() + read_pos_, read_size);
    read_pos_ += read_size;
    return read_size;
  }

  size_t longest_line_len() { return longest_line_len_; }
  int remaining() { return buffer_.length() - read_pos_; }

private:
  const size_t chunk_size_;
  size_t longest_line_len_;
  std::string buffer_;
  size_t read_pos_;
};

// Parametrize tests with different end line characters
class LinebufReaderTest : public ::testing::TestWithParam<const char*> {
};

TEST_P(LinebufReaderTest, LargeChunkReading) {
  LinebufReader reader;
  InputStreamSimulator input(10000, GetParam());  // reading large chunks of data.
  // Here, we expect that everything essentially shows up with the first
  // read. Also we assume that everything fits into the reader buffer.

  // TODO(hzeller): there must be a better way to pass a member function call
  // through std::functional than wrapping it in a lambda
  reader.Update([&input](char *buf, size_t size) {
      return input.Read(buf, size);
    });
  EXPECT_EQ(0, input.remaining());
  EXPECT_EQ(std::string(kSampleLines[0]), reader.ReadLine());
  EXPECT_EQ(std::string(kSampleLines[1]), reader.ReadLine());
  EXPECT_EQ(std::string(kSampleLines[2]), reader.ReadLine());
  EXPECT_EQ(std::string(kSampleLines[3]), reader.ReadLine());
  EXPECT_EQ(NULL, reader.ReadLine());
}

TEST_P(LinebufReaderTest, SmallChunkReading) {
  LinebufReader reader;
  InputStreamSimulator input(1, GetParam());  // One byte at a time.
  int expected_next_sample = 0;

  while (input.remaining() > 0) {
    // TODO(hzeller): there must be a better way to pass a member function call
    // through std::functional than wrapping it in a lambda
    reader.Update([&input](char *buf, size_t size) {
        return input.Read(buf, size);
      });
    const char *potential_line = reader.ReadLine();
    if (potential_line != NULL) {
      fprintf(stderr, "Got line: '%s'\n", potential_line);
      EXPECT_EQ(std::string(kSampleLines[expected_next_sample]),
                potential_line);
      expected_next_sample += 1;
    } else {
      fprintf(stderr, ".");  // Indicating that we have nothing yet.
    }
  }
  EXPECT_EQ(kSampleLineCount, expected_next_sample);
}

TEST_P(LinebufReaderTest, TightBufferReading) {
  InputStreamSimulator input(1, GetParam());  // One byte at a time.
  // We allocate a reader whose entire buffer just fits a line. This should
  // still work.
  LinebufReader reader(input.longest_line_len() + strlen(GetParam()));
  int expected_next_sample = 0;

  while (input.remaining() > 0) {
    // TODO(hzeller): there must be a better way to pass a member function call
    // through std::functional than wrapping it in a lambda
    reader.Update([&input](char *buf, size_t size) {
        return input.Read(buf, size);
      });
    const char *potential_line = reader.ReadLine();
    if (potential_line != NULL) {
      EXPECT_EQ(std::string(kSampleLines[expected_next_sample]),
                potential_line);
      expected_next_sample += 1;
    }
  }
  EXPECT_EQ(kSampleLineCount, expected_next_sample);
}

INSTANTIATE_TEST_CASE_P(PortableEndLineTests,
                        LinebufReaderTest,
                        ::testing::Values("\n", "\r", "\r\n"));

// TODO(hzeller): more testing
//   - Implementation of a more graceful handling if our buffer is too small
//     to hold a full line.
int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
