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
#ifndef _BEAGLEG_LINEBUF_READER_H
#define _BEAGLEG_LINEBUF_READER_H

#include <stddef.h>
#include <unistd.h>

#include <functional>

// Tokenizes a non-contiguous sequence of incoming data into lines.
//
// A buffer that can be updated with new bytes and then allows for extracting
// lines separated by newlines up to that point.
//
// This is to be used in conjunction with file descriptor event management
// such as select() or poll() which yield a random amount of bytes from a
// file-descriptor.
//
// Usage:
// void NewDataReady() {
//   reader.Update(/* with new data, see method signatures */);
//   // Get all the lines we have available now
//   while ((line = reader.ReadAndConsumeLine()) {
//      // do something with line
//   }
// }
class LinebufReader {
 public:
  // A function to read from some data source. Similar to read(2), it gets
  // a buffer and a maximums size it can write to and returns the number
  // of bytes it has actually written.
  // Returns a positive number for read bytes, zero if we are end-of-stream
  // and a negative number to indicate error.
  typedef std::function<ssize_t(char *buf, size_t size)> ReadFun;

  // The "buffer_size" determines the longest line we expect at maximum.
  // TODO(hzeller): right now, we don't gracefully deal with overlong lines.
  explicit LinebufReader(size_t buffer_size = 16384);
  ~LinebufReader();

  // Update content. It will be calling the ReadFun exactly once and updates
  // its internal buffer.
  // After this, you may call ReadLine() to extract as many lines as had
  // been waiting.
  // If you made sure that there is data available before calling Update(),
  // this will not block.
  // Returns the value returned by ReadFun.
  ssize_t Update(ReadFun read_fun);

  // The tpical way this will be called: with a file descriptor that has some
  // bytes ready.
  ssize_t Update(int fd) {
    return Update([fd](char *buf, size_t len) { return read(fd, buf, len); });
  }

  // Return a current line if it is available. The line is a nul terminated
  // c-string without the newline character(s).
  //
  // If there is no current line pending, or it is incomplete, returns NULL.
  // It is a good idea to call this after a call to Update() in a loop until
  // you reach NULL to empty the buffer before the next Update() comes in.
  //
  // Returned data is only valid until next call to Update() or
  // ReadAndConsumeLine()..
  const char *ReadAndConsumeLine();

  // Get the current incomplete line. This is useful to receive the final
  // data when finishing in case of a missing newline.
  // TODO(hzeller): maybe a function to get the remaining buffer when we
  // are closing the connection ? There might be some incomplete line in there.
  const char *IncompleteLine();

  void Flush() {
    content_start_ = buffer_start_;
    content_end_ = buffer_start_;
  }

  // Currently stored in buffer.
  size_t size() const { return content_end_ - content_start_; }

 private:
  const size_t len_;
  char *const buffer_start_;
  const char *const buffer_end_;
  char *content_start_;
  char *content_end_;

  bool cr_seen_;
};

#endif  // _BEAGLEG_LINEBUF_READER_H
