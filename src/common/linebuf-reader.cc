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
#include "common/linebuf-reader.h"

#include "common/string-util.h"

LinebufReader::LinebufReader(size_t buffer_size)
    : len_(buffer_size),
      buffer_start_(new char[len_]),
      buffer_end_(buffer_start_ + len_),
      content_start_(buffer_start_),
      content_end_(buffer_start_),
      cr_seen_(false) {}
LinebufReader::~LinebufReader() { delete[] buffer_start_; }

ssize_t LinebufReader::Update(const ReadFun &read_fun) {
  if (content_start_ - buffer_start_ > (int)(len_ / 2)) {
    const size_t copy_len = size();
    memmove(buffer_start_, content_start_, copy_len);
    content_start_ = buffer_start_;
    content_end_ = buffer_start_ + copy_len;
  }
  const ssize_t r = read_fun(content_end_, buffer_end_ - content_end_);
  // TODO(hzeller): if we get zero, we should consider this as end-of-stream
  // and potentially regard the buffer as 'complete' even if it doesn't have a
  // full line yet.
  if (r >= 0) content_end_ += r;  // so, what if r < 0 ?
  return r;
}

const char *LinebufReader::IncompleteLine() {
  *content_end_ = '\n';
  content_end_++;
  return ReadAndConsumeLine();
}

const char *LinebufReader::ReadAndConsumeLine() {
  for (char *i = content_start_; i < content_end_; ++i) {
    if (cr_seen_ && *i == '\n') {
      cr_seen_ = false;
      content_start_ = i + 1;
      continue;
    }
    if (*i == '\r' || *i == '\n') {
      cr_seen_ = (*i == '\r');
      *i = '\0';
      const char *line = content_start_;
      content_start_ = i + 1;
      return line;
    }
  }
  return nullptr;
}
