/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2015 Henner Zeller <h.zeller@acm.org>
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
#ifndef _BEAGLEG_STRING_UTIL_H
#define _BEAGLEG_STRING_UTIL_H

#include <assert.h>
#include <stddef.h>
#include <string.h>

#include <ostream>
#include <string>
#include <vector>

#if __cplusplus >= 201703L
#include <string_view>
#endif

// Define this with empty, if you're not using gcc.
#define PRINTF_FMT_CHECK(fmt_pos, args_pos) \
  __attribute__((format(printf, fmt_pos, args_pos)))

namespace beagleg {
#if __cplusplus >= 201703L
using string_view = ::std::string_view;
#else
// Our version of c++17 std::string_view in case c++17 is not available yet.
// It essentially points at a chunk of data of a particular
// length. Pointer + length.
// Allows to have keep cheap substrings of strings without copy while still
// have a type-safe, length-aware piece of string.
class string_view {
 public:
  typedef const char *iterator;

  string_view() : data_(NULL), len_(0) {}
  string_view(const char *data, size_t len) : data_(data), len_(len) {}

  // We want implicit conversions from these types
  string_view(const std::string &s)  // NOLINT
      : data_(s.data()), len_(s.length()) {}
  string_view(const char *str)  // NOLINT
      : data_(str), len_(strlen(str)) {}

  string_view substr(size_t pos, size_t len) const {
    assert(pos + len <= len_);
    return string_view(data_ + pos, len);
  }
  string_view substr(size_t pos) const { return substr(pos, length() - pos); }

  bool operator==(const string_view &other) const {
    if (len_ != other.len_) return false;
    if (data_ == other.data_) return true;
    return strncmp(data_, other.data_, len_) == 0;
  }

  char operator[](size_t pos) const { return data_[pos]; }
  const char *data() const { return data_; }
  size_t length() const { return len_; }
  bool empty() const { return len_ == 0; }

  iterator begin() const { return data_; }
  iterator end() const { return data_ + len_; }

 private:
  const char *data_;
  size_t len_;
};

inline std::ostream &operator<<(std::ostream &o, string_view s) {
  return o.write(s.data(), s.length());
}
#endif
}  // namespace beagleg

// Trim beagleg::string_view of whitespace font and back and returned trimmed
// string.
beagleg::string_view TrimWhitespace(beagleg::string_view s);

// Lowercase the string (simple ASCII) and return as newly allocated
// std::string
std::string ToLower(beagleg::string_view in);

// Test if given beagleg::string_view is prefix of the other.
bool HasPrefix(beagleg::string_view s, beagleg::string_view prefix);

// Formatted printing into a string.
std::string StringPrintf(const char *format, ...) PRINTF_FMT_CHECK(1, 2);

// Split a string at any of the given separator characters.
std::vector<beagleg::string_view> SplitString(beagleg::string_view s,
                                              beagleg::string_view separators);

// Parse a decimal from a beagleg::string_view.
int64_t ParseDecimal(beagleg::string_view s, int64_t fallback);

#undef PRINTF_FMT_CHECK
#endif  // _BEAGLEG_STRING_UTIL_H
