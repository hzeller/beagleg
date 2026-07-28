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

#include <cstdint>
#include <string>
#include <vector>

#if __cplusplus >= 201703L
#include <string_view>
#else
#include <cstring>
#include <ostream>
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
  static const size_t npos = -1;

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
  const char &front() const { return data_[0]; }
  const char *data() const { return data_; }
  size_t length() const { return len_; }
  size_t size() const { return len_; }
  bool empty() const { return len_ == 0; }

  iterator begin() const { return data_; }
  iterator end() const { return data_ + len_; }

  size_t find_first_of(char c) const {
    const char *found = strchr(data_, c);
    return found ? found - data_ : npos;
  }

  // Not optimized. Just make work.
  size_t find(string_view s, size_t pos = 0) const {
    const size_t self_len = this->size();
    const size_t s_len = s.size();

    if (pos > self_len) return npos;
    if (s_len == 0) return pos;
    if (s_len > self_len - pos) return npos;

    using traits_type = std::char_traits<char>;

    // single char..
    const char *data = this->data();
    if (s_len == 1) {
      const char *ptr = traits_type::find(data + pos, self_len - pos, s[0]);
      return ptr ? static_cast<size_t>(ptr - data) : npos;
    }

    // multi-char
    const char first_char = s[0];
    const size_t max_search_pos = self_len - s_len;

    for (size_t i = pos; i <= max_search_pos; ++i) {
      // Find candidate match for the first character
      const char *candidate =
        traits_type::find(data + i, max_search_pos - i + 1, first_char);
      if (!candidate) return npos;  // not even first char found.

      i = static_cast<size_t>(candidate - data);

      // Check if the remainder of the substring matches
      if (traits_type::compare(candidate + 1, s.data() + 1, s_len - 1) == 0) {
        return i;
      }
    }

    return npos;
  }

  void remove_prefix(size_t n) {
    n = n < len_ ? n : len_;
    data_ += n;
    len_ -= n;
  }

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

// Parse a number from a beagleg::string_view into "result". Returns the
// end of the number on success, nullptr otherwise.
// So this can be used in a simple boolean context for simple success
// testing, but as well in parsing context where advancing to the next position
// is needed.
const char *convert_strto32(beagleg::string_view s, int32_t *result);
const char *convert_strto64(beagleg::string_view s, int64_t *result);
const char *convert_strtof(beagleg::string_view s, float *result);
const char *convert_strtod(beagleg::string_view s, double *result);

// Possible convenience functions
// bool consume_strtof(beagleg::string_view *s, float *result);

// Parse integer. On success, return parsed number, fallback otherwise.
int64_t ParseInt64(beagleg::string_view s, int64_t fallback);

#undef PRINTF_FMT_CHECK
#endif  // _BEAGLEG_STRING_UTIL_H
