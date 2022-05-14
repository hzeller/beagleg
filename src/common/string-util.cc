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

#include "common/string-util.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <algorithm>
#include <charconv>

std::string_view TrimWhitespace(std::string_view s) {
  std::string_view::iterator start = s.begin();
  while (start < s.end() && isspace(*start)) start++;
  std::string_view::iterator end = s.end() - 1;
  while (end > start && isspace(*end)) end--;
  return std::string_view(start, end + 1 - start);
}

std::string ToLower(std::string_view in) {
  std::string result(in.length(), ' ');
  std::transform(in.begin(), in.end(), result.begin(), ::tolower);
  return result;
}

bool HasPrefix(std::string_view s, std::string_view prefix) {
  if (s.length() < prefix.length()) return false;
  return strncmp(s.data(), prefix.data(), prefix.length()) == 0;
}

static inline bool contains(std::string_view str, char c) {
  return str.find_first_of(c) != std::string_view::npos;
}

std::vector<std::string_view> SplitString(std::string_view s,
                                          std::string_view separators) {
  std::vector<std::string_view> result;
  std::string_view::iterator i = s.begin();
  std::string_view::iterator start = i;
  for (/**/; i != s.end(); ++i) {
    if (contains(separators, *i)) {
      result.emplace_back(start, i - start);
      start = i + 1;
    }
  }
  result.emplace_back(start, i - start);
  return result;
}

bool SafeParseDecimal(std::string_view s, int64_t *result) {
  while (s.length() && isspace(s.front())) s.remove_prefix(1);
  if (s.length() && s.front() == '+') s.remove_prefix(1);
  auto success = std::from_chars(s.begin(), s.end(), *result, 10);
  return success.ec == std::errc();
}

// Parse decimal and return on success or return fallback value otherwise.
int64_t ParseDecimal(std::string_view s, int64_t fallback) {
  int64_t result;
  return SafeParseDecimal(s, &result) ? result : fallback;
}

static void vAppendf(std::string *str, const char *format, va_list ap) {
  const size_t orig_len = str->length();
  const size_t space = 1024;  // there should be better ways to do this...
  str->resize(orig_len + space);
  int written = vsnprintf((char *)str->data() + orig_len, space, format, ap);
  str->resize(orig_len + written);
}

std::string StringPrintf(const char *format, ...) {
  va_list ap;
  va_start(ap, format);
  std::string result;
  vAppendf(&result, format, ap);
  va_end(ap);
  return result;
}
