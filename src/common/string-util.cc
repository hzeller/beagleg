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

#include "string-util.h"

#include <algorithm>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

StringPiece TrimWhitespace(const StringPiece &s) {
  StringPiece::iterator start = s.begin();
  while (start < s.end() && isspace(*start))
    start++;
  StringPiece::iterator end = s.end() - 1;
  while (end > start && isspace(*end))
    end--;
  return StringPiece(start, end + 1 - start);
}

std::string ToLower(const StringPiece &in) {
  std::string result(in.length(), ' ');
  std::transform(in.begin(), in.end(), result.begin(), ::tolower);
  return result;
}

bool HasPrefix(const StringPiece &s, const StringPiece &prefix) {
  if (s.length() < prefix.length()) return false;
  return strncmp(s.data(), prefix.data(), prefix.length()) == 0;
}

static inline bool contains(const StringPiece &str, char c) {
  for (char i : str) {
    if (i == c) return true;
  }
  return false;
}

std::vector<StringPiece> SplitString(const StringPiece &s,
                                     const StringPiece &separators) {
  std::vector<StringPiece> result;
  StringPiece::iterator i = s.begin();
  StringPiece::iterator start = i;
  for (/**/; i != s.end(); ++i) {
    if (contains(separators, *i)) {
      result.push_back(StringPiece(start, i - start));
      start = i + 1;
    }
  }
  result.push_back(StringPiece(start, i - start));
  return result;
}

long ParseDecimal(const StringPiece &s, long fallback) {
  std::string as_str = s.ToString();  // we need safe c-str()
  char *endptr = NULL;
  long result = strtol(as_str.c_str(), &endptr, 10);
  if (*endptr != '\0') return fallback;
  return result;
}

static void vAppendf(std::string *str, const char *format, va_list ap) {
  const size_t orig_len = str->length();
  const size_t space = 1024;   // there should be better ways to do this...
  str->resize(orig_len + space);
  int written = vsnprintf((char*)str->data() + orig_len, space, format, ap);
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
