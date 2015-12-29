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
