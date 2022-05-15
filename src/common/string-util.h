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
#include <string_view>
#include <vector>

// Define this with empty, if you're not using gcc.
#define PRINTF_FMT_CHECK(fmt_pos, args_pos) \
  __attribute__((format(printf, fmt_pos, args_pos)))

// Trim std::string_view of whitespace font and back and returned trimmed
// string.
std::string_view TrimWhitespace(std::string_view s);

// Lowercase the string (simple ASCII) and return as newly allocated
// std::string
std::string ToLower(std::string_view in);

// Test if given std::string_view is prefix of the other.
bool HasPrefix(std::string_view s, std::string_view prefix);

// Formatted printing into a string.
std::string StringPrintf(const char *format, ...) PRINTF_FMT_CHECK(1, 2);

// Split a string at any of the given separator characters.
std::vector<std::string_view> SplitString(std::string_view s,
                                          std::string_view separators);

// Parse a number from a std::string_view into "result". Returns the
// end of the number on success, nullptr otherwise.
// So this can be used in a simple boolean context for simple success
// testing, but as well in parsing context where advancing to the next position
// is needed.
const char *convert_strto32(std::string_view s, int32_t *result);
const char *convert_strto64(std::string_view s, int64_t *result);
const char *convert_strtof(std::string_view s, float *result);
const char *convert_strtod(std::string_view s, double *result);

// Possible convenience functions
// bool consume_strtof(std::string_view *s, float *result);

// Parse integer. On success, return parsed number, fallback otherwise.
int64_t ParseInt64(std::string_view s, int64_t fallback);

#undef PRINTF_FMT_CHECK
#endif  // _BEAGLEG_STRING_UTIL_H
