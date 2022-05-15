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
#include <type_traits>

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

template <typename value_type>
static const char* convert_strto_num(std::string_view s, value_type *result) {
  while (!s.empty() && isspace(s.front())) s.remove_prefix(1);
  if (!s.empty() && s.front() == '+') s.remove_prefix(1);
  auto success = std::from_chars(s.data(), s.data() + s.size(), *result);
  return (success.ec == std::errc()) ? success.ptr : nullptr;
}
const char *convert_strto32(std::string_view s, int32_t *result) {
  return convert_strto_num<int32_t>(s, result);
}
const char *convert_strto64(std::string_view s, int64_t *result) {
  return convert_strto_num<int64_t>(s, result);
}

/*
 * So, unfortunately not all c++17 implemenetations actually provide
 * a std::from_chars() for floating point numbers; available only >= g++-11
 * for instance.
 *
 * So here we use the c++ detection pattern to determine if that function
 * is available, otherwise fall back to slower best-effort implementation that
 * copies it to a nul-terminated buffer, then calls the old strtof(), strotd()
 * functions.
 */
template <typename T, typename = void>
struct from_chars_available : std::false_type {};
template <typename T>
struct from_chars_available<
  T, std::void_t<decltype(std::from_chars(
       std::declval<const char *>(), std::declval<const char *>(),
       std::declval<T &>()))>> : std::true_type {};

template <typename T>
inline constexpr bool from_chars_available_v = from_chars_available<T>::value;

// Copy everything that looks like a number into output iterator.
static void CopyNumberTo(const char *in_begin, const char *in_end,
                         char* out_begin, const char *out_end) {
  const char *src = in_begin;
  char *dst = out_begin;
  const char *extra_allowed = "+-.";
  bool have_point = false;
  out_end -= 1; // Allow space for 0 termination.
  while (src < in_end
         && (isdigit(*src) || index(extra_allowed, *src))
         && dst < out_end) {
    // The sign is only allowed in the first character of the buffer
    if ((*src == '+' || *src == '-') && dst != out_begin) break;
    // only allow one decimal point
    if (*src == '.') {
      if (have_point)
        break;
      else
        have_point = true;
    }
    *dst++ = *src++;
  }
  *dst = '\0';
}

template <typename T, T (*strto_fallback_fun)(const char *, char **)>
static const char *convert_strto_ieee(std::string_view s, T *result) {
  if constexpr (from_chars_available_v<T>) {
    return convert_strto_num<T>(s, result);
  }

  // Fallback in case std::from_chars() does not exist for this type. Here,
  // we just call the corresponding C-function, but first have to copy
  // the number to a local buffer, as that one requires \0-termination.
  char buffer[32];

  // Need to skip whitespace first to not use up our buffer for that.
  std::string_view n = s;
  while (!n.empty() && isspace(n.front())) n.remove_prefix(1);

  CopyNumberTo(n.data(), n.data() + n.size(), buffer, buffer + sizeof(buffer));
  char *endptr = nullptr;
  *result = strto_fallback_fun(buffer, &endptr);
  if (endptr == buffer) return nullptr;  // Error.

  // Now, convert our offset back relateive to the original string.
  return n.data() + (endptr - buffer);
}

const char *convert_strtof(std::string_view s, float *result) {
  return convert_strto_ieee<float, strtof>(s, result);
}

const char*convert_strtod(std::string_view s, double *result) {
  return convert_strto_ieee<double, strtod>(s, result);
}

// Parse decimal and return on success or return fallback value otherwise.
int64_t ParseInt64(std::string_view s, int64_t fallback) {
  int64_t result;
  return convert_strto64(s, &result) ? result : fallback;
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
