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

#include "config-parser.h"

#include <assert.h>
#include <ctype.h>

#include <algorithm>
#include <fstream>
#include <iostream>

#include "common/logging.h"
#include "common/string-util.h"

// Parse a super-simple multiplicative expression.
static double ParseDoubleExpression(const char *input, double fallback,
                                    char **end) {
  const char *full_expr = input;
  double value = strtod(input, end);
  if (*end == input) return fallback;
  for (;;) {
    while (isspace(**end)) ++*end;
    const char op = **end;
    if (op != '/' && op != '*') {
      return value;  // done. Not an operation.
    }
    ++*end;
    while (isspace(**end)) ++*end;
    input = *end;
    double operand;
    if (*input == '(') {
      operand = ParseDoubleExpression(input + 1, 1.0, end);
      if (**end != ')') {
        fprintf(stderr, "Mismatching parenthesis in '%s'\n", full_expr);
        return fallback;
      } else {
        ++*end;
      }
    } else {
      operand = strtod(input, end);
    }
    if (*end == input) return fallback;
    if (op == '/')
      value /= operand;
    else if (op == '*')
      value *= operand;
  }
  return value;
}

bool ConfigParser::Reader::ParseString(const std::string &value,
                                       std::string *result) {
  *result = value;
  return true;
}

bool ConfigParser::Reader::ParseInt(const std::string &value, int *result) {
  char *end;
  *result = strtol(value.c_str(), &end, 10);
  return *end == '\0';
}

bool ConfigParser::Reader::ParseBool(const std::string &value, bool *result) {
  if (value == "1" || value == "yes" || value == "true") {
    *result = true;
    return true;
  }
  if (value == "0" || value == "no" || value == "false") {
    *result = false;
    return true;
  }
  return false;
}

bool ConfigParser::Reader::ParseFloatExpr(const std::string &value,
                                          float *result) {
  char *end;
  double eval = ParseDoubleExpression(value.c_str(), 1.0, &end);
  if (end == NULL || *end == '\0') {
    *result = eval;
    if (*result < 0) {
      Log_error("Expected positive value.");
      return false;
    }
    return true;
  }
  return false;
}

void ConfigParser::Reader::ReportError(int line_no, const std::string &msg) {
  Log_error("Line %d: %s", line_no, msg.c_str());
}

bool ConfigParser::SetContentFromFile(const char *filename) {
  if (!filename) return false;
  std::ifstream file_stream(filename, std::ios::binary);
  content_.assign(std::istreambuf_iterator<char>(file_stream),
                  std::istreambuf_iterator<char>());
  return file_stream.good();
}

void ConfigParser::SetContent(beagleg::string_view content) {
  content_.assign(content.begin(), content.end());
}

// Extract next line out of source; returns line, modifies "source"
// to point to next
// Modifies source.
static beagleg::string_view NextLine(beagleg::string_view *source) {
  beagleg::string_view result;
  if (source->length() == 0) return result;
  const beagleg::string_view::iterator start = source->begin();
  beagleg::string_view::iterator endline = start;
  for (/**/; endline != source->end(); ++endline) {
    // Whatever newline or comment comes first terminates our resulting line...
    if (!result.data() &&
        (*endline == '#' || *endline == '\r' || *endline == '\n')) {
      result = beagleg::string_view(start, endline - start);
    }
    if (*endline == '\n') {  // ... but we wait until \n to reposition source
      *source = beagleg::string_view(endline + 1,
                                     source->length() - (endline - start) - 1);
      return result;
    }
  }
  // Encountered last line without final newline.
  result = beagleg::string_view(start, endline - start);
  *source = beagleg::string_view(source->end(), 0);
  return result;
}

static std::string CanonicalizeName(const beagleg::string_view s) {
  return ToLower(TrimWhitespace(s));
}

bool ConfigParser::EmitConfigValues(Reader *reader) const {
  bool success = true;
  bool current_section_interested = false;
  std::string current_section;
  int line_no = 0;
  beagleg::string_view content_data(content_.data(), content_.length());
  beagleg::string_view line = NextLine(&content_data);
  for (/**/; line.data() != NULL; line = NextLine(&content_data)) {
    ++line_no;
    line = TrimWhitespace(line);
    if (line.empty()) continue;

    // Sections start with '['
    if (line[0] == '[') {
      if (line[line.length() - 1] != ']') {
        reader->ReportError(line_no, "Section line does not end in ']'");
        success = false;
        current_section_interested = false;  // rest is probably bogus.
        continue;
      }

      const beagleg::string_view section = line.substr(1, line.length() - 2);
      current_section = CanonicalizeName(section);
      current_section_interested =
        reader->SeenSection(line_no, current_section);
    } else {
      beagleg::string_view::iterator eq_pos =
        std::find(line.begin(), line.end(), '=');
      if (eq_pos == line.end()) {
        reader->ReportError(line_no, "name=value pair expected.");
        success = false;
        continue;
      }
      if (current_section_interested) {
        const std::string name = CanonicalizeName(
          beagleg::string_view(line.begin(), eq_pos - line.begin()));
        const beagleg::string_view value_piece = TrimWhitespace(
          beagleg::string_view(eq_pos + 1, line.end() - eq_pos - 1));
        std::string value(value_piece.begin(), value_piece.end());
        bool could_parse = reader->SeenNameValue(line_no, name, value);
        if (!could_parse) {
          reader->ReportError(
            line_no,
            StringPrintf("In section [%s]: Couldn't handle '%s = %s'",
                         current_section.c_str(), name.c_str(), value.c_str()));
        }
        success &= could_parse;
      }
    }
  }
  return success;
}
