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

#include <algorithm>
#include <fstream>
#include <iostream>

#include <assert.h>
#include <ctype.h>

#include "logging.h"

namespace {
// A StringPiece essentially points at a chunk of data of a particular
// length. Pointer + length.
// It allows to have keep cheap substrings of strings without copy.
// TOOD(hzeller): if needed somewhere else, put in separate header.
class StringPiece {
public:
  typedef const char* iterator;
  StringPiece() : data_(NULL), len_(0) {}
  StringPiece(const char *data, size_t len)
    : data_(data), len_(len) {}

  StringPiece substr(size_t pos, size_t len) const {
    assert(pos + len <= len_);
    return StringPiece(data_ + pos, len);
  }

  void assign(const char *data, size_t len) {
    data_ = data;
    len_ = len;
  }

  std::string ToString() const {
    return std::string(data_, len_);
  }

  const char operator[](size_t pos) const { return data_[pos]; }
  const char *data() const { return data_; }
  size_t length() const { return len_; }
  bool empty() const { return len_ == 0; }

  iterator begin() const { return data_; }
  iterator end() const { return data_ + len_; }

private:
  const char *data_;
  size_t len_;
};

void TrimWhitespace(StringPiece *s) {
  StringPiece::iterator start = s->begin();
  while (start < s->end() && isspace(*start))
    start++;
  StringPiece::iterator end = s->end() - 1;
  while (end > start && isspace(*end))
    end--;
  s->assign(start, end + 1 - start);
}

} // namespace

void ConfigParser::EventReceiver::ReportError(int line_no,
                                              const std::string &msg) {
  std::cerr << line_no << ":" << msg << std::endl;
}

ConfigParser::ConfigParser() : parse_success_(true) {}

bool ConfigParser::SetContentFromFile(const char *filename) {
  parse_success_ = true;
  std::ifstream file_stream(filename, std::ios::binary);
  content_.assign(std::istreambuf_iterator<char>(file_stream),
                  std::istreambuf_iterator<char>());
  return file_stream.good();
}

void ConfigParser::SetContent(const std::string &content) {
  parse_success_ = true;
  content_ = content;
}

// Extract next line out of source. Takes 
// Modifies source.
static StringPiece NextLine(StringPiece *source) {
  StringPiece result;
  if (source->length() == 0)
    return result;
  const StringPiece::iterator start = source->begin();
  StringPiece::iterator endline = start;
  for (/**/; endline != source->end(); ++endline) {
    // Whatever comes first terminates our line.
    if (!result.data() &&
        (*endline == '#' || *endline == '\r' || *endline == '\n')) {
      result.assign(start, endline - start);
    }
    if (*endline == '\n') {
      source->assign(endline + 1, source->length() - (endline - start) - 1);
      return result;
    }
  }
  // Encountered last line without final newline.
  result.assign(start, endline - start);
  source->assign(source->end(), 0);
  return result;
}

bool ConfigParser::EmitConfigValues(EventReceiver *event_receiver) {
  // The first pass collects all the parse errors and emits them. Later on,
  // we refuse to run another time.
  if (!parse_success_)
    return false;
  bool success = true;
  bool current_section_interested = false;
  int line_no = 0;
  StringPiece content_data(content_.data(), content_.length());
  StringPiece line = NextLine(&content_data);
  for (/**/; success && line.data() != NULL; line = NextLine(&content_data)) {
    ++line_no;
    TrimWhitespace(&line);
    if (line.empty())
      continue;

    // Sections start with '['
    if (line[0] == '[') {
      if (line[line.length() - 1] != ']') {
        event_receiver->ReportError(line_no, "Section line does not end in ']'");
        parse_success_ = false;
        current_section_interested = false;  // rest is probably bogus.
        continue;
      }

      StringPiece section = line.substr(1, line.length() - 2);
      TrimWhitespace(&section);
      current_section_interested = event_receiver
        ->SeenSection(line_no, section.ToString());
    }
    else {
      StringPiece::iterator eq_pos = std::find(line.begin(), line.end(), '=');
      if (eq_pos == line.end()) {
        event_receiver->ReportError(line_no, "name=value pair expected.");
        parse_success_ = false;
        continue;
      }
      if (current_section_interested) {
        StringPiece name(line.begin(), eq_pos - line.begin());
        TrimWhitespace(&name);
        StringPiece value(eq_pos + 1, line.end() - eq_pos - 1);
        TrimWhitespace(&value);
        success &= event_receiver->SeenNameValue(line_no,
                                                 name.ToString(),
                                                 value.ToString());
      }
    }
  }
  return parse_success_ && success;
}

std::vector<std::string> ConfigParser::GetUnclaimedSections() const {
  std::vector<std::string> result;
  return result;
}
