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
#ifndef _BEAGLEG_CONFIG_PARSER_H
#define _BEAGLEG_CONFIG_PARSER_H

#include <string>
#include <vector>

#include "common/string-util.h"

// The config parser reads a configuration file and passes tokenized
// values to a ConfigParser::Reader.
class ConfigParser {
public:
  // A reader has to be implemented by a subsystem that needs configuration
  // from the file.
  class Reader {
  public:
    virtual ~Reader() {}

    // Inform about new section. If the Reader is interested in
    // name/value pairs seen in that section, it should return 'true'.
    virtual bool SeenSection(int line_no, const std::string &section_name) = 0;

    // SeenNameValue() is only called if this Reader expressed interest
    // in the current section. Returns 'true' if it could deal with the
    // name/value, 'false' if there was an error and the configuration should
    // be deemed invalid.
    virtual bool SeenNameValue(int line_no,
                               const std::string &name,
                               const std::string &value) = 0;

    virtual void ReportError(int line_no, const std::string &msg);

  protected:
    // Convenience functions that can be used in derived readers.
    // All the Accept() functions are done in the way that they always return
    // 'true' if the expected name is not matched, otherwise they return the
    // outcome of parsing the value. That way, they can be chained with &&
    static bool ParseString(const std::string &value, std::string *result);
    static bool ParseInt(const std::string &value, int *result);
    static bool ParseBool(const std::string &value, bool *result);
    static bool ParseFloatExpr(const std::string &value, float *result);
  };

  // Create a config parser.
  ConfigParser();

  // Set content of configuration by reading from the file. Returns 'true'
  // if reading the file was successful.
  // (Note, this will read in the whole configuration file in memory, but
  // they are by their very nature smallish in size).
  // Overwrites any previous content.
  bool SetContentFromFile(const char *filename);

  // Set content of configuration file as one string. Typically useful in
  // unit tests.
  // Overwrites any previous content.
  void SetContent(StringPiece content);

  // Emit configuration values to the Reader for all sections it is interested
  // in.
  //
  // This method can be called many times with different Readers. The
  // ConfigParser is handed to various subsystems of the machine-config for
  // each of them to configure itself. This allows the ConfigParser to focus on
  // tokenization and the configured subsections to ingest name/values.
  //
  // The parser informes the reader about new sections it has seen
  // (calling SeenSection()) and asks if the reader is interested in
  // name/value pairs in that section. If so, it provides the reader with these
  // calling SeenNameValue().
  //
  // Returns 'true' if configuration file could be parsed (no syntax errors,
  // and all calls to SeenNameValue() returned true).
  //
  // Reader-ownership is not taken over.
  bool EmitConfigValues(Reader *reader);

private:
  std::string content_;
  bool parse_success_;
};
#endif // _BEAGLEG_CONFIG_PARSER_H
