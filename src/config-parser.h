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

#include <map>
#include <string>
#include <vector>

class ConfigParser {
public:
  class EventReceiver {
  public:
    virtual ~EventReceiver() {}

    // Inform about new section. If the EventReceiver is interested int
    // name/value pairs seen in that section, it should return 'true'.
    virtual bool SeenSection(int line_no, const std::string &section_name) = 0;

    // SeenNameValue() is only called if this EventReceiver expressed interest
    // in the current section. Returns 'true' if it could deal with the
    // name/value, 'false' if there was an error and the configuration should
    // be deemed invalid.
    virtual bool SeenNameValue(int line_no,
                               const std::string &name,
                               const std::string &value) = 0;

    virtual void ReportError(int line_no, const std::string &msg);
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
  void SetContent(const std::string &content);

  // Emit configuration values to the EventReceiver. Returns 'true' if
  // configuration file could be parsed (no syntax errors, and all calls to
  // SeenNameValue() returned true).
  // EventReceiver is not taken over.
  bool EmitConfigValues(EventReceiver *event_receiver);

  // Get a list of sections that have not been claimed yet. Depending on the
  // situation, that might indicate typos in the configuration file.
  // This requires to have EmitConfigValues() at least once.
  std::vector<std::string> GetUnclaimedSections() const;

private:
  std::string content_;
  bool parse_success_;
  std::map<std::string, bool> claimed_sections_;
};
#endif // _BEAGLEG_CONFIG_PARSER_H
