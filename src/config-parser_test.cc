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

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <iostream>

using ::testing::Return;

namespace {
class MockConfigReader : public ConfigParser::Reader {
 public:
  MOCK_METHOD2(SeenSection, bool(int line_no, const std::string &section_name));
  MOCK_METHOD3(SeenNameValue, bool(int line_no, const std::string &name,
                                   const std::string &value));
  MOCK_METHOD2(ReportError, void(int line_no, const std::string &msg));
};

}  // namespace

TEST(ConfigParserTest, BasicParser) {
  ConfigParser p;
  p.SetContent(
    " [ SOME-section ]\n"
    " # just comment\n"
    "\n\n"
    " some line = bar  # foo\n"
    "another line = x\r\n"
    "  something = unfinished-line");
  MockConfigReader events;
  EXPECT_CALL(events, SeenSection(1, "some-section")).WillOnce(Return(true));
  EXPECT_CALL(events, SeenNameValue(5, "some line", "bar"))
    .WillOnce(Return(true));
  EXPECT_CALL(events, SeenNameValue(6, "another line", "x"))
    .WillOnce(Return(true));
  EXPECT_CALL(events, SeenNameValue(7, "something", "unfinished-line"))
    .WillOnce(Return(true));
  EXPECT_TRUE(p.EmitConfigValues(&events));
}

TEST(ConfigParserTest, ErrorReporting) {
  ConfigParser p;
  p.SetContent(
    " incomplete line\n"
    "[ incomplete-section \n");

  MockConfigReader events;
  EXPECT_CALL(events, ReportError(1, "name=value pair expected."));
  EXPECT_CALL(events, ReportError(2, "Section line does not end in ']'"));

  EXPECT_FALSE(p.EmitConfigValues(&events));
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
