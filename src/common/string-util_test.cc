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

#include <gtest/gtest.h>

TEST(StringUtilTest, TrimWhitespace) {
  EXPECT_EQ(beagleg::string_view("hello"), TrimWhitespace(" \t  hello \n\r  "));
  EXPECT_TRUE(TrimWhitespace(" \t ").empty());
}

TEST(StringUtilTest, ASCIIToLower) {
  EXPECT_EQ("hello world", ToLower("Hello WORLD"));
  EXPECT_EQ("hello world", ToLower("Hello WORLD"));
}

TEST(StringUtilTest, HasPrefix) {
  EXPECT_TRUE(HasPrefix("hello world", "hello"));
  EXPECT_FALSE(HasPrefix("hello world", "hellO"));
}

TEST(StringUtilTest, SplitString) {
  std::vector<beagleg::string_view> result = SplitString("foo", ",");
  EXPECT_EQ(1, (int)result.size());
  EXPECT_EQ(beagleg::string_view("foo"), result[0]);

  result = SplitString(",hello, world", ",");
  EXPECT_EQ(3, (int)result.size());
  EXPECT_EQ(beagleg::string_view(""), result[0]);
  EXPECT_EQ(beagleg::string_view("hello"), result[1]);
  EXPECT_EQ(beagleg::string_view(" world"), result[2]);

  // Also test with trailing, empty field
  result = SplitString(",hello, world,", ",");
  EXPECT_EQ(4, (int)result.size());
  EXPECT_EQ(beagleg::string_view(""), result[0]);
  EXPECT_EQ(beagleg::string_view("hello"), result[1]);
  EXPECT_EQ(beagleg::string_view(" world"), result[2]);
  EXPECT_EQ(beagleg::string_view(""), result[3]);
}

TEST(StringUtilTest, ParseDecimal) {
  // Make sure we can parse beyond 32 bit.
  EXPECT_EQ(12345678901234LL, ParseDecimal("12345678901234", -1));

  // Make sure we're not assumming a nul-byte at a particular point
  beagleg::string_view longer_string("4255");
  EXPECT_EQ(42, ParseDecimal(longer_string.substr(0, 2), -1));
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
