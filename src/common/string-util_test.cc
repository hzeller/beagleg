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
  EXPECT_EQ("hello", TrimWhitespace(" \t  hello \n\r  "));
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
  std::vector<std::string_view> result = SplitString("foo", ",");
  EXPECT_EQ(1, (int)result.size());
  EXPECT_EQ("foo", result[0]);

  result = SplitString(",hello, world", ",");
  EXPECT_EQ(3, (int)result.size());
  EXPECT_EQ("", result[0]);
  EXPECT_EQ("hello", result[1]);
  EXPECT_EQ(" world", result[2]);

  // Also test with trailing, empty field
  result = SplitString(",hello, world,", ",");
  EXPECT_EQ(4, (int)result.size());
  EXPECT_EQ("", result[0]);
  EXPECT_EQ("hello", result[1]);
  EXPECT_EQ(" world", result[2]);
  EXPECT_EQ("", result[3]);
}

TEST(StringUtilTest, ParseDecimalInt) {
  int64_t value;
  EXPECT_FALSE(safe_strto64("hello", &value));
  EXPECT_TRUE(safe_strto64("123", &value));
  EXPECT_EQ(123, value);
  EXPECT_TRUE(safe_strto64("+456", &value));
  EXPECT_EQ(456, value);
  EXPECT_TRUE(safe_strto64("-789", &value));
  EXPECT_EQ(-789, value);
  EXPECT_TRUE(safe_strto64(" 123 ", &value));
  EXPECT_EQ(123, value);

  // Make sure we can parse beyond 32 bit.
  EXPECT_EQ(12345678901234LL, ParseInt64("12345678901234", -1));

  // Make sure we're not assumming a nul-byte at a particular point
  std::string_view longer_string("4255");
  EXPECT_EQ(42, ParseInt64(longer_string.substr(0, 2), -1));
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
