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

#include <gtest/gtest.h>

TEST(StringUtilTest, TrimWhitespace) {
    EXPECT_EQ(StringPiece("hello"), TrimWhitespace(" \t  hello \n\r  "));
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

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
