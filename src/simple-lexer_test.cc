/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2016 Henner Zeller <h.zeller@acm.org>
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

#include "simple-lexer.h"

#include <gtest/gtest.h>

enum MyKeywords {
    NO_KEYWORD,
    KEYWORD_IF,
    KEYWORD_ELSE,
    KEYWORD_ELSEIF,
    KEYWORD_LT,
    KEYWORD_LE
};

TEST(SimpleLexerTest, SimpleKeywordMatching) {
    SimpleLexer<MyKeywords> lexer;
    lexer.AddKeyword("if",     KEYWORD_IF);
    lexer.AddKeyword("else",   KEYWORD_ELSE);
    lexer.AddKeyword("elseif", KEYWORD_ELSEIF);
    lexer.AddKeyword("<",      KEYWORD_LT);
    lexer.AddKeyword("<=",     KEYWORD_LE);

    const char *word = "nothing";
    EXPECT_EQ(NO_KEYWORD, lexer.MatchNext(&word));
    EXPECT_EQ(std::string("nothing"), word);

    word = "elsx";
    EXPECT_EQ(NO_KEYWORD, lexer.MatchNext(&word));
    EXPECT_EQ(std::string("elsx"), word);

    word = "else";
    EXPECT_EQ(KEYWORD_ELSE, lexer.MatchNext(&word));
    EXPECT_EQ(std::string(""), word);

    word = "elsefoo";
    EXPECT_EQ(KEYWORD_ELSE, lexer.MatchNext(&word));
    EXPECT_EQ(std::string("foo"), word);

    word = "elseifbar";
    EXPECT_EQ(KEYWORD_ELSEIF, lexer.MatchNext(&word));
    EXPECT_EQ(std::string("bar"), word);

    word = "<";
    EXPECT_EQ(KEYWORD_LT, lexer.MatchNext(&word));
    EXPECT_EQ(std::string(""), word);

    word = "<=";
    EXPECT_EQ(KEYWORD_LE, lexer.MatchNext(&word));
    EXPECT_EQ(std::string(""), word);

    word = "<hello";
    EXPECT_EQ(KEYWORD_LT, lexer.MatchNext(&word));
    EXPECT_EQ(std::string("hello"), word);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
