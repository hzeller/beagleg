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
#include <map>
#include <vector>

class SimpleLexerBase {
public:
  ~SimpleLexerBase();

protected:
  SimpleLexerBase();
  // Add a keyword and the corresponding value it should be associated with.
  // "value" needs to be > 0.
  void AddKeywordIntValue(const char *keyword, int value);
  int ConsumeKeyword(const char **input);
  const char *ReverseMapToString(int value);

private:
  class Node;
  typedef std::map<int, const char*> KeywordMap;

  void AddKeywordAtNode(Node *node, const char *keyword, int value);

  Node *root_;
  std::vector<Node*> to_delete_;
  KeywordMap keyword_mappings_;
};

// A simple keyword matcher that can deal with ambiguous prefixes
// (e.g. '<' and '<=').
// It is matching keywords and maps it to a particular enumeration.
// to match keywords that are mapped to a particular enumeration.
// Only requirement: The Enum needs to map value zero (0) to 'invalid keyword'.
template <typename Enum>
class SimpleLexer : public SimpleLexerBase {
public:
  // Register a keyword and the corresponding value it should be associated
  // with. The "value" needs to be > 0 (as 0 means 'no keyword').
  // The keyword is remembered for the AsString() function. If multiple
  // keywords are added with the same enum, the first is remembered.
  void AddKeyword(const char *keyword, Enum value) {
    AddKeywordIntValue(keyword, value);
  }

  // Get string respresentation of keyword enum or NULL if it does not exist.
  const char *AsString(Enum e) { return ReverseMapToString(e); }

  // Attempts to consume next keyword greedily (longer match wins).
  // Advances the input the number of consumed characters if there was
  // a match.
  // The "input" pointer is modified to point to the first non-whitespace
  // character after the matched keyword on success.
  // Returns value of matched keyword or 0 if there was no match.
  Enum MatchNext(const char **input) {
    return static_cast<Enum>(ConsumeKeyword(input));
  }

  // Expect and consume a particular keyword, otherwise return false.
  bool ExpectNext(const char **input, Enum e) {
    const char *end = *input;
    if (MatchNext(&end) == e) {
      *input = end;
      return true;
    }
    return false;
  }
};
