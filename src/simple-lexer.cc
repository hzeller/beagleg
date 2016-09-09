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

#include <stdlib.h>
#include <assert.h>
#include <ctype.h>
#include <string.h>

static const char *skip_white(const char *line) {
  while (*line && isspace(*line))
    line++;
  return line;
}

class SimpleLexerBase::Node {
public:
  Node() : value(0) {
    memset(&next_character, 0, 256*sizeof(Node*));
  }

  Node *next_character[256];
  int value;
};

SimpleLexerBase::SimpleLexerBase() : root_(new Node()) {
}

SimpleLexerBase::~SimpleLexerBase() {
  for (size_t i = 0; i < to_delete_.size(); ++i) {
    delete to_delete_[i];
  }
  delete root_;
}

void SimpleLexerBase::AddKeywordIntValue(const char *keyword, int value) {
  assert(value > 0);
  AddKeywordAtNode(root_, keyword, value);
  if (keyword_mappings_.find(value) == keyword_mappings_.end()) {
    keyword_mappings_[value] = keyword;
  }
}

const char *SimpleLexerBase::ReverseMapToString(int value) {
  if (value == 0) return "?";
  KeywordMap::const_iterator found = keyword_mappings_.find(value);
  if (found == keyword_mappings_.end())
    return NULL;
  return found->second;
}

void SimpleLexerBase::AddKeywordAtNode(Node *node, const char *keyword,
                                       int value) {
  if (*keyword == '\0') {
    assert(node->value == 0);  // otherwise, the same keyword is already there.
    node->value = value;
  } else {
    assert(!isspace(*keyword));  // Keywords cannot have spaces.
    const int up = toupper(*keyword);
    if (node->next_character[up] == NULL) {
      node->next_character[up] = new Node();
      to_delete_.push_back(node->next_character[up]);
    }
    AddKeywordAtNode(node->next_character[up], keyword + 1, value);
  }
}

int SimpleLexerBase::ConsumeKeyword(const char **input) {
  int result = 0;
  const char *word = *input;
  word = skip_white(word);
  for (const Node *node = root_; node; ++word) {
    if (node->value > 0) {
      *input = skip_white(word);
      result = node->value;
    }
    node = node->next_character[toupper(*word)];
  }
  return result;
}
