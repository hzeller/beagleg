/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2018 Henner Zeller <h.zeller@acm.org>
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
#ifndef _BEAGLEG_HERSHEY_H_
#define _BEAGLEG_HERSHEY_H_

#include <functional>

#include "common/string-util.h"

// -- Functions to draw ASCII text in the Hershey simplex font.

// Determine the width of the text if drawn with DrawText()
float TextWidth(StringPiece str, float size);

enum class TextAlign { kLeft, kCenter, kRight };

// Draw a text at position (x,y) with the given alignment and size,
// output is sent to the 2D output 'device' function that receives.
//   "do_line"  - a boolean saying if we should line or move to the position.
//   "x", "y"   - the position to moveto/lineto
// The function makes it independent of any output device and easy to
// adapt in any environment.
void DrawText(StringPiece str, float tx, float ty, TextAlign align, float size,
               std::function<void(bool do_line, float x, float y)> draw);

#endif // _BEAGLEG_HERSHEY_H_
