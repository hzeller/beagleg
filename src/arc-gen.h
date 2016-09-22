/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2015 H Hartley Sweeten <hsweeten@visionengravers.com>
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
#ifndef _BEAGLEG_ARC_GEN_H
#define _BEAGLEG_ARC_GEN_H

#include "gcode-parser.h"

void spline_gen(AxesRegister *position,     // start position. Will be updated.
                const AxesRegister &cp1,    // Offset from start to first control point.
                const AxesRegister &cp2,    // Offset from target to second control point.
                const AxesRegister &target, // Target position.
                void (*segment_output)(void *, const AxesRegister &),
                void *segment_output_user_data);

#endif  // _BEAGLEG_ARC_GEN_H
