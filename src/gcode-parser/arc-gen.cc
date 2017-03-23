/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2015 H Hartley Sweeten <hsweeten@visionengravers.com>
 *    and author who implemented Smoothieware Robot::append_arc()
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

#include <math.h>
#include <stdio.h>

#include <functional>

#if 0
#  include "logging.h"
#endif

#include "gcode-parser.h"

// Arc generation based on smoothieware implementation
// https://github.com/Smoothieware/Smoothieware.git
// src/modules/robot/Robot.cpp - Robot::append_arc()
//
// Simplified: we don't care about using trigonometric optimization
// tricks (CPU fast enough) but rather have readable code.
//
// Normal axis is the axis perpendicular to the plane the arc is
// created in.

// Small arc segments increase accuracy, though might create some
// CPU load. With 0.1mm arc segments, we can generate about 90 meter/second
// of arcs with the BeagleBone Black CPU. Sufficient :)
#define MM_PER_ARC_SEGMENT      0.1

// Generate an arc. Input is the
static void arc_gen(enum GCodeParserAxis normal_axis,  // Normal axis
                    bool is_cw,                        // 0 CCW, 1 CW
                    AxesRegister *position_out,   // start position. Will be updated.
                    const AxesRegister &center,     // Offset to center.
                    const AxesRegister &target,     // Target position.
                    std::function<void(const AxesRegister&)> segment_output) {
  // Depending on the normal vector, pre-calc plane
  enum GCodeParserAxis plane[3];
  switch (normal_axis) {
  case AXIS_Z: plane[0] = AXIS_X; plane[1] = AXIS_Y; plane[2] = AXIS_Z; break;
  case AXIS_X: plane[0] = AXIS_Y; plane[1] = AXIS_Z; plane[2] = AXIS_X; break;
  case AXIS_Y: plane[0] = AXIS_X; plane[1] = AXIS_Z; plane[2] = AXIS_Y; break;
  default:
    return;   // Invalid axis.
  }

  // Alias reference for readable use with [] operator
  AxesRegister &position = *position_out;
  AxesRegister offset;
  offset[AXIS_X] = center[AXIS_X] - position[AXIS_X];
  offset[AXIS_Y] = center[AXIS_Y] - position[AXIS_Y];
  offset[AXIS_Z] = center[AXIS_Z] - position[AXIS_Z];
  const float radius = sqrtf(offset[plane[0]] * offset[plane[0]] +
                             offset[plane[1]] * offset[plane[1]]);
  const float center_0 = position[plane[0]] + offset[plane[0]];
  const float center_1 = position[plane[1]] + offset[plane[1]];
  const float linear_travel = target[plane[2]] - position[plane[2]];
  const float rt_0 = target[plane[0]] - center_0;
  const float rt_1 = target[plane[1]] - center_1;

  float r_0 = -offset[plane[0]]; // Radius vector from center to current location
  float r_1 = -offset[plane[1]];

#if 0
  Log_debug("arc from %c,%c: %.3f,%.3f to %.3f,%.3f (radius:%.3f) helix %c:%.3f\n",
            gcodep_axis2letter(plane[0]), gcodep_axis2letter(plane[1]),
            position[plane[0]], position[plane[1]],
            target[plane[0]], target[plane[1]], radius,
            gcodep_axis2letter(plane[2]), linear_travel);
#endif

  // CCW angle between position and target from circle center.
  float angular_travel = atan2(r_0*rt_1 - r_1*rt_0, r_0*rt_0 + r_1*rt_1);
  if (is_cw) {
    if (angular_travel >= 0) angular_travel -= 2*M_PI;
  } else {
    if (angular_travel <= 0) angular_travel += 2*M_PI;
  }

  // Find the distance for this gcode in the axes we care.
  const float mm_of_travel = hypotf(angular_travel*radius, fabs(linear_travel));

  // We don't care about non-XYZ moves (e.g. extruder)
  if (mm_of_travel < 0.00001)
    return;

  // Figure out how many segments for this gcode
  const int segments = floorf(mm_of_travel / MM_PER_ARC_SEGMENT);

  const float theta_per_segment = angular_travel / segments;
  const float linear_per_segment = linear_travel / segments;

  for (int i = 1; i < segments; i++) { // Increment (segments-1)
    const float cos_Ti = cosf(i * theta_per_segment);
    const float sin_Ti = sinf(i * theta_per_segment);
    r_0 = -offset[plane[0]] * cos_Ti + offset[plane[1]] * sin_Ti;
    r_1 = -offset[plane[0]] * sin_Ti - offset[plane[1]] * cos_Ti;

    // Update arc_target location
    position[plane[0]] = center_0 + r_0;
    position[plane[1]] = center_1 + r_1;
    position[plane[2]] += linear_per_segment;

    // Emit
    segment_output(position);
  }

  // Ensure last segment arrives at target location.
  for (int axis = AXIS_X; axis <= AXIS_Z; axis++) {
    position[(GCodeParserAxis)axis] = target[(GCodeParserAxis)axis];
  }
  segment_output(position);
}

static AxesRegister calc_bezier_point(float t,
                                      const AxesRegister &p0,
                                      const AxesRegister &p1,
                                      const AxesRegister &p2,
                                      const AxesRegister &p3) {
  const float u = 1.0f - t;
  const float uu = u * u;
  const float tt = t * t;
  const float uuu = uu * u;
  const float ttt = tt * t;

  AxesRegister p = p0;
  p[AXIS_X] = uuu * p0[AXIS_X];               // first term
  p[AXIS_Y] = uuu * p0[AXIS_Y];
  p[AXIS_X] += (3.0f * uu * t * p1[AXIS_X]);  // second term
  p[AXIS_Y] += (3.0f * uu * t * p1[AXIS_Y]);
  p[AXIS_X] += (3.0f * u * tt * p2[AXIS_X]);  // third term
  p[AXIS_Y] += (3.0f * u * tt * p2[AXIS_Y]);
  p[AXIS_X] += (ttt * p3[AXIS_X]);            // forth term
  p[AXIS_Y] += (ttt * p3[AXIS_Y]);
  return p;
}

static void spline_gen(const AxesRegister &start,
                       const AxesRegister &cp1,
                       const AxesRegister &cp2,
                       const AxesRegister &target,
                std::function<void (const AxesRegister& pos)> segment_output) {
#if 0
  Log_debug("spline_gen: start:%.3f,%.3f cp1:%.3f,%.3f cp2:%.3f,%.3f end:%.3f,%.3f\n",
            position[AXIS_X], position[AXIS_Y],
            cp1[AXIS_X], cp1[AXIS_Y],
            cp2[AXIS_X], cp2[AXIS_Y],
            target[AXIS_X], target[AXIS_Y]);
#endif

  for (float t = 0; t < 1; t += 0.01f) {
    segment_output(calc_bezier_point(t, start, cp1, cp2, target));
  }
  segment_output(target);
}

void GCodeParser::EventReceiver::arc_move(float feed_mm_p_sec,
                                          GCodeParserAxis normal_axis,
                                          bool clockwise,
                                          const AxesRegister &start,
                                          const AxesRegister &center,
                                          const AxesRegister &end) {
  AxesRegister position = start;
  arc_gen(normal_axis, clockwise, &position,
          center, end, [this, feed_mm_p_sec](const AxesRegister &pos) {
            coordinated_move(feed_mm_p_sec, pos);
          });
}

void GCodeParser::EventReceiver::spline_move(float feed_mm_p_sec,
                                             const AxesRegister &start,
                                             const AxesRegister &cp1,
                                             const AxesRegister &cp2,
                                             const AxesRegister &end) {
  spline_gen(start, cp1, cp2, end,
             [this, feed_mm_p_sec](const AxesRegister &pos) {
               coordinated_move(feed_mm_p_sec, pos);
             });
}
