/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
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

#include "arc-gen.h"

// Arc generation based on smoothieware implementation
// https://github.com/Smoothieware/Smoothieware.git
// src/modules/robot/Robot.cpp - Robot::append_arc()
//
// G2 or G3 <X- Y- Z- I- J- K- P- F->
//
// XY-plane (G17)  normal-axis: Z
//   X Y : end point of arc
//   Z   : helix (linear_travel)
//   I J : X Y offset to center of arc
//   K   : not used
//   P   : number of turns (currently only 1 turn is generated - 0-90 degree arc)
//   F   : feedrate
//
// ZX-plane (G18)  normal-axis: Y
//   X Z : end point of arc
//   Y   : helix (linear_travel)
//   I K : X Z offset to center of arc
//   J   : not used
//   P   : number of turns (currently only 1 turn is generated - 0-90 degree arc)
//   F   : feedrate
//
// YZ-plane (G19)  normal-axis: X
//   Y Z : end point of arc
//   X   : helix (linear_travel)
//   J K : Y Z offset to center of arc
//   I   : not used
//   P   : number of turns (currently only 1 turn is generated - 0-90 degree arc)
//   F   : feedrate

// these need to be tuned (they are the default values from the smoothieware code)
#define MM_PER_ARC_SEGMENT	0.1
#define N_ARC_CORRECTION	5

// Generate an arc. Input is the
void arc_gen(enum GCodeParserAxis normal_axis,  // Normal axis of the arc-plane
	     char is_cw,                        // 0 CCW, 1 CW
	     float position[],   // start position. Will be updated.
	     float offset[],     // Offset to center.
	     float target[],     // Target position.
	     void (*segment_output)(void *, float[]),
             void *segment_output_user_data) {
  // Depending on the normal vector, pre-calc plane
  enum GCodeParserAxis plane[3];
  switch (normal_axis) {
  case AXIS_Z: plane[0] = AXIS_X; plane[1] = AXIS_Y; plane[2] = AXIS_Z; break;
  case AXIS_X: plane[0] = AXIS_Y; plane[1] = AXIS_Z; plane[2] = AXIS_X; break;
  case AXIS_Y: plane[0] = AXIS_X; plane[1] = AXIS_Z; plane[2] = AXIS_Y; break;
  default:
    return;   // Invalid axis.
  }

  // Scary math
  float radius = sqrtf(offset[plane[0]]*offset[plane[0]] +
                       offset[plane[1]]*offset[plane[1]]);
  float center_0 = position[plane[0]] + offset[plane[0]];
  float center_1 = position[plane[1]] + offset[plane[1]];
  float linear_travel = target[plane[2]] - position[plane[2]];
  float r_0 = -offset[plane[0]]; // Radius vector from center to current location
  float r_1 = -offset[plane[1]];
  float rt_0 = target[plane[0]] - center_0;
  float rt_1 = target[plane[1]] - center_1;

  fprintf(stderr, "arc from %c,%c: %.3f,%.3f to %.3f,%.3f (radius:%.3f) helix %c:%.3f\n",
	  gcodep_axis2letter(plane[0]), gcodep_axis2letter(plane[1]),
	  position[plane[0]], position[plane[1]],
	  target[plane[0]], target[plane[1]], radius,
	  gcodep_axis2letter(plane[2]), linear_travel);

  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_0*rt_1 - r_1*rt_0, r_0*rt_0 + r_1*rt_1);
  if (angular_travel < 0) angular_travel += 2 * M_PI;
  if (is_cw) angular_travel -= 2 * M_PI;

  // Find the distance for this gcode
  float mm_of_travel = hypotf(angular_travel*radius, fabs(linear_travel));

  // We don't care about non-XYZ moves (for example the extruder produces some of those)
  if (mm_of_travel < 0.00001)
    return;

  // Figure out how many segments for this gcode
  int segments = floorf(mm_of_travel / MM_PER_ARC_SEGMENT);

  float theta_per_segment = angular_travel / segments;
  float linear_per_segment = linear_travel / segments;

  /*
   * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
   * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
   * r_T = [cos(phi) -sin(phi);
   * sin(phi) cos(phi] * r ;
   * For arc generation, the center of the circle is the axis of rotation and the radius vector is
   * defined from the circle center to the initial position. Each line segment is formed by successive
   * vector rotations. This requires only two cos() and sin() computations to form the rotation
   * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
   * all float numbers are single precision on the Arduino. (True float precision will not have
   * round off issues for CNC applications.) Single precision error can accumulate to be greater than
   * tool precision in some cases. Therefore, arc path correction is implemented.
   *
   * Small angle approximation may be used to reduce computation overhead further. This approximation
   * holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
   * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
   * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
   * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
   * issue for CNC machines with the single precision Arduino calculations.
   * This approximation also allows mc_arc to immediately insert a line segment into the planner
   * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
   * a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead.
   * This is important when there are successive arc motions.
   */

  // Vector rotation matrix values
  float cos_T = 1 - 0.5 * theta_per_segment * theta_per_segment; // Small angle approximation
  float sin_T = theta_per_segment;

  float arc_target[3];
  float sin_Ti;
  float cos_Ti;
  int i;
  int count = 0;

  // Initialize the linear axis
  arc_target[plane[2]] = position[plane[2]];

  for (i = 1; i < segments; i++) { // Increment (segments-1)
    if (count < N_ARC_CORRECTION) {
      // Apply vector rotation matrix
      float rot = r_0 * sin_T + r_1 * cos_T;
      r_0 = r_0 * cos_T - r_1 * sin_T;
      r_1 = rot;
      count++;
    } else {
      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti = cosf(i * theta_per_segment);
      sin_Ti = sinf(i * theta_per_segment);
      r_0 = -offset[plane[0]] * cos_Ti + offset[plane[1]] * sin_Ti;
      r_1 = -offset[plane[0]] * sin_Ti - offset[plane[1]] * cos_Ti;
      count = 0;
    }

    // Update arc_target location
    arc_target[plane[0]] = center_0 + r_0;
    arc_target[plane[1]] = center_1 + r_1;
    arc_target[plane[2]] += linear_per_segment;

    // Append this segment to the queue
    for (int axis = 0; axis <= AXIS_Z; axis++) {
      position[axis] = arc_target[axis];
    }
    segment_output(segment_output_user_data, position);
  }

  // Ensure last segment arrives at target location.
  for (int axis = 0; axis <= AXIS_Z; axis++) {
    position[axis] = target[axis];
  }
  segment_output(segment_output_user_data, position);
}
