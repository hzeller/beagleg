/* -*- mode: c; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2014 Henner Zeller <h.zeller@acm.org>
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

#include "gcode-parser.h"

#include <assert.h>  // remove.

#include <ctype.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/types.h>
#include <unistd.h>

//-- This is the more detailed public API. If it ever is needed (e.g.
//   to keep track of multiple streams at once, put in gcode-parser.h.
//   Till then, gcode_parse_stream is sufficient.
typedef struct GCodeParser GCodeParser_t;  // Opaque parser object type.

// These are internal implementation details of creating a GCodeParser
// object. This was exposed before publically, but is not really needed, as
// users use the facade gcodep_parse_stream() function.
// Could be made a public API if needed.

// Initialize parser.
// The "callbacks"-struct contains the functions the parser calls on parsing.
// Returns an opaque type used in the parse function.
// Does not take ownership of the provided pointer.
static GCodeParser_t *gcodep_new(struct GCodeParserCb *callbacks);

static void gcodep_delete(GCodeParser_t *object);  // Opposite of gcodep_new()

// Main workhorse: Parse a gcode line, call callbacks if needed.
// If "err_stream" is non-NULL, sends error messages that way.
static void gcodep_parse_line(GCodeParser_t *obj, const char *line,
                              FILE *err_stream);
// -- End public API

typedef float AxesRegister[GCODE_NUM_AXES];

const AxisBitmap_t kAllAxesBitmap =
  ((1 << AXIS_X) | (1 << AXIS_Y) | (1 << AXIS_Z)| (1 << AXIS_E)
   | (1 << AXIS_A) | (1 << AXIS_B) | (1 << AXIS_C)
   | (1 << AXIS_U) | (1 << AXIS_V) | (1 << AXIS_W));

struct GCodeParser {
  struct GCodeParserCb callbacks;
  FILE *err_msg;
  int modal_g0_g1;
  int line_number;
  int provided_axes;
  float unit_to_mm_factor;      // metric: 1.0; imperial 25.4
  char axis_is_absolute[GCODE_NUM_AXES];
  AxesRegister relative_zero;  // reference, set by G92 commands
  AxesRegister axes_pos;
};

static void dummy_gcode_start(void *user) {}
static void dummy_gcode_finished(void *user) {}
static void dummy_gcode_command_executed(void *user, char letter, float val) {}

static void dummy_set_speed_factor(void *user, float f) {
  fprintf(stderr, "GCodeParser: set_speed_factor(%.1f)\n", f);
}
static void dummy_set_temperature(void *user, float f) {
  fprintf(stderr, "GCodeParser: set_temperature(%.1f)\n", f);
}
static void dummy_set_fanspeed(void *user, float speed) {
  fprintf(stderr, "GCodeParser: set_fanspeed(%.0f)\n", speed);
}
static void dummy_wait_temperature(void *user) {
  fprintf(stderr, "GCodeParser: wait_temperature()\n");
}
static void dummy_dwell(void *user, float f) {
  fprintf(stderr, "GCodeParser: dwell(%.1f)\n", f);
}
static void dummy_motors_enable(void *user, char b) {
  fprintf(stderr, "GCodeParser: %s motors\n", b ? "enable" : "disable");
}
static void dummy_move(void *user, float feed, const float *axes) {
  fprintf(stderr, "GCodeParser: move(X=%.3f,Y=%.3f,Z=%.3f,E=%.3f,...);",
	  axes[AXIS_X], axes[AXIS_Y], axes[AXIS_Z], axes[AXIS_E]);
  if (feed > 0)
    fprintf(stderr, " feed=%.1f\n", feed);
  else
    fprintf(stderr, "\n");
}
static void dummy_go_home(void *user, AxisBitmap_t axes, float *new_pos) {
  fprintf(stderr, "GCodeParser: go-home(0x%02x)\n", axes);
  memset(new_pos, GCODE_NUM_AXES, sizeof(*new_pos));
}
static int dummy_probe_axis(void *user, float feed, enum GCodeParserAxis axis) {
  fprintf(stderr, "GCodeParser: probe-axis(%c)", gcodep_axis2letter(axis));
  if (feed > 0)
    fprintf(stderr, " feed=%.1f\n", feed);
  else
    fprintf(stderr, "\n");
  return 0;
}
static const char *dummy_unprocessed(void *user, char letter, float value,
				     const char *remaining) {
  fprintf(stderr, "GCodeParser: unprocessed('%c', %d, '%s')\n",
	  letter, (int) value, remaining);
  return NULL;
}
static void dummy_idle(void *user) {
  fprintf(stderr, "GCodeParser: input idle\n");
}

static void set_all_axis_to_absolute(GCodeParser_t *p, char value) {
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    p->axis_is_absolute[i] = value;
  }
}

char gcodep_axis2letter(enum GCodeParserAxis axis) {
  switch (axis) {
  case AXIS_X: return 'X';
  case AXIS_Y: return 'Y';
  case AXIS_Z: return 'Z';
  case AXIS_E: return 'E';
  case AXIS_A: return 'A';
  case AXIS_B: return 'B';
  case AXIS_C: return 'C';
  case AXIS_U: return 'U';
  case AXIS_V: return 'V';
  case AXIS_W: return 'W';
  case GCODE_NUM_AXES: return '?';
    // no default to have compiler warn about new values.
  }
  return '?';
}

// Returns the GCodeParserAxis value or GCODE_NUM_AXES if out of range.
enum GCodeParserAxis gcodep_letter2axis(char letter) {
  switch (letter) {
  case 'X': case 'x': return AXIS_X;
  case 'Y': case 'y': return AXIS_Y;
  case 'Z': case 'z': return AXIS_Z;
  case 'E': case 'e': return AXIS_E;
  case 'A': case 'a': return AXIS_A;
  case 'B': case 'b': return AXIS_B;
  case 'C': case 'c': return AXIS_C;
  case 'U': case 'u': return AXIS_U;
  case 'V': case 'v': return AXIS_V;
  case 'W': case 'w': return AXIS_W;
  }
  return GCODE_NUM_AXES;
}

static struct GCodeParser *gcodep_new(struct GCodeParserCb *callbacks) {
  GCodeParser_t *result = (GCodeParser_t*)malloc(sizeof(*result));
  memset(result, 0x00, sizeof(*result));

  // Initial values for various constants.
  result->unit_to_mm_factor = 1.0f;
  set_all_axis_to_absolute(result, 1);

  // Setting up all callbacks
  if (callbacks) {
    memcpy(&result->callbacks, callbacks, sizeof(*callbacks));
  }

  // Set some reasonable defaults for unprovided callbacks:
  if (!result->callbacks.gcode_start)
    result->callbacks.gcode_start = &dummy_gcode_start;
  if (!result->callbacks.gcode_finished)
    result->callbacks.gcode_finished = &dummy_gcode_finished;
  if (!result->callbacks.go_home)
    result->callbacks.go_home = &dummy_go_home;
  if (!result->callbacks.probe_axis)
    result->callbacks.probe_axis = &dummy_probe_axis;
  if (!result->callbacks.set_fanspeed)
    result->callbacks.set_fanspeed = &dummy_set_fanspeed;
  if (!result->callbacks.set_speed_factor)
    result->callbacks.set_speed_factor = &dummy_set_speed_factor;
  if (!result->callbacks.set_temperature)
    result->callbacks.set_temperature = &dummy_set_temperature;
  if (!result->callbacks.wait_temperature)
    result->callbacks.wait_temperature = &dummy_wait_temperature;
  if (!result->callbacks.dwell)
    result->callbacks.dwell = &dummy_dwell;
  if (!result->callbacks.motors_enable)
    result->callbacks.motors_enable = &dummy_motors_enable;
  if (!result->callbacks.coordinated_move)
    result->callbacks.coordinated_move = &dummy_move;
  if (!result->callbacks.rapid_move)
    result->callbacks.rapid_move = result->callbacks.coordinated_move;
  if (!result->callbacks.unprocessed)
    result->callbacks.unprocessed = &dummy_unprocessed;
  if (!result->callbacks.gcode_command_done)
    result->callbacks.gcode_command_done = &dummy_gcode_command_executed;
  if (!result->callbacks.input_idle)
    result->callbacks.input_idle = &dummy_idle;
  return result;
}

static void gcodep_delete(struct GCodeParser *parser) {
  free(parser);
}

static const char *skip_white(const char *line) {
  while (*line && isspace(*line))
    line++;
  return line;
}

// Parse next letter/number pair.
// Returns the remaining line or NULL if end reached.
static const char *gcodep_parse_pair_with_linenumber(int line_num,
                                                     const char *line,
                                                     char *letter, float *value,
                                                     FILE *err_stream) {
  // TODO: error callback when we have errors with messages.
  if (line == NULL)
    return NULL;
  line = skip_white(line);

  if (*line == '\0' || *line == ';' || *line == '%')
    return NULL;

  if (*line == '(') {  // Comment between words; e.g. G0(move) X1(this axis)
    while (*line && *line != ')')
      line++;
    line = skip_white(line + 1);
    if (*line == '\0') return NULL;
  }

  *letter = toupper(*line++);
  if (*line == '\0') {
    fprintf(err_stream ? err_stream : stderr,
	    "// Line %d G-Code Syntax Error: expected value after '%c'\n",
            line_num, *letter);
    return NULL;
  }
  // If this line has a checksum, we ignore it. In fact, the line is done.
  if (*letter == '*')
    return NULL;
  while (*line && isspace(*line))
    line++;

  // Parsing with strtof() can be problematic if the line does
  // not contain spaces, and strof() sees the sequence 0X... as it treats that
  // as hex value. E.g. G0X1. Unlikely, but let's do a nasty workaround:
  char *repair_x = (*(line+1) == 'x' || *(line+1) == 'X') ? (char*)line+1 : NULL;
  if (repair_x) *repair_x = '\0';  // pretend that is the end of number.

  char *endptr;
  *value = strtof(line, &endptr);

  if (repair_x) *repair_x = 'X';  // Put the 'X' back.

  if (line == endptr) {
    fprintf(err_stream ? err_stream : stderr, "// Line %d G-Code Syntax Error:"
	    " Letter '%c' is not followed by a number `%s`.\n",
            line_num, *letter, line);
    return NULL;
  }
  line = endptr;

  line = skip_white(line); // Makes the line better to deal with.
  return line;  // We parsed something; return whatever is remaining.
}

// Internally used version
static const char *gparse_pair(struct GCodeParser *p, const char *line,
                               char *letter, float *value) {
  return gcodep_parse_pair_with_linenumber(p->line_number, line,
                                           letter, value, p->err_msg);
}

// public version.
const char *gcodep_parse_pair(const char *line,
                              char *letter, float *value,
                              FILE *err_stream) {
  return gcodep_parse_pair_with_linenumber(-1, line, letter, value, err_stream);
}


static const char *handle_home(struct GCodeParser *p, const char *line) {
  AxisBitmap_t homing_flags = 0;
  char axis_l;
  float dummy;
  const char *remaining_line;
  while ((remaining_line = gparse_pair(p, line, &axis_l, &dummy))) {
    const enum GCodeParserAxis axis = gcodep_letter2axis(axis_l);
    if (axis == GCODE_NUM_AXES)
      break;  //  Possibly start of new command.
    homing_flags |= (1 << axis);
    line = remaining_line;
  }
  if (homing_flags == 0) homing_flags = kAllAxesBitmap;
  // home the selected axes (last machine position will be updated)
  float new_position[GCODE_NUM_AXES] = {0};
  p->callbacks.go_home(p->callbacks.user_data, homing_flags,
                       new_position);

  // now update the world position
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (homing_flags & (1 << i)) {
      p->axes_pos[i] = new_position[i];
      p->relative_zero[i] = new_position[i];
    }
  }

  return line;
}

static const char *handle_rebase(struct GCodeParser *p, const char *line) {
  char axis_l;
  float value;
  const char *remaining_line;
  while ((remaining_line = gparse_pair(p, line, &axis_l, &value))) {
    const float unit_val = value * p->unit_to_mm_factor;
    const enum GCodeParserAxis axis = gcodep_letter2axis(axis_l);
    if (axis == GCODE_NUM_AXES)
      break;    // Possibly start of new command.
    p->relative_zero[axis] = p->axes_pos[axis] - unit_val;

    line = remaining_line;
  }
  return line;
}

// Set a parameter on a user callback.
// These all have the form foo(void *userdata, float value)
static const char *set_param(struct GCodeParser *p, char param_letter,
			     void (*value_setter)(void *, float), float factor,
			     const char *line) {
  char letter;
  float value;
  const char *remaining_line = gparse_pair(p, line, &letter, &value);
  if (remaining_line != NULL && letter == param_letter) {
    value_setter(p->callbacks.user_data, factor * value);
    return remaining_line;
  }
  return line;
}

static float abs_axis_pos(struct GCodeParser *p,
			  const enum GCodeParserAxis axis,
			  const float unit_value) {
  float pos = (p->axis_is_absolute[axis]) ? p->relative_zero[axis] : p->axes_pos[axis];
  return pos + unit_value;
}

static float f_param_to_feedrate(const float unit_value) {
  return unit_value / 60.0f;  // feedrates are units per minute.
}

static const char *handle_move(struct GCodeParser *p,
			       const char *line, int force_change) {
  char axis_l;
  float value;
  int any_change = force_change;
  float feedrate = -1;
  const char *remaining_line;
  while ((remaining_line = gparse_pair(p, line, &axis_l, &value))) {
    const float unit_value = value * p->unit_to_mm_factor;
    if (axis_l == 'F') {
      feedrate = f_param_to_feedrate(unit_value);
      any_change = 1;
    }
    else {
      const enum GCodeParserAxis update_axis = gcodep_letter2axis(axis_l);
      if (update_axis == GCODE_NUM_AXES)
        break;  // Invalid axis: possibley start of new command.
      p->axes_pos[update_axis] = abs_axis_pos(p, update_axis, unit_value);
      any_change = 1;
    }
    line = remaining_line;
  }

  if (any_change) {
    if (p->modal_g0_g1)
      p->callbacks.coordinated_move(p->callbacks.user_data, feedrate, p->axes_pos);
    else
      p->callbacks.rapid_move(p->callbacks.user_data, feedrate, p->axes_pos);
  }
  return line;
}

// Arc generation based on smoothieware implementation
// https://github.com/Smoothieware/Smoothieware.git
// src/modules/robot/Robot.cpp - Robot::append_arc()
// currently only supports XY-plane
//
// G2 or G3 <X- Y- Z- I- J- K- P- F->
//
// XY-plane (G17)
//   X Y : end point of arc
//   Z   : helix (linear_travel)
//   I J : X Y offset to center of arc
//   K   : not used
//   P   : number of turns (currently only 1 turn is generated - 0-90 degree arc)
//   F   : feedrate
//
// XZ-plane (G18)
//   X Z : end point of arc
//   Y   : helix (linear_travel)
//   I K : X Z offset to center of arc
//   J   : not used
//   P   : number of turns (currently only 1 turn is generated - 0-90 degree arc)
//   F   : feedrate
//
// YZ-plane (G19)
//   Y Z : end point of arc
//   X   : helix (linear_travel)
//   J K : Y Z offset to center of arc
//   I   : not used
//   P   : number of turns (currently only 1 turn is generated - 0-90 degree arc)
//   F   : feedrate

// these need to be tuned (they are the default values from the smoothieware code)
#define MM_PER_ARC_SEGMENT	0.5
#define N_ARC_CORRECTION	5

static const char *handle_arc(struct GCodeParser *p,
			      const char *line,
			      int is_cw) {
  const char *remaining_line;
  float end[3];		// absolute end point of arc
  float offset[3];	// relative offset (center point) of arc
  float feedrate = -1;
  float value;
  char letter;
  int turns = 1;
  enum GCodeParserAxis axis;

  // default the end points to the current position and clear the offset
  for (axis = 0; axis <= AXIS_Z; axis++) {
    end[axis] = p->axes_pos[axis];
    offset[axis] = 0;
  }

  while ((remaining_line = gparse_pair(p, line, &letter, &value))) {
    const float unit_value = value * p->unit_to_mm_factor;
    if (letter == 'X') end[AXIS_X] = abs_axis_pos(p, AXIS_X, unit_value);
    else if (letter == 'Y') end[AXIS_Y] = abs_axis_pos(p, AXIS_Y, unit_value);
    else if (letter == 'Z') end[AXIS_Z] = abs_axis_pos(p, AXIS_Z, unit_value);
    else if (letter == 'I') offset[AXIS_X] = unit_value;
    else if (letter == 'J') offset[AXIS_Y] = unit_value;
    else if (letter == 'K') offset[AXIS_Z] = unit_value;
    else if (letter == 'F') feedrate = f_param_to_feedrate(unit_value);
    else if (letter == 'P') turns = (int)value;	// currently ignored
    else break;

    line = remaining_line;
  }

  // Should the arc parameters be sanity checked?
  if (turns < 0 || turns > 4) {
    fprintf(stderr, "G-Code Syntax Error: handle_arc: turns=%d (must be 1-4)\n",
            turns);
    return remaining_line;
  }

  // Scary math
  float radius = sqrtf(offset[AXIS_X]*offset[AXIS_X] + offset[AXIS_Y]*offset[AXIS_Y]);
  float center_x = p->axes_pos[AXIS_X] + offset[AXIS_X];
  float center_y = p->axes_pos[AXIS_Y] + offset[AXIS_Y];
  float linear_travel = end[AXIS_Z] - p->axes_pos[AXIS_Z];
  float r_x = -offset[AXIS_X]; // Radius vector from center to current location
  float r_y = -offset[AXIS_Y];
  float rt_x = end[AXIS_X] - center_x;
  float rt_y = end[AXIS_Y] - center_y;

  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_x*rt_y - r_y*rt_x, r_x*rt_x + r_y*rt_y);
  if (angular_travel < 0) angular_travel += 2 * M_PI;
  if (is_cw) angular_travel -= 2 * M_PI;

  // Find the distance for this gcode
  float mm_of_travel = hypotf(angular_travel*radius, fabs(linear_travel));

  // We don't care about non-XYZ moves (for example the extruder produces some of those)
  if (mm_of_travel < 0.00001)
    return remaining_line;

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
  arc_target[AXIS_Z] = p->axes_pos[AXIS_Z];

  for (i = 1; i < segments; i++) { // Increment (segments-1)
    if (count < N_ARC_CORRECTION) {
      // Apply vector rotation matrix
      float rot = r_x * sin_T + r_y * cos_T;
      r_x = r_x * cos_T - r_y * sin_T;
      r_y = rot;
      count++;
    } else {
      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti = cosf(i * theta_per_segment);
      sin_Ti = sinf(i * theta_per_segment);
      r_x = -offset[AXIS_X] * cos_Ti + offset[AXIS_Y] * sin_Ti;
      r_y = -offset[AXIS_X] * sin_Ti - offset[AXIS_Y] * cos_Ti;
      count = 0;
    }

    // Update arc_target location
    arc_target[AXIS_X] = center_x + r_x;
    arc_target[AXIS_Y] = center_y + r_y;
    arc_target[AXIS_Z] += linear_per_segment;

    // Append this segment to the queue
    for (axis = 0; axis <= AXIS_Z; axis++)
      p->axes_pos[axis] = arc_target[axis];
    p->callbacks.coordinated_move(p->callbacks.user_data, feedrate, p->axes_pos);
  }

  // Ensure last segment arrives at target location.
  for (axis = 0; axis <= AXIS_Z; axis++)
    p->axes_pos[axis] = end[axis];
  p->callbacks.coordinated_move(p->callbacks.user_data, feedrate, p->axes_pos);

  return line;
}

static const char *handle_z_probe(struct GCodeParser *p, const char *line) {
  char letter;
  float value;
  float feedrate = -1;
  float new_pos = 0;
  const char *remaining_line;
  while ((remaining_line = gparse_pair(p, line, &letter, &value))) {
    const float unit_value = value * p->unit_to_mm_factor;
    if (letter == 'F') feedrate = f_param_to_feedrate(unit_value);
    else if (letter == 'Z') new_pos = value * p->unit_to_mm_factor;
    else break;
    line = remaining_line;
  }
  // probe for the travel endstop
  if (p->callbacks.probe_axis(p->callbacks.user_data, feedrate, AXIS_Z)) {
    // FIXME: the machine and world positions should refelct the same position
    p->axes_pos[AXIS_Z] = new_pos;
    p->relative_zero[AXIS_Z] = 0;
  }
  return line;
}

// Note: changes here should be documented in G-code.md as well.
static void gcodep_parse_line(struct GCodeParser *p, const char *line,
                              FILE *err_stream) {
  ++p->line_number;
  void *const userdata = p->callbacks.user_data;
  struct GCodeParserCb *cb = &p->callbacks;
  p->err_msg = err_stream;  // remember as 'instance' variable.
  char letter;
  float value;
  while ((line = gparse_pair(p, line, &letter, &value))) {
    char processed_command = 1;
    if (letter == 'G') {
      switch ((int) value) {
      case 0: p->modal_g0_g1 = 0; line = handle_move(p, line, 0); break;
      case 1: p->modal_g0_g1 = 1; line = handle_move(p, line, 0); break;
      case 2: line = handle_arc(p, line, 1); break;
      case 3: line = handle_arc(p, line, 0); break;
      case 4: line = set_param(p, 'P', cb->dwell, 1.0f, line); break;
      case 20: p->unit_to_mm_factor = 25.4f; break;
      case 21: p->unit_to_mm_factor = 1.0f; break;
      case 28: line = handle_home(p, line); break;
      case 30: line = handle_z_probe(p, line); break;
      case 70: p->unit_to_mm_factor = 25.4f; break;
      case 71: p->unit_to_mm_factor = 1.0f; break;
      case 90: set_all_axis_to_absolute(p, 1); break;
      case 91: set_all_axis_to_absolute(p, 0); break;
      case 92: line = handle_rebase(p, line); break;
      default: line = cb->unprocessed(userdata, letter, value, line); break;
      }
    }
    else if (letter == 'M') {
      switch ((int) value) {
      case 2: cb->input_idle(userdata); break;
      case 17: cb->motors_enable(userdata, 1); break;
      case 18: cb->motors_enable(userdata, 0); break;
      case 30: cb->input_idle(userdata); break;
      case 82: p->axis_is_absolute[AXIS_E] = 1; break;
      case 83: p->axis_is_absolute[AXIS_E] = 0; break;
      case 84: cb->motors_enable(userdata, 0); break;
      case 104: line = set_param(p, 'S', cb->set_temperature, 1.0f, line); break;
      case 106: line = set_param(p, 'S', cb->set_fanspeed, 1.0f, line); break;
      case 107: cb->set_fanspeed(userdata, 0); break;
      case 109:
	line = set_param(p, 'S', cb->set_temperature, 1.0f, line);
	cb->wait_temperature(userdata);
	break;
      case 116: cb->wait_temperature(userdata); break;
      case 220:
	line = set_param(p, 'S', cb->set_speed_factor, 0.01f, line);
	break;
      default: line = cb->unprocessed(userdata, letter, value, line); break;
      }
    }
    else if (letter == 'N') {
      // Line number? Yeah, ignore for now :)
      processed_command = 0;
    }
    else {
      const enum GCodeParserAxis axis = gcodep_letter2axis(letter);
      if (axis == GCODE_NUM_AXES) {
        line = cb->unprocessed(userdata, letter, value, line);
      } else {
        // This line must be a continuation of a previous G0/G1 command.
        // Update the axis position then handle the move.
        const float unit_value = value * p->unit_to_mm_factor;
        p->axes_pos[axis] = abs_axis_pos(p, axis, unit_value);
	line = handle_move(p, line, 1);
	// make gcode_command_done() think this was a 'G0/G1' command
	letter = 'G';
	value = p->modal_g0_g1;
      }
    }
    if (processed_command) {
      cb->gcode_command_done(userdata, letter, value);
    }
  }
  p->err_msg = NULL;
}

// It is usually good to shut down gracefully, otherwise the PRU keeps running.
// So we're intercepting signals and exit gcode_machine_control_from_stream()
// cleanly.
static volatile char caught_signal = 0;
static void receive_signal() {
  caught_signal = 1;
  static char msg[] = "Caught signal. Shutting down ASAP.\n";
  (void)write(STDERR_FILENO, msg, sizeof(msg)); // void, but gcc still warns :/
}
static void arm_signal_handler() {
  caught_signal = 0;
  signal(SIGTERM, &receive_signal);  // Regular kill
  signal(SIGINT, &receive_signal);   // Ctrl-C
}
static void disarm_signal_handler() {
  signal(SIGTERM, SIG_DFL);  // Regular kill
  signal(SIGINT, SIG_DFL);   // Ctrl-C
}

// Public facade function.
int gcodep_parse_stream(int input_fd,
                        struct GCodeParserCb *parse_events,
                        FILE *err_stream) {
  if (err_stream) {
    // Output needs to be unbuffered, otherwise they'll never make it.
    setvbuf(err_stream, NULL, _IONBF, 0);
  }

  FILE *gcode_stream = fdopen(input_fd, "r");
  if (gcode_stream == NULL) {
    perror("Opening gcode stream");
    return 1;
  }

  fd_set read_fds;
  struct timeval wait_time;
  int select_ret;
  char in_idle_mode = 0;
  wait_time.tv_sec = 0;
  wait_time.tv_usec = 50 * 1000;
  FD_ZERO(&read_fds);

  GCodeParser_t *parser = gcodep_new(parse_events);
  arm_signal_handler();
  char buffer[8192];
  parser->callbacks.gcode_start(parse_events->user_data);
  while (!caught_signal) {
    // Read with timeout. If we don't get anything on our input, but it
    // is not finished yet, we tell our
    FD_SET(input_fd, &read_fds);
    // When we're already in idle mode, we're not interested in change.
    wait_time.tv_usec = in_idle_mode ? 500 * 1000 : 50 * 1000;
    select_ret = select(input_fd + 1, &read_fds, NULL, NULL, &wait_time);
    if (select_ret < 0)
      break;
    if (select_ret == 0) {  // timeout.
      if (!in_idle_mode) {
        parser->callbacks.input_idle(parse_events->user_data);
        in_idle_mode = 1;
      }
      continue;
    }

    // Filedescriptor readable. Now wait for a line to finish.
    if (fgets(buffer, sizeof(buffer), gcode_stream) == NULL)
      break;
    in_idle_mode = 0;
    gcodep_parse_line(parser, buffer, err_stream);
  }
  disarm_signal_handler();

  if (err_stream) {
    fflush(err_stream);
  }
  fclose(gcode_stream);
  gcodep_delete(parser);

  parser->callbacks.gcode_finished(parse_events->user_data);

  return caught_signal ? 2 : 0;
}
