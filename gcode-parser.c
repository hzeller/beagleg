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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

typedef float AxesRegister[GCODE_NUM_AXES];

const unsigned char kAllAxesBitmap = 
  ((1 << AXIS_X) | (1 << AXIS_Y) | (1 << AXIS_Z)
   | (1 << AXIS_E) | (1 << AXIS_A) | (1 << AXIS_B) | (1 << AXIS_C));

struct GCodeParser {
  struct GCodeParserCb callbacks;
  void *cb_userdata;
  FILE *msg;
  int provided_axes;
  float unit_to_mm_factor;      // metric: 1.0; imperial 25.4
  char axis_is_absolute[GCODE_NUM_AXES];
  AxesRegister relative_zero;  // reference, set by G92 commands
  AxesRegister axes_pos;
};

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
static void dummy_go_home(void *user, unsigned char flags) {
  fprintf(stderr, "GCodeParser: go-home(0x%02x)\n", flags);
}
static const char *dummy_unprocessed(void *user, char letter, float value,
				     const char *remaining) {
  fprintf(stderr, "GCodeParser: unprocessed('%c', %d, '%s')\n",
	  letter, (int) value, remaining);
  return NULL;
}

static void set_all_axis_to_absolute(GCodeParser_t *p, char value) {
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    p->axis_is_absolute[i] = value;
  }
}

struct GCodeParser *gcodep_new(struct GCodeParserCb *callbacks,
			       void *userdata) {
  GCodeParser_t *result = (GCodeParser_t*)malloc(sizeof(*result));
  memset(result, 0x00, sizeof(*result));

  // Initial values for various constants.
  result->unit_to_mm_factor = 1.0f;
  set_all_axis_to_absolute(result, 1);

  // Setting up all callbacks
  if (callbacks) {
    memcpy(&result->callbacks, callbacks, sizeof(*callbacks));
  }
  result->cb_userdata = userdata;

  // Set some reasonable defaults for unprovided callbacks:
  if (!result->callbacks.go_home)
    result->callbacks.go_home = &dummy_go_home;
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

  return result;
}

void gcodep_delete(struct GCodeParser *parser) {
  free(parser);
}

static const char *skip_white(const char *line) {
  while (*line && isspace(*line))
    line++;
  return line;
}

// Parse next letter/number pair.
// Returns the remaining line or NULL if end reached.
const char *gcodep_parse_pair(const char *line, char *letter, float *value,
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
	    "// G-Code Syntax Error: expected value after '%c'\n", *letter);
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
    fprintf(err_stream ? err_stream : stderr, "// G-Code Syntax Error: "
	    "Letter '%c' is not followed by a number.\n", *letter);
    return NULL;
  }
  line = endptr;

  line = skip_white(line); // Makes the line better to deal with.
  return line;  // We parsed something; return whatever is remaining.
}

static const char *handle_home(struct GCodeParser *p, const char *line) {
  unsigned char homing_flags = 0;
  char axis;
  float dummy;
  const char *remaining_line;
  while ((remaining_line = gcodep_parse_pair(line, &axis, &dummy, p->msg))) {
    char done = 0;
    switch (axis) {
    case 'X': homing_flags |= (1 << AXIS_X); break;
    case 'Y': homing_flags |= (1 << AXIS_Y); break;
    case 'Z': homing_flags |= (1 << AXIS_Z); break;
    case 'E': homing_flags |= (1 << AXIS_E); break;
    case 'A': homing_flags |= (1 << AXIS_A); break;
    case 'B': homing_flags |= (1 << AXIS_B); break;
    case 'C': homing_flags |= (1 << AXIS_C); break;
    default:
      done = 1;  // Possibly start of new command.
      break;
    }
    if (done) break;
    line = remaining_line;
  }
  if (homing_flags == 0) homing_flags = kAllAxesBitmap;
  p->callbacks.go_home(p->cb_userdata, homing_flags);

  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (homing_flags & (1 << i)) {
      p->axes_pos[i] = 0;
      p->relative_zero[i] = 0;
    }
  }

  return line;
}

static const char *handle_rebase(struct GCodeParser *p, const char *line) {
  char axis;
  float value;
  const char *remaining_line;
  while ((remaining_line = gcodep_parse_pair(line, &axis, &value, p->msg))) {
    const float unit_value = value * p->unit_to_mm_factor;
    char done = 0;
    switch (axis) {
    case 'X': p->relative_zero[AXIS_X] = p->axes_pos[AXIS_X] - unit_value; break;
    case 'Y': p->relative_zero[AXIS_Y] = p->axes_pos[AXIS_Y] - unit_value; break;
    case 'Z': p->relative_zero[AXIS_Z] = p->axes_pos[AXIS_Z] - unit_value; break;
    case 'E': p->relative_zero[AXIS_E] = p->axes_pos[AXIS_E] - unit_value; break;
    case 'A': p->relative_zero[AXIS_A] = p->axes_pos[AXIS_A] - unit_value; break;
    case 'B': p->relative_zero[AXIS_B] = p->axes_pos[AXIS_B] - unit_value; break;
    case 'C': p->relative_zero[AXIS_C] = p->axes_pos[AXIS_C] - unit_value; break;
    default:
      done = 1;  // Possibly start of new command.
      break;
    }
    if (done)
      break;
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
  const char *remaining_line = gcodep_parse_pair(line, &letter, &value, p->msg);
  if (remaining_line != NULL && letter == param_letter) {
    value_setter(p->cb_userdata, factor * value);   // value on a user-callback
    return remaining_line;
  }
  return line;
}

static const char *handle_move(struct GCodeParser *p,
			       void (*fun_move)(void *, float, const float *),
			       const char *line) {
  char axis;
  float value;
  int any_change = 0;
  float feedrate = -1;
  const char *remaining_line;
  char done = 0;
  int update_axis = -1;
  while ((remaining_line = gcodep_parse_pair(line, &axis, &value, p->msg))) {
    const float unit_value = value * p->unit_to_mm_factor;
    switch (axis) {
    case 'F': feedrate = unit_value / 60.0; any_change = 1; break;
    case 'X': update_axis = AXIS_X; break;
    case 'Y': update_axis = AXIS_Y; break;
    case 'Z': update_axis = AXIS_Z; break;
    case 'E': update_axis = AXIS_E; break;
    case 'A': update_axis = AXIS_A; break;
    case 'B': update_axis = AXIS_B; break;
    case 'C': update_axis = AXIS_C; break;
    default:
      done = 1;  // Possibly start of new command.
      break;
    }

    if (update_axis >= 0) {
      if (p->axis_is_absolute[update_axis]) {
	p->axes_pos[update_axis] = p->relative_zero[update_axis] + unit_value;
      }
      else {
	p->axes_pos[update_axis] += unit_value;
      }
      any_change = 1;
    }
    if (done)
      break;
    line = remaining_line;
    update_axis = -1;
  }

  if (any_change) fun_move(p->cb_userdata, feedrate, p->axes_pos);
  return line;
}

void gcodep_parse_line(struct GCodeParser *p, const char *line,
		       FILE *err_stream) {
  void *const userdata = p->cb_userdata;
  struct GCodeParserCb *cb = &p->callbacks;
  p->msg = err_stream;  // remember as 'instance' variable.
  char letter;
  float value;
  while ((line = gcodep_parse_pair(line, &letter, &value, p->msg))) {
    if (letter == 'G') {
      switch ((int) value) {
      case 0: line = handle_move(p, cb->rapid_move, line); break;
      case 1: line = handle_move(p, cb->coordinated_move, line); break;
      case 4: line = set_param(p, 'P', cb->dwell, 1.0f, line); break;
      case 20: p->unit_to_mm_factor = 25.4f; break;
      case 21: p->unit_to_mm_factor = 1.0f; break;
      case 28: line = handle_home(p, line); break;
      case 90: set_all_axis_to_absolute(p, 1); break;
      case 91: set_all_axis_to_absolute(p, 0); break;
      case 92: line = handle_rebase(p, line); break;
      default: line = cb->unprocessed(userdata, letter, value, line); break;
      }
    }
    else if (letter == 'M') {
      switch ((int) value) {
      case 17: cb->motors_enable(userdata, 1); break;
      case 18: cb->motors_enable(userdata, 0); break;
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
    }
    else {
      line = cb->unprocessed(userdata, letter, value, line);
    }
  }
  p->msg = NULL;
}
