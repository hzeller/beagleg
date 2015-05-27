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

#include "arc-gen.h"

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
// The config contains the functions the parser calls on parsing.
// Returns an opaque type used in the parse function.
// Assumes the machine is in the machine home position.
// Does not take ownership of the provided pointer.
static GCodeParser_t *gcodep_new(struct GCodeParserConfig *config);
static void gcodep_delete(GCodeParser_t *object);  // Opposite of gcodep_new()

// Reset parser, mostly reset relative coordinate systems to whatever they
// should be in the beginning.
// Does _not_ reset the machine position.
static void gcodep_program_start_defaults(GCodeParser_t *object);

// Main workhorse: Parse a gcode line, call callbacks if needed.
// If "err_stream" is non-NULL, sends error messages that way.
static void gcodep_parse_line(GCodeParser_t *obj, const char *line,
                              FILE *err_stream);
// -- End public API

const AxisBitmap_t kAllAxesBitmap =
  ((1 << AXIS_X) | (1 << AXIS_Y) | (1 << AXIS_Z)| (1 << AXIS_E)
   | (1 << AXIS_A) | (1 << AXIS_B) | (1 << AXIS_C)
   | (1 << AXIS_U) | (1 << AXIS_V) | (1 << AXIS_W));

struct GCodeParser {
  struct GCodeParserCb callbacks;
  char program_in_progress;
  FILE *err_msg;
  int modal_g0_g1;
  int line_number;
  int provided_axes;
  float unit_to_mm_factor;               // metric: 1.0; imperial 25.4
  char axis_is_absolute[GCODE_NUM_AXES]; // G90 or G91 active.

  // The axes_pos is the current absolute position of the machine
  // in the work-cube. It always is positive in the range of
  // (0,0,0,...) -> (range_x, range_y, range_z,...)
  AxesRegister axes_pos;

  // All the following coordinates are absolute positions.
  AxesRegister origin_machine;             // homing position.
  AxesRegister origin_g92;

  // TODO: origin_g54, origin_g55 ... These must come in the configuration
  // as they can't be set

  // The current active origin is the machine absolute position
  // to which all coordinates given are relative.
  const float *current_origin;      // active origin.

  enum GCodeParserAxis arc_normal;  // normal vector of arcs.
};

static void dummy_gcode_start(void *user) {}
static void dummy_gcode_finished(void *user) {}
static void dummy_inform_display_offset(void *user, const float *o) {
  fprintf(stderr, "GCodeParser: display offset [");
  for (int i = 0; i < GCODE_NUM_AXES;  ++i) {
    fprintf(stderr, "%s%.3f", i == 0 ? "" : ", ", o[i]);
  }
  fprintf(stderr, "\n");
}

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
static char dummy_move(void *user, float feed, const float *axes) {
  fprintf(stderr, "GCodeParser: move(X=%.3f,Y=%.3f,Z=%.3f,E=%.3f,...);",
	  axes[AXIS_X], axes[AXIS_Y], axes[AXIS_Z], axes[AXIS_E]);
  if (feed > 0)
    fprintf(stderr, " feed=%.1f\n", feed);
  else
    fprintf(stderr, "\n");
  return 1;
}
static void dummy_go_home(void *user, AxisBitmap_t axes) {
  fprintf(stderr, "GCodeParser: go-home(0x%02x)\n", axes);
}
static char dummy_probe_axis(void *user, float feed, enum GCodeParserAxis axis,
                             float *reached_pos) {
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

static void set_current_origin(struct GCodeParser *p, float *origin) {
  p->current_origin = origin;
  p->callbacks.inform_origin_offset(p->callbacks.user_data, origin);
}

static struct GCodeParser *gcodep_new(struct GCodeParserConfig *config) {
  GCodeParser_t *result = (GCodeParser_t*)malloc(sizeof(*result));
  memset(result, 0x00, sizeof(*result));

  memcpy(result->origin_machine, config->machine_origin,
         sizeof(result->origin_machine));

  // Setting up all callbacks
  result->callbacks = config->callbacks;

  // Set some reasonable defaults for unprovided callbacks:
  if (!result->callbacks.gcode_start)
    result->callbacks.gcode_start = &dummy_gcode_start;
  if (!result->callbacks.gcode_finished)
    result->callbacks.gcode_finished = &dummy_gcode_finished;
  if (!result->callbacks.inform_origin_offset)
    result->callbacks.inform_origin_offset = &dummy_inform_display_offset;
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

  gcodep_program_start_defaults(result);

  return result;
}

static void gcodep_delete(struct GCodeParser *parser) {
  free(parser);
}

// Reset coordinate systems etc. that should be assumed at
// the beginnig of a program.
static void gcodep_program_start_defaults(GCodeParser_t *object) {
  // Initial values for various constants.
  object->unit_to_mm_factor = 1.0f;     // G21
  set_all_axis_to_absolute(object, 1);  // G90

  // Initially, G92 origin is as well where the machine is home.
  memcpy(object->origin_g92, object->origin_machine,
         sizeof(object->origin_g92));

  object->arc_normal = AXIS_Z;  // Arcs in XY-plane

  set_current_origin(object, object->origin_machine);
}

static void gcodep_finish_program_and_reset(GCodeParser_t *object) {
  void *const userdata = object->callbacks.user_data;
  object->callbacks.gcode_finished(userdata);

  gcodep_program_start_defaults(object);
  object->program_in_progress = 0;
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
  p->callbacks.go_home(p->callbacks.user_data, homing_flags);

  // Now update the world position
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (homing_flags & (1 << i)) {
      p->axes_pos[i] = p->origin_machine[i];
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
    // This sets the given value to be the new zero.
    p->origin_g92[axis] = p->axes_pos[axis] - unit_val;

    line = remaining_line;
  }
  set_current_origin(p, p->origin_g92);
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
  float pos = ((p->axis_is_absolute[axis])
               ? p->current_origin[axis]
               : p->axes_pos[axis]);
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
  AxesRegister new_pos;
  memcpy(new_pos, p->axes_pos, sizeof(new_pos));

  while ((remaining_line = gparse_pair(p, line, &axis_l, &value))) {
    const float unit_value = value * p->unit_to_mm_factor;
    if (axis_l == 'F') {
      feedrate = f_param_to_feedrate(unit_value);
      any_change = 1;
    }
    else {
      const enum GCodeParserAxis update_axis = gcodep_letter2axis(axis_l);
      if (update_axis == GCODE_NUM_AXES)
        break;  // Invalid axis: possibly start of new command.
      new_pos[update_axis] = abs_axis_pos(p, update_axis, unit_value);
      any_change = 1;
    }
    line = remaining_line;
  }

  char did_move = 0;
  if (any_change) {
    if (p->modal_g0_g1) {
      did_move = p->callbacks.coordinated_move(p->callbacks.user_data,
                                               feedrate, new_pos);
    } else {
      did_move = p->callbacks.rapid_move(p->callbacks.user_data,
                                         feedrate, new_pos);
    }
  }
  if (did_move) {
    memcpy(p->axes_pos, new_pos, sizeof(p->axes_pos));
  }
  return line;
}

struct ArcCallbackData {
  struct GCodeParser *parser;
  float feedrate;
};

static void arc_callback(void *data, float new_pos[]) {
  struct ArcCallbackData *cbinfo = (struct ArcCallbackData*) data;
  struct GCodeParser *p = cbinfo->parser;
  p->callbacks.coordinated_move(p->callbacks.user_data,
                                cbinfo->feedrate, new_pos);
}

// For we just generate an arc by emitting many small steps for
// now. TODO(hzeller): actually generate a curve profile for that.
// With G17, G18, G19, the plane was selected before.
// X, Y, Z: new position (two in the plane, one for the helix.
// I, J, K: offset to center, corresponding to X,Y,Z.
//          Only the two in the given plane are relevant.
// F      : Optional feedrate.
// P      : number of turns. currently ignored.
// R      : TODO: implement. Modern systems allow that.
static const char *handle_arc(struct GCodeParser *p,
			      const char *line,
			      int is_cw) {
  const char *remaining_line;
  float target[GCODE_NUM_AXES];
  float offset[GCODE_NUM_AXES] = {0};
  float feedrate = -1;
  float value;
  char letter;
  int turns = 1;

  memcpy(target, p->axes_pos, GCODE_NUM_AXES * sizeof(*target));

  while ((remaining_line = gparse_pair(p, line, &letter, &value))) {
    const float unit_value = value * p->unit_to_mm_factor;
    if (letter == 'X') target[AXIS_X] = abs_axis_pos(p, AXIS_X, unit_value);
    else if (letter == 'Y') target[AXIS_Y] = abs_axis_pos(p, AXIS_Y, unit_value);
    else if (letter == 'Z') target[AXIS_Z] = abs_axis_pos(p, AXIS_Z, unit_value);
    else if (letter == 'I') offset[AXIS_X] = unit_value;
    else if (letter == 'J') offset[AXIS_Y] = unit_value;
    else if (letter == 'K') offset[AXIS_Z] = unit_value;
    else if (letter == 'F') feedrate = f_param_to_feedrate(unit_value);
    else if (letter == 'P') turns = (int)value;	// currently ignored
    // TODO: 'R'
    else break;

    line = remaining_line;
  }

  // Should the arc parameters be sanity checked?
  if (turns < 0 || turns > 4) {
    fprintf(stderr, "G-Code Syntax Error: handle_arc: turns=%d (must be 1-4)\n",
            turns);
    return remaining_line;
  }

  struct ArcCallbackData cb_arc_data;
  cb_arc_data.parser = p;
  cb_arc_data.feedrate = feedrate;
  arc_gen(p->arc_normal, is_cw, p->axes_pos, offset, target,
          &arc_callback, &cb_arc_data);

  return line;
}

static const char *handle_z_probe(struct GCodeParser *p, const char *line) {
  char letter;
  float value;
  float feedrate = -1;
  float probe_thickness = 0;
  const char *remaining_line;
  while ((remaining_line = gparse_pair(p, line, &letter, &value))) {
    const float unit_value = value * p->unit_to_mm_factor;
    if (letter == 'F') feedrate = f_param_to_feedrate(unit_value);
    else if (letter == 'Z') probe_thickness = value * p->unit_to_mm_factor;
    else break;
    line = remaining_line;
  }
  // probe for the travel endstop
  float probed_pos;
  if (p->callbacks.probe_axis(p->callbacks.user_data, feedrate, AXIS_Z,
                              &probed_pos)) {
    p->axes_pos[AXIS_Z] = probed_pos;
    // Doing implicit G92 here. Is this what we want ?
    p->origin_g92[AXIS_Z] = probed_pos - probe_thickness;
    set_current_origin(p, p->origin_g92);
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
    if (!p->program_in_progress) {
      cb->gcode_start(userdata);
      p->program_in_progress = 1;
    }
    char processed_command = 1;
    if (letter == 'G') {
      switch ((int) value) {
      case  0: p->modal_g0_g1 = 0; line = handle_move(p, line, 0); break;
      case  1: p->modal_g0_g1 = 1; line = handle_move(p, line, 0); break;
      case  2: line = handle_arc(p, line, 1); break;
      case  3: line = handle_arc(p, line, 0); break;
      case  4: line = set_param(p, 'P', cb->dwell, 1.0f, line); break;
      case 17: p->arc_normal = AXIS_Z; break;
      case 18: p->arc_normal = AXIS_Y; break;
      case 19: p->arc_normal = AXIS_X; break;
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
      case  2: gcodep_finish_program_and_reset(p); break;
      case 17: cb->motors_enable(userdata, 1); break;
      case 18: cb->motors_enable(userdata, 0); break;
      case 30: gcodep_finish_program_and_reset(p); break;
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
int gcodep_parse_stream(struct GCodeParserConfig *config, int input_fd,
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
  wait_time.tv_sec = 0;
  wait_time.tv_usec = 50 * 1000;
  FD_ZERO(&read_fds);

  GCodeParser_t *parser = gcodep_new(config);
  arm_signal_handler();
  char buffer[8192];
  while (!caught_signal) {
    // Read with timeout. If we don't get anything on our input, but it
    // is not finished yet, we tell our
    FD_SET(input_fd, &read_fds);
    // When we're already in idle mode, we're not interested in change.
    wait_time.tv_usec = 50 * 1000;
    select_ret = select(input_fd + 1, &read_fds, NULL, NULL, &wait_time);
    if (select_ret < 0)  // Broken stream.
      break;

    if (select_ret == 0) {  // Timeout. Regularly call.
      parser->callbacks.input_idle(config->callbacks.user_data);
      continue;
    }

    // Filedescriptor readable. Now wait for a line to finish.
    if (fgets(buffer, sizeof(buffer), gcode_stream) == NULL)
      break;
    gcodep_parse_line(parser, buffer, err_stream);
  }
  disarm_signal_handler();

  if (err_stream) {
    fflush(err_stream);
  }
  fclose(gcode_stream);

  if (parser->program_in_progress) {
    parser->callbacks.gcode_finished(config->callbacks.user_data);
  }

  gcodep_delete(parser);

  return caught_signal ? 2 : 0;
}
