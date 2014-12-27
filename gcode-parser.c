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

#include <ctype.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

//-- This is the more detailed public API. If it ever is needed (e.g.
//   to keep track of multiple streams at once, put in gcode-parser.h.
//   Till then, gcode_parse_stream is sufficient.
typedef struct GCodeParser GCodeParser_t;  // Opaque parser object type.

// Initialize parser.
// The "callbacks"-struct contains the functions the parser calls on parsing.
// Returns an opaque type used in the parse function.
// Does not take ownership of the provided pointer.
GCodeParser_t *gcodep_new(struct GCodeParserCb *callbacks);

void gcodep_delete(GCodeParser_t *object);  // Opposite of gcodep_new()

// Main workhorse: Parse a gcode line, call callbacks if needed.
// If "err_stream" is non-NULL, sends error messages that way.
void gcodep_parse_line(GCodeParser_t *obj, const char *line, FILE *err_stream);
// -- End public API

typedef float AxesRegister[GCODE_NUM_AXES];

const AxisBitmap_t kAllAxesBitmap =
  ((1 << AXIS_X) | (1 << AXIS_Y) | (1 << AXIS_Z)| (1 << AXIS_E)
   | (1 << AXIS_A) | (1 << AXIS_B) | (1 << AXIS_C)
   | (1 << AXIS_U) | (1 << AXIS_V) | (1 << AXIS_W));

struct GCodeParser {
  struct GCodeParserCb callbacks;
  FILE *msg;
  int line_number;
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
static void dummy_go_home(void *user, AxisBitmap_t axes) {
  fprintf(stderr, "GCodeParser: go-home(0x%02x)\n", axes);
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

struct GCodeParser *gcodep_new(struct GCodeParserCb *callbacks) {
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
                                           letter, value, p->msg);
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

  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (homing_flags & (1 << i)) {
      p->axes_pos[i] = 0;
      p->relative_zero[i] = 0;
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

static const char *handle_move(struct GCodeParser *p,
			       void (*fun_move)(void *, float, const float *),
			       const char *line) {
  char axis_l;
  float value;
  int any_change = 0;
  float feedrate = -1;
  const char *remaining_line;
  while ((remaining_line = gparse_pair(p, line, &axis_l, &value))) {
    const float unit_value = value * p->unit_to_mm_factor;
    if (axis_l == 'F') {
      feedrate = unit_value / 60.0;  // feedrates are per minute.
      any_change = 1;
    }
    else {
      const enum GCodeParserAxis update_axis = gcodep_letter2axis(axis_l);
      if (update_axis == GCODE_NUM_AXES)
        break;  // Invalid axis: possibley start of new command.
      if (p->axis_is_absolute[update_axis]) {
        p->axes_pos[update_axis] = p->relative_zero[update_axis] + unit_value;
      } else {
        p->axes_pos[update_axis] += unit_value;
      }
      any_change = 1;
    }
    line = remaining_line;
  }

  if (any_change) fun_move(p->callbacks.user_data, feedrate, p->axes_pos);
  return line;
}

// Note: changes here should be documented in G-code.md as well.
void gcodep_parse_line(struct GCodeParser *p, const char *line,
		       FILE *err_stream) {
  ++p->line_number;
  void *const userdata = p->callbacks.user_data;
  struct GCodeParserCb *cb = &p->callbacks;
  p->msg = err_stream;  // remember as 'instance' variable.
  char letter;
  float value;
  while ((line = gparse_pair(p, line, &letter, &value))) {
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
                        struct GCodeParserCb *parse_events, FILE *err_stream) {
  if (err_stream) {
    // Output needs to be unbuffered, otherwise they'll never make it.
    setvbuf(err_stream, NULL, _IONBF, 0);
  }

  FILE *gcode_stream = fdopen(input_fd, "r");
  if (gcode_stream == NULL) {
    perror("Opening gcode stream");
    return 1;
  }

  GCodeParser_t *parser = gcodep_new(parse_events);
  arm_signal_handler();
  char buffer[8192];
  while (!caught_signal && fgets(buffer, sizeof(buffer), gcode_stream)) {
    gcodep_parse_line(parser, buffer, err_stream);
  }
  disarm_signal_handler();

  if (err_stream) {
    fflush(err_stream);
  }
  fclose(gcode_stream);
  gcodep_delete(parser);
  
  return caught_signal ? 2 : 0;
}
