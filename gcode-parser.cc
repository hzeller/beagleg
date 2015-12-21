/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
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

// TODO: when passing 'remaining', clean it from \n
//  - M117 doesn't come with extra newline
//  - M888 P0 (example non-existent M code) would be warn-printed nicely.

#include "gcode-parser.h"

#include "arc-gen.h"

#include <assert.h>  // remove.

#include <ctype.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <sys/types.h>
#include <unistd.h>

static const char *gcodep_parse_pair_with_linenumber(int line_num,
                                                     const char *line,
                                                     char *letter, float *value,
                                                     FILE *err_stream);

const AxisBitmap_t kAllAxesBitmap =
  ((1 << AXIS_X) | (1 << AXIS_Y) | (1 << AXIS_Z)| (1 << AXIS_E)
   | (1 << AXIS_A) | (1 << AXIS_B) | (1 << AXIS_C)
   | (1 << AXIS_U) | (1 << AXIS_V) | (1 << AXIS_W));

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

GCodeParser::Config::Config() {
  bzero(machine_origin, sizeof(machine_origin));
}

// -- default implementation of parse events.
void GCodeParser::Events::gcode_start() {}
void GCodeParser::Events::gcode_finished() {}
void GCodeParser::Events::wait_for_start() {}
void GCodeParser::Events::inform_origin_offset(const float *o) {
  fprintf(stderr, "GCodeParser: (display) origin offset [");
  for (int i = 0; i < GCODE_NUM_AXES;  ++i) {
    fprintf(stderr, "%s%c:%.3f", i == 0 ? "" : ", ",
            gcodep_axis2letter((enum GCodeParserAxis)i),o[i]);
  }
  fprintf(stderr, "]\n");
}

void GCodeParser::Events::gcode_command_done(char letter, float val) {}

void GCodeParser::Events::set_speed_factor(float f) {
  fprintf(stderr, "GCodeParser: set_speed_factor(%.1f)\n", f);
}
void GCodeParser::Events::set_temperature(float f) {
  fprintf(stderr, "GCodeParser: set_temperature(%.1f)\n", f);
}
void GCodeParser::Events::set_fanspeed(float speed) {
  fprintf(stderr, "GCodeParser: set_fanspeed(%.0f)\n", speed);
}
void GCodeParser::Events::wait_temperature() {
  fprintf(stderr, "GCodeParser: wait_temperature()\n");
}
void GCodeParser::Events::dwell(float f) {
  fprintf(stderr, "GCodeParser: dwell(%.1f)\n", f);
}
void GCodeParser::Events::motors_enable(bool b) {
  fprintf(stderr, "GCodeParser: %s motors\n", b ? "enable" : "disable");
}
bool GCodeParser::Events::coordinated_move(float feed, const float *axes) {
  fprintf(stderr, "GCodeParser: move(X=%.3f,Y=%.3f,Z=%.3f,E=%.3f,...);",
	  axes[AXIS_X], axes[AXIS_Y], axes[AXIS_Z], axes[AXIS_E]);
  if (feed > 0)
    fprintf(stderr, " feed=%.1f\n", feed);
  else
    fprintf(stderr, "\n");
  return 1;
}
bool GCodeParser::Events::rapid_move(float feed, const float *axes) {
  return coordinated_move(feed, axes);
}

void GCodeParser::Events::go_home(AxisBitmap_t axes) {
  fprintf(stderr, "GCodeParser: go-home(0x%02x)\n", axes);
}
bool GCodeParser::Events::probe_axis(float feed, enum GCodeParserAxis axis,
                                     float *reached_pos) {
  fprintf(stderr, "GCodeParser: probe-axis(%c)", gcodep_axis2letter(axis));
  if (feed > 0)
    fprintf(stderr, " feed=%.1f\n", feed);
  else
    fprintf(stderr, "\n");
  return false;
}
const char *GCodeParser::Events::unprocessed( char letter, float value,
                                             const char *remaining) {
  fprintf(stderr, "GCodeParser: unprocessed('%c', %d, '%s')\n",
	  letter, (int) value, remaining);
  return NULL;
}
void GCodeParser::Events::input_idle() {
  fprintf(stderr, "GCodeParser: input idle\n");
}


// We keep the implementation with all its unnecessary details for the user
// in this implementation.
class GCodeParser::Impl {
public:
  Impl(const GCodeParser::Config &config, GCodeParser::Events *parse_events);

  void ParseLine(const char *line, FILE *err_stream);
  int ParseStream(int input_fd, FILE *err_stream);

private:
  // Reset parser, mostly reset relative coordinate systems to whatever they
  // should be in the beginning.
  // Does _not_ reset the machine position.
  void InitProgramDefaults() {
    unit_to_mm_factor = 1.0f;        // G21
    set_all_axis_to_absolute(true);  // G90
    reset_G92();                     // No global offset.

    arc_normal = AXIS_Z;  // Arcs in XY-plane

    set_current_origin(origin_machine, global_offset_g92);

    // Some initial machine states emitted as events.
    callbacks->set_speed_factor(1.0);
    callbacks->set_fanspeed(0);
    callbacks->set_temperature(0);
  }

  void set_all_axis_to_absolute(bool value) {
    for (int i = 0; i < GCODE_NUM_AXES; ++i) {
      axis_is_absolute[i] = value;
    }
  }

  void reset_G92() {
    bzero(global_offset_g92, sizeof(global_offset_g92));
  }

  void set_current_origin(const float *origin, const float *offset) {
    current_origin = origin;
    current_global_offset = offset;
    AxesRegister visible_origin;
    memcpy(visible_origin, origin, sizeof(visible_origin));
    if (current_global_offset) {
      for (int i = 0; i < GCODE_NUM_AXES; ++i) {
        visible_origin[i] += current_global_offset[i];
      }
    }
    callbacks->inform_origin_offset(visible_origin);
  }

  void finish_program_and_reset() {
    callbacks->gcode_finished();

    InitProgramDefaults();
    program_in_progress = false;
  }

  const char *gparse_pair(const char *line, char *letter, float *value) {
    return gcodep_parse_pair_with_linenumber(line_number, line,
                                             letter, value, err_msg);
  }

  float abs_axis_pos(const enum GCodeParserAxis axis, const float unit_value) {
    float relative_to = ((axis_is_absolute[axis])
                         ? current_origin[axis]
                         : axes_pos[axis]);
    float offset = current_global_offset ? current_global_offset[axis] : 0.0f;
    return relative_to + unit_value + offset;
  }

  const char *handle_home(const char *line);
  const char *handle_G92(float sub_command, const char *line);
  const char *handle_move(const char *line, int force_change);
  const char *handle_arc(const char *line, int is_cw);
  const char *handle_z_probe(const char *line);

  // Read parameter from letter "param_letter" and call the event callback
  // "ValueSetter" in the events callbacks. Pass the parameter, multiplied
  // with "factor", to the member function.
  typedef void (Events::*EventValueSetter)(float d);
  const char *set_param(char param_letter, EventValueSetter setter,
                        float factor, const char *line);

  GCodeParser::Events *const callbacks;
  const GCodeParser::Config config;

  bool program_in_progress;

  FILE *err_msg;
  int modal_g0_g1;
  int line_number;
  int provided_axes;
  float unit_to_mm_factor;               // metric: 1.0; imperial 25.4
  bool axis_is_absolute[GCODE_NUM_AXES]; // G90 or G91 active.

  // The axes_pos is the current absolute position of the machine
  // in the work-cube. It always is positive in the range of
  // (0,0,0,...) -> (range_x, range_y, range_z,...)
  AxesRegister axes_pos;

  // All the following coordinates are absolute positions. They
  // can be choosen as active origin.
  AxesRegister origin_machine;     // homing position.
  // ... + other origins.

  AxesRegister global_offset_g92;
  // ... tool ofset ...

  // The current active origin is the machine absolute position with
  // respect to the machine cube. All GCode absolute positions are relative
  // to that absolute position.
  const float *current_origin;         // active origin.

  // Active offsets from origin. They can be NULL if not active.
  const float *current_global_offset;  // offset relative to current origin.
  // more: tool offset.

  enum GCodeParserAxis arc_normal;  // normal vector of arcs.
};

GCodeParser::Impl::Impl(const GCodeParser::Config &parse_config,
                        GCodeParser::Events *parse_events)
  : callbacks(parse_events), config(parse_config),
    program_in_progress(false),
    err_msg(NULL), modal_g0_g1(0),
    line_number(0), provided_axes(0),
    unit_to_mm_factor(1.0),  // G21
    current_origin(NULL), current_global_offset(NULL),
    arc_normal(AXIS_Z)
{
  assert(callbacks);  // otherwise, this is not very useful.

  reset_G92();

  // This is the only time we set the const value. Cast.
  // TODO(hzeller): get from config.
  memcpy((float*) origin_machine, config.machine_origin, sizeof(origin_machine));

  // When we initialize the machine, we assume the axes to
  // be at the origin (but it better is G28-ed later)
  memcpy(axes_pos, config.machine_origin, sizeof(axes_pos));

  InitProgramDefaults();
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

const char *GCodeParser::Impl::handle_home(const char *line) {
  AxisBitmap_t homing_flags = 0;
  char axis_l;
  float dummy;
  const char *remaining_line;
  while ((remaining_line = gparse_pair(line, &axis_l, &dummy))) {
    const enum GCodeParserAxis axis = gcodep_letter2axis(axis_l);
    if (axis == GCODE_NUM_AXES)
      break;  //  Possibly start of new command.
    homing_flags |= (1 << axis);
    line = remaining_line;
  }
  if (homing_flags == 0) homing_flags = kAllAxesBitmap;
  callbacks->go_home(homing_flags);

  // Now update the world position
  for (int i = 0; i < GCODE_NUM_AXES; ++i) {
    if (homing_flags & (1 << i)) {
      axes_pos[i] = origin_machine[i];
    }
  }

  return line;
}

// Set relative coordinate system
const char *GCodeParser::Impl::handle_G92(float sub_command, const char *line) {
  // It is safe to compare raw float values here, as long as we give float
  // literals. They have been parsed from literals as well.
  if (sub_command == 92.0f) {
    char axis_l;
    float value;
    const char *remaining_line;
    while ((remaining_line = gparse_pair(line, &axis_l, &value))) {
      const float unit_val = value * unit_to_mm_factor;
      const enum GCodeParserAxis axis = gcodep_letter2axis(axis_l);
      if (axis == GCODE_NUM_AXES)
        break;    // Possibly start of new command.
      // This sets the given value to be the new zero.
      global_offset_g92[axis] = (axes_pos[axis] - unit_val) -
        current_origin[axis];

      line = remaining_line;
    }
    set_current_origin(current_origin, global_offset_g92);
  }
  else if (sub_command == 92.1f) {   // Reset
    reset_G92();
    set_current_origin(current_origin, global_offset_g92);
  }
  else if (sub_command == 92.2f) {   // Suspend
    set_current_origin(current_origin, NULL);
  }
  else if (sub_command == 92.3f) {   // Restore
    set_current_origin(current_origin, global_offset_g92);
  }
  return line;
}

// Set a parameter on a callback that takes exacly one float.
const char *GCodeParser::Impl::set_param(char param_letter,
                                         EventValueSetter setter,
                                         float factor, const char *line) {
  char letter;
  float value;
  const char *remaining_line = gparse_pair(line, &letter, &value);
  if (remaining_line != NULL && letter == param_letter) {
    (callbacks->*setter)(factor * value);   // TODO
    return remaining_line;
  }
  return line;
}

static float f_param_to_feedrate(const float unit_value) {
  return unit_value / 60.0f;  // feedrates are units per minute.
}

const char *GCodeParser::Impl::handle_move(const char *line, int force_change) {
  char axis_l;
  float value;
  int any_change = force_change;
  float feedrate = -1;
  const char *remaining_line;
  AxesRegister new_pos;
  memcpy(new_pos, axes_pos, sizeof(new_pos));

  while ((remaining_line = gparse_pair(line, &axis_l, &value))) {
    const float unit_value = value * unit_to_mm_factor;
    if (axis_l == 'F') {
      feedrate = f_param_to_feedrate(unit_value);
      any_change = 1;
    }
    else {
      const enum GCodeParserAxis update_axis = gcodep_letter2axis(axis_l);
      if (update_axis == GCODE_NUM_AXES)
        break;  // Invalid axis: possibly start of new command.
      new_pos[update_axis] = abs_axis_pos(update_axis, unit_value);
      any_change = 1;
    }
    line = remaining_line;
  }

  char did_move = 0;
  if (any_change) {
    if (modal_g0_g1) {
      did_move = callbacks->coordinated_move(feedrate, new_pos);
    } else {
      did_move = callbacks->rapid_move(feedrate, new_pos);
    }
  }
  if (did_move) {
    memcpy(axes_pos, new_pos, sizeof(axes_pos));
  }
  return line;
}

struct ArcCallbackData {
  GCodeParser::Events *callbacks;
  float feedrate;
};

static void arc_callback(void *data, float new_pos[]) {
  struct ArcCallbackData *cbinfo = (struct ArcCallbackData*) data;
  cbinfo->callbacks->coordinated_move(cbinfo->feedrate, new_pos);
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
const char *GCodeParser::Impl::handle_arc(const char *line, int is_cw) {
  const char *remaining_line;
  float target[GCODE_NUM_AXES];
  float offset[GCODE_NUM_AXES] = {0};
  float feedrate = -1;
  float value;
  char letter;
  int turns = 1;

  memcpy(target, axes_pos, GCODE_NUM_AXES * sizeof(*target));

  while ((remaining_line = gparse_pair(line, &letter, &value))) {
    const float unit_value = value * unit_to_mm_factor;
    if (letter == 'X') target[AXIS_X] = abs_axis_pos(AXIS_X, unit_value);
    else if (letter == 'Y') target[AXIS_Y] = abs_axis_pos(AXIS_Y, unit_value);
    else if (letter == 'Z') target[AXIS_Z] = abs_axis_pos(AXIS_Z, unit_value);
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
  cb_arc_data.callbacks = callbacks;
  cb_arc_data.feedrate = feedrate;
  arc_gen(arc_normal, is_cw, axes_pos, offset, target,
          &arc_callback, &cb_arc_data);

  return line;
}

const char *GCodeParser::Impl::handle_z_probe(const char *line) {
  char letter;
  float value;
  float feedrate = -1;
  float probe_thickness = 0;
  const char *remaining_line;
  while ((remaining_line = gparse_pair(line, &letter, &value))) {
    const float unit_value = value * unit_to_mm_factor;
    if (letter == 'F') feedrate = f_param_to_feedrate(unit_value);
    else if (letter == 'Z') probe_thickness = value * unit_to_mm_factor;
    else break;
    line = remaining_line;
  }
  // Probe for the travel endstop
  float probed_pos;
  if (callbacks->probe_axis(feedrate, AXIS_Z, &probed_pos)) {
    axes_pos[AXIS_Z] = probed_pos;
    // Doing implicit G92 here. Is this what we want ? Later, this might
    // be part of tool-offset or something.
    global_offset_g92[AXIS_Z] = (axes_pos[AXIS_Z] - probe_thickness)
      - current_origin[AXIS_Z];
    set_current_origin(current_origin, global_offset_g92);
  }
  return line;
}

// Note: changes here should be documented in G-code.md as well.
void GCodeParser::Impl::ParseLine(const char *line, FILE *err_stream) {
  ++line_number;
  err_msg = err_stream;  // remember as 'instance' variable.
  char letter;
  float value;
  while ((line = gparse_pair(line, &letter, &value))) {
    if (!program_in_progress) {
      callbacks->gcode_start();
      program_in_progress = true;
    }
    char processed_command = 1;
    if (letter == 'G') {
      switch ((int) value) {
      case  0: modal_g0_g1 = 0; line = handle_move(line, 0); break;
      case  1: modal_g0_g1 = 1; line = handle_move(line, 0); break;
      case  2: line = handle_arc(line, 1); break;
      case  3: line = handle_arc(line, 0); break;
      case  4: line = set_param('P', &GCodeParser::Events::dwell, 1.0f, line); break;
      case 17: arc_normal = AXIS_Z; break;
      case 18: arc_normal = AXIS_Y; break;
      case 19: arc_normal = AXIS_X; break;
      case 20: unit_to_mm_factor = 25.4f; break;
      case 21: unit_to_mm_factor = 1.0f; break;
      case 28: line = handle_home(line); break;
      case 30: line = handle_z_probe(line); break;
      case 70: unit_to_mm_factor = 25.4f; break;
      case 71: unit_to_mm_factor = 1.0f; break;
      case 90: set_all_axis_to_absolute(true); break;
      case 91: set_all_axis_to_absolute(false); break;
      case 92: line = handle_G92(value, line); break;
      default: line = callbacks->unprocessed(letter, value, line); break;
      }
    }
    else if (letter == 'M') {
      switch ((int) value) {
      case  2: finish_program_and_reset(); break;
      case 17: callbacks->motors_enable(true); break;
      case 18: callbacks->motors_enable(false); break;
      case 24: callbacks->wait_for_start(); break;
      case 30: finish_program_and_reset(); break;
      case 82: axis_is_absolute[AXIS_E] = true; break;
      case 83: axis_is_absolute[AXIS_E] = false; break;
      case 84: callbacks->motors_enable(false); break;
      case 104: line = set_param('S', &GCodeParser::Events::set_temperature, 1.0f, line); break;
      case 106: line = set_param('S', &GCodeParser::Events::set_fanspeed, 1.0f, line); break;
      case 107: callbacks->set_fanspeed(0); break;
      case 109:
	line = set_param('S', &GCodeParser::Events::set_temperature, 1.0f, line);
	callbacks->wait_temperature();
	break;
      case 116: callbacks->wait_temperature(); break;
      case 220:
	line = set_param('S', &GCodeParser::Events::set_speed_factor, 0.01f, line);
	break;
      default: line = callbacks->unprocessed(letter, value, line); break;
      }
    }
    else if (letter == 'N') {
      // Line number? Yeah, ignore for now :)
      processed_command = 0;
    }
    else {
      const enum GCodeParserAxis axis = gcodep_letter2axis(letter);
      if (axis == GCODE_NUM_AXES) {
        line = callbacks->unprocessed(letter, value, line);
      } else {
        // This line must be a continuation of a previous G0/G1 command.
        // Update the axis position then handle the move.
        const float unit_value = value * unit_to_mm_factor;
        axes_pos[axis] = abs_axis_pos(axis, unit_value);
	line = handle_move(line, 1);
	// make gcode_command_done() think this was a 'G0/G1' command
	letter = 'G';
	value = modal_g0_g1;
      }
    }
    if (processed_command) {
      callbacks->gcode_command_done(letter, value);
    }
  }
  err_msg = NULL;
}

// It is usually good to shut down gracefully, otherwise the PRU keeps running.
// So we're intercepting signals and exit gcode_machine_control_from_stream()
// cleanly.
static volatile char caught_signal = 0;
static void receive_signal(int signo) {
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
int GCodeParser::Impl::ParseStream(int input_fd, FILE *err_stream) {
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
      callbacks->input_idle();
      continue;
    }

    // Filedescriptor readable. Now wait for a line to finish.
    if (fgets(buffer, sizeof(buffer), gcode_stream) == NULL)
      break;
    ParseLine(buffer, err_stream);
  }
  disarm_signal_handler();

  if (err_stream) {
    fflush(err_stream);
  }
  fclose(gcode_stream);

  if (program_in_progress) {
    callbacks->gcode_finished();
  }

  return caught_signal ? 2 : 0;
}

GCodeParser::GCodeParser(const Config &config, Events *parse_events)
  : impl_(new Impl(config, parse_events)) {
}
GCodeParser::~GCodeParser() {
  delete impl_;
}
void GCodeParser::ParseLine(const char *line, FILE *err_stream) {
  impl_->ParseLine(line, err_stream);
}
int GCodeParser::ParseStream(int input_fd, FILE *err_stream) {
  return impl_->ParseStream(input_fd, err_stream);
}
const char *GCodeParser::ParsePair(const char *line,
                                   char *letter, float *value,
                                   FILE *err_stream) {
  return gcodep_parse_pair_with_linenumber(-1, line, letter, value, err_stream);
}
