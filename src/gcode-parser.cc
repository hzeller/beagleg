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
//  - Rotational axes are not dealt with properly at this point. E.g.
//    degrees also get multiplied with imperial factors :)

#include "gcode-parser.h"

#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/select.h>
#include <sys/types.h>
#include <unistd.h>

#include "arc-gen.h"
#include "logging.h"
#include "string-util.h"

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

// We keep the implementation with all its unnecessary details for the user
// in this implementation.
class GCodeParser::Impl {
public:
  Impl(const GCodeParser::Config &config,
       GCodeParser::EventReceiver *parse_events, bool allow_m111);
  ~Impl();

  void ParseLine(GCodeParser *owner, const char *line, FILE *err_stream);
  int ParseStream(GCodeParser *owner, int input_fd, FILE *err_stream);
  const char *gcodep_parse_pair_with_linenumber(int line_num,
                                                const char *line,
                                                char *letter,
                                                float *value,
                                                FILE *err_stream);
private:
  enum DebugLevel {
    DEBUG_NONE   = 0,
    DEBUG_PARSER = (1 << 0)
  };

  // Reset parser, mostly reset relative coordinate systems to whatever they
  // should be in the beginning.
  // Does _not_ reset the machine position.
  void InitProgramDefaults() {
    unit_to_mm_factor_ = 1.0f;        // G21
    set_all_axis_to_absolute(true);   // G90
    reset_G92();                      // No global offset.
    set_current_offset(global_offset_g92_);

    arc_normal_ = AXIS_Z;  // Arcs in XY-plane

    // Some initial machine states emitted as events.
    callbacks->set_speed_factor(1.0);
    callbacks->set_fanspeed(0);
    callbacks->set_temperature(0);
  }

  void set_all_axis_to_absolute(bool value) {
    for (int i = 0; i < GCODE_NUM_AXES; ++i) {
      axis_is_absolute_[i] = value;
    }
  }

  void reset_G92() {
    global_offset_g92_ = kZeroOffset;
  }

  // The following methods set the origin and offset and also
  // inform the event receiver about this change.
  void set_current_offset(const AxesRegister &offset) {
    current_global_offset_ = &offset;
    inform_origin_offset_change();
  }
  // no need yet for set_current_origin().

  void inform_origin_offset_change() {
    AxesRegister visible_origin(*current_origin_);
    for (int i = 0; i < GCODE_NUM_AXES; ++i) {
      visible_origin[i] += current_global_offset()[i];
    }
    callbacks->inform_origin_offset(visible_origin);
  }

  void finish_program_and_reset() {
    callbacks->gcode_finished(false);

    InitProgramDefaults();
    program_in_progress_ = false;
  }

  const char *gparse_pair(const char *line, char *letter, float *value) {
    return gcodep_parse_pair_with_linenumber(line_number_, line,
                                             letter, value, err_msg_);
  }

  float abs_axis_pos(const enum GCodeParserAxis axis, const float unit_value) {
    float relative_to = ((axis_is_absolute_[axis])
                         ? current_origin()[axis]+current_global_offset()[axis]
                         : axes_pos_[axis]);
    return relative_to + unit_value;
  }

  const char *handle_home(const char *line);
  const char *handle_G10(const char *line);
  void change_coord_system(float sub_command);
  const char *handle_G92(float sub_command, const char *line);
  const char *handle_move(const char *line, bool force_change);
  const char *handle_arc(const char *line, bool is_cw);
  const char *handle_z_probe(const char *line);
  const char *handle_M111(const char *line);

  // Read parameter from letter "param_letter" and call the event callback
  // "ValueSetter" in the events callbacks. Pass the parameter, multiplied
  // with "factor", to the member function.
  typedef void (EventReceiver::*EventValueSetter)(float d);
  const char *set_param(char param_letter, EventValueSetter setter,
                        float factor, const char *line);

  const char *gcode_parse_parameter(int line_num, const char *line,
                                    int *param_num, float *param_val,
                                    bool query);

  // Read parameter. Do range check.
  bool read_parameter(int num, float *result) {
    if (num < 0 || num >= config.num_parameters) {
      gprintf(GLOG_SEMANTIC_ERR,
              "reading unsupported parameter number (%d)\n", num);
      return false;
    }
    *result = config.parameters[num];
    return true;
  }

  // Read parameter. Do range check.
  bool store_parameter(int num, float value) {
    // zero parameter can never be written.
    if (num <= 0 || num >= config.num_parameters) {
      gprintf(GLOG_SEMANTIC_ERR,
              "writing unsupported parameter number (%d)\n", num);
      return false;
    }
    config.parameters[num] = value;
    return true;
  }

  const AxesRegister &current_origin() const { return *current_origin_; }
  const AxesRegister &current_global_offset() const { return *current_global_offset_; }

  enum GCodePrintLevel {
    GLOG_INFO,
    GLOG_SYNTAX_ERR,
    GLOG_SEMANTIC_ERR,
  };
  void gprintf(enum GCodePrintLevel level, const char *format, ...);

  static AxesRegister kZeroOffset;

  GCodeParser::EventReceiver *const callbacks;
  const GCodeParser::Config config;

  bool program_in_progress_;

  FILE *err_msg_;
  int modal_g0_g1_;
  int line_number_;
  float unit_to_mm_factor_;               // metric: 1.0; imperial 25.4
  bool axis_is_absolute_[GCODE_NUM_AXES]; // G90 or G91 active.

  // The axes_pos is the current absolute position of the machine
  // in the work-cube. It always is positive in the range of
  // (0,0,0,...) -> (range_x, range_y, range_z,...)
  // This represents the current position of the machine.
  AxesRegister axes_pos_;

  // -- Origins
  // All the following coordinates are absolute positions. They
  // can be choosen as active origin.
  // The home position is not necessarily {0,0,0...}, but it is where
  // the end-switches are.
  const AxesRegister home_position_;
  // Coordinate systems 1-9 (G54 to G59.3) (zero-indexed)
  AxesRegister coord_system_[9];

  // -- Offsets
  AxesRegister global_offset_g92_;
  // ... + more; e.g. tool ofset ...

  // The current active origin is the machine absolute position with
  // respect to the machine cube. All GCode absolute positions are relative
  // to that absolute position. Just points to the actual register.
  const AxesRegister *current_origin_;         // active origin.

  // Active offsets from origin. They can be kNullOffset
  const AxesRegister *current_global_offset_;  // offset rel. to current origin.
  // more: tool offset.

  enum GCodeParserAxis arc_normal_;  // normal vector of arcs.

  unsigned int debug_level_;  // OR-ed bits from DebugLevel enum
  bool allow_m111_;
};

AxesRegister GCodeParser::Impl::kZeroOffset;

GCodeParser::Impl::Impl(const GCodeParser::Config &parse_config,
                        GCodeParser::EventReceiver *parse_events,
                        bool allow_m111)
  : callbacks(parse_events), config(parse_config),
    program_in_progress_(false),
    err_msg_(NULL), modal_g0_g1_(0),
    line_number_(0),
    unit_to_mm_factor_(1.0),  // G21
    // When we initialize the machine, we assume the axes to
    // be at the origin (but it better is G28-ed later)
    // TODO(hzeller): this might not be what we want. There are situations
    // in which we know the machine is in some other position (e.g. restarting
    // a job). So that needs to be more flexible ans possibly be part of the
    // incoming config.
    axes_pos_(config.machine_origin),
    home_position_(config.machine_origin),
    current_origin_(&home_position_), current_global_offset_(&kZeroOffset),
    arc_normal_(AXIS_Z),
    debug_level_(DEBUG_NONE), allow_m111_(allow_m111)
{
  assert(callbacks);  // otherwise, this is not very useful.
  reset_G92();
  InitProgramDefaults();
}

GCodeParser::Impl::~Impl() {
}

// gcode-printf. Prints message to stream or stderr.
// level
//   0    Information
//   1    G-Code Syntax Error with line number
//   2    G-Code Syntax Error
void GCodeParser::Impl::gprintf(enum GCodePrintLevel level,
                                const char *format, ...) {
  FILE *stream = err_msg_;
  if (stream == NULL) stream = stderr;
  switch (level) {
  case GLOG_INFO:
    fprintf(stream, "// ");
    break;
  case GLOG_SYNTAX_ERR:
    fprintf(stream, "// Line %d: G-Code Syntax Error: ", line_number_);
    break;
  case GLOG_SEMANTIC_ERR:
    fprintf(stream, "// Line %d: G-Code Error: ", line_number_);
    break;
  }
  va_list ap;
  va_start(ap, format);
  vfprintf(stream, format, ap);
  va_end(ap);
}

static const char *skip_white(const char *line) {
  while (*line && isspace(*line))
    line++;
  return line;
}

static float ParseGcodeNumber(const char *line, const char **endptr) {
  // We need to copy the number into a temporary buffer as strtof() does
  // not accept an end-limiter.
  char buffer[40];
  const char *src = line;
  char *dst = buffer;
  const char *end = buffer + sizeof(buffer) - 1;
  const char *extra_allowed = "+-.";
  while (*src && (isdigit(*src) || index(extra_allowed, *src)) && dst < end) {
    *dst++ = *src++;
  }
  *dst = '\0';
  char *parsed_end;
  const float result = strtof(buffer, &parsed_end);
  if (parsed_end == dst) {
    *endptr = src;
  } else {
    *endptr = line;
  }
  return result;
}

const char *GCodeParser::Impl::gcode_parse_parameter(
  int line_num, const char *line, int *param_num, float *param_val,
  bool query)
{
  *param_num = 0;
  *param_val = 0.0f;
  line = skip_white(line);
  if (*line == '\0') {
    gprintf(GLOG_SYNTAX_ERR, "expected value after '#'\n");
    return NULL;
  }

  bool indexed = false;
  if (*line == '#') {  // indexed parameter access
    line++;
    indexed = true;
  }

  const char *endptr;
  *param_num = (int)ParseGcodeNumber(line, &endptr);
  if (line == endptr) {
    gprintf(GLOG_SYNTAX_ERR,
            "'#' is not followed by a number but '%s'\n", line);
    return NULL;
  }
  if (indexed) {
    float val;
    if (!read_parameter(*param_num, &val))
      return NULL;
    *param_num = val;
  }
  if (*param_num <= 0 || *param_num > (int)config.num_parameters-1) {
    gprintf(GLOG_SYNTAX_ERR, "unsupported parameter number (%d)\n", *param_num);
    return NULL;
  }
  line = endptr;
  line = skip_white(line);

  if (*line == '=') {
    line++;
    float new_val = ParseGcodeNumber(line, &endptr);
    if (line == endptr) {
      gprintf(GLOG_SYNTAX_ERR,
              "'#%d=' is not followed by a number but '%s'\n",
              *param_num, line);
      return NULL;
    }
    line = endptr;
    store_parameter(*param_num, new_val);
    *param_val = new_val;
  } else {
    read_parameter(*param_num, param_val);
    if (query) {
      gprintf(GLOG_INFO, "%s%d%s = %f\n",
              indexed ? "[" : "", *param_num, indexed ? "]" : "", *param_val);
    }
  }
  line = skip_white(line); // Makes the line better to deal with.
  return line;  // We parsed something; return whatever is remaining.
}

// Parse next letter/number pair.
// Returns the remaining line or NULL if end reached.
const char *GCodeParser::Impl::gcodep_parse_pair_with_linenumber(
  int line_num, const char *line,
  char *letter, float *value,
  FILE *err_stream)
{
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

  if (*line == '#') {  // parameter set/get without a letter
    line++;
    int param_num;
    float param_val;
    const char *remaining_line = gcode_parse_parameter(line_num, line,
                                                       &param_num, &param_val,
                                                       true);

    // recursive call to parse the letter/number pair
    line = remaining_line;
    return gcodep_parse_pair_with_linenumber(line_num, line, letter, value, err_stream);
  }

  *letter = toupper(*line++);
  if (*line == '\0') {
    gprintf(GLOG_SYNTAX_ERR, "expected value after '%c'\n", *letter);
    return NULL;
  }
  // If this line has a checksum, we ignore it. In fact, the line is done.
  if (*letter == '*')
    return NULL;
  line = skip_white(line);

  const char *endptr;
  if (*line == '#') {  // parameter get with a letter
    line++;
    int param_num;
    endptr = gcode_parse_parameter(line_num, line, &param_num, value, false);
    if (endptr == NULL) {
      return NULL;   // got some error.
    }
    if (line == endptr) {
      gprintf(GLOG_SYNTAX_ERR,
              "Letter '%c' is not followed by a parameter number but '%s'\n",
              *letter, line);
      return NULL;
    }
  } else {
    *value = ParseGcodeNumber(line, &endptr);

    if (line == endptr) {
      gprintf(GLOG_SYNTAX_ERR,
              "Letter '%c' is not followed by a number but '%s'\n",
              *letter, line);
      return NULL;
    }
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
      axes_pos_[i] = home_position_[i];
    }
  }

  return line;
}

// Set coordinate system data
const char *GCodeParser::Impl::handle_G10(const char *line) {
  AxesRegister coords;
  int l_val = -1;
  int p_val = -1;
  bool have_val[GCODE_NUM_AXES];
  for (int i = 0; i < GCODE_NUM_AXES; ++i)
    have_val[i] = false;

  char letter;
  float value;
  const char *remaining_line;
  while ((remaining_line = gparse_pair(line, &letter, &value))) {
    if (letter == 'L') l_val = (int)value;
    else if (letter == 'P') p_val = (int)value;
    else {
      const enum GCodeParserAxis axis = gcodep_letter2axis(letter);
      const float unit_val = value * unit_to_mm_factor_;
      if (axis == GCODE_NUM_AXES)
        break;  //  Possibly start of new command.
      coords[axis] = unit_val;
      have_val[axis] = true;
    }
    line = remaining_line;
  }
  if (l_val == 2 && p_val >= 1 && p_val <= 9) {
    for (int i = 0; i < GCODE_NUM_AXES; ++i)
      if (!have_val[i]) coords[i] = coord_system_[p_val-1][i];
    coord_system_[p_val-1] = coords;
    // Now update the parameters
    int offset = (p_val-1) * 20;
    for (int i = 0; i < GCODE_NUM_AXES; ++i) {
      store_parameter(5221 + offset + i, coords[i]);
    }
  } else {
    gprintf(GLOG_SYNTAX_ERR, "handle_G10: invalid L or P value\n");
  }
  return line;
}

void GCodeParser::Impl::change_coord_system(float sub_command) {
  int coord_system = -1;
  switch ((int)sub_command) {
  case 54: coord_system = 1; break;
  case 55: coord_system = 2; break;
  case 56: coord_system = 3; break;
  case 57: coord_system = 4; break;
  case 58: coord_system = 5; break;
  case 59:
    if (sub_command == 59.0f) { coord_system = 6; break; }
    if (sub_command == 59.1f) { coord_system = 7; break; }
    if (sub_command == 59.2f) { coord_system = 8; break; }
    if (sub_command == 59.3f) { coord_system = 9; break; }
    // fallthru
  default:
    gprintf(GLOG_SYNTAX_ERR, "invalid coordinate system\n");
    return;
  }
  store_parameter(5220, coord_system);
  current_origin_ = &coord_system_[coord_system-1];
  inform_origin_offset_change();
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
      const float unit_val = value * unit_to_mm_factor_;
      const enum GCodeParserAxis axis = gcodep_letter2axis(axis_l);
      if (axis == GCODE_NUM_AXES)
        break;    // Possibly start of new command.
      // This sets the given value to be the new zero.
      global_offset_g92_[axis] = (axes_pos_[axis] - unit_val) -
        current_origin()[axis];

      line = remaining_line;
    }
    set_current_offset(global_offset_g92_);
  }
  else if (sub_command == 92.1f) {   // Reset
    reset_G92();
    set_current_offset(global_offset_g92_);
  }
  else if (sub_command == 92.2f) {   // Suspend
    set_current_offset(kZeroOffset);
  }
  else if (sub_command == 92.3f) {   // Restore
    set_current_offset(global_offset_g92_);
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

const char *GCodeParser::Impl::handle_move(const char *line, bool force_change) {
  char axis_l;
  float value;
  bool any_change = force_change;
  float feedrate = -1;
  const char *remaining_line;
  AxesRegister new_pos = axes_pos_;

  while ((remaining_line = gparse_pair(line, &axis_l, &value))) {
    const float unit_value = value * unit_to_mm_factor_;
    if (axis_l == 'F') {
      feedrate = f_param_to_feedrate(unit_value);
      any_change = true;
    }
    else {
      const enum GCodeParserAxis update_axis = gcodep_letter2axis(axis_l);
      if (update_axis == GCODE_NUM_AXES)
        break;  // Invalid axis: possibly start of new command.
      new_pos[update_axis] = abs_axis_pos(update_axis, unit_value);
      any_change = true;
    }
    line = remaining_line;
  }

  char did_move = 0;
  if (any_change) {
    if (modal_g0_g1_) {
      did_move = callbacks->coordinated_move(feedrate, new_pos);
    } else {
      did_move = callbacks->rapid_move(feedrate, new_pos);
    }
  }
  if (did_move) {
    axes_pos_ = new_pos;
  }
  return line;
}

struct ArcCallbackData {
  GCodeParser::EventReceiver *callbacks;
  float feedrate;
};

static void arc_callback(void *data, const AxesRegister &new_pos) {
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
const char *GCodeParser::Impl::handle_arc(const char *line, bool is_cw) {
  const char *remaining_line;
  AxesRegister target = axes_pos_;
  AxesRegister offset;
  float feedrate = -1;
  float value;
  char letter;
  int turns = 1;

  while ((remaining_line = gparse_pair(line, &letter, &value))) {
    const float unit_value = value * unit_to_mm_factor_;
    if (letter == 'X') target[AXIS_X] = abs_axis_pos(AXIS_X, unit_value);
    else if (letter == 'Y') target[AXIS_Y] = abs_axis_pos(AXIS_Y, unit_value);
    else if (letter == 'Z') target[AXIS_Z] = abs_axis_pos(AXIS_Z, unit_value);
    else if (letter == 'I') offset[AXIS_X] = unit_value;
    else if (letter == 'J') offset[AXIS_Y] = unit_value;
    else if (letter == 'K') offset[AXIS_Z] = unit_value;
    else if (letter == 'F') feedrate = f_param_to_feedrate(unit_value);
    else if (letter == 'P') turns = (int)value; // currently ignored
    // TODO: 'R'
    else break;

    line = remaining_line;
  }

  // Should the arc parameters be sanity checked?
  if (turns != 1) {
    // Currently, we ignore turns, so we check that it is exactly one.
    gprintf(GLOG_SYNTAX_ERR, "handle_arc: turns=%d (must be 1)\n", turns);
    return remaining_line;
  }

  struct ArcCallbackData cb_arc_data;
  cb_arc_data.callbacks = callbacks;
  cb_arc_data.feedrate = feedrate;
  arc_gen(arc_normal_, is_cw, &axes_pos_, offset, target,
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
    const float unit_value = value * unit_to_mm_factor_;
    if (letter == 'F') feedrate = f_param_to_feedrate(unit_value);
    else if (letter == 'Z') probe_thickness = value * unit_to_mm_factor_;
    else break;
    line = remaining_line;
  }
  // Probe for the travel endstop
  float probed_pos;
  if (callbacks->probe_axis(feedrate, AXIS_Z, &probed_pos)) {
    axes_pos_[AXIS_Z] = probed_pos;
    // Doing implicit G92 here. Is this what we want ? Later, this might
    // be part of tool-offset or something.
    global_offset_g92_[AXIS_Z] = (axes_pos_[AXIS_Z] - probe_thickness)
      - current_origin()[AXIS_Z];
    set_current_offset(global_offset_g92_);
  }
  return line;
}

const char *GCodeParser::Impl::handle_M111(const char *line) {
  if (allow_m111_) {
    int level = -1;
    char letter;
    float value;
    const char *remaining_line;
    // Read all the 'S' parameters. Well, we expect exactly one.
    while ((remaining_line = gparse_pair(line, &letter, &value))) {
      if (letter != 'S')
        break;  // possibly next command.
      level = (int)value;
      line = remaining_line;
    }
    if (level >= 0) debug_level_ = level;
  } else {
    line = NULL;  // consume the full line.
  }
  return line;
}

// Note: changes here should be documented in G-code.md as well.
void GCodeParser::Impl::ParseLine(GCodeParser *owner,
                                  const char *line, FILE *err_stream) {
  if (debug_level_ & DEBUG_PARSER) {
    Log_debug("GCodeParser| %s", line);
  }

  ++line_number_;
  err_msg_ = err_stream;  // remember as 'instance' variable.
  char letter;
  float value;
  while ((line = gparse_pair(line, &letter, &value))) {
    if (!program_in_progress_) {
      callbacks->gcode_start(owner);
      program_in_progress_ = true;
    }
    char processed_command = 1;
    if (letter == 'G') {
      switch ((int) value) {
      case  0: modal_g0_g1_ = 0; line = handle_move(line, false); break;
      case  1: modal_g0_g1_ = 1; line = handle_move(line, false); break;
      case  2: line = handle_arc(line, true); break;
      case  3: line = handle_arc(line, false); break;
      case  4: line = set_param('P', &GCodeParser::EventReceiver::dwell,
                                1.0f, line);
        break;
      case 10: line = handle_G10(line); break;
      case 17: arc_normal_ = AXIS_Z; break;
      case 18: arc_normal_ = AXIS_Y; break;
      case 19: arc_normal_ = AXIS_X; break;
      case 20: unit_to_mm_factor_ = 25.4f; break;
      case 21: unit_to_mm_factor_ = 1.0f; break;
      case 28: line = handle_home(line); break;
      case 30: line = handle_z_probe(line); break;
      case 54: case 55: case 56: case 57: case 58: case 59:
        change_coord_system(value);
        break;
      case 70: unit_to_mm_factor_ = 25.4f; break;
      case 71: unit_to_mm_factor_ = 1.0f; break;
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
      case 82: axis_is_absolute_[AXIS_E] = true; break;
      case 83: axis_is_absolute_[AXIS_E] = false; break;
      case 84: callbacks->motors_enable(false); break;
      case 104: line = set_param('S',
                                 &GCodeParser::EventReceiver::set_temperature,
                                 1.0f, line);
        break;
      case 106: line = set_param('S', &GCodeParser::EventReceiver::set_fanspeed,
                                 1.0f, line);
        break;
      case 107: callbacks->set_fanspeed(0); break;
      case 109:
        line = set_param('S', &GCodeParser::EventReceiver::set_temperature,
                         1.0f, line);
        callbacks->wait_temperature();
        break;
      case 116: callbacks->wait_temperature(); break;
      case 111: line = handle_M111(line); break;
      case 220:
        line = set_param('S', &GCodeParser::EventReceiver::set_speed_factor,
                         0.01f, line);
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
        const float unit_value = value * unit_to_mm_factor_;
        axes_pos_[axis] = abs_axis_pos(axis, unit_value);
        line = handle_move(line, true);
        // make gcode_command_done() think this was a 'G0/G1' command
        letter = 'G';
        value = modal_g0_g1_;
      }
    }
    if (processed_command) {
      callbacks->gcode_command_done(letter, value);
    }
  }
  err_msg_ = NULL;
}

// It is usually good to shut down gracefully, otherwise the PRU keeps running.
// So we're intercepting signals and exit gcode_machine_control_from_stream()
// cleanly.
static volatile bool caught_signal = false;
static void receive_signal(int signo) {
  static char msg[] = "Caught signal. Shutting down ASAP.\n";
  if (!caught_signal) {
    write(STDERR_FILENO, msg, sizeof(msg));
  }
  caught_signal = true;
}
static void arm_signal_handler() {
  caught_signal = false;
  struct sigaction sa = {};
  sa.sa_handler = receive_signal;
  sa.sa_flags = SA_RESETHAND;  // oneshot, no restart
  sigaction(SIGTERM, &sa, NULL);  // Regular kill
  sigaction(SIGINT, &sa, NULL);   // Ctrl-C

  // Other, internal problems that should never happen, but
  // can trigger multiple times before we gain back control
  // to shut down as cleanly as possible. These are not one-shot.
  sa.sa_flags = 0;
  sigaction(SIGSEGV, &sa, NULL);
  sigaction(SIGBUS, &sa, NULL);
  sigaction(SIGFPE, &sa, NULL);
}
static void disarm_signal_handler() {
  signal(SIGTERM, SIG_DFL);  // Regular kill
  signal(SIGINT, SIG_DFL);   // Ctrl-C
  signal(SIGSEGV, SIG_DFL);
  signal(SIGBUS, SIG_DFL);
  signal(SIGFPE, SIG_DFL);
}

// Public facade function.
int GCodeParser::Impl::ParseStream(GCodeParser *owner,
                                   int input_fd, FILE *err_stream) {
  if (err_stream) {
    // Output needs to be unbuffered, otherwise they'll never make it.
    setvbuf(err_stream, NULL, _IONBF, 0);
  }

  FILE *gcode_stream = fdopen(input_fd, "r");
  if (gcode_stream == NULL) {
    Log_error("While opening stream from fd %d: %s", input_fd, strerror(errno));
    return 1;
  }

  bool is_processing = true;
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
    if (select_ret < 0)  { // Broken stream.
      Log_error("select(): %s", strerror(errno));
      break;
    }

    if (select_ret == 0) {  // Timeout. Regularly call.
      callbacks->input_idle(is_processing);
      is_processing = false;
      continue;
    }

    is_processing = true;

    // Filedescriptor readable. Now wait for a line to finish.
    if (fgets(buffer, sizeof(buffer), gcode_stream) == NULL)
      break;

    ParseLine(owner, buffer, err_stream);
  }
  disarm_signal_handler();

  if (caught_signal)
    return 2;

  if (err_stream) {
    fflush(err_stream);
  }
  fclose(gcode_stream);

  // always call gcode_finished() to disable motors at end of stream
  callbacks->gcode_finished(true);

  return 0;
}

GCodeParser::GCodeParser(const Config &config, EventReceiver *parse_events,
                         bool allow_m111)
  : impl_(new Impl(config, parse_events, allow_m111)) {
}
GCodeParser::~GCodeParser() {
  delete impl_;
}
void GCodeParser::ParseLine(const char *line, FILE *err_stream) {
  impl_->ParseLine(this, line, err_stream);
}
int GCodeParser::ParseStream(int input_fd, FILE *err_stream) {
  return impl_->ParseStream(this, input_fd, err_stream);
}
const char *GCodeParser::ParsePair(const char *line,
                                   char *letter, float *value,
                                   FILE *err_stream) {
  return impl_->gcodep_parse_pair_with_linenumber(-1, line, letter, value,
                                                  err_stream);
}
