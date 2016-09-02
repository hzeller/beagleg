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
  int error_count() const { return error_count_; }

private:
  enum DebugLevel {
    DEBUG_NONE        = 0,
    DEBUG_PARSER      = (1 << 0),
    DEBUG_EXPRESSION  = (1 << 1)
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

    last_spline_cp2_ = kZeroOffset;
    have_first_spline_ = false;

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

  enum Operation {
    NO_OPERATION = 0,

    // binary operations
    POWER,                                              // precedence 4
    DIVIDED_BY, MODULO, TIMES,                          // precedence 3
    AND2, EXCLUSIVE_OR, MINUS, NON_EXCLUSIVE_OR, PLUS,  // precedence 2
    RIGHT_BRACKET,                                      // precedence 1

    // unary operations
    ABS, ACOS, ASIN, ATAN, COS, EXP, FIX, FUP, LN, ROUND, SIN, SQRT, TAN
  };

  const char *op_string(Operation op) {
    switch (op) {
    default:
    case NO_OPERATION:      return "?";
    case POWER:             return "**";
    case DIVIDED_BY:        return "/";
    case MODULO:            return "MOD";
    case TIMES:             return "*";
    case AND2:              return "AND";
    case EXCLUSIVE_OR:      return "XOR";
    case MINUS:             return "-";
    case NON_EXCLUSIVE_OR:  return "OR";
    case PLUS:              return "+";
    case RIGHT_BRACKET:     return "]";
    case ABS:               return "ABS";
    case ACOS:              return "ACOS";
    case ASIN:              return "ASIN";
    case ATAN:              return "ATAN";
    case COS:               return "COS";
    case EXP:               return "EXP";
    case FIX:               return "FIX";
    case FUP:               return "FUP";
    case LN:                return "LN";
    case ROUND:             return "ROUND";
    case SIN:               return "SIN";
    case SQRT:              return "SQRT";
    case TAN:               return "TAN";
    }
  }

  int precedence(Operation op) {
    if (op == RIGHT_BRACKET) return 1;
    if (op == POWER) return 4;
    if (op >= AND2) return 2;
    return 3;
  }

  bool execute_unary(float *value, Operation op);
  const char *gcodep_atan(const char *line, float *value);
  const char *gcodep_operation_unary(const char *line, Operation *op);
  const char *gcodep_unary(const char *line, float *value);

  bool execute_binary(float *left, Operation op, float *right);
  const char *gcodep_operation(const char *line, Operation *op);

  const char *gcodep_expression(const char *line, float *value);

  const char *gcodep_value(const char *line, float *value);

  const char *gcodep_set_parameter(const char *line);

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
  const char *handle_spline(float sub_command, const char *line);
  const char *handle_z_probe(const char *line);
  const char *handle_M111(const char *line);

  // Read parameter from letter "param_letter" and call the event callback
  // "ValueSetter" in the events callbacks. Pass the parameter, multiplied
  // with "factor", to the member function.
  typedef void (EventReceiver::*EventValueSetter)(float d);
  const char *set_param(char param_letter, EventValueSetter setter,
                        float factor, const char *line);

  const char *gcodep_parameter(const char *line, float *value);

  // Read parameter. Do range check.
  bool read_parameter(int num, float *result) {
    if (num < 0 || num >= (int)config.num_parameters) {
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
    if (num <= 0 || num >= (int)config.num_parameters) {
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
    GLOG_EXPRESSION
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

  AxesRegister last_spline_cp2_;
  bool have_first_spline_;

  unsigned int debug_level_;  // OR-ed bits from DebugLevel enum
  bool allow_m111_;

  // TODO(hzeller): right now, we hook the error count to the gprintf(), but
  // maybe this needs to be more explicit.
  int error_count_;
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
    debug_level_(DEBUG_NONE), allow_m111_(allow_m111), error_count_(0)
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
//   3    Expression handling information (conditional on M111 S2)
void GCodeParser::Impl::gprintf(enum GCodePrintLevel level,
                                const char *format, ...) {
  FILE *stream = err_msg_;
  if (stream == NULL) stream = stderr;
  switch (level) {
  case GLOG_EXPRESSION:
    if (!(debug_level_ & DEBUG_EXPRESSION))
      return;
    // fallthru
  case GLOG_INFO:
    fprintf(stream, "// ");
    break;
  case GLOG_SYNTAX_ERR:
    fprintf(stream, "// Line %d: G-Code Syntax Error: ", line_number_);
    ++error_count_;
    break;
  case GLOG_SEMANTIC_ERR:
    fprintf(stream, "// Line %d: G-Code Error: ", line_number_);
    ++error_count_;
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

// Parse number from "line" and store in "value". Returns the position in the
// string after the value had been parsed; if there was an error parsing,
// returns the beginning of the line.
static const char *ParseGcodeNumber(const char *line, float *value) {
  line = skip_white(line);
  // We need to copy the number into a temporary buffer as strtof() does
  // not accept an end-limiter.
  char buffer[40];
  const char *src = line;
  char *dst = buffer;
  const char *end = buffer + sizeof(buffer) - 1;
  const char *extra_allowed = "+-.";
  bool have_point = false;
  while (*src && (isdigit(*src) || index(extra_allowed, *src)) && dst < end) {
    // the sign is only allowed in the first character of the buffer
    if ((*src == '+' || *src == '-') && dst != buffer) break;
    // only allow one decimal point
    if (*src == '.') {
      if (have_point) break;
      else have_point = true;
    }
    *dst++ = *src++;
  }
  *dst = '\0';
  char *parsed_end;
  *value = strtof(buffer, &parsed_end);

  return (parsed_end == dst) ? src : line;
}

const char *GCodeParser::Impl::gcodep_parameter(const char *line, float *value) {
  line = skip_white(line);
  if (*line == '\0') {
    gprintf(GLOG_SYNTAX_ERR, "expected value after '#'\n");
    return NULL;
  }

  float index;
  const char *endptr;
  endptr = gcodep_value(line, &index);
  if (endptr == NULL) {
    gprintf(GLOG_SYNTAX_ERR,
            "'#' is not followed by a number but '%s'\n", line);
    return NULL;
  }
  line = endptr;

  read_parameter((int)index, value);

  return line;  // We parsed something; return whatever is remaining.
}

bool GCodeParser::Impl::execute_unary(float *value, Operation op) {
  float val;
  switch (op) {
  case ABS:
    val = fabsf(*value);
    break;
  case ACOS:
    if (*value < -1.0f || *value > 1.0f) {
      gprintf(GLOG_SYNTAX_ERR, "ACOS argument out of range\n");
      return false;
    }
    val = (acosf(*value) * 180.0f) / M_PI;
    break;
  case ASIN:
    if (*value < -1.0f || *value > 1.0f) {
      gprintf(GLOG_SYNTAX_ERR, "ASIN argument out of range\n");
      return false;
    }
    val = (asinf(*value) * 180.0f) / M_PI;
    break;
  case COS:
    val = cosf((*value * M_PI) / 180.0f);
    break;
  case EXP:
    val = expf(*value);
    break;
  case FIX:
    val = floorf(*value);
    break;
  case FUP:
    val = ceilf(*value);
    break;
  case LN:
    if (*value <= 0.0f) {
      gprintf(GLOG_SYNTAX_ERR, "Zero or negative argument to LN\n");
      return false;
    }
    val = logf(*value);
    break;
  case ROUND:
    val = (double)((int)(*value + ((*value < 0.0f) ? -0.5f : 0.5f)));
    break;
  case SIN:
    val = sinf((*value * M_PI) / 180.0f);
    break;
  case SQRT:
    if (*value < 0.0f) {
      gprintf(GLOG_SYNTAX_ERR, "Negative argument to SQRT\n");
      return false;
    }
    val = sqrtf(*value);
    break;
  case TAN:
    val = tanf((*value * M_PI) / 180.0f);
    break;
  default:
    gprintf(GLOG_SYNTAX_ERR, "Attempt to execute unknown unary operation\n");
    return false;
  }
  gprintf(GLOG_EXPRESSION, "%s[%f] -> %f\n", op_string(op), *value, val);
  *value = val;
  return true;
}

const char *GCodeParser::Impl::gcodep_atan(const char *line, float *value) {
  if (*line != '/') {
    gprintf(GLOG_SYNTAX_ERR, "expected '/' after ATAN got '%s'\n", line);
    return NULL;
  }
  line++;

  if (*line != '[') {
    gprintf(GLOG_SYNTAX_ERR, "expected '[' after ATAN/ got '%s'\n", line);
    return NULL;
  }
  line++;

  float value2;
  const char *endptr;
  endptr = gcodep_expression(line, &value2);
  if (endptr == NULL) {
    gprintf(GLOG_SYNTAX_ERR, "expected value got '%s'\n", line);
    return NULL;
  }
  line = endptr;

  float val = (atan2f(*value, value2) * 180.0f) / M_PI;
  gprintf(GLOG_EXPRESSION, "%s[%f]/[%f] -> %f\n",
          op_string(ATAN), *value, value2, val);
  *value = val;
  return line;
}

const char *GCodeParser::Impl::gcodep_operation_unary(const char *line,
                                                      Operation *op) {
  const char c = toupper(*line);
  line++;
  switch (c) {
  case 'A':
    if (toupper(*line)     == 'B' &&
        toupper(*(line+1)) == 'S') {
      line += 2;
      *op = ABS;
    } else if (toupper(*line)     == 'C' &&
               toupper(*(line+1)) == 'O' &&
               toupper(*(line+2)) == 'S') {
      line += 3;
      *op = ACOS;
    } else if (toupper(*line)     == 'S' &&
               toupper(*(line+1)) == 'I' &&
               toupper(*(line+2)) == 'N') {
      line += 3;
      *op = ASIN;
    } else if (toupper(*line)     == 'T' &&
               toupper(*(line+1)) == 'A' &&
               toupper(*(line+2)) == 'N') {
      line += 3;
      *op = ATAN;
    }
    break;
  case 'C':
    if (toupper(*line)     == 'O' &&
        toupper(*(line+1)) == 'S') {
      line += 2;
      *op = COS;
    }
    break;
  case 'E':
    if (toupper(*line)     == 'X' &&
        toupper(*(line+1)) == 'P') {
      line += 2;
      *op = EXP;
    }
    break;
  case 'F':
    if (toupper(*line)     == 'I' &&
        toupper(*(line+1)) == 'X') {
      line += 2;
      *op = FIX;
    } else if (toupper(*line)     == 'U' &&
               toupper(*(line+1)) == 'P') {
      line += 2;
      *op = FUP;
    }
    break;
  case 'L':
    if (toupper(*line) == 'N') {
      line++;
      *op = LN;
    }
    break;
  case 'R':
    if (toupper(*line)     == 'O' &&
        toupper(*(line+1)) == 'U' &&
        toupper(*(line+2)) == 'N' &&
        toupper(*(line+3)) == 'D') {
      line += 4;
      *op = ROUND;
    }
    break;
  case 'S':
    if (toupper(*line)   == 'I' &&
        toupper(*(line+1)) == 'N') {
      line += 2;
      *op = SIN;
    } else if (toupper(*line)     == 'Q' &&
               toupper(*(line+1)) == 'R' &&
               toupper(*(line+2)) == 'T') {
      line += 3;
      *op = SQRT;
    }
    break;
  case 'T':
    if (toupper(*line)     == 'A' &&
        toupper(*(line+1)) == 'N') {
      line += 2;
      *op = TAN;
    }
    break;
  default:
    *op = NO_OPERATION;
    break;
  }

  return (*op == NO_OPERATION) ? NULL : line;
}

const char *GCodeParser::Impl::gcodep_unary(const char *line, float *value) {
  Operation op;
  const char *endptr;

  line = skip_white(line);

  endptr = gcodep_operation_unary(line, &op);
  if (endptr == NULL) {
    gprintf(GLOG_SYNTAX_ERR, "unknown unary got '%s'\n", line);
    return NULL;
  }
  line = skip_white(endptr);

  if (*line != '[') {
    gprintf(GLOG_SYNTAX_ERR, "expected '[' got '%s'\n", line);
    return NULL;
  }
  line = skip_white(line + 1);

  endptr = gcodep_expression(line, value);
  if (endptr == NULL) {
    gprintf(GLOG_SYNTAX_ERR, "expected value got '%s'\n", line);
    return NULL;
  }
  line = skip_white(endptr);

  if (op == ATAN) {
    endptr = gcodep_atan(line, value);
    if (endptr == NULL) {
      gprintf(GLOG_SYNTAX_ERR, "expected value got '%s'\n", line);
      return NULL;
    }
    line = skip_white(endptr);
  } else {
    if (!execute_unary(value, op)) {
      gprintf(GLOG_SYNTAX_ERR, "unary operation failed\n");
      return NULL;
    }
  }

  return line;
}

bool GCodeParser::Impl::execute_binary(float *left, Operation op, float *right) {
  float val = *left;
  switch (op) {
  case POWER:
    if (*left < 0.0f && floor(*right) != *right) {
      gprintf(GLOG_SYNTAX_ERR, "Attempt to raise negative to non-integer power\n");
      return false;
    }
    val = powf(*left, *right);
    break;
  case DIVIDED_BY:
    if (*right == 0.0f) {
      gprintf(GLOG_SYNTAX_ERR, "Attempt to divide by zero\n");
      return false;
    }
    val = *left / *right;
    break;
  case MODULO:
    val = fmodf(*left, *right);
    // always calculates a positive answer
    if (val < 0.0f)
      val += fabsf(*right);
    break;
  case TIMES:
    val = *left * *right;
    break;
  case AND2:
    val = (*left == 0.0f || *right == 0.0f) ? 0.0f : 1.0f;
    break;
  case EXCLUSIVE_OR:
    val = (((*left == 0.0f) && (*right != 0.0f)) ||
           ((*left != 0.0f) && (*right == 0.0f))) ? 1.0f : 0.0f;
    break;
  case MINUS:
    val = *left - *right;
    break;
  case NON_EXCLUSIVE_OR:
    val = ((*left != 0.0f) || (*right != 0.0f)) ? 1.0f : 0.0f;
    break;
  case PLUS:
    val = *left + *right;
    break;
  default:
    gprintf(GLOG_SYNTAX_ERR, "Attempt to execute unknown binary operation\n");
    return false;
  }
  gprintf(GLOG_EXPRESSION, "[%f %s %f] -> %f\n",
          *left, op_string(op), *right, val);
  *left = val;
  return true;
}

const char *GCodeParser::Impl::gcodep_operation(const char *line,
                                                Operation *op) {
  char c = toupper(*line);
  line++;
  switch (c) {
  case '+':
    *op = PLUS;
    break;
  case '-':
    *op = MINUS;
    break;
  case '/':
    *op = DIVIDED_BY;
    break;
  case '*':
    if (*line == '*') {
      line++;
      *op = POWER;
    } else {
      *op = TIMES;
    }
    break;
  case ']':
    *op = RIGHT_BRACKET;
    break;
  case 'A':
    if (toupper(*line)     == 'N' &&
        toupper(*(line+1)) == 'D') {
      line += 2;
      *op = AND2;
    }
    break;
  case 'M':
    if (toupper(*line)     == 'O' &&
        toupper(*(line+1)) == 'D') {
      line += 2;
      *op = MODULO;
    }
    break;
  case 'O':
    if (toupper(*line) == 'R') {
      line++;
      *op = NON_EXCLUSIVE_OR;
    }
    break;
  case 'X':
    if (toupper(*line)     == 'O' &&
        toupper(*(line+1)) == 'R') {
      line += 2;
      *op = EXCLUSIVE_OR;
    }
    break;
  default:
    *op = NO_OPERATION;
    break;
  }

  return (*op == NO_OPERATION) ? NULL : line;
}

// the expression stack needs to be at least one greater than the max precedence
#define MAX_STACK   5

const char *GCodeParser::Impl::gcodep_expression(const char *line, float *value) {
  float vals[MAX_STACK];
  Operation ops[MAX_STACK];
  int stack = 0;
  const char *endptr;
  line = skip_white(line);

  for (ops[0] = NO_OPERATION; ops[0] != RIGHT_BRACKET; ) {
    endptr = gcodep_value(line, &vals[stack]);
    if (endptr == NULL) {
      if (*line == '-') {
        // make [-expression] work like [-1 * expression]
        line++;
        vals[stack] = -1.0f;
        ops[stack] = TIMES;
        stack++;
        continue;
      }
      gprintf(GLOG_SYNTAX_ERR, "expected value got '%s'\n", line);
      return NULL;
    }
    line = endptr;

    endptr = gcodep_operation(line, &ops[stack]);
    if (endptr == NULL) {
      gprintf(GLOG_SYNTAX_ERR, "unknown operator '%s'\n", line);
      return NULL;
    }
    line = skip_white(endptr);

    // handle the first left value and single value for the unary operations
    if (stack == 0) {
      if (ops[stack] == RIGHT_BRACKET)
        break;
      stack++;
      continue;
    }

    if (precedence(ops[stack]) > precedence(ops[stack - 1])) {
      stack++;
      if (stack >= MAX_STACK) {
        gprintf(GLOG_SYNTAX_ERR, "stack overflow\n");
        return NULL;
      }
    } else {  // precedence of latest operator is <= previous precedence
      for ( ; precedence(ops[stack]) <= precedence(ops[stack - 1]); ) {
        if (!execute_binary(&vals[stack - 1], ops[stack - 1], &vals[stack]))
          return NULL;

        ops[stack - 1] = ops[stack];
        if (stack > 1 && precedence(ops[stack - 1]) <= precedence(ops[stack - 2]))
          stack--;
        else
          break;
      }
    }
  }
  *value = vals[0];

  return line;
}

// Parse a value out of the line.
// The value may be a number, a parameter value, a unary function, or an
// expression.
const char *GCodeParser::Impl::gcodep_value(const char *line, float *value) {
  char c = toupper(*line);
  if (isalpha(c)) c = 'U';  // indicates a unary in the switch below

  const char *endptr;
  switch (c) {
  case '\0':
    endptr = NULL;
    break;
  case '[':
    endptr = gcodep_expression(line + 1, value);
    break;
  case '#':
    endptr = gcodep_parameter(line + 1, value);
    break;
  case 'U':
    endptr = gcodep_unary(line, value);
    break;
  default:
    endptr = ParseGcodeNumber(line, value);
    break;
  }
  if (line == endptr || endptr == NULL)
    return NULL;

  line = skip_white(endptr); // Makes the line better to deal with.
  return line;
}

const char *GCodeParser::Impl::gcodep_set_parameter(const char *line) {
  float value;
  const char *endptr;
  endptr = gcodep_value(line, &value);
  if (endptr == NULL) {
    gprintf(GLOG_SYNTAX_ERR,
            "gcodep_set_parameter: expected value after '#' got '%s'\n",
            line);
    return NULL;
  }
  line = skip_white(endptr);

  int index = (int)value;

  if (*line != '=') {
    gprintf(GLOG_SYNTAX_ERR,
            "gcodep_set_parameter: expected '=' after '#%d' got '%s'\n",
            index, line);
    return NULL;
  }
  line = skip_white(line+1);

  endptr = gcodep_value(line, &value);
  if (endptr == NULL) {
    gprintf(GLOG_SYNTAX_ERR,
            "gcodep_set_parameter: expected value after '#%d=' got '%s'\n",
            index, line);
    return NULL;
  }
  line = skip_white(endptr);

  store_parameter(index, value);

  gprintf(GLOG_EXPRESSION, "#%d=%f\n", index, value);

  return line;
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

  const char *endptr;
  if (*line == '#') {  // parameter set without a letter
    line++;
    endptr = gcodep_set_parameter(line);
    if (endptr == NULL)
      return NULL;

    // recursive call to parse the letter/number pair
    line = endptr;
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

  endptr = gcodep_value(line, value);
  if (endptr == NULL) {
    gprintf(GLOG_SYNTAX_ERR,
            "Letter '%c' is not followed by a number but '%s'\n",
            *letter, line);
    return NULL;
  }
  line = endptr;

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

  bool did_move = false;
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

// The algorithm used here is based on finding the midpoint M of the line
// L between the current point and the end point of the arc. The center
// of the arc lies on a line through M perpendicular to L.
//
// If the value of the radius argument is negative, that means [NCMS,
// page 21] that an arc larger than a semicircle is to be made.
// Otherwise, an arc of a semicircle or less is made.
static bool arc_radius_to_center(float start1, float start2,
                                 float end1, float end2,
                                 float radius, bool is_cw,
                                 float *center1, float *center2) {
  if (end1 == start1 && end2 == start2)
    return false;               // start point same as end point

  const float abs_radius = fabs(radius);
  const float mid1 = (end1 + start1) / 2.0f;
  const float mid2 = (end2 + start2) / 2.0f;

  float half_length = hypotf(mid1 - end1, mid2 - end2);
  if ((half_length / abs_radius) > (1 + 1e-6))
    return false;               // radius to small to reach end point
  if ((half_length / abs_radius) > (1 - 1e-6))
    half_length = abs_radius;   // allow a small error for semicircle

  float theta;
  if ((is_cw && radius > 0) || (!is_cw && radius < 0))
    theta = atan2f(end2 - start2, end1 - start1) - (M_PI / 2.0f);
  else
    theta = atan2f(end2 - start2, end1 - start1) + (M_PI / 2.0f);

  const float turn2 = asinf(half_length / abs_radius);
  const float offset = abs_radius * cosf(turn2);

  // return the offset to the center of the arc
  *center1 = (mid1 + (offset * cosf(theta))) - start1;
  *center2 = (mid2 + (offset * sinf(theta))) - start2;

  return true;
}

// For we just generate an arc by emitting many small steps for
// now. TODO(hzeller): actually generate a curve profile for that.
// With G17, G18, G19, the plane was selected before.
// X, Y, Z: new position (two in the plane, one for the helix.
// I, J, K: offset to center, corresponding to X,Y,Z.
//          Only the two in the given plane are relevant.
// F      : Optional feedrate.
// P      : number of turns. currently ignored.
// R      : arc radius
const char *GCodeParser::Impl::handle_arc(const char *line, bool is_cw) {
  const char *remaining_line;
  AxesRegister target = axes_pos_;
  AxesRegister offset = kZeroOffset;
  float feedrate = -1;
  float radius = 0.0f;
  bool have_ijk = false;
  bool have_r = false;
  float value;
  char letter;
  int turns = 1;

  while ((remaining_line = gparse_pair(line, &letter, &value))) {
    const float unit_value = value * unit_to_mm_factor_;
    if (letter == 'X') target[AXIS_X] = abs_axis_pos(AXIS_X, unit_value);
    else if (letter == 'Y') target[AXIS_Y] = abs_axis_pos(AXIS_Y, unit_value);
    else if (letter == 'Z') target[AXIS_Z] = abs_axis_pos(AXIS_Z, unit_value);
    else if (letter == 'I') { offset[AXIS_X] = unit_value; have_ijk = true; }
    else if (letter == 'J') { offset[AXIS_Y] = unit_value; have_ijk = true; }
    else if (letter == 'K') { offset[AXIS_Z] = unit_value; have_ijk = true; }
    else if (letter == 'F') feedrate = f_param_to_feedrate(unit_value);
    else if (letter == 'P') turns = (int)value; // currently ignored
    else if (letter == 'R') { radius = unit_value; have_r = true; }
    else break;

    line = remaining_line;
  }

  if (!have_ijk && !have_r) {
    gprintf(GLOG_SYNTAX_ERR, "handle_arc: missing IJK or R\n");
    return line;
  } else if (have_ijk && have_r) {
    gprintf(GLOG_SYNTAX_ERR, "handle_arc: mixed IJK and R\n");
    return line;
  }
  if (turns != 1) {
    // Currently, we ignore turns, so we check that it is exactly one.
    gprintf(GLOG_SYNTAX_ERR, "handle_arc: turns=%d (must be 1)\n", turns);
    return line;
  }

  if (have_ijk) {
    switch (arc_normal_) {
    case AXIS_Z:
      if (offset[AXIS_Z] != 0.0f) {
        gprintf(GLOG_SYNTAX_ERR, "handle_arc: invalid K in XY plane\n");
        return line;
      }
      break;
    case AXIS_X:
      if (offset[AXIS_X] != 0.0f) {
        gprintf(GLOG_SYNTAX_ERR, "handle_arc: invalid I in YZ plane\n");
        return line;
      }
      break;
    case AXIS_Y:
      if (offset[AXIS_Y] != 0.0f) {
        gprintf(GLOG_SYNTAX_ERR, "handle_arc: invalid J in ZX plane\n");
        return line;
      }
      break;
    default:
      return line;  // invalid plane
    }
  } else if (have_r) {
    bool have_center;
    switch (arc_normal_) {
    case AXIS_Z:
      have_center = arc_radius_to_center(axes_pos_[AXIS_X], axes_pos_[AXIS_Y],
                                         target[AXIS_X], target[AXIS_Y],
                                         radius, is_cw,
                                         &offset[AXIS_X], &offset[AXIS_Y]);
      break;
    case AXIS_X:
      have_center = arc_radius_to_center(axes_pos_[AXIS_Y], axes_pos_[AXIS_Z],
                                         target[AXIS_Y], target[AXIS_Z],
                                         radius, is_cw,
                                         &offset[AXIS_Y], &offset[AXIS_Z]);
      break;
    case AXIS_Y:
      have_center = arc_radius_to_center(axes_pos_[AXIS_X], axes_pos_[AXIS_Z],
                                         target[AXIS_X], target[AXIS_Z],
                                         radius, is_cw,
                                         &offset[AXIS_X], &offset[AXIS_Z]);
      break;
    default:
      return line;  // invalid plane
    }
    if (!have_center) {
      gprintf(GLOG_SYNTAX_ERR, "handle_arc: unable to handle radius\n");
      return line;
    }
  }

  struct ArcCallbackData cb_arc_data;
  cb_arc_data.callbacks = callbacks;
  cb_arc_data.feedrate = feedrate;
  arc_gen(arc_normal_, is_cw, &axes_pos_, offset, target,
          &arc_callback, &cb_arc_data);

  return line;
}

// G5 X- Y- <I- J-> P- Q- (Cubic spline)
//   I - X relative offset from start point to first control point
//   J - Y relative offset from start point to first control point
//   P - X relative offset from start point to second control point
//   Q - Y relative offset from start point to second control point
//
// G5 creates a cubic B-spline in the XY plane with the X and Y axes only.
// P and Q must be specified for every G5 command.
//
// For the first G5 command in a series of G5 commands, I and J must be
// specified. For subsequent G5 commands, either I and I must be specified,
// or neither. If I and J are unspecified, the starting direction of this
// cubic will automaticall match the ending direction of the previous cubic
// (as if I and J are the negation of the previous P and Q).
//
// For example, to program a curvy N shape:
//
// G90 G17
// G0 X0 Y0
// G5 I0 J30 P0 Q-30 X10 Y10
//
// A second curvy N that attaches smoothly to this one can now be made without
// specifying I and J:
//
// G5 P0 Q-30 X20 Y20
//
// It is an error if:
//   1) P and Q are not both specified.
//   2) Just one of I or J are specified.
//   3) I or J are unspecified in the first of a series of G5 commands.
//   4) An axis other than X or Y is specified.
//   5) The active plane is not G17.
//
// G5.1 X- Y- I- J- (Quadratic spline)
//   I - X relative offset from start point to control point
//   J - Y relative offset from start point to control point
//
// G5.1 creates a quadratic B-spline in the XY plane with the X and Y axis
// only. Not specifying I or J gives zero offset for the specified axis, so
// one or both must be given.
//
// For example, to program a parabola, through the origin from X-20 Y-40 to
// X20 Y40:
//
// G90 G17
// G0 X-20 Y40
// G5.1 X20 I20 J-80
//
// It is an error if:
//   1) Both I and J offset are unspecified or zero.
//   2) An axis other than X or Y is specified.
//   3) The active plane is not G17.
//
// G5.2 ...  G5.3 (NURBS Block)
// Not currently supported.
const char *GCodeParser::Impl::handle_spline(float sub_command, const char *line) {
  if (arc_normal_ != AXIS_Z) {
    gprintf(GLOG_SEMANTIC_ERR, "handle_spline: not in XY plane\n");
    return NULL;
  }

  bool is_cubic;
  if (sub_command == 5.0f) {
    is_cubic = true;
  } else if (sub_command == 5.1f) {
    is_cubic = false;
  } else {
    gprintf(GLOG_SEMANTIC_ERR, "handle_spline: G%.1f is not supported\n",
            sub_command);
    return NULL;
  }

  AxesRegister target = axes_pos_;
  AxesRegister cp1 = kZeroOffset;
  AxesRegister cp2 = kZeroOffset;
  bool have_i = false;
  bool have_j = false;
  bool have_p = false;
  bool have_q = false;
  const char *remaining_line;
  float value;
  char letter;
  while ((remaining_line = gparse_pair(line, &letter, &value))) {
    const float unit_value = value * unit_to_mm_factor_;
    if (letter == 'X') target[AXIS_X] = abs_axis_pos(AXIS_X, unit_value);
    else if (letter == 'Y') target[AXIS_Y] = abs_axis_pos(AXIS_Y, unit_value);
    else if (letter == 'I') { cp1[AXIS_X] = unit_value; have_i = true; }
    else if (letter == 'J') { cp1[AXIS_Y] = unit_value; have_j = true; }
    else if (letter == 'P') { cp2[AXIS_X] = unit_value; have_p = true; }
    else if (letter == 'Q') { cp2[AXIS_Y] = unit_value; have_q = true; }
    else {
      gprintf(GLOG_SEMANTIC_ERR, "handle_spline: invalid axis specified\n");
      return NULL;
    }
    line = remaining_line;
  }

  if (is_cubic) {
    if (!have_p && !have_q) {
      gprintf(GLOG_SEMANTIC_ERR, "handle_spline: G5 missing P and Q\n");
      return NULL;
    }
    if ((have_i && !have_j) || (!have_i && have_j)) {
      gprintf(GLOG_SEMANTIC_ERR, "handle_spline: G5 missing I or J\n");
      return NULL;
    }
    if (!have_i && !have_j) {
      if (!have_first_spline_) {
        gprintf(GLOG_SEMANTIC_ERR, "handle_spline: G5 missing I and J\n");
        return NULL;
      } else {
        cp1[AXIS_X] = -last_spline_cp2_[AXIS_X];
        cp1[AXIS_Y] = -last_spline_cp2_[AXIS_Y];
      }
    }
    last_spline_cp2_ = cp2;
    have_first_spline_ = true;

    // convert the control points from offsets to absolutes
    cp1[AXIS_X] += axes_pos_[AXIS_X];
    cp1[AXIS_Y] += axes_pos_[AXIS_Y];
    cp2[AXIS_X] += target[AXIS_X];
    cp2[AXIS_Y] += target[AXIS_Y];
  } else {
    if (!have_i && !have_j) {
      gprintf(GLOG_SEMANTIC_ERR, "handle_spline: G5.1 missing I and J\n");
      return NULL;
    }
    if (cp1[AXIS_X] == 0.0f && cp1[AXIS_Y] == 0.0f) {
      gprintf(GLOG_SEMANTIC_ERR, "handle_spline: G5.1 I and J are zero\n");
      return NULL;
    }
    have_first_spline_ = false;

    // convert the quadratic control point to two cubic control points
    AxesRegister _cp1;
    AxesRegister _cp2;
    _cp1[AXIS_X] = axes_pos_[AXIS_X] + (cp1[AXIS_X] * 2.0f) / 3.0f;
    _cp1[AXIS_Y] = axes_pos_[AXIS_Y] + (cp1[AXIS_Y] * 2.0f) / 3.0f;
    _cp2[AXIS_X] = _cp1[AXIS_X] + (cp1[AXIS_X] * 1.0f) / 3.0f;
    _cp2[AXIS_Y] = _cp1[AXIS_Y] + (cp1[AXIS_Y] * 1.0f) / 3.0f;

    cp1 = _cp1;
    cp2 = _cp2;
  }

  struct ArcCallbackData cb_arc_data;
  cb_arc_data.callbacks = callbacks;
  cb_arc_data.feedrate = -1;
  spline_gen(&axes_pos_, cp1, cp2, target, &arc_callback, &cb_arc_data);

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
    bool last_spline = have_first_spline_;
    have_first_spline_ = false;
    bool processed_command = true;
    if (letter == 'G') {
      switch ((int) value) {
      case  0: modal_g0_g1_ = 0; line = handle_move(line, false); break;
      case  1: modal_g0_g1_ = 1; line = handle_move(line, false); break;
      case  2: line = handle_arc(line, true); break;
      case  3: line = handle_arc(line, false); break;
      case  4: line = set_param('P', &GCodeParser::EventReceiver::dwell,
                                1.0f, line);
        break;
      case  5:
        have_first_spline_ = last_spline;
        line = handle_spline(value, line);
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
      processed_command = false;
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
int GCodeParser::error_count() const { return impl_->error_count(); }

const char *GCodeParser::ParsePair(const char *line,
                                   char *letter, float *value,
                                   FILE *err_stream) {
  return impl_->gcodep_parse_pair_with_linenumber(-1, line, letter, value,
                                                  err_stream);
}
