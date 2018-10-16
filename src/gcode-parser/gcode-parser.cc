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

#include "common/logging.h"
#include "common/string-util.h"

#include "simple-lexer.h"

const AxisBitmap_t kAllAxesBitmap =
  ((1 << AXIS_X) | (1 << AXIS_Y) | (1 << AXIS_Z) |
   (1 << AXIS_A) | (1 << AXIS_B) | (1 << AXIS_C) |
   (1 << AXIS_U) | (1 << AXIS_V) | (1 << AXIS_W) |
   (1 << AXIS_E));

char gcodep_axis2letter(enum GCodeParserAxis axis) {
  switch (axis) {
  case AXIS_X: return 'X';
  case AXIS_Y: return 'Y';
  case AXIS_Z: return 'Z';
  case AXIS_A: return 'A';
  case AXIS_B: return 'B';
  case AXIS_C: return 'C';
  case AXIS_U: return 'U';
  case AXIS_V: return 'V';
  case AXIS_W: return 'W';
  case AXIS_E: return 'E';
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
  case 'A': case 'a': return AXIS_A;
  case 'B': case 'b': return AXIS_B;
  case 'C': case 'c': return AXIS_C;
  case 'U': case 'u': return AXIS_U;
  case 'V': case 'v': return AXIS_V;
  case 'W': case 'w': return AXIS_W;
  case 'E': case 'e': return AXIS_E;
  }
  return GCODE_NUM_AXES;
}

static const char *const kCoordinateSystemNames[9] = {
  "G54", "G55", "G56", "G57", "G58", "G59", "G59.1", "G59.2", "G59.3"
};

// We keep the implementation with all its unnecessary details for the user
// in this implementation.
class GCodeParser::Impl {
public:
  Impl(const GCodeParser::Config &config,
       GCodeParser::EventReceiver *parse_events);
  ~Impl();

  void ParseBlock(GCodeParser *owner, const char *line, FILE *err_stream);
  int ParseStream(GCodeParser *owner, int input_fd, FILE *err_stream);
  const char *gcodep_parse_pair_with_linenumber(int line_num,
                                                const char *line,
                                                char *letter,
                                                float *value,
                                                FILE *err_stream);
  int error_count() const { return error_count_; }
  EventReceiver *callbacks() { return callbacks_; }

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
    set_ijk_absolute(false);          // G91.1
    reset_G92();                      // No global offset.
    set_current_offset(global_offset_g92_, "H");

    arc_normal_ = AXIS_Z;  // Arcs in XY-plane

    last_spline_cp2_ = kZeroOffset;
    have_first_spline_ = false;

    do_while_ = false;

    // Some initial machine states emitted as events.
    callbacks()->set_speed_factor(1.0);
    callbacks()->set_fanspeed(0);
    callbacks()->set_temperature(0);
  }

  void InitCoordSystems();

  void set_all_axis_to_absolute(bool value) {
    for (GCodeParserAxis a : AllAxes()) {
      axis_is_absolute_[a] = value;
    }
    modal_absolute_g90_ = value;
  }

  void set_ijk_absolute(bool absolute) {
    if (absolute) {
      gprintf(GLOG_SEMANTIC_ERR, "TODO: We don't support absolute IJK yet\n");
    }
    ijk_is_absolute_ = absolute;
  }

  void reset_G92() {
    global_offset_g92_ = kZeroOffset;
  }

  // The following methods set the origin and offset and also
  // inform the event receiver about this change.
  void set_current_offset(const AxesRegister &offset, const char *name) {
    current_global_offset_ = &offset;
    inform_origin_offset_change(name);
  }
  // no need yet for set_current_origin().

  void inform_origin_offset_change(const char *name) {
    AxesRegister visible_origin(*current_origin_);
    for (GCodeParserAxis a : AllAxes()) {
      visible_origin[a] += current_global_offset()[a];
    }
    callbacks()->inform_origin_offset(visible_origin, name);
  }

  void finish_program_and_reset() {
    callbacks()->gcode_finished(false);

    InitProgramDefaults();
    program_in_progress_ = false;
  }

  enum Operation {
    NO_OPERATION = 0,

    // binary operations
    BINARY_OP_START,
    POWER,                                              // precedence 5
    DIVIDED_BY, MODULO, TIMES,                          // precedence 4
    AND2, EXCLUSIVE_OR, MINUS, NON_EXCLUSIVE_OR, PLUS,  // precedence 3
    EQ, NE, GT, GE, LT, LE,                             // precedence 2
    RIGHT_BRACKET,                                      // precedence 1

    // unary operations
    UNARY_OP_START,
    ABS, ACOS, ASIN, ATAN, COS, EXP, FIX, FUP, LN, ROUND, SIN, SQRT, TAN
  };

  int precedence(Operation op) {
    if (op == RIGHT_BRACKET) return 1;
    if (op == POWER) return 5;
    if (op >= EQ) return 2;
    if (op >= AND2) return 3;
    return 4;
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

  void gcodep_conditional(const char *line);

  void gcodep_while_end();
  void gcodep_while_do(const char *line);
  void gcodep_while_start(const char *line);

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
  void handle_G90_G91(float value);
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

  // Read name of parameter (after #) which is either a number or a
  // non-alphanumeric character.
  const char *read_param_name(const char *line, std::string *result);

  // Read parameter. Do range check.
  bool read_parameter(StringPiece param_name, float *result) const {
    if (config_.parameters == NULL)
      return false;
    param_name = TrimWhitespace(param_name);
    Config::ParamMap::const_iterator found
      = config_.parameters->find(ToLower(param_name));
    if (found != config_.parameters->end()) {
      *result = found->second;
      return true;
    } else {
      *result = 0;
      return false;
    }
  }

  // Read parameter. Do range check.
  bool store_parameter(StringPiece param_name, float value) {
    if (config_.parameters == NULL)
      return false;
    param_name = TrimWhitespace(param_name);
    // zero parameter can never be written.
    if (param_name == "0" || atoi(param_name.ToString().c_str()) >= 5400) {
      gprintf(GLOG_SEMANTIC_ERR, "writing unsupported parameter number (%s)\n",
              param_name.ToString().c_str());
      return false;
    }
    (*config_.parameters)[ToLower(param_name)] = value;
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

  static const AxesRegister kZeroOffset;

  GCodeParser::EventReceiver *const callbacks_;
  GCodeParser::Config config_;

  SimpleLexer<Operation> op_parse_;

  enum ControlKeyword {
    NO_CONTROL_KEYWORD,
    CK_IF, CK_THEN, CK_ELSE, CK_ELSEIF,
    CK_WHILE, CK_DO, CK_END
  };
  SimpleLexer<ControlKeyword> control_parse_;

  bool program_in_progress_;

  FILE *err_msg_;
  int modal_g0_g1_;
  int line_number_;
  float unit_to_mm_factor_;               // metric: 1.0; imperial 25.4

  // We distinguish axes here, because in 3D printers, the E-axis can be
  // set relative independently of the other axes.
  bool axis_is_absolute_[GCODE_NUM_AXES]; // G90 or G91 active.
  bool ijk_is_absolute_ = false;
  bool modal_absolute_g90_ = true;        // All axes, but E might differ

  // The axes_pos is the current absolute position of the machine
  // in the work-cube. It always is positive in the range of
  // (0,0,0,...) -> (range_x, range_y, range_z,...)
  // This represents the current position of the machine.
  AxesRegister axes_pos_;

  // -- Origins
  // All the following coordinates are absolute positions. They
  // can be choosen as active origin.
  // The machine origin or home position is not necessarily {0,0,0...}, but
  // it is where the end-switches are.
  const AxesRegister machine_origin_;

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

  GCodeParser *while_owner_;
  FILE *while_err_stream_;
  bool do_while_;
  std::string while_condition_;
  std::string while_loop_;

  unsigned int debug_level_;  // OR-ed bits from DebugLevel enum
  bool allow_m111_;

  // TODO(hzeller): right now, we hook the error count to the gprintf(), but
  // maybe this needs to be more explicit.
  int error_count_;
};

const AxesRegister GCodeParser::Impl::kZeroOffset;

GCodeParser::Impl::Impl(const GCodeParser::Config &parse_config,
                        GCodeParser::EventReceiver *parse_events)
  : callbacks_(parse_events), config_(parse_config),
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
    axes_pos_(config_.machine_origin),
    machine_origin_(config_.machine_origin),
    // The current origin is the same as the home position (where the home
    // switches are) That means for CNC machines with machine origins e.g.
    // on the top right, that all valid coordinates to stay within the
    // machine cube are negative.
    current_origin_(&machine_origin_),
    current_global_offset_(&kZeroOffset),
    arc_normal_(AXIS_Z),
    while_err_stream_(NULL), do_while_(false),
    debug_level_(DEBUG_NONE),
    error_count_(0)
{
  assert(callbacks_);  // otherwise, this is not very useful.
  reset_G92();
  InitProgramDefaults();
  InitCoordSystems();
  op_parse_.AddKeyword("+",  PLUS);
  op_parse_.AddKeyword("-",  MINUS);
  op_parse_.AddKeyword("/",  DIVIDED_BY);
  op_parse_.AddKeyword("MOD", MODULO);
  op_parse_.AddKeyword("*",  TIMES);
  op_parse_.AddKeyword("**", POWER);
  op_parse_.AddKeyword("==", EQ); op_parse_.AddKeyword("EQ", EQ);
  op_parse_.AddKeyword("!=", NE); op_parse_.AddKeyword("NE", NE);
  op_parse_.AddKeyword(">",  GT); op_parse_.AddKeyword("GT", GT);
  op_parse_.AddKeyword(">=", GE); op_parse_.AddKeyword("GE", GE);
  op_parse_.AddKeyword("<",  LT); op_parse_.AddKeyword("LT", LT);
  op_parse_.AddKeyword("<=", LE); op_parse_.AddKeyword("LE", LE);
  op_parse_.AddKeyword("AND", AND2); op_parse_.AddKeyword("&&",  AND2);
  op_parse_.AddKeyword("OR", NON_EXCLUSIVE_OR);
  op_parse_.AddKeyword("||", NON_EXCLUSIVE_OR);
  op_parse_.AddKeyword("XOR", EXCLUSIVE_OR);
  op_parse_.AddKeyword("]",  RIGHT_BRACKET);

  op_parse_.AddKeyword("abs",  ABS);
  op_parse_.AddKeyword("tan",  TAN); op_parse_.AddKeyword("atan", ATAN);
  op_parse_.AddKeyword("sin",  SIN); op_parse_.AddKeyword("asin", ASIN);
  op_parse_.AddKeyword("cos",  COS); op_parse_.AddKeyword("acos", ACOS);
  op_parse_.AddKeyword("exp",  EXP); op_parse_.AddKeyword("ln",   LN);
  op_parse_.AddKeyword("fix",  FIX);
  op_parse_.AddKeyword("fup",  FUP);
  op_parse_.AddKeyword("round", ROUND);
  op_parse_.AddKeyword("sqrt", SQRT);

  control_parse_.AddKeyword("if",     CK_IF);
  control_parse_.AddKeyword("then",   CK_THEN);
  control_parse_.AddKeyword("else",   CK_ELSE);
  control_parse_.AddKeyword("elseif", CK_ELSEIF);
  control_parse_.AddKeyword("while",  CK_WHILE);
  control_parse_.AddKeyword("do",     CK_DO);
  control_parse_.AddKeyword("end",    CK_END);
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

// Parameter/variable names can be simple integers (traditional NIST), or
// a named one.
// Returns the remainder of the line or NULL if parameter name could not
// be parsed.
const char* GCodeParser::Impl::read_param_name(const char *line,
                                               std::string *result) {
  line = skip_white(line);
  if (*line == '\0') {
    gprintf(GLOG_SYNTAX_ERR, "expected value after '#'\n");
    return NULL;
  }

  const bool bracketed = (*line == '<');   // #<foo>-style variables.
  if (bracketed)
    ++line;

  const bool numeric_parameter = *line == '#' || isdigit(*line);
  if (numeric_parameter && bracketed) {
    gprintf(GLOG_SYNTAX_ERR, "The #<> bracket syntax is only allowed for "
            "the alphanumeric parameters (at %s)", line);
    return NULL;
  }

  // if (!numeric_parameter && strict_nist) warn("using extension");
  if (numeric_parameter) {
    float index;
    const char *endptr = gcodep_value(line, &index);
    if (endptr == NULL) {
      gprintf(GLOG_SYNTAX_ERR,
              "'#' is not followed by a number but '%s'\n", line);
      return NULL;
    }
    line = endptr;
    *result = StringPrintf("%d", (int) index);
  } else {
    result->clear();
    // Allowing alpha-numeric parameters.
    while (*line
           && ((*line >= '0' && *line <= '9')
               || (*line >= 'A' && *line <= 'Z')
               || (*line >= 'a' && *line <= 'z')
               || *line == '_'
               || (bracketed && isspace(*line)))) {
      if (!isspace(*line)) {
        result->append(1, *line);
      }
      ++line;
    }
  }

  if (bracketed) {
    if (*line != '>') {
      gprintf(GLOG_SYNTAX_ERR, "Missed closing bracket for parameter <%s>\n",
              result->c_str());
      return NULL;
    }
    ++line;
  }

  return result->empty() ? NULL : skip_white(line);
}

const char *GCodeParser::Impl::gcodep_parameter(const char *line, float *value) {
  std::string param_name;
  line = read_param_name(line, &param_name);
  if (line == NULL) return NULL;

  read_parameter(param_name, value);

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
  gprintf(GLOG_EXPRESSION, "%s[%f] -> %f\n", op_parse_.AsString(op),
          *value, val);
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
          op_parse_.AsString(ATAN), *value, value2, val);
  *value = val;
  return line;
}

const char *GCodeParser::Impl::gcodep_operation_unary(const char *line,
                                                      Operation *op) {
  *op = op_parse_.MatchNext(&line);
  if (*op > UNARY_OP_START) {
    return line;
  } else {
    *op = NO_OPERATION;
    return NULL;
  }
}

const char *GCodeParser::Impl::gcodep_unary(const char *line, float *value) {
  Operation op;
  const char *endptr;

  line = gcodep_operation_unary(line, &op);
  if (line == NULL) {
    gprintf(GLOG_SYNTAX_ERR, "unknown unary got '%s'\n", line);
    return NULL;
  }

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
  case EQ:
    val = (*left == *right) ? 1.0f : 0.0f;
    break;
  case NE:
    val = (*left != *right) ? 1.0f : 0.0f;
    break;
  case GT:
    val = (*left > *right) ? 1.0f : 0.0f;
    break;
  case GE:
    val = (*left >= *right) ? 1.0f : 0.0f;
    break;
  case LT:
    val = (*left < *right) ? 1.0f : 0.0f;
    break;
  case LE:
    val = (*left <= *right) ? 1.0f : 0.0f;
    break;
  default:
    gprintf(GLOG_SYNTAX_ERR, "Attempt to execute unknown binary operation\n");
    return false;
  }
  gprintf(GLOG_EXPRESSION, "[%f %s %f] -> %f\n",
          *left, op_parse_.AsString(op), *right, val);
  *left = val;
  return true;
}

const char *GCodeParser::Impl::gcodep_operation(const char *line,
                                                Operation *op) {
  *op = op_parse_.MatchNext(&line);
  if (*op > NO_OPERATION && *op < UNARY_OP_START) {
    return line;
  } else {
    *op = NO_OPERATION;
    return NULL;
  }
}

// the expression stack needs to be at least one greater than the max precedence
#define MAX_STACK   6

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
        line = skip_white(line+1);
        if (*line == '-') {
          gprintf(GLOG_SYNTAX_ERR, "double-negative detected\n");
          return NULL;
        }
        // make [-expression] work like [-1 * expression]
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
  std::string param_name;
  line = read_param_name(line, &param_name);
  if (line == NULL) return NULL;
  const char *log_name = param_name.c_str();

  float value;
  if (*line     == '+' &&
      *(line+1) == '+') {
    line = skip_white(line+2);
    read_parameter(param_name, &value);
    value++;
    store_parameter(param_name, value);
    gprintf(GLOG_EXPRESSION, "#%s++ -> #%s=%f\n", log_name, log_name, value);
    return line;
  }
  if (*line     == '-' &&
      *(line+1) == '-') {
    line = skip_white(line+2);
    read_parameter(param_name, &value);
    value--;
    store_parameter(param_name, value);
    gprintf(GLOG_EXPRESSION, "#%s-- -> #%s=%f\n", log_name, log_name, value);
    return line;
  }

  Operation op = NO_OPERATION;
  if (*line     == '+' &&
      *(line+1) == '=') {
    line = skip_white(line+2);
    op = PLUS;
  } else if (*line     == '-' &&
             *(line+1) == '=') {
    line = skip_white(line+2);
    op = MINUS;
  } else if (*line     == '*' &&
             *(line+1) == '=') {
    line = skip_white(line+2);
    op = TIMES;
  } else if (*line     == '/' &&
             *(line+1) == '=') {
    line = skip_white(line+2);
    op = DIVIDED_BY;
  } else if (*line == '=') {
    line = skip_white(line+1);
  } else {
    if (*line == '\0') {
      value = 0.0;
      read_parameter(param_name, &value);
      gprintf(GLOG_INFO, "#%s = %f\n", log_name, value);
    } else {
      gprintf(GLOG_SYNTAX_ERR,
              "gcodep_set_parameter: expected '=' after '#%s' got '%s'\n",
              log_name, line);
    }
    return NULL;
  }

  // Assignment
  const char *endptr;
  endptr = gcodep_value(line, &value);
  if (endptr == NULL) {
    gprintf(GLOG_SYNTAX_ERR,
            "gcodep_set_parameter: expected value after '#%s=' got '%s'\n",
            log_name, line);
    return NULL;
  }
  line = skip_white(endptr);

  if (op != NO_OPERATION) {
    float left;
    read_parameter(param_name, &left);
    if (!execute_binary(&left, op, &value))
      return NULL;
    value = left;
  } else {
    // see if this is a ternary operation '? :'
    if (*line == '?') {
      bool condition = (value != 0.0f);
      line = skip_white(line+1);

      endptr = gcodep_value(line, &value);
      if (endptr == NULL) {
        gprintf(GLOG_SYNTAX_ERR,
                "gcodep_set_parameter: expected value after '#%s=[%d] ? ' got '%s'\n",
                log_name, line, condition);
        return NULL;
      }
      line = skip_white(endptr);
      float true_value = value;

      if (*line == ':') {
        line = skip_white(line+1);
        endptr = gcodep_value(line, &value);
        if (endptr == NULL) {
          gprintf(GLOG_SYNTAX_ERR,
                  "gcodep_set_parameter: expected value after '#%s=[%d] ? %f :' got '%s'\n",
                  log_name, line, condition, true_value);
          return NULL;
        }
        line = skip_white(endptr);

        if (condition)
          value = true_value;
      } else {
        gprintf(GLOG_SYNTAX_ERR,
                "gcodep_set_parameter: expected ':' after '#%s=[%d] ? %f' got '%s'\n",
                log_name, line, condition, true_value);
        return NULL;
      }
    }
  }

  store_parameter(param_name, value);
  callbacks()->gcode_command_done('#', value);

  gprintf(GLOG_EXPRESSION, "#%s=%f\n", log_name, value);

  return line;
}

void GCodeParser::Impl::gcodep_conditional(const char *line) {
  if (*line != '[') {
    gprintf(GLOG_SYNTAX_ERR, "expected '[' after IF got '%s'\n", line);
    return;
  }

  float value = 0.0f;
  const char *endptr;
  endptr = gcodep_expression(line + 1, &value);
  if (line == endptr || endptr == NULL)
    return;

  const bool condition = (value == 1.0f) ? true : false;

  line = skip_white(endptr);
  if (!control_parse_.ExpectNext(&line, CK_THEN)) {
    gprintf(GLOG_SEMANTIC_ERR, "unsupported IF [...] %s (expected 'then')\n",
            line);
    return;
  }

  line = skip_white(line);
  if (condition == true) {
    // parse the true condition
    if (*line == '#') {
      gcodep_set_parameter(++line);
    } else {
      gprintf(GLOG_SYNTAX_ERR, "expected '#' after IF [...] THEN got '%s'\n",
              line);
    }
  } else {
    bool have_else = false;
    // see if there is an ELSE
    // (TODO: make these with control_parser_)
    while (*line != '\0') {
      if (toupper(*line)     == 'E' &&
          toupper(*(line+1)) == 'L' &&
          toupper(*(line+2)) == 'S' &&
          toupper(*(line+3)) == 'E') {
        line = skip_white(line+4);
        have_else = true;
        break;
      } else {
        line++;
      }
    }
    if (have_else) {
      // ELSEIF
      if (toupper(*line)     == 'I' &&
          toupper(*(line+1)) == 'F') {
        line = skip_white(line+2);
        gcodep_conditional(line);
        return;
      }

      // parse the false condition
      if (*line == '#') {
        gcodep_set_parameter(++line);  // TODO: error handling ?
      } else {
        gprintf(GLOG_SYNTAX_ERR, "expected '#' after IF [...] THEN ... ELSE "
                "got '%s'\n", line);
        return;
      }
    }
  }
}

void GCodeParser::Impl::gcodep_while_end() {
    int loops = 0;
    while (1) {
      const char *line = while_condition_.c_str();
      const char *endptr;
      float value;
      // the '[' was already parsed
      endptr = gcodep_expression(line, &value);
      if (endptr == NULL) {
        gprintf(GLOG_SYNTAX_ERR, "expected value got '%s'\n", line);
        return;
      }
      if (value == 0.0f)
        break;

      line = skip_white(endptr);
      if (control_parse_.ExpectNext(&line, CK_DO)) {
        std::vector<StringPiece> piece = SplitString(while_loop_, "\n");
        for (size_t i=0; i < piece.size(); i++)
          ParseBlock(while_owner_, piece[i].ToString().c_str(),
                     while_err_stream_);
      } else {
        gprintf(GLOG_SYNTAX_ERR, "expected DO got '%s'\n", line);
        return;
      }
      loops++;
    }
    gprintf(GLOG_INFO, "Executed %d loops\n", loops);
}

void GCodeParser::Impl::gcodep_while_do(const char *line) {
  if (control_parse_.ExpectNext(&line, CK_END)) {
    do_while_ = false;
    gcodep_while_end();
    return;
  }

  while_loop_ += line;
}

// WHILE [conditionalexpression is true] DO
// ...
// END
void GCodeParser::Impl::gcodep_while_start(const char *line) {
  if (*line != '[') {
    gprintf(GLOG_SYNTAX_ERR, "expected '[' after WHILE got '%s'\n", line);
    return;
  }
  line = skip_white(line+1);

  while_condition_ = line;
  while_loop_ = "";
  do_while_= true;
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

  if (do_while_) {
    gcodep_while_do(line);
    return NULL;
  }

  if (*line == '(') {  // Comment between words; e.g. G0(move) X1(this axis)
    while (*line && *line != ')')
      line++;
    line = skip_white(line + 1);
    if (*line == '\0') return NULL;
  }

  if (control_parse_.ExpectNext(&line, CK_IF)) {
    gcodep_conditional(skip_white(line));
    return NULL;
  }

  if (control_parse_.ExpectNext(&line, CK_WHILE)) {
    gcodep_while_start(skip_white(line));
    return NULL;
  }

  const char *endptr;
  if (*line == '#') {  // parameter set without a letter
    line++;
    endptr = gcodep_set_parameter(line);
    if (endptr == NULL)
      return NULL;

    // recursive call to parse the letter/number pair
    line = endptr;
    return gcodep_parse_pair_with_linenumber(line_num, line, letter, value,
                                             err_stream);
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
  callbacks()->go_home(homing_flags);

  // Now update the world position
  for (GCodeParserAxis a : AllAxes()) {
    if (homing_flags & (1 << a)) {
      axes_pos_[a] = machine_origin_[a];
    }
  }

  return line;
}

// Initialize the coordiate systems from the loaded parameters
void GCodeParser::Impl::InitCoordSystems() {
  float value;
  for (int i = 0; i < 9; ++i) {
    bool set = false;
    int offset = i * 20;
    value = 0.0;
    std::string coords = "";
    for (GCodeParserAxis axis : AllAxes()) {
      read_parameter(StringPrintf("%d", 5221 + offset + axis), &value);
      coord_system_[i][axis] = machine_origin_[axis] + value;
      if (axis <= AXIS_Y || value)
        coords += StringPrintf(" %c:%.3f", gcodep_axis2letter(axis), value);
      if (value) set = true;
    }
    if (set) {  // "undefined" message is pretty noisy, only print if set.
      Log_debug("%s offset%s", kCoordinateSystemNames[i],
                set ? coords.c_str() : " undefined");
    }
  }

  if (!read_parameter("5220", &value) || value < 1 || value > 9) {
    value = 1;     // If not set or invalid, force G54
    store_parameter("5220", value);
  }

  const int coord_system = (int)value - 1;
  assert(coord_system >= 0 && coord_system < 9);  // enforced above.

  current_origin_ = &coord_system_[coord_system];
  inform_origin_offset_change(kCoordinateSystemNames[coord_system]);
  Log_debug("Using Coordinate system #5220=%d: %s",
            coord_system + 1, kCoordinateSystemNames[coord_system]);
}

// Set coordinate system data
const char *GCodeParser::Impl::handle_G10(const char *line) {
  AxesRegister coords;
  int l_val = -1;
  int p_val = -1;
  bool have_val[GCODE_NUM_AXES];
  for (GCodeParserAxis a : AllAxes()) {
    have_val[a] = false;
  }

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

  // Right now, we only do G10 origin offsets. If we do more, calling various
  // functions would probably be good.

  if (l_val != 2) {
    gprintf(GLOG_SEMANTIC_ERR, "G10: can only handle G10 L2\n");
    return line;
  }

  if (p_val < 1 || p_val > 9) {
    gprintf(GLOG_SEMANTIC_ERR, "G10 L2 P%d - coordinate system P-value needs "
            "to be between P1..P9\n", p_val);
    return line;
  }

  const int cs = p_val - 1;   // Target coordinate system.

  // Update coordinate system with values that changed.
  for (GCodeParserAxis a : AllAxes()) {
    if (!have_val[a]) continue;
    coord_system_[cs][a] = modal_absolute_g90_
      ? machine_origin_[a] + coords[a]
      : coord_system_[cs][a] + coords[a];
  }

  // Now update the parameters
  const int variable_offset = cs * 20;
  for (GCodeParserAxis a : AllAxes()) {
    if (!have_val[a]) continue;
    // We always store the absolute offset from home.
    store_parameter(StringPrintf("%d", 5221 + variable_offset + a),
                    coord_system_[cs][a] - machine_origin_[a]);
  }
  if (current_origin_ == &coord_system_[cs]) {
    // If this was our currently active coordiate system, we need to inform
    // about the new display offsets.
    inform_origin_offset_change(kCoordinateSystemNames[cs]);
  }

  return line;
}

void GCodeParser::Impl::handle_G90_G91(float value) {
  if (value == 90.0f)
    set_all_axis_to_absolute(true);
  else if (value == 91.0f)
    set_all_axis_to_absolute(false);
  else if (value == 90.1f)
    set_ijk_absolute(true);
  else if (value == 91.1f)
    set_ijk_absolute(false);
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
    gprintf(GLOG_SYNTAX_ERR, "invalid coordinate system %.1f\n", sub_command);
    return;
  }
  store_parameter("5220", coord_system);
  current_origin_ = &coord_system_[coord_system-1];
  inform_origin_offset_change(kCoordinateSystemNames[coord_system-1]);
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
    set_current_offset(global_offset_g92_, "G92");
  }
  else if (sub_command == 92.1f) {   // Reset
    reset_G92();
    set_current_offset(global_offset_g92_, "");
  }
  else if (sub_command == 92.2f) {   // Suspend
    set_current_offset(kZeroOffset, "");
  }
  else if (sub_command == 92.3f) {   // Restore
    set_current_offset(global_offset_g92_, "G92");
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
    (callbacks()->*setter)(factor * value);   // TODO
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
  AxisBitmap_t affected_axes = 0;

  while ((remaining_line = gparse_pair(line, &axis_l, &value))) {
    const float unit_value = value * unit_to_mm_factor_;
    if (axis_l == 'F') {
      feedrate = f_param_to_feedrate(unit_value);
      any_change = true;
    }
    else if (axis_l == 'S') {
      callbacks()->change_spindle_speed(value);
    }
    else {
      const enum GCodeParserAxis update_axis = gcodep_letter2axis(axis_l);
      if (update_axis == GCODE_NUM_AXES)
        break;  // Invalid axis: possibly start of new command.
      new_pos[update_axis] = abs_axis_pos(update_axis, unit_value);
      any_change = true;
      affected_axes |= (1 << update_axis);
    }
    line = remaining_line;
  }

  bool did_move = false;
  if (any_change) {
    callbacks()->clamp_to_range(affected_axes, &new_pos);
    if (modal_g0_g1_) {
      did_move = callbacks()->coordinated_move(feedrate, new_pos);
    } else {
      did_move = callbacks()->rapid_move(feedrate, new_pos);
    }
  }
  if (did_move) {
    axes_pos_ = new_pos;
  }
  return line;
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

  // Quick hack: we should've calculated with absolute center above.
  // Also TODO: ijk_is_absolute_ needs to be honored.
  AxesRegister absolute_center = axes_pos_;
  absolute_center[AXIS_X] += offset[AXIS_X];
  absolute_center[AXIS_Y] += offset[AXIS_Y];
  absolute_center[AXIS_Z] += offset[AXIS_Z];
  if (callbacks()->arc_move(feedrate, arc_normal_, is_cw,
                            axes_pos_, absolute_center, target))
    axes_pos_ = target;
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

  if (callbacks()->spline_move(-1, axes_pos_, cp1, cp2, target))
    axes_pos_ = target;
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
  if (callbacks()->probe_axis(feedrate, AXIS_Z, &probed_pos)) {
    axes_pos_[AXIS_Z] = probed_pos;
    // Doing implicit G92 here. Is this what we want ? Later, this might
    // be part of tool-offset or something.
    global_offset_g92_[AXIS_Z] = (axes_pos_[AXIS_Z] - probe_thickness)
      - current_origin()[AXIS_Z];
    set_current_offset(global_offset_g92_, "G30");
  }
  return line;
}

const char *GCodeParser::Impl::handle_M111(const char *line) {
  if (config_.allow_m111) {
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
void GCodeParser::Impl::ParseBlock(GCodeParser *owner,
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
      callbacks()->gcode_start(owner);
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
      case 90: case 91: handle_G90_G91(value);  break;
      case 92: line = handle_G92(value, line); break;
      default: line = callbacks()->unprocessed(letter, value, line); break;
      }
    }
    else if (letter == 'M') {
      switch ((int) value) {
      case  2: finish_program_and_reset(); break;
      case 17: callbacks()->motors_enable(true); break;
      case 18: callbacks()->motors_enable(false); break;
      case 24: callbacks()->wait_for_start(); break;
      case 30: finish_program_and_reset(); break;
      case 82: axis_is_absolute_[AXIS_E] = true; break;
      case 83: axis_is_absolute_[AXIS_E] = false; break;
      case 84: callbacks()->motors_enable(false); break;
      case 104: line = set_param('S',
                                 &GCodeParser::EventReceiver::set_temperature,
                                 1.0f, line);
        break;
      case 106: line = set_param('S', &GCodeParser::EventReceiver::set_fanspeed,
                                 1.0f, line);
        break;
      case 107: callbacks()->set_fanspeed(0); break;
      case 109:
        line = set_param('S', &GCodeParser::EventReceiver::set_temperature,
                         1.0f, line);
        callbacks()->wait_temperature();
        break;
      case 116: callbacks()->wait_temperature(); break;
      case 111: line = handle_M111(line); break;
      case 220:
        line = set_param('S', &GCodeParser::EventReceiver::set_speed_factor,
                         0.01f, line);
        break;
      case 500: config_.SaveParams(); break;
      case 501: config_.LoadParams(); break;
      default: line = callbacks()->unprocessed(letter, value, line); break;
      }
    }
    else if (letter == 'F') {
      // Feedrate is sometimes used in absence of a move command.
      const float unit_value = value * unit_to_mm_factor_;
      const float feedrate = f_param_to_feedrate(unit_value);
      callbacks()->coordinated_move(feedrate, axes_pos_);  // No move, just feed
    }
    else if (letter == 'N') {
      // Line number? Yeah, ignore for now :)
      processed_command = false;
    }
    else {
      const enum GCodeParserAxis axis = gcodep_letter2axis(letter);
      if (axis == GCODE_NUM_AXES) {
        line = callbacks()->unprocessed(letter, value, line);
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
      callbacks()->gcode_command_done(letter, value);
    }
  }
  err_msg_ = NULL;
}

GCodeParser::GCodeParser(const Config &config, EventReceiver *parse_events)
  : impl_(new Impl(config, parse_events)) {
}
GCodeParser::~GCodeParser() {
  delete impl_;
}
void GCodeParser::ParseBlock(const char *line, FILE *err_stream) {
  impl_->ParseBlock(this, line, err_stream);
}

bool GCodeParser::ReadFile(FILE *input_gcode_stream, FILE *err_stream) {
  if (input_gcode_stream == nullptr) return false;
  char buffer[8192];   // "8kB ought to be enough for everybody"
  while (fgets(buffer, sizeof(buffer), input_gcode_stream) != nullptr) {
    impl_->ParseBlock(this, buffer, err_stream);
  }
  if (err_stream) {
    fflush(err_stream);
  }
  fclose(input_gcode_stream);

  // always call gcode_finished() to disable motors at end of stream
  impl_->callbacks()->gcode_finished(true);
  return true;
}

int GCodeParser::error_count() const { return impl_->error_count(); }

const char *GCodeParser::ParsePair(const char *line,
                                   char *letter, float *value,
                                   FILE *err_stream) {
  return impl_->gcodep_parse_pair_with_linenumber(-1, line, letter, value,
                                                  err_stream);
}
