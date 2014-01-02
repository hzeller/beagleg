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
  int provided_axes;
  float unit_to_mm_factor;  // metric: 1.0; imperial 25.4
  int is_absolute;
  float current_feedrate;
  AxesRegister relative_zero;  // reference, set by G92 commands
  AxesRegister axes_pos;
};

static void dummy_set_feedrate(void *user, float f) {
  fprintf(stderr, "GCodeParser: set_feedrate(%.2f)\n", f);
}

static void dummy_move(void *user, const float *axes) {
  fprintf(stderr, "GCodeParser: move(X=%.3f,Y=%.3f,Z=%.3f,E=%.3f,...)\n",
	  axes[AXIS_X], axes[AXIS_Y], axes[AXIS_Z], axes[AXIS_E]);
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

struct GCodeParser *gcodep_new(struct GCodeParserCb *callbacks,
			       void *userdata) {
  GCodeParser_t *result = (GCodeParser_t*)malloc(sizeof(*result));
  memset(result, 0x00, sizeof(*result));
  if (callbacks) {
    memcpy(&result->callbacks, callbacks, sizeof(*callbacks));
  }
  result->cb_userdata = userdata;

  // Set some reasonable defaults for unprovided callbacks:
  if (!result->callbacks.go_home)
      result->callbacks.go_home = &dummy_go_home;
  if (!result->callbacks.set_feedrate)
    result->callbacks.set_feedrate = &dummy_set_feedrate;
  if (!result->callbacks.coordinated_move)
    result->callbacks.coordinated_move = &dummy_move;
  if (!result->callbacks.rapid_move)
    result->callbacks.rapid_move = result->callbacks.coordinated_move;
  if (!result->callbacks.unprocessed)
    result->callbacks.unprocessed = &dummy_unprocessed;
  result->unit_to_mm_factor = 1.0f;
  result->is_absolute = 1;
  return result;
}

void gcodep_delete(struct GCodeParser *parser) {
  free(parser);
}

// Parse next letter/number pair.
// Returns the remaining line or NULL if end reached.
static const char *parse_next_pair(const char *line,
				   char *letter, float *value) {
  if (line == NULL)
    return NULL;
  while (*line && isspace(*line))
    line++;
  if (*line == '\0' || *line == ';' || *line == '%')
    return NULL;
  if (sscanf(line, "%c%f", letter, value) != 2) {
    fprintf(stderr, "Error parsing '%s'\n", line); // TODO: error callback.
    return NULL;
  }
  while (*line && !isspace(*line))
    line++;  // Skip text we just parsed
  while (*line && isspace(*line))
    line++;  // Skip whitespace, maybe we're done anyway.
  return line;  // never NULL because we successfully parsed something.
}

static const char *handle_home(struct GCodeParser *p, const char *line) {
  memset(p->axes_pos, 0x00, sizeof(AxesRegister));
  memset(p->relative_zero, 0x00, sizeof(AxesRegister));

  unsigned char homing_flags = 0;
  char axis;
  float dummy;
  while ((line = parse_next_pair(line, &axis, &dummy))) {
    switch (axis) {
    case 'X': homing_flags |= (1 << AXIS_X); break;
    case 'Y': homing_flags |= (1 << AXIS_X); break;
    case 'Z': homing_flags |= (1 << AXIS_X); break;
    case 'E': homing_flags |= (1 << AXIS_X); break;
    case 'A': homing_flags |= (1 << AXIS_X); break;
    case 'B': homing_flags |= (1 << AXIS_X); break;
    case 'C': homing_flags |= (1 << AXIS_X); break;
    default:
      p->callbacks.go_home(p->cb_userdata,
			   homing_flags != 0 ? homing_flags : kAllAxesBitmap);
      return p->callbacks.unprocessed(p->cb_userdata, axis, dummy, line);
    }
  }
  p->callbacks.go_home(p->cb_userdata,
		       homing_flags != 0 ? homing_flags : kAllAxesBitmap);
  return NULL;
}

static const char *handle_rebase(struct GCodeParser *p, const char *line) {
  char axis;
  float value;
  while ((line = parse_next_pair(line, &axis, &value))) {
    const float unit_value = value * p->unit_to_mm_factor;
    switch (axis) {
    case 'X': p->relative_zero[AXIS_X] = p->axes_pos[AXIS_X] - unit_value; break;
    case 'Y': p->relative_zero[AXIS_Y] = p->axes_pos[AXIS_Y] - unit_value; break;
    case 'Z': p->relative_zero[AXIS_Z] = p->axes_pos[AXIS_Z] - unit_value; break;
    case 'E': p->relative_zero[AXIS_E] = p->axes_pos[AXIS_E] - unit_value; break;
    case 'A': p->relative_zero[AXIS_A] = p->axes_pos[AXIS_A] - unit_value; break;
    case 'B': p->relative_zero[AXIS_B] = p->axes_pos[AXIS_B] - unit_value; break;
    case 'C': p->relative_zero[AXIS_C] = p->axes_pos[AXIS_C] - unit_value; break;
    default:
      return p->callbacks.unprocessed(p->cb_userdata, axis, value, line);
    }
  }
  return NULL;
}

static const char *handle_move(struct GCodeParser *p,
			       void (*fun_move)(void *, const float *),
			       const char *line) {
  char axis;
  float value;
  int any_change = 0;

#define UPDATE_AXIS(name, val) do {			\
    if (p->is_absolute) {				\
      p->axes_pos[name] = p->relative_zero[name] + val;	\
    }							\
    else {						\
      p->axes_pos[AXIS_X] += val;			\
    }							\
    any_change = 1;					\
  } while(0)
  
  while ((line = parse_next_pair(line, &axis, &value))) {
    const float unit_value = value * p->unit_to_mm_factor;
    switch (axis) {
    case 'F': {
      if (p->current_feedrate != unit_value) {
	p->callbacks.set_feedrate(p->cb_userdata, unit_value);
	p->current_feedrate = unit_value;
      }
      break;
    }
    case 'X': UPDATE_AXIS(AXIS_X, unit_value); break;
    case 'Y': UPDATE_AXIS(AXIS_Y, unit_value); break;
    case 'Z': UPDATE_AXIS(AXIS_Z, unit_value); break;
    case 'E': UPDATE_AXIS(AXIS_E, unit_value); break;
    case 'A': UPDATE_AXIS(AXIS_A, unit_value); break;
    case 'B': UPDATE_AXIS(AXIS_B, unit_value); break;
    case 'C': UPDATE_AXIS(AXIS_C, unit_value); break;
    default:
      // Uh, got someething unexpected. Flush current move, the call
      if (any_change) fun_move(p->cb_userdata, p->axes_pos);
      return p->callbacks.unprocessed(p->cb_userdata, axis, value, line);
    }
  }
#undef UPDATE_AXIS

  if (any_change) fun_move(p->cb_userdata, p->axes_pos);
  return NULL;
}

void gcodep_parse_line(struct GCodeParser *p, const char *line) {
  // TODO: strip of Nxx   *xx
  char letter;
  float value;
  while ((line = parse_next_pair(line, &letter, &value))) {
    if (letter == 'G') {
      switch ((int) value) {
      case 0: line = handle_move(p, p->callbacks.rapid_move, line); break;
      case 1: line = handle_move(p, p->callbacks.coordinated_move, line); break;
      case 20: p->unit_to_mm_factor = 25.4f; break;
      case 21: p->unit_to_mm_factor = 1.0f; break;
      case 28: line = handle_home(p, line); break;
      case 90: p->is_absolute = 1; break;
      case 91: p->is_absolute = 0; break;
      case 92: line = handle_rebase(p, line); break;
      default:
	line = p->callbacks.unprocessed(p->cb_userdata, letter, value, line);
	break;
      }
    }
    else {
      line = p->callbacks.unprocessed(p->cb_userdata, letter, value, line);
    }
  }
}
