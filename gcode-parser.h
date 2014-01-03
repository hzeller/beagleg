/*
 * (c) 2013, 1014 Henner Zeller <h.zeller@acm.org>
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

/*
 * Parser for commong G-codes. Translates movements into absolute mm coordinates
 * and calls callbacks on changes.
 * Simple implementation, by no means complete.
 */

typedef struct GCodeParser GCodeParser_t;  // Opaque type with the parser object.

enum GGodeParserAxes {
  AXIS_X, AXIS_Y, AXIS_Z,
  AXIS_E,
  AXIS_A, AXIS_B, AXIS_C,
  GCODE_NUM_AXES
};

// Callbacks called by the parser.
// The units in these callbacks are alway mm and always absolute. The parser
// takes care of interpreting G20/G21, G90/G91/G92; the output units are always
// in absolute millimeter. (TODO: rotational axes are probably to be handled
// differently).
struct GCodeParserCb {
  // Home all the axis whose bit is set. e.g. (1<<AXIS_X) for X
  void (*go_home)(void *, unsigned char axis_bitmap);

  // Set feedrate for the following G-commands. Parameter is in mm/min
  void (*set_feedrate)(void *, float);

  void (*set_fanspeed)(void *, float);  // 0 == off; 255 == full speed

  // Set temperature to given temperature in Celsius.
  void (*set_temperature)(void *, float);
  void (*wait_temperature)(void *);
  void (*dwell)(void *, float);

  void (*disable_motors)(void *);

  // Coordinated move to absolute coordinates of motor vector.
  // (Unused axes always stay at 0).
  // Parameter: vector, number of elements (Axes are: X, Y, Z, E, A, B, C)
  void (*coordinated_move)(void *, const float *);
  void (*rapid_move)(void *, const float *);   // like coordinated_move()

  // Parameters: letter + value of the command that could not be processed.
  // string of rest of line.
  // Should return pointer to remaining line after processed or NULL if it
  // consumed it all.
  const char *(*unprocessed)(void *, char letter, float value, const char *);
};


// Initialize parser. It gets a string of maximum 8 characters naming the
// axis and their correspondence to the output array. E.g. "XYZEABC"
// Returns an opaque type used in the parse functions.
// Does not take ownership of the axes string or callbacks.
GCodeParser_t *gcodep_new(struct GCodeParserCb *callbacks,
			  void *callback_context);
void gcodep_delete(GCodeParser_t *object);

// Parse a gcode line, call callbacks if needed.
void gcodep_parse_line(GCodeParser_t *obj, const char *line);
