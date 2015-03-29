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
#ifndef _BEAGLEG_GCODE_PARSER_H
#define _BEAGLEG_GCODE_PARSER_H
/*
 * Parser for commong G-codes. Translates movements into absolute mm coordinates
 * and calls callbacks on changes.
 * All un-implemented G- and M-Codes are sent back via a callback for the
 * user to handle.
 *
 * See G-code.md for documentation.
 */

#include <stdint.h>
#include <stdio.h>

typedef uint32_t AxisBitmap_t;

// Axis supported by this parser.
enum GCodeParserAxis {
  AXIS_X, AXIS_Y, AXIS_Z,
  AXIS_E,
  AXIS_A, AXIS_B, AXIS_C,
  AXIS_U, AXIS_V, AXIS_W,
  GCODE_NUM_AXES
};

// Maps axis enum to letter. AXIS_Z -> 'Z'
char gcodep_axis2letter(enum GCodeParserAxis axis);

// Case-insensitively maps an axis letter to the GCodeParserAxis enumeration 
// value.
// Returns GCODE_NUM_AXES on invalid character.
enum GCodeParserAxis gcodep_letter2axis(char letter);

// Callbacks called by the parser and to be implemented by the user
// with meaningful actions.
//
// The units in these callbacks are always mm and always absolute: the parser
// takes care of interpreting G20/G21, G90/G91/G92 internally.
// (TODO: rotational axes are probably to be handled differently).
//
// The first parameter in any callback is the "user_data" data pointer member.
struct GCodeParserCb {
  void *user_data;  // Context which is passed in each call of these functions.

  void (*gcode_start)(void *);             // FYI: Start parsing. Use for initialization.
  void (*gcode_finished)(void *);          // FYI: Finished parsing.

  // G28: Home all the axis whose bit is set. e.g. (1<<AXIS_X) for X
  void (*go_home)(void *, AxisBitmap_t axis_bitmap);

  void (*set_speed_factor)(void *, float); // M220 feedrate factor 0..1
  void (*set_fanspeed)(void *, float);     // M106, M107: speed 0...255
  void (*set_temperature)(void *, float);  // M104, M109: Set temp. in Celsius.
  void (*wait_temperature)(void *);        // M109, M116: Wait for temp. reached.
  void (*dwell)(void *, float);            // G4: dwell for milliseconds.
  void (*motors_enable)(void *, char b);   // M17,M84,M18: Switch on/off motors
                                           // b == 1: on, b == 0: off.

  // G1 (coordinated move) and G0 (rapid move). Move to absolute coordinates. 
  // First parameter is the userdata.
  // Second parameter is feedrate in mm/sec if provided, or -1 otherwise.
  //   (typically, the user would need to remember the positive values).
  // The third parameter is an array of absolute coordinates (in mm), indexed
  // by GCodeParserAxis.
  void (*coordinated_move)(void *, float feed_mm_p_sec, const float[]);  // G1
  void (*rapid_move)(void *, float feed_mm_p_sec, const float[]);        // G0

  // Hand out G-code command that could not be interpreted.
  // Parameters: letter + value of the command that was not understood,
  // string of rest of line.
  // Should return pointer to remaining line after processed (after all consumed
  // parameters) or NULL if the whole remaining line was consumed.
  // Implementors might want to use gcodep_parse_pair() if they need to read
  // G-code words from the remaining line.
  const char *(*unprocessed)(void *, char letter, float value, const char *);
};

// Read and parse GCode from input_fd and call callbacks
// in "parse_events". Error messages are sent to "err_stream" if non-NULL.
// Reads until EOF. The input file descriptor is closed.
int gcodep_parse_stream(int input_fd,
                        struct GCodeParserCb *parse_events, FILE *err_stream);

// Utility function: Parses next pair in the line of G-code (e.g. 'P123' is
// a pair of the letter 'P' and the value '123').
// Takes care of skipping whitespace, comments etc.
//
// If "err_stream" is non-NULL, sends error messages that way.
//
// Can be used by implementors of unprocessed() to parse the remainder of the
// line they received.
//
// Parses "line". If a pair could be parsed, returns non-NULL value and
// fills in variables pointed to by "letter" and "value".
//
// Returns the remainder of the line or NULL if no pair has been found and the
// end-of-string has been reached.
const char *gcodep_parse_pair(const char *line, char *letter, float *value,
			      FILE *err_stream);

#endif  // _BEAGLEG_GCODE_PARSER_H_
