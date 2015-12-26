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
#include "container.h"

// Axis supported by this parser.
enum GCodeParserAxis {
  AXIS_X, AXIS_Y, AXIS_Z,
  AXIS_E,
  AXIS_A, AXIS_B, AXIS_C,
  AXIS_U, AXIS_V, AXIS_W,
  GCODE_NUM_AXES
};

// Convenient type: a register to store machine coordinates.
typedef FixedArray<float, GCODE_NUM_AXES> AxesRegister;

// Bitmap of axis bools. Test with usual (1<<AXIS_X)
typedef uint32_t AxisBitmap_t;

// Maps axis enum to letter. AXIS_Z -> 'Z'
char gcodep_axis2letter(enum GCodeParserAxis axis);

// Case-insensitively maps an axis letter to the GCodeParserAxis enumeration
// value.
// Returns GCODE_NUM_AXES on invalid character.
enum GCodeParserAxis gcodep_letter2axis(char letter);

class GCodeParser {
public:
  // Parse Event Callbacks called by the parser and to be implemented by the
  // user with meaningful actions.
  //
  // The units in these callbacks are always mm and always absolute: the parser
  // takes care of interpreting G20/G21, G90/G91/G92 internally.
  // (TODO: rotational axes are probably to be handled differently).
  // Also see ../G-code.md
  class Events {
  public:
    virtual ~Events() {}
    virtual void gcode_start();    // Start program. Use for initialization.
    virtual void gcode_finished(); // End of program or stream.

    // The parser handles relative positions and coordinate systems internally,
    // so the machine does not have to worry about that.
    // But for display purposes, the machine might be interested in the current
    // offset that applies.
    // This callback informs about the current origin, relative to the (0/0/0)
    // machine cube.
    // If needed after the callback returns, the receiver needs to make a copy
    // of the register.
    virtual void inform_origin_offset(const AxesRegister& offset);

    // "gcode_command_done" is always executed when a command is completed,
    // which is after internally executed ones (such as G21) or commands that
    // have triggered a callback. Mostly FYI, you can use this for logging or
    // might use this to send "ok\n" depending on the client implementation.
    virtual void gcode_command_done(char letter, float val);

    // If the input has been idle and we haven't gotten any new line for more
    // than 50ms, this function is called (repeately, until there is input again).
    // Use this to do whatever other maintenance might be needed.
    virtual void input_idle();

    // G24: Start/resume
    // If machine has a START_GPIO this callback will wait until it is low
    // before continuing to parse commands.
    virtual void wait_for_start();

    // G28: Home all the axis whose bit is set. e.g. (1<<AXIS_X) for X
    // After that, the parser assume to be at the machine_origin as set in
    // the GCodeParserConfig for the given axes.
    virtual void go_home(AxisBitmap_t axis_bitmap);

    // G30: Probe Z axis to travel_endstop. Returns 'true' if the receiver
    // successfully probed the position and returned it in "probed_position".
    // The value represents the actual position reached within the machine
    // cube for the queried axis (in mm).
    virtual bool probe_axis(float feed_mm_p_sec, enum GCodeParserAxis axis,
                            float *probed_position);

    virtual void set_speed_factor(float factor);   // M220 feedrate factor 0..1
    virtual void set_fanspeed(float value);        // M106, M107: speed 0...255
    virtual void set_temperature(float degrees_c); // M104, M109: Set temp. in Celsius
    virtual void wait_temperature();       // M109, M116: Wait for temp. reached.
    virtual void dwell(float time_ms);     // G4: dwell for milliseconds.
    virtual void motors_enable(bool enable);   // M17, M84, M18: Switch on/off motors

    // G1 (coordinated move) and G0 (rapid move). Move to absolute coordinates.
    // First parameter is feedrate in mm/sec if provided, or -1 otherwise.
    //   (typically, the user would need to remember the positive values).
    // The second parameter is an array of absolute coordinates (already
    // preprocessed to be in millimeter), indexed by GCodeParserAxis.
    // Returns 1, if the move was successful or 0 if the machine could not move
    // the machine (e.g. because it was beyond limits or other condition).
    virtual bool coordinated_move(float feed_mm_p_sec,
                                  const AxesRegister &absolute_pos);  // G1
    virtual bool rapid_move(float feed_mm_p_sec,
                            const AxesRegister &absolute_pos);        // G0

    // Hand out G-code command that could not be interpreted.
    // Parameters: letter + value of the command that was not understood,
    // string of rest of line (the letter is always upper-case).
    // Should return pointer to remaining line that has not been processed
    // or NULL if the whole remaining line was consumed (or you want to skip
    // this part).
    // Implementors might want to use GCodeParser::ParsePair() if they need
    // to read G-code words from the remaining line.
    virtual const char *unprocessed(char letter, float value,
                                    const char *rest_of_line);
  };

  // Configuration for the parser.
  struct Config {
    // The machine origin. This is where the end-switches are. Typically,
    // for CNC machines, that might have Z at the highest point for instance,
    // while 3D printers have Z at zero.
    AxesRegister machine_origin;

    // TODO(hzeller): here, there should be references to registers
    // G54..G59. They can be set externally to GCode by some operating
    // terminal, but need to be accessed in gcode.
    //float *position_register[6];  // register G54..G59
  };

public:
  // Create a parser with the given config, emitting parse events
  // to "parse_events".
  GCodeParser(const Config &config, Events *parse_events);
  ~GCodeParser();

  // Main workhorse: Parse a gcode line, call callbacks if needed.
  // If "err_stream" is non-NULL, sends error messages that way.
  void ParseLine(const char *line, FILE *err_stream);

  // Read and parse GCode from "input_fd" and call callbacks.
  // Error messages are sent to "err_stream" if non-NULL.
  // Reads until EOF (returns 0) or signal occured (returns 2).
  // The input file descriptor is closed.
  int ParseStream(int input_fd, FILE *err_stream);

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
  // fills in variables pointed to by "letter" and "value". "letter" is guaranteed
  // to be upper-case.
  //
  // Returns the remainder of the line or NULL if no pair has been found and the
  // end-of-string has been reached.
  static const char *ParsePair(const char *line, char *letter, float *value,
                               FILE *err_stream);

private:
  class Impl;
  Impl *impl_;
};


#endif  // _BEAGLEG_GCODE_PARSER_H_
