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
 *
 *   enum GCodeParserAxis - convenient type representing axes in GCode
 *   AxesRegister         - a useful container containing positions of all axes
 *   GCodeParser          - The parser that turns GCode to machine callbacks.
 */

#include <stdint.h>
#include <stdio.h>

#include <string>
#include <map>

#include "common/container.h"

// Axis supported by this parser.
// Sequence matters, as these determine the 52xx variables
enum GCodeParserAxis {
  AXIS_X, AXIS_Y, AXIS_Z,
  AXIS_A, AXIS_B, AXIS_C,
  AXIS_U, AXIS_V, AXIS_W,
  AXIS_E,
  GCODE_NUM_AXES
};
typedef EnumIterable<GCodeParserAxis, AXIS_X, GCODE_NUM_AXES> AllAxes;

// Convenient type: a register to store machine coordinates.
typedef FixedArray<float, GCODE_NUM_AXES, GCodeParserAxis> AxesRegister;

// Bitmap of axis bools. Test with usual (1<<AXIS_X)
typedef uint32_t AxisBitmap_t;

// Maps axis enum to letter. AXIS_Z -> 'Z'
char gcodep_axis2letter(enum GCodeParserAxis axis);

// Case-insensitively maps an axis letter to the GCodeParserAxis enumeration
// value.
// Returns GCODE_NUM_AXES on invalid character.
enum GCodeParserAxis gcodep_letter2axis(char letter);

inline bool is_rotational_axis(GCodeParserAxis axis) {
  return axis == AXIS_A || axis == AXIS_B || axis == AXIS_C;
}

// A parser for GCode that handles all the nitty gritty details of GCode,
// then sends the resulting machine path to the EventReceiver callbacks.
//
// The parser handles coordinate systems, offsets, variables, expression
// evaluation, loops etc. and sends callbacks with plain numbers in absolute
// coordinates based on (0,0,0) of the machine cube.
//
// The parser neesds a configuration and an implementation of the EventReceiver
// that processes the callbacks coming from the parser.
class GCodeParser {
public:
  class EventReceiver;
  struct Config;

  // Create a parser with the given config, emitting parse events
  // to "parse_events".
  GCodeParser(const Config &config, EventReceiver *parse_events);
  ~GCodeParser();

  // Main workhorse: Parse a gcode block (a line), call callbacks if needed.
  // If "err_stream" is non-NULL, sends error messages that way.
  void ParseBlock(const char *line, FILE *err_stream);

  // Convenience function: Read gcode from file. This reads the file
  // line-by-line, parses these blocks and call the EventReceiver.
  // Closes input stream after EOF.
  // The input is expected to be a stream with no stalls, so no input_idle()
  // will be called (Reading from a socket ? Use GCodeStreamer instead.).
  //
  // Error messages are sent to "err_stream" if non-NULL.
  // Returns true on success.
  bool ReadFile(FILE *input_gcode_stream, FILE *output_err_stream);

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
  // fills in variables pointed to by "letter" and "value". "letter" is
  // guaranteed to be upper-case.
  //
  // Returns the remainder of the line or NULL if no pair has been found and the
  // end-of-string has been reached.
  //
  // Resolves variables.
  const char *ParsePair(const char *line, char *letter, float *value,
                        FILE *err_stream);

  // Number of errors seen.
  int error_count() const;

private:
  class Impl;
  Impl *impl_;
};

// Configuration for the parser.
struct GCodeParser::Config {
  typedef std::map<std::string, float> ParamMap;
  Config() : parameters(NULL) {}
  Config(const std::string &filename) : parameters(NULL), paramfile(filename) {}

  bool LoadParams();
  bool SaveParams() const;

  // Allow using M111 to change debug messages.
  bool allow_m111 = false;

  // The machine origin. This is where the end-switches are. Typically,
  // for CNC machines, that might have Z at the highest point for instance,
  // while 3D printers have Z at zero.
  AxesRegister machine_origin;

  // The NIST-RS274NGC parameters/variables.
  // This maps the name to the value of the parameter. The original RS274
  // only supports integer variables, but we allow arbitrary variable names.
  ParamMap *parameters;

private:
  const std::string paramfile;
};

// Parse Event Callbacks called by the parser and to be implemented by the
// user with meaningful actions.
//
// The units in these callbacks are always mm and always absolute: the parser
// takes care of interpreting G20/G21, G90/G91/G92, coordinate systems,
// expression evaluation etc. internally.
//
// Rotational axes units are in degree (but not well tested yet; 'should'
// work, but no guarantees).
//
// Also see ../G-code.md
class GCodeParser::EventReceiver {
public:
  virtual ~EventReceiver() {}
  // Start program. Use for initialization. Informs about gcode parser so that
  // it is possible to call ParsePair().
  virtual void gcode_start(GCodeParser *parser) = 0;

  // End of program or stream.
  virtual void gcode_finished(bool end_of_stream) {}

  // The parser handles relative positions and coordinate systems internally,
  // so the machine does not have to worry about that: all move commands
  // are always given relative to (0,0,0) of the machine cube.
  //
  // For display purposes however, the machine might be interested in the
  // current offset that applies.
  // This callback informs about the current origin used for absolute
  // positions in GCode.
  // The "offset" if provided relative to (0,0,0) of the machine cube.
  // (If needed after the callback returns, the receiver needs to make a copy
  //  of the register.)
  // The "named_offset" gives the name of the coordinate system that is,
  // e.g. 'G54'.
  virtual void inform_origin_offset(const AxesRegister &offset,
                                    const char *named_offset) {}

  // "gcode_command_done" is always executed when a command is completed,
  // which is after internally executed ones (such as G21) or commands that
  // have triggered a callback. Mostly FYI, you can use this for logging or
  // might use this to send "ok\n" depending on the client implementation.
  virtual void gcode_command_done(char letter, float val) {}

  // If the input has been idle and we haven't gotten any new line for more
  // than 50ms, this function is called (repeately, until there is input again).
  // Use this to do whatever other maintenance might be needed.
  // The first call to input_idle() after data has been processed
  // will have "is_first" set.
  virtual void input_idle(bool is_first) {}

  // G24: Start/resume. Waits for the start input if available.
  virtual void wait_for_start() {}

  // G28: Home all the axis whose bit is set. e.g. (1<<AXIS_X) for X
  // After that, the parser assume to be at the machine_origin as set in
  // the GCodeParserConfig for the given axes.
  virtual void go_home(AxisBitmap_t axis_bitmap) = 0;

  // G30: Probe Z axis to travel_endstop. Returns 'true' if the receiver
  // successfully probed the position and returned it in "probed_position".
  // The value represents the actual position reached within the machine
  // cube for the queried axis (in mm).
  virtual bool probe_axis(float feed_mm_p_sec, enum GCodeParserAxis axis,
                          float *probed_position) { return false; }

  // TODO: M3/M4 should be dealt with in the parser.
  //TODOvirtual void set_spindle_on(bool ccw, float value) {}
  //TODOvirtual void set_spindle_off() {}

  // Change of spindle speed in the current mode.
  // This might be part of a G0/G1 code.
  virtual void change_spindle_speed(float value) {}

  virtual void set_speed_factor(float factor) = 0;// M220 feedrate factor 0..1
  virtual void set_fanspeed(float value) = 0;     // M106, M107: speed 0...255
  virtual void set_temperature(float degrees_c)=0; // M104, M109: Set temp. in Celsius
  virtual void wait_temperature() = 0;    // M109, M116: Wait for temp. reached.
  virtual void dwell(float time_ms) = 0;     // G4: dwell for milliseconds.
  virtual void motors_enable(bool enable) = 0;   // M17, M84, M18: Switch on/off motors

  // Give receiver an opportunity to modify a target coordinate, e.g. clamp
  // ranges before executing a G0/G1 move to prevent hitting a range-check
  // later.
  // Note, if you clamp values, following GCode will be relative to the new
  // coordinate, so thoughtful consideration is needed; often it is better
  // to go into soft-EStop in range conditions.
  // (typically, only Z-clamping makes sense, so this is what it is called
  // for now).
  // [This API might change before things are fully settled]
  virtual void clamp_to_range(AxisBitmap_t affected, AxesRegister *axes) {}

  // G1 (coordinated move) and G0 (rapid move). Move to absolute coordinates.
  // First parameter is feedrate in mm/sec if provided, or -1 otherwise.
  //   (typically, the user would need to remember the positive values).
  // The second parameter is an array of absolute coordinates (already
  // preprocessed to be in millimeter), indexed by GCodeParserAxis.
  // Returns true if the move was successful or false if the machine could
  // not move (e.g. because it was beyond limits or other condition).
  virtual bool coordinated_move(float feed_mm_p_sec,
                                const AxesRegister &absolute_pos) = 0;  // G1
  virtual bool rapid_move(float feed_mm_p_sec,
                          const AxesRegister &absolute_pos) = 0;        // G0

  // G2, G3
  // Arc in a circular motion from current position around the "center"
  // coordinate to the "end" coordinate. These coordinates are absolute.
  // The normal axis is one of AXIS_X...AXIS_Z (most common probably AXIS_Z).
  // Movement outside the axes orthogonal to the normal axis are linearly
  // interpolated from their current position (e.g. creating a spiral).
  //
  // The default implementation linearlizes it and calls coordinated_move()
  // with small line segments.
  //
  // TODO(hzeller): We could probably generalize this by having a
  //  'normal vector' instead of normal_axis + clockwise. This would allow for
  //  arbitrarily placed arcs in space (but there is no GCode for it).
  virtual void arc_move(float feed_mm_p_sec,
                        GCodeParserAxis normal_axis, bool clockwise,
                        const AxesRegister &start,
                        const AxesRegister &center,
                        const AxesRegister &end);

  // G5, G5.1
  // Move in a cubic spine from absolute "start" to "end" given the absolute
  // control points "cp1" and "cp2".
  // The default implementation linearlizes curve and calls coordinated_move()
  // with the segments.
  virtual void spline_move(float feed_mm_p_sec,
                           const AxesRegister &start,
                           const AxesRegister &cp1, const AxesRegister &cp2,
                           const AxesRegister &end);

  // Hand out G-code command that could not be interpreted.
  // Parameters: letter + value of the command that was not understood,
  // string of rest of line (the letter is always upper-case).
  // Should return pointer to remaining line that has not been processed
  // or NULL if the whole remaining line was consumed (or you want to skip
  // this part).
  // Implementors might want to use GCodeParser::ParsePair() if they need
  // to read G-code words from the remaining line.
  virtual const char *unprocessed(char letter, float value,
                                  const char *rest_of_line) = 0;
};

#endif  // _BEAGLEG_GCODE_PARSER_H_
