BealgeG G-Code interpretation
-----------------------------

##Synatx

G-Code is essentially a pair of 'letters' and 'numbers' that tell the machine
what to do. It is well described somewhere else, so this is only a
quick overview.
G-Codes are typically organized in lines. This is a typical command:

    G1 X10 Y20 ; move to X-coordinate 10 and Y=20
    G1X0Y8     ; no need for spaces between pairs - but readability sucks.

`G1` means: move in a coordinated move, i.e. the axis move in a way that the
final move resembles a straight line in the N-dimensional space they are in.
Comments can be at the end of line and start with a semicolon.

There can be comments _between_ pairs with parenthesis. This is not supported
by every G-Code interpreter, but BeagleG does:

    G1(coordinated move) X10(to this position)


##API
G-code parsing as provided by [the G-Code parse API](./gcode-parser.h) receives
G-code either from a string or a file-descriptor and calls parametrized
parse-callbacks representing slightly more higher-level commands.
The callbacks are defined in a struct

```c
struct GCodeParserCb {
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
```

##Supported commands

The following commands are supported. A place-holder of `[coordinates]` means
a combination of axis koordinates (such as `X10 Y20`) and an optional feedrate
(`F1000`).
Current set of supported axis-letters is X, Y, Z, E, A, B, C, U, V, W.

Callbacks that take coordinates are _always_ pre-converted to
machine-absolute and metric. The codes G20/G21 and G90/G91/G92 as well as
M82, M83 are handled internally to always output absolute, metric coordinates.

Commands that are not recognized are passed on to the `unprocessed()` callback
for the user to handle (see description in API).

Line numbers `Nxx` and checksums `*xx` are parsed and discarded, but ignored
for now.

###G Codes

Command          | Callback             | Description
---------------- |----------------------|------------------------------------
G0 [coordinates] | `rapid_move()`       | Move to coordinates
G1 [coordinates] | `coordinated_move()` | Like G0, but guarantee linear move
G4 Pnnn          | `dwell()`            | Dwell (wait) for nnn milliseconds.
G20              | -                    | Set coordinates to inches.
G21              | -                    | Set coordinates to millimeter.
G28 [coordinates]| `handle_home()`      | Home the machine on given axes.
G90              | -                    | Coordinates are absolute.
G91              | -                    | Coordinates are relative.
G92 [coordinates]| -                    | Set position to be the new zero.

###M Codes

Command          | Callback              | Description
-----------------|-----------------------|-----------------------------
M17              | `motors_enable()`     | Switch on motors.
M18              | `motors_enable()`     | Switch off motors.
M84              | `motors_enable()`     | Switch off motors.
M82              | -                     | Set E-axis to absolute.
M83              | -                     | Set E-axis to relative.
M104 Snnn        | `set_temperature()`   | Set temperature in celsius.
M116             | `wait_temperature()`  | Wait for temperature to be reached
M109 Snnn        | `set_t.., wait_t..()` | Combination of M104, M116: Set temperature and wait for it to be reached.
M106             | `set_fanspeed()`      | set speed of fan; 0..255
M107             | `set_fanspeed(0)`     | switch off fan.
M220 Snnn        | `set_speed_factor()`  | Set output speed factor.

###M Codes dealt with by machine-control
The standard M-Code are directly handled by the G-code parser and result
in callbacks. Other not quite standard G-codes are handled in machine-control.

Command          | Description
-----------------|----------------------------------------
M105             | Get current extruder temperature.
M114             | Get current position; coordinate units in mm.
M115             | Get firmware version.
