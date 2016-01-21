BealgeG G-Code interpretation
-----------------------------

##Syntax

G-Code is essentially a pair of 'letters' and 'numbers' that tell the machine
what to do. It is well described [somewhere else][Intro GCode], so this is only a
quick overview.
G-Codes are typically organized in lines. These are typical commands:

    G1 X10 Y20 ; move to X-coordinate 10 and Y=20
    G1X0Y8     ; no need for spaces between pairs - but readability sucks.
    G1 X10 (small move) Y200 (big move) ; comments can be in-line with parenthesis
    G1 X10 Y100 G1 X0 Y0  ; multiple commands can be in one line.

`G1` means: move in a coordinated move, i.e. the axis move in a way that the
final move resembles a straight line in the N-dimensional space they are in.
Comments can be at the end of line and start with a semicolon.

There can be comments _between_ pairs with parenthesis. This is not supported
by every G-Code interpreter, but BeagleG does:

    G1(coordinated move) X10(to this position)

## Supported commands

Supported commands are currently added on a need-to-have basis. They are a subset
of G-Codes found documented in [LinuxCNC] and [RepRap Wiki].

The following commands are supported. A place-holder of `[coordinates]` means
a combination of axis coordinates (such as `X10 Y20`) and an optional feedrate
(`F1000`).
Current set of supported axis-letters is X, Y, Z, E, A, B, C, U, V, W (the
`--axis-mapping` flag decides which make it to motors).

Line numbers `Nxx` and checksums `*xx` are parsed, but discarded and ignored
for now.

### G Codes

Command          | Callback             | Description
---------------- |----------------------|------------------------------------
G0 [coordinates] | `rapid_move()`       | Move to coordinates
G1 [coordinates] | `coordinated_move()` | Like G0, but guarantee linear move
G2 [end] [offset]| `coordinated_move()` | Clockwise arc
G3 [end] [offset]| `coordinated_move()` | Counterclockwise arc
G4 Pnnn          | `dwell()`            | Dwell (wait) for nnn milliseconds.
G17              | -                    | XY plane selection.
G18              | -                    | ZX plane selection.
G19              | -                    | YZ plane selection.
G20              | -                    | Set coordinates to inches.
G21              | -                    | Set coordinates to millimeter.
G28 [coordinates]| `handle_home()`      | Home the machine on given axes.
G30 [Z<thick>]   | `handle_z_probe()`   | Z Probe, with optional target thickness.
G70              | -                    | Set coordinates to inches.
G71              | -                    | Set coordinates to millimeter.
G90              | -                    | Coordinates are absolute.
G91              | -                    | Coordinates are relative.
G92 [coordinates]| -                    | Set position to be the new zero.
G92.1            | -                    | Reset G92 offset
G92.2            | -                    | Suspend G92 offset
G92.3            | -                    | Restore G92 offset

### M Codes

Command          | Callback              | Description
-----------------|-----------------------|-----------------------------
M2               | `gcode_finished()`    | Program end. Resets back to defaults.
M24              | `wait_for_start()`    | Start/resume a program. Waits for the start input if available.
M17              | `motors_enable()`     | Switch on motors.
M18              | `motors_enable()`     | Switch off motors.
M30              | `gcode_finished()`    | Program end. Resets back to defaults.
M84              | `motors_enable()`     | Switch off motors.
M82              | -                     | Set E-axis to absolute.
M83              | -                     | Set E-axis to relative.
M104 Snnn        | `set_temperature()`   | Set temperature in celsius.
M116             | `wait_temperature()`  | Wait for temperature to be reached
M109 Snnn        | `set_t.., wait_t..()` | Combination of M104, M116: Set temperature and wait for it to be reached.
M106 Snnn        | `set_fanspeed()`      | set speed of fan; 0..255
M107             | `set_fanspeed(0)`     | switch off fan.
M111 Snnn        | -                     | Set debug level.
M220 Snnn        | `set_speed_factor()`  | Set output speed factor.

### M Codes dealt with by gcode-machine-control
The standard M-Code are directly handled by the G-code parser and result
in parametrized callbacks. Other not quite standard G-codes are handled in
[gcode-machine-control](./gcode-machine-control.c) when receiving
the `unprocessed()` callback (see API below):

Command          | Description
-----------------|----------------------------------------
M0               | Unconditional stop, sets Software E-Stop (only on capes that have ESTOP_SW_GPIO)
M3 Sxx           | Spindle On Clockwise at speed Sxx
M4 Sxx           | Spindle On Counterclockwise at speed Sxx
M5               | Spindle Off
M7               | Turn mist on
M8               | Turn flood on
M9               | Turn all coolant off
M10              | Turn on vacuum
M11              | Turn off vacuum
M42 Pnn          | Get state of AUX Pin nn.
M42 Pnn Sxx      | Set AUX Pin nn to value xx
M62 Pnn          | Set AUX Pin nn to 1
M63 Pnn          | Set AUX Pin nn to 0
M64 Pnn          | Set AUX Pin nn to 1; updates immediately, independent of buffered moves.
M65 Pnn          | Set AUX Pin nn to 0; updates immediately, independent of buffered moves.
M80              | ATX Power On (only on capes that have MACHINE_PWR_GPIO)
M81              | ATX Power Off (only on capes that have MACHINE_PWR_GPIO)
M105             | Get current extruder temperature.
M114             | Get current position; coordinate units in mm.
M115             | Get firmware version.
M117             | Display message.
M119             | Get endstop status.
M120             | Enable pause switch detection.
M121             | Disable pause switch detection.
M245             | Start cooler
M246             | Stop cooler
M355             | Turn case lights on/off
M999             | Clear Software E-Stop (only on capes that have ESTOP_SW_GPIO)

### Feedrate in Euclidian space
The axes X, Y, and Z are dealt with specially by `gcode-machine-control`: they are
understood as representing coordinates in an Euclidian space (not entirely
unwarranted :) ) and thus applies a feedrate in a way that the resulting
path sees the given speed in space, not each individual axis:

    G28 G1 X100      F100  ; moves X with feedrate 100mm/min
    G28 G1 X100 Y100 F100  ; moves X and Y with feedrate 100/sqrt(2) ~ 70.7mm/min

## API
G-code parsing as provided by [the G-Code parse API](./gcode-parser.h) receives
G-code from a file-descriptor (via the `int gcodep_parse_stream()` function)
and calls parametrized parse-callbacks representing slightly more higher-level commands.

The coordinates passed to callbacks are _always_ pre-converted to machine-absolute and
metric to make implementation of the callback receivers easy.
The codes `G20`/`G21`/`G70`/`G71` and `G90`/`G91`/`G92` as well as
`M82`, `M83` are handled internally to always output absolute, metric coordinates.

Currently, the GCode parser also implements `G2` and `G3` and emits line-segments with
`coordinated_move()` callbacks (this should probably move outside the parser).

Commands that are not recognized are passed on to the `unprocessed()` callback
for the user to handle (see description in API).

[LinuxCNC]: http://linuxcnc.org/docs/html/gcode.html
[RepRap Wiki]: http://reprap.org/wiki/G-code
[Intro GCode]: http://en.wikipedia.org/wiki/G-code
