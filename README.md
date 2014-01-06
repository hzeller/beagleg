BeagleG
=======

Experimental step-motor controller using the PRU capability of the
Beaglebone Black to create precisely timed stepper-pulses (just velocity right
now, no acceleration or jerk implemented).

The motor-interface API allows to enqueue step-{count, frequency}
of 8 steppers that are controlled in a coordinated move (G1), with real-time
controlled steps at rates that can go well beyond 500kHz.
So: sufficient even for advanced step motors and drivers :)

The `send-gcode` test-program is parsing G-Code, extracting axes moves and
enqueues them to the realtime unit.

## APIs
The functionality is encapsulated in independently usable APIs.

   - `motor-interface.h` : C-API to enqueue motor moves, that are
      executed in the PRU.
   - `gcode-parser.h` : C-API that parses G-Code and calls callbacks, while
      taking care of many internals, e.g. it automatically translates everything
      into metric, absolute coordinates.
   - `determine-print-stats.h`: C-API to determine some basic stats about
      a G-Code file; it processes the entire file and determines estimated
      print time, filament used etc. Implementation is mostly an example using
      gcode-parser.h.
   - `gcode-machine-control.h` : highlevel C-API to control a machine via
      G-Code: it reads G-Code and emits the necessary machine commands.

## Machine control binary
To test things properly, there is a G-Code interpreter that you can either give
a filename, or a port to listen to.

    Usage: ./send-gcode [options] [<gcode-filename>]
    Options:
      -f <factor> : Print speed factor (Default 1.0).
      -m <rate>   : Max. feedrate (Default 600mm/s).
      -l <port>   : Listen on this TCP port on 0.0.0.0.
      -n          : Dryrun; don't send to motors (Default: off).
      -P          : Verbose: Print motor commands (Default: off).
      -S          : Synchronous: don't queue (Default: off).
      -R          : Repeat file forever.

The G-Code understands axes X, Y, Z, E, A, B, C and maps them to stepper [0..6].

### Examples

    ./send-gcode -f 10 -m 1000 -R myfile.gcode

Output the file `myfile.gcode` in 10x the original speed, with a feedrate
capped at 1000mm/s. Repeat this file forever (say you want to stress-test).


    ./send-gcode -l 4444

Listen on TCP port 4444 for incoming connections and execute G-Codes over this
line. So you could use `telnet` to have an interactive session or send a file
with `socat`:

     cat myfile.gcode | socat -t5 - TCP4:beaglebone-hostname:4444

Use `socat`, don't use the ancient `nc` (netcat) - its buffering seems to be
broken so that it can get stuck. With `socat`, it should be possible to connect
to a pseudo-terminal in case your printer-software only talks to a terminal
(haven't tried that yet, please let me know if it works).

Note, there can only be one open TCP connection at any given time.

## Pinout

These are the GPIO bits associated with the motor outputs. The actual physical
pins are all over the place on the Beaglebone Black extension headers P8 and P9,
check the mapping in your BBB documentation.

    Axis G-Code name  |  X   Y   Z   E   A   B   C  <unassigned>
    Step     : GPIO-0 |  2,  3,  4,  5,  7, 14, 15, 20
    Direction: GPIO-1 | 12, 13, 14, 15, 16, 17, 18, 19

For your interface: note this is 3.3V level (and assume not more than ~4mA).

## Build
The Makefile is assuming that you build this either on the Beaglebone Black
directly, or using a cross compiler (see Makefile).
You need to have checked out https://github.com/beagleboard/am335x_pru_package
which provides the pasm PRU assembler and the library to push this code to the
PRU.

## License
BeagleG is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

## TODO
   - Implement acceleration and jerk.
   - Read end-switches
   - Needed for full 3D printer solution: add PWM for heaters.
   - Fast stop without waiting for queues to empty, but still be able to
     recover exact last position.
   - ...
