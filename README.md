BeagleG
=======

Experimental step-motor controller using the PRU capability of the
Beaglebone Black to create precisely timed stepper-pulses (just velocity right
now, no acceleration or jerk implemented).

The motor-interface API allows to enqueue step-{count, frequency}
of 8 steppers that are controlled in a coordinated move (G1), with real-time
controlled steps at rates that can go well beyond 500khz.
So: sufficient even for advanced step motors and drivers :)

The `send-gcode` test-program is parsing G-code, extracting axes moves and
enqueues them to the realtime unit.

## APIs
The functionality is encapsulated in independently usable APIs.

   - `motor-interface.h` : C-API to enqueue motor moves, that are executed
     in the PRU.
   - `gcode-parser.h` : C-API that parses G-code and calls callbacks, while
     taking care of many internals, e.g. it automatically translates everything
     into metric, absolute coordinates.
   - `determine-print-stats.h`: C-API to determine some basic stats about
      a G-Code file; it processes the entire file and determines estimated
      print time, filament used etc. Mostly an example for gcode-parser.h.

## G-Code interpreter
To test things properly, there is a G-code interpreter.

    Usage: ./send-gcode [options] <gcode-filename>
    Options:
      -f <factor> : Print speed factor (Default 1.0).
      -m <rate>   : Max. feedrate (Default 600mm/s).
      -p          : Print motor commands to console (Default: off).
      -n          : Dryrun; don't send to motors (Default: off).
      -s          : Synchronous: don't queue (Default: off).
      -l          : Loop file forever.

The G-code understands axes X, Y, Z, E, A, B, C and maps them to stepper [0..6].

## Pinout

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
