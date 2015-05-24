BeagleG
=======

(I am experimenting with the acceleration planning and jerk right now, so don't expect
 everything 100% to work every time)

Step-motor controller (and eventually 3D printer controller) using the PRU
capability of the Beaglebone Black to create precisely timed stepper-pulses for
acceleration and travel (right now: trapezoidal motion profile).

See example here: http://www.youtube.com/watch?v=hIEY9077D64

The motor-operations API allows to enqueue operations with speed changes
(transition between segments) or fixed speed (travel) of 8 steppers that are
controlled in a coordinated move (G1), with real-time
controlled steps at rates that can go beyond 500kHz.
So: sufficient even for advanced step motors and drivers :)

The jerk/{accl-,decel-}eration motion profile is entirely
created within the PRU from parameters sent by the host CPU (i.e. BeagleBone ARM)
via a ring-buffer.
The host CPU prepares the data, such as parsing the [G-Code](./G-code.md) and
doing travel planning, while all the real-time critical parts are
done in the PRU. The host program needs less than 1% CPU-time processing a
typical G-Code file.

The `machine-control` program is parsing G-Code, extracting axes moves and
enqueues them to the realtime unit.

## APIs
The functionality is encapsulated in independently usable APIs.

   - [gcode-parser.h](./gcode-parser.h) : C-API for parsing
      [G-Code](./G-code.md) that calls callback parse events, while taking
      care of many internals, e.g. it automatically translates
      everything into metric, absolute coordinates.

   - [gcode-machine-control.h](./gcode-machine-control.h) : highlevel C-API to
      control a machine via G-Code: it receives G-Code events (implementing the
      callbacks called by the parser), does motion
      planning, axis mapping, speed/accleration segment joining and emits
      the necessary machine commands to MotorOperations.
      Depends on the motor-operations and gcode-parser APIs.
      Provides the functionality provided by the `machine-control` binary.

   - [motor-operations.h](./motor-operations.h) : Low-level motor motion C-API.
      Receives travel speeds or speed transitions and prepares parameters
      for the discrete approximation in the motion-queue backend.
      The print-stats binary has a different implementation that uses it to
      determine print-time calculations.

   - [motion-queue.h](./motion-queue.h) : Even lower level interface: queue
      between motor-operations and hardware creating motion profiles in realtime.
      The implementation is done in the BeagleBone-PRU, but is separated out
      enough that it is not dependent on it: The required operations could be
      implemented in microcontrollers or FPGAs (32 bit operations help...).
      There is a simulation implementation (sim-firmware.c) that illustrates
      what to do with the parameters. The simulation just outputs the would-be
      result as CSV file (good for debugging).

   - `determine-print-stats.h`: C-API to determine some basic stats about
      a G-Code file; it processes the entire file and determines estimated
      print time, filament used etc. Implementation is mostly an example using
      gcode-parser.h.
      Used in the `gcode-print-stats` binary.

The interfaces are typically C-structs with function pointers which allows for
easy testing or simple translation into languages such as Go or C++.

## Build
To build, we need the BeagleG code and the PRU assembler with supporting library.
The BeagleG repository is set up in a way that the PRU assembler is checked out via
a sub-module to make these things simple.

Clone the BeagleG repository with the `--recursive` flag to get this sub-module

     git clone --recursive https://github.com/hzeller/beagleg.git

(If you are a github user, you might want to use the git protocol).

Then just

     cd beagleg
     make

If you are looking at the code and developing on a non-Beaglebone machine, pass an empty
`ARM_COMPILE_FLAGS` environment variable:

     ARM_COMPILE_FLAGS="" make

## Getting started
Before you can use beagleg and get meaningful outputs on the GPIO pins,
we have to tell the pin multiplexer to connect them to the output pins. For
that, just run the start-devicetree-overlay.sh script with your hardware
to install the device overlay.

    sudo hardware/start-devicetree-overlay.sh hardware/BUMPS/BeagleG.dts

See the [Hardware page](./hardware) how to enable the cape at boot time.

## Machine control binary
To control a machine with G-Code, use the `machine-control` binary.
This either takes a filename or a TCP port to listen on.

    Usage: ./machine-control [options] [<gcode-filename>]
    Options:
      --steps-mm <axis-steps>   : steps/mm, comma separated[*] (Default 160,160,160,40,0, ...).
                                  (negative for reverse)
      --max-feedrate <rate> (-m): Max. feedrate per axis (mm/s), comma separated[*] (Default: 200,200,90,10,0, ...).
      --accel <accel>       (-a): Acceleration per axis (mm/s^2), comma separated[*] (Default 4000,4000,1000,10000,0, ...).
      --axis-mapping            : Axis letter mapped to which motor connector (=string pos)
                                  Use letter or '_' for empty slot.
                                  You can use the same letter multiple times for mirroring.
                                  Use lowercase to reverse. (Default: 'XYZEA')
      --channel-layout          : Driver channel (0..7) mapped to which motor connector (=string pos)
                                  This depends on the harware mapping of the cape (Default for BUMPS: '23140').
      --port <port>         (-p): Listen on this TCP port for GCode.
      --bind-addr <bind-ip> (-b): Bind to this IP (Default: 0.0.0.0).
      -f <factor>               : Print speed factor (Default 1.0).
      -n                        : Dryrun; don't send to motors (Default: off).
      -P                        : Verbose: Print motor commands (Default: off).
      -S                        : Synchronous: don't queue (Default: off).
      --loop[=count]            : Loop file number of times (no value: forever)
    [*] All comma separated axis numerical values are in the sequence X,Y,Z,E,A,B,C,U,V,W
    (the actual mapping to a connector happens with --channel-layout and --axis-mapping,
    the default values map the channels left to right on the Bumps-board as X,Y,Z,E,A)
    You can either specify --port <port> to listen for commands or give a GCode-filename
    All numbers can optionally be given as fraction, e.g. --steps-mm '3200/6.35,200/3'

The G-Code understands logical axes X, Y, Z, E, A, B, C, U, V, and W,
while `machine-control` maps these to physical output connectors,
by default "XYZEA".
This can be changed with the `--axis-mapping` flag. This flag maps the
logical axis (such as 'Y') to a physical connector location on the
cape -- the position in the string represents the position of the connector. The
first letter in that string corresponds to the first connector, the second
to the second and so on.

Use lower-case axis names to invese the axis; this is
useful for mirroring axes, e.g. `--axis-mapping=XxYZ` has three axes, with
two motors connected to the first two connectors with a mirrored X-axis.

More details about the G-Code code parsed and handled can be found in the
[G-Code documentation](./G-code.md).

### Examples

    sudo ./machine-control -f 10 --max-feedrate 1000 --loop myfile.gcode

Output the file `myfile.gcode` in 10x the original speed, with a feedrate
capped at 1000mm/s. Repeat this file forever (say you want to stress-test).

    echo "G1 X100 F10000 G1 X0 F1000" | sudo ./machine-control /dev/stdin

This command directly executes some GCode coming from stdin. This is in
particular useful when you're calibrating your machine and need to work on
little tweaks.

    sudo ./machine-control --port 4444

Listen on TCP port 4444 for incoming connections and execute G-Codes over this
line. So you could use `telnet beaglebone-hostname 4444` to have an interactive
session or send a file with `socat`:

     cat myfile.gcode | socat -t5 - TCP4:beaglebone-hostname:4444

Use `socat`, don't use the ancient `nc` (netcat) - its buffering seems to be
broken so that it can get stuck. With `socat`, it should be possible to connect
to a pseudo-terminal in case your printer-software only talks to a terminal
(haven't tried that yet, please let me know if it works).

Note, there can only be one open TCP connection at any given time.

### Axis to Motor mapping

Each board has a number of connectors for motors and switches to which you connect your
physical motors to.
To intuitively map these connector positions to logical axes names, the `machine-control`
binary can be configured with command line flags with a string that associates the motor
position (=string position) with the actual axis (name of axis).

In the following example configuration of a [Bumps cape][bumps] with 5 connectors,
the X axis on the very left
(with a plugged in motor), second slot empty, third is 'Z', fourth (second-last) is E, and
finally the Y axis is on the very right (more space for two connectors which I
need for my Type-A machine).
The mapping is configured with:

        ./machine-control --axis-mapping "X_ZEY"  ...

(TODO: make example for mirrored axes)

Similarly, the `--min-endswitch` and `--max-endswitch` takes a string with axis letters,
where the string position represents the connector position on your cape and the letter the
corresponding axis this switch is for (Upper case: used for homing; lower case: just general
axis endstop).

![Bumps board][BUMPS-img]

### Configuration tip
For a particular machine, you might have some settings you always want to
use, and maybe add some comments. So create a file that contains all the
command line options

    $ cat type-a.config
    # Configuration for Type-A machine series 1, Motors @ 28V
    --steps-mm 1600/20,1600/20,800
    --max-feedrate 900,900,90
    --accel 18000,8000,1500
    --axis-mapping XYyZE   # y mirrored

Now, you can invoke `machine-control` like this

    sudo ./machine-control $(sed 's/#.*//g' type-a.config) --port 4444

or, simpler, if you don't have any comments in the configuration file:

    sudo ./machine-control $(cat type-a.config) --port 4444

The `sed` command passes the configuration, but removes the comment characters.

## G-Code stats binary
There is a binary `gcode-print-stats` to extract information from the G-Code
file e.g. accurate expected print-time, Object height (=maximum Z-axis),
filament length.

    Usage: ./gcode-print-stats [options] <gcode-file> [<gcode-file> ..]
    Options:
            -m <max-feedrate> : Maximum feedrate in mm/s
            -f <factor>       : Speedup-factor for print
            -H                : Toggle print header line
    Use filename '-' for stdin.

The output is in column form, so you can use standard tools to process them.
For instance, from a bunch of gcode files, find the one that takes the longest
time

    ./gcode-print-stats *.gcode | sort -k2 -n

## License
BeagleG is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

## TODO
   - Do planning: no need to decelerate fully if we're going on an (almost)
     straight line between line segments.
   - Needed for full 3D printer solution: add PWM for heaters.
   - Fast pause without waiting for queues to empty, but still be able to
     recover exact last position. That way pause/resume is possible.
   - ...

[BUMPS]: https://github.com/hzeller/bumps
[BUMPS-img]: ./img/bumps-connect.jpg
