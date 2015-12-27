BeagleG
=======

Step-motor controller for CNC-like devices (or 3D printers) using the
PRU (Programmable Realtime Unit) of the Beaglebone Black to create precisely
timed and fast stepper-pulses for acceleration and travel.
(And with fast, we're talking up to 1Mhz fast. For 8 motors in parallel.
In a controlled move (G1). So this is not a limit in real-world applications).

Works with a cape designed by the author (the [BUMPS] cape), but also provides
relatively easy adaption to new hardware (currently: support for CRAMPS). See
[hardware](./hardware) subdirectory.

This was one of my very early tests:
[![First Test][run-vid]](http://youtu.be/hIEY9077D64)

The {accl-,decel-}eration and travel motion profile is entirely
created within the PRU from parameters sent by the host CPU decoupled via a
ring-buffer.
The BeagleBone main CPU prepares the data, such as parsing the
[G-Code](./G-code.md) and doing travel planning, while all the real-time
critical parts are done in the PRU. The host CPU typically needs less
than 1% CPU-time doing everything else (and there is no need for a real-time
kernel).

The main `machine-control` program is parsing G-Code, extracting axes moves and
enqueues them to the realtime unit. It can receive G-Code from a file or
socket (you can just telnet to it for an interactive session, how cool is that?).

## APIs
The functionality is implemented in a stack of independently usable APIs.

   - [gcode-parser.h](./src/gcode-parser.h) : C++-API for parsing
      [G-Code](./G-code.md) that calls callback parse events, while taking
      care of many internals, e.g. interpreting slightly different dialects and
      automatically translates everything into metric, absolute coordinates for
      ease of downstream receivers. This API in itself is independent of the
      rest, so it might be useful in other contexts as well.

   - [gcode-machine-control.h](./src/gcode-machine-control.h) : highlevel
      C++-API to control a machine via G-Code: it receives G-Code events, does
      motion planning, axis mapping, speed/accleration segment joining and emits
      the resuling motor commands to MotorOperations.
      Depends on the gcode-parser APIs as input and motor-operations as output.
      Provides the functionality provided by the `machine-control` binary.

   - [motor-operations.h](./src/motor-operations.h) : Low-level motor motion
      C++-API.
      Receives travel speeds and speed transitions from gcode machine control
      planner. This is a good place to implement stepmotor driver backends.
      The implementation here prepares parameters for the discrete
      approximation in the PRU motion-queue backend.
      The print-stats binary has a different implementation that uses it to
      determine print-time calculations, simulating how long motor movements
      would take.

   - [motion-queue.h](./src/motion-queue.h) : Even lower level interface: queue
      between motor-operations and hardware creating motion profiles in realtime.
      The implementation is done in the BeagleBone-PRU, but is separated out
      enough that it is not dependent on it: The required operations could be
      implemented in microcontrollers or FPGAs (32 bit operations help...).
      There is a simulation implementation (sim-firmware.cc) that illustrates
      what to do with the parameters. The simulation just outputs the would-be
      result as CSV file (good for debugging).

   - [determine-print-stats.h](./src/determine-print-stats.h): Highlevel API
      facade to determine some basic stats about a G-Code file; it processes
      the entire file and determines estimated print time, filament used etc.
      Since this takes the actual travel planning into account, the values are
      a correct prediction of the actual print or CNC time.

The interfaces are C++ objects.

## Build
To build, we need the BeagleG code and the PRU assembler with supporting library.
The BeagleG repository is set up in a way that the PRU assembler is checked out via
a git sub-module to make it simple.

Clone the BeagleG repository with the `--recursive` flag to get this sub-module

     git clone --recursive https://github.com/hzeller/beagleg.git

(If you are a github user, you might want to use the git protocol).

Then just

     cd beagleg
     make

If you are looking at the code and developing on a non-Beaglebone machine,
pass an empty `ARM_COMPILE_FLAGS` environment variable:

     ARM_COMPILE_FLAGS="" make

In particular if you work on BeagleG development, is advisable to run the tests.
To avoid re-inventing the testing-framework wheel, we use the
[Google test framework](https://github.com/google/googletest), for which there
are typically already packages available:

    sudo aptitude install libgtest-dev google-mock cmake
    make test
    # Or, for more thorough memory-leak or initialization issue check:
    make valgrind-test

## Getting started
Before you can use beagleg and get meaningful outputs on the GPIO pins,
we have to tell the pin multiplexer to connect them to the output pins. For
that, just run the start-devicetree-overlay.sh script with your hardware
to install the device overlay. You find it in the `hardware/` subdirectory.

    sudo hardware/start-devicetree-overlay.sh hardware/BUMPS/BeagleG.dts

See the [Hardware page](./hardware) how to enable the cape at boot time.

## Machine control binary
To control a machine with G-Code, use the `machine-control` binary.
This either takes a filename or a TCP port to listen on.

```
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
  --range <range-mm>    (-r): Comma separated range of of axes in mm (0..range[axis]). Only
                                values > 0 are actively clipped. (Default: 100,100,100,-1,-1, ...)
  --min-endswitch           : Axis letter mapped to which endstop connector for negative travel (=string pos)
                                Use letter or '_' for unused endstop.
                                Use uppercase if endstop is used for homimg, lowercase if used for travel limit.
  --max-endswitch           : Axis letter mapped to which endstop connector for positive travel (=string pos)
                                Use letter or '_' for unused endstop.
                                Use uppercase if endstop is used for homimg, lowercase if used for travel limit.
  --home-order              : Order to home axes, all axes involved with homing should be listed (Default: ZXY)
  --require-homing          : If set, machine refuses to work unless homed
  --disable-range-check     : Don't limit at machine bounds. Dangerous.
  --endswitch-polarity      : 'Hit' polarity for each endstop connector (=string pos).
                                Use '1' or '+' for logic high trigger.
                                Use '0' or '-' for logic low trigger.
                                Use '_' for unused endstops.
  --threshold-angle         : Threshold angle of XY vectors to ignore speed changes (Default=10.0)
  --port <port>         (-p): Listen on this TCP port for GCode.
  --bind-addr <bind-ip> (-b): Bind to this IP (Default: 0.0.0.0).
  --logfile <logfile>   (-l): Logfile to use. If empty, messages go to syslog (Default: /dev/stderr).
  --daemon              (-d): Run as daemon.
  -f <factor>               : Print speed factor (Default 1.0).
  -n                        : Dryrun; don't send to motors (Default: off).
  -P                        : Verbose: Print motor commands (Default: off).
  -S                        : Synchronous: don't queue (Default: off).
  --loop[=count]            : Loop file number of times (no value: forever; equal sign with value important.)
[*] All comma separated axis numerical values are in the sequence X,Y,Z,E,A,B,C,U,V,W
(the actual mapping to a connector happens with --axis-mapping,
the default values map the channels left to right on the Bumps-board as X,Y,Z,E,A)
You can either specify --port <port> to listen for commands or give a GCode-filename
All numbers can be given as multiplicative expression
which makes microstepping and unit conversions more readable
e.g. --steps-mm '16*200/(25.4/4),8*200/4'
```

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

Note, there can only be one open TCP connection at any given time (after all, there is
only one physical machine).

### Axis to Motor mapping

Each board has a number of connectors for motors and switches to which you connect your
physical motors and end-switches to.

To intuitively map these connector positions to logical axes names, the `machine-control`
binary can be configured with command line flags with a string that associates the motor
position (=string position) with the actual axis (name of axis). You can map the same axis
letter to multiple positions, if you mirror motors. Lower- and uppper-case of that letter
flip the direction.

In the following example configuration of a [Bumps cape][bumps] with 5 connectors,
we want the `X` axis on the very left, then another, mirrored X axis motor, turning in the
opposite direction (lowercase `x`), third is `Z`,
fourth (second-last) is `E` (extrusion for a 3D printer), and finally the `Y` axis is on the
very right.

The mapping is configured with:

        ./machine-control --axis-mapping "XxZEY"  ...

Similarly, the `--min-endswitch` and `--max-endswitch` takes a string with axis letters,
where the string position represents the connector position on your cape and the letter the
corresponding axis this switch is for (Upper case: used for homing; lower case: just general
axis endstop).

![Bumps board][BUMPS-img]
(Just for reference. This has one missing Pololu driver, so the second connector wouldn't work).

### Configuration tip
For a particular machine, you might have some settings you always want to
use, and maybe add some comments. So create a file that contains all the
command line options

    $ cat type-a.config
    # Configuration for Type-A machine series 1, Motors @ 28V
    --steps-mm 8*200/20,8*200/20,800
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

(TODO: make it share the same configuration as machine-control)

## License
BeagleG is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

## TODO
Not everything I'd like to have is implemented yet, but getting closer as weekend
hacking permits.

   - Needed for full 3D printer solution: add PWM/PID-loop for heaters.
   - Fast pause without waiting for queues to empty, but still be able to
     recover exact last position. That way pause/resume is possible.
   - ...

[run-vid]: ./img/beagleg-vid-thumb.jpg
[BUMPS]: https://github.com/hzeller/bumps
[BUMPS-img]: ./img/bumps-connect.jpg
