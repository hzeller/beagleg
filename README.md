BeagleG
=======

Step-motor controller (and eventually 3D printer controller) using the PRU
capability of the Beaglebone Black to create precisely timed stepper-pulses for
acceleration and travel.

See example here: http://www.youtube.com/watch?v=hIEY9077D64

The motor-interface API allows to enqueue
step-{count, {start,travel,end}-frequency}
of 8 steppers that are controlled in a coordinated move (G1), with real-time
controlled steps at rates that can go beyond 500kHz.
So: sufficient even for advanced step motors and drivers :)

The [acceleration - travel - deceleration] motion profile is entirely
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

   - [motor-interface.h](./motor-interface.h) : Low-level motor move C-API to
      enqueue motor moves, that are executed in the PRU.

   - [gcode-parser.h](./gcode-parser.h) : C-API for parsing
      [G-Code](./G-code.md) that calls callback parse events, while taking
      care of many internals, e.g. it automatically translates
      everything into metric, absolute coordinates.

   - [gcode-machine-control.h](./machine-control.h) : highlevel C-API to
      control a machine via G-Code: it reads G-Code from a stream and emits
      the necessary machine commands.
      Depends on the motor-interface and gcode-parser APIs.
      Provides the functionality provided by the `machine-control` binary.

   - `determine-print-stats.h`: C-API to determine some basic stats about
      a G-Code file; it processes the entire file and determines estimated
      print time, filament used etc. Implementation is mostly an example using
      gcode-parser.h.
      Used in the `gcode-print-stats` binary.

## Build
The Makefile is assuming that you build this either on the Beaglebone Black
directly, or using a cross compiler (see Makefile).

You need to have checked out https://github.com/beagleboard/am335x_pru_package
which provides the pasm PRU assembler and the library to push this code to the
PRU.

    # Check out and build am335 package
    git clone https://github.com/beagleboard/am335x_pru_package.git
    cd am335x_pru_package/
    cd pru_sw/utils/pasm_source ; ./linuxbuild ; cd -
    CROSS_COMPILE="" make -C pru_sw/app_loader/interface/
    cd ..

    # Check out BeagleG and build
    git clone https://github.com/hzeller/beagleg.git
    cd beagleg
    make

(If you are a github user, you might want to use the git protocol).

If you run into compile problems, make sure to have both, am335x_pru_package and
beagleg up-to-date from git.

## Getting started
Before you can use beagleg and get meaningful outputs on the GPIO pins,
we have to tell the pin multiplexer to connect them to the output pins. For
that, just run the beagleg-cape-pinmux.sh script that installs the device
overlay.

    sudo ./beagleg-cape-pinmux.sh

See below to enable the cape at boot time.

## Machine control binary
To control a machine with G-Code, use the `machine-control` binary.
This either takes a filename or a TCP port to listen on.

    Usage: ./machine-control [options] [<gcode-filename>]
    Options:
      --steps-mm <axis-steps>   : steps/mm, comma separated (Default 160,160,160,40,0, ...).
      --max-feedrate <rate> (-m): Max. feedrate per axis (mm/s), comma separated (Default: 200,200,90,10,0, ...).
      --accel <accel>       (-a): Acceleration per axis (mm/s^2), comma separated (Default 4000,4000,1000,10000,0, ...).
      --axis-mapping            : Axis letter mapped to which motor connector (=string pos)
                                  Use letter or '_' for empty slot. (Default: 'XYZEABC')
      --port <port>         (-p): Listen on this TCP port.
      --bind-addr <bind-ip> (-b): Bind to this IP (Default: 0.0.0.0).
      -f <factor>               : Print speed factor (Default 1.0).
      -n                        : Dryrun; don't send to motors (Default: off).
      -P                        : Verbose: Print motor commands (Default: off).
      -S                        : Synchronous: don't queue (Default: off).
      -R                        : Repeat file forever.
    All comma separated axis numerical values are in the sequence X,Y,Z,E,A,B,C,U,V,W
    You can either specify --port <port> to listen for commands or give a filename

The G-Code understands logical axes X, Y, Z, E, A, B, C, U, V, and W,
while `machine-control` maps these to physical output connectors,
by default "XYZEA".
This can be changed with the `--axis-mapping` flag. This flag maps the
logical axis (such as 'Y') to a physical connector location on the
cape -- the position in the string represents the position of the connector.

More details about the G-Code code parsed and handled can be found in the
[G-Code documentation](./G-code.md).

### Examples

    sudo ./machine-control -f 10 -m 1000 -R myfile.gcode

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

### Configuration tip
For a particular machine, you might have some settings you always want to
use, and maybe add some comments. So create a file that contains all the
command line options

    $ cat type-a.config
    # Configuration for Type-A machine series 1, Motors @ 28V
    --steps-mm 75.075,150.14,800   # x has half the steps than y
    --max-feedrate 900,900,90
    --accel 18000,8000,1500
    --axis-mapping X_ZEY   # y on the double-connector

Now, you can invoke `machine-control` like this

    sudo ./machine-control $(sed 's/#.*//g' type-a.config) --port 4444

or, simpler, if you don't have any comments in the configuration file:

    sudo ./machine-control $(cat type-a.config) --port 4444

The `sed` command dumps the configuration, but removes the comment characters.

## Pinout

These are the GPIO bits associated with the motor outputs. The actual physical
pins are all over the place on the Beaglebone Black extension headers P8 and P9,
see table below.

Before we can use all pins, we need to tell the Beaglebone Black pin multiplexer
which we're going to use for GPIO. For that, we need to install a device
tree overlay. Just run the script beagleg-cape-pinmux.sh as root

    sudo ./beagleg-cape-pinmux.sh

This registers the cape, as you can confirm by looking at the slots:

    $ cat /sys/devices/bone_capemgr.*/slots
     0: 54:PF---
     1: 55:PF---
     2: 56:PF---
     3: 57:PF---
     4: ff:P-O-L Bone-LT-eMMC-2G,00A0,Texas Instrument,BB-BONE-EMMC-2G
     5: ff:P-O-L Bone-Black-HDMI,00A0,Texas Instrument,BB-BONELT-HDMI
     8: ff:P-O-L Override Board Name,00A0,Override Manuf,BeagleG

Now, all pins are mapped to be used by beagleg. This is the pinout

       Driver channel |  0     1     2     3     4      5     6     7
    Step     : GPIO-0 |  2,    3,    4,    5,    7,    14,   15,   20
           BBB Header |P9-22 P9-21 P9-18 P9-17 P9-42A P9-26 P9-24 P9-41A
                      |
    Direction: GPIO-1 | 12,   13,   14,   15,    16,    17,  18,   19
           BBB Header |P8-12 P8-11 P8-16 P8-15 P9-15 P9-23  P9-14 P9-16

Motor enable for all motors is on `GPIO-1`, bit 28, P9-12
(The mapping right now was done because these are consecutive GPIO pins that
can be used, but the mapping to P9-42A (P11-22) and P9-41A (P11-21) should
probably move to an unambiguated pin)

The mapping from axis to driver channel happens in two steps, see documentation
in `struct MachineControlConfig` about the configuration options `channel_layout`
and `axis_mapping`. The first describes the mapping of driver channels to
connector positions on the cape (which might differ due to board layout reasons),
the second the mapping of G-code axes (such as 'X' or 'Y') to the
connector position. While the 'channel_layout' is
configured in the code currently (and dependent on the cape hardware), the
axis mapping can be set with the `--axis-mapping` flag.

In the following [Bumps cape][bumps], the X axis on the very left (with a plugged
in motor), second slot empty, third is 'Z', fourth (second-last) is E, and
finally the Y axis is on the very right (more space for two connectors which I
need for my Type-A machine).
The mapping is configured with:

        ./machine-control --axis-mapping "X_ZEY"  ...

![Bumps output mapping][bumps-cape]

(The [Bumps cape][bumps] was designed to work with BeagleG).

If you build your own cape: note all logic levels are 3.3V (and assume not more
than ~4mA). The RAMPS driver board for instance only works if you power the
5V input with 3.3V, so that the Pololu inputs detect the logic level properly.

This is an early experimental manual cape interfacing to a RAMPS adapter:
![Manual Cape][manual-cape]

At the middle/bottom of the test board you see a headpone connector: many of
the early experiments didn't have yet a stepper motor installed, but just
listening to the step-frequency.

Not yet supported, coming soon:
   * More PINS of GPIO-0 will be used
       * Two AUX outputs on GPIO-0 30, 31. This controls the medium current Aux open drain connectors at the bottom left on the [Bumps board][bumps].
       * 3 end-switch inputs on GPIO-0 23, 26, 27
   * The PWM outputs are on GPIO-2 2, 3, 4, 5 which are also pins Timer 4, 5, 6, 7. Plan is to use the AM335x Timer functionality in their PWM mode. These control the two high current PWM outputs (screw terminals top right) and the two medium current open drain pwm connectors on the Bumps board (connector top left).w
   * Analog inputs AIN0, AIN1, AIN2 will be used for temperature reading.

## Load cape device tree at bootup
While loading the pinmux manually with

    sudo ./beagleg-cape-pinmux.sh

initializes the pinmux now, we have to do this every time after boot. Also,
we'd like to have the cape installed as early as possible in the boot process
to properly set all the output values to safe values.

For that, we essentially have to add

    optargs=capemgr.enable_partno=BeagleG

To the `/boot/uboot/uEnv.txt` file to let the kernel know to enable that cape.

The kernel looks for the firmware in /lib/firmware - since at boot time the
root-fs is not mounted yet, just the init-rd ramdisk, we need to make sure
to have it in the uInitrd filesystem.

There is a script that does both of these things. This might depend on your
distribution, so take a look at the script first to check that it does what
it should do (it will abort on the first error, so probably it won't do damage).
In particular the unpacking/repacking of the initrd file might be specific to
your distribution.

    sudo ./beagleg-install-cape.sh BeagleG-00A0.dtbo

After a reboot, you should see the cape to be enabled early on in the boot
process (Power PWM LEDs switch off).

## G-Code stats binary
There is a binary `gcode-print-stats` to extract information from the G-Code
file e.g. estimated print-time, Object height (=maximum Z-axis), filament
length.

    Usage: ./gcode-print-stats [options] <gcode-file> [<gcode-file> ..]
    Options:
            -m <max-feedrate> : Maximum feedrate in mm/s
            -f <factor>       : Speedup-factor for print
    Use filename '-' for stdin.

The output is in column form, so you can use standard tools to process them.
For instance, from a bunch of gcode files, find the one that takes the longest
time

    ./gcode-print-stats *.gcode | sort -k2 -n

*Note: this binary is currently not taking acceleration into account, so the
 estimated times are off*

## License
BeagleG is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

## TODO
   - Read end-switches
   - Do planning: no need to decelerate fully if we're going on an (almost)
     straight line between line segments.
   - Needed for full 3D printer solution: add PWM for heaters.
   - Fast pause without waiting for queues to empty, but still be able to
     recover exact last position. That way pause/resume is possible.
   - ...

[manual-cape]: https://github.com/hzeller/beagleg/raw/master/img/manual-ramps-cape.jpg
[bumps-cape]: https://github.com/hzeller/beagleg/raw/master/img/bumps-connect.jpg
[bumps]: https://github.com/hzeller/bumps
