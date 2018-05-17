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

## Install
### System configuration

In order to run BeagleG on your BeagleBone you will need to be sure
that uio_pruss kernel module has been installed and loaded in the kernel.
You can easily test if the module it's available by running:

    if lsmod | grep "uio_pruss" &> /dev/null ; then echo "The kernel is BeagleG ready"; else echo "Need uio_pruss module"; fi

then just load it using:

    sudo modprobe uio_pruss

or add it to /etc/modules to make it persistent:

    sudo sh -c 'echo uio_pruss >> /etc/modules'

if the uio_pruss module is not available, you might be running a 4.x kernel
from TI; they are experimenting with a new way to connect to the PRU (remoteproc)
which is not marked stable or ready yet, and is not backward compatible with
older kernels still commonly run on BeagleBones, so we can't use it in BeagleG
yet. You can see that if you ask `uname -r` returns something like
`4.4.12-ti-r32` - with a `ti` after the version number.

In that case, you will need to
[change your kernel version](http://elinux.org/BeagleBoardDebian#Install_Latest_Kernel_Image) to a bone prefix one (e.g. `4.4.14-bone`).

#### Newer kernels
Newest beagleboard kernels support both remoteproc and uio_pruss modules.
In order to use the uio_pruss one simply follow the directives that you can find inside `/boot/uEnv.txt`:

```
###PRUSS OPTIONS
###pru_rproc (4.4.x-ti kernel)
#uboot_overlay_pru=/lib/firmware/AM335X-PRU-RPROC-4-4-TI-00A0.dtbo
###pru_uio (4.4.x-ti & mainline/bone kernel)
uboot_overlay_pru=/lib/firmware/AM335X-PRU-UIO-00A0.dtbo
```

Some 4.4 linux kernel versions do not have the timers drivers enabled.
In order to be able to use the PWM you would need to recompile the kernel with
CONFIG_OMAP_DM_TIMER=y.

In order to use BeagleG **without** the PWM TIMERS support, you can compile beagleg with:
```
CONFIG_FLAGS=-D_DISABLE_PWM_TIMERS make
```


### Build
To build, we need the BeagleG code and the PRU assembler with supporting library.
The BeagleG repository is set up in a way that the PRU assembler is checked out via
a git sub-module to make it simple.

Clone the BeagleG repository with the `--recursive` flag to get this sub-module

```
git clone --recursive https://github.com/hzeller/beagleg.git
```

(If you are a github user, you might want to use the git protocol).

Then just

```
cd beagleg
make
```

If you have an older debian wheezy with a gcc 4.6 as default compiler, you need
to install a g++ 4.7 first to be able to compile; then set the CXX variable
to this compiler when running make:

```
sudo apt-get install g++-4.7
CXX=g++-4.7 make
```

## Getting started
Before you can use beagleg and get meaningful outputs on the GPIO pins,
we have to tell the pin multiplexer to connect them to the output pins. For
that, just run the `start-devicetree-overlay.sh` script with your hardware
to install the device overlay. You find it in the `hardware/` subdirectory.

    sudo hardware/start-devicetree-overlay.sh hardware/BUMPS/BeagleG.dts

See the [Hardware page](./hardware) how to enable the cape at boot time.

## Machine control binary
To control a machine with G-Code, use the `machine-control` binary.
This either takes a filename or a TCP port to listen on.

```
Usage: ./machine-control [options] [<gcode-filename>]
Options:
  -c, --config <config-file> : Configuration file. (Required)
  -p, --port <port>          : Listen on this TCP port for GCode.
  -b, --bind-addr <bind-ip>  : Bind to this IP (Default: 0.0.0.0).
  -l, --logfile <logfile>    : Logfile to use. If empty, messages go to syslog (Default: /dev/stderr).
  -d, --daemon               : Run as daemon.
      --priv <uid>[:<gid>]   : After opening GPIO: drop privileges to this (default: daemon:daemon)
      --help                 : Display this help text and exit.

Mostly for testing and debugging:
  -f <factor>                : Feedrate speed factor (Default 1.0).
  -n                         : Dryrun; don't send to motors, no GPIO or PRU needed (Default: off).
  -P                         : Verbose: Show some more debug output (Default: off).
  -S                         : Synchronous: don't queue (Default: off).
      --loop[=count]         : Loop file number of times (no value: forever; equal sign with value important.)
      --allow-m111           : Allow changing the debug level with M111 (Default: off).

Configuration file overrides:
     --homing-required       : Require homing before any moves (require-homing = yes).
     --nohoming-required     : (Opposite of above^): Don't require homing before any moves (require-homing = no).
     --norange-check         : Disable machine limit checks. (range-check = no).
```

The axis configurations (max feedrate, acceleration, travel, motor mapping,...)
is configured in a [configuration file like in this example](./sample.config).

The G-Code understands logical axes X, Y, Z, E, A, B, C, U, V, and W.

More details about the G-Code code parsed and handled can be found in the
[G-Code documentation](./G-code.md).

### Examples

For testing your motor settings, you might initially just have a simple
file that you want to loop over:

    sudo ./machine-control -c my.config -f 10 --loop myfile.gcode

Output the file `myfile.gcode` in 10x the original speed, repeat this file
forever (say you want to stress-test).

    echo "G1 X100 F10000 G1 X0 F1000" | sudo ./machine-control /dev/stdin

This command directly executes some GCode coming from stdin. This is in
particular useful when you're calibrating your machine and need to work on
little tweaks.

    sudo ./machine-control -c my.config --port 4444

Listen on TCP port 4444 for incoming connections and execute G-Codes over this
line. So you could use `telnet beaglebone-hostname 4444` to have an interactive
session or send a file with `socat`:

     cat myfile.gcode | socat -t5 - TCP4:beaglebone-hostname:4444

Use `socat`, don't use the ancient `nc` (netcat) - its buffering seems to be
broken so that it can get stuck. With `socat`, it should be possible to connect
to a pseudo-terminal in case your printer-software only talks to a terminal
(haven't tried that yet, please let me know if it works).

Note, there can only be one open TCP connection at any given time (after all,
there is only one physical machine).

## G-Code stats binary
There is a binary `gcode-print-stats` to extract information from the G-Code
file e.g. accurate expected print-time, Object height (=maximum Z-axis),
filament length. This is in particular useful because many GCode runtime
estimators are widely off; this is accurate to the second because it takes all
acceleration phases into account.

```
Usage: ./gcode-print-stats [options] <gcode-file> [<gcode-file> ..]
Options:
        -c <config>       : Machine config
        -f <factor>       : Speedup-factor for feedrate.
        -H                : Toggle print header line
Use filename '-' for stdin.
```

The output is in column form, so you can use standard tools to process them.
For instance, from a bunch of gcode files, find the one that takes the longest
time

    ./gcode-print-stats -c my.config *.gcode | sort -k2 -n

## Cape

The [BUMPS]-cape is one of the capes to use, it was developed together with
BeagleG (but it is not widely distributed yet).
BeagleG also works with the CRAMPS board, which is a popular motor driver cape
for the BeagleBone Black. You can easily adapt your own hardware, check the
[hardware](./hardware) sub-directory.

![Bumps board][BUMPS-img]

### Axis to Motor mapping

Each board has a number of connectors for motors and switches to which you
connect your physical motors and end-switches to.

To map these connector positions to logical axes names, the `machine-control`
binary has a configuration file in which you can configure not only the
various axis parameters (max speed, acceleration, steps/mm), but also assign
these axes to motor drivers provided by the cape (`motor_1`, `motor_2`,...)
and end switches (`switch_1`, `switch_2`,...) to logical functions
(e.g. `min_x`). See the [annotated config file](./sample.config).

## Development

If you want to use the nicely seprated sub-APIs of BeagleG programmatically
or want to get involved in the development, check
the [Development](./Development.md) page.

<a href="Development.md"><img src="./img/machine-control.png" width="128"/><img src="./img/sample-gcode2ps-isometric.png" width="140"/></a>

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
[ccache]: https://ccache.samba.org/
