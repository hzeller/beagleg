Hardware support
================

While BeagleG currently is tied to the am335x family with a PRU it can support various hardware
that wire up the outputs to stepmotor drivers. There are various [BeagleBone-Black] capes that
provide such stepper motor drivers for 3D printers and CNC machines.

This directory contains sub-directories with the name of the particular hardware. Each directory
contains the necessary hardware description used by BeagleG. You need to enable the hardware
you intend to use in the toplevel Makefile.

   * [BUMPS/](./BUMPS) From the maker of BeagleG, there comes the [BUMPS] board.
     It is designed to get the best possible performance by making sure that related
     pins (e.g. all the step outputs) are all on one GPIO port to be able to minimize the number
     of IO operations needed (native implementation).

   * [CRAMPS/](./CRAMPS) The [CRAMPS] ("Cape-RAMPS") board is a popular cape.

[BeagleBone-Black]: http://beagleboard.org/BLACK
[BUMPS]: http://github.com/hzeller/bumps
[CRAMPS]: http://reprap.org/wiki/CRAMPS
