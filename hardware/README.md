Hardware support
================

While BeagleG currently is tied to the BeagleBone Black **am335x** CPU family
utilizing its neat PRU, it can support various hardware that wire up the
outputs to stepmotor drivers.

There are various [BeagleBone-Black] capes that provide such stepper motor
drivers for 3D printers and CNC machines.

This directory contains sub-directories with the name of the particular hardware. Each directory
contains the necessary hardware description used by BeagleG. You need to enable the hardware
you intend to use in the toplevel Makefile.

   * [BUMPS/](./BUMPS/) The [BUMPS] board was initially developed for BeagleG
     before there were any other boards available.
   * [CRAMPS/](./CRAMPS/) The [CRAMPS] board by Charles Steinkuehler is a
     popular cape.
   * [Pockegotion](./Pockegotion/) Work in progress for a cape for the
     [PocketBeagle].

(If you have access to other boards and run them with BeagleG, consider adding the support and
send a pull request)

## Set up the pins needed

Load the pin-mapping with the `config-pin` script from the directory that
contains your hardware mapping; for BUMPS, this would be:

```
/opt/source/bb.org-overlays/tools/beaglebone-universal-io/config-pin -f BUMPS/bumps.pins
```

(Note: Older versions of this documentation were loading a device tree here,
but these days things are simpler making use of the [beaglebone-universal-io]
GPIO pin-mapper.)

## Adding support for new hardware

To add support for a new cape, you need to create a subdirectory with the name
of the cape you want to add.

The directory should contain at least a README or README.md describing the
board and provide references where it can be found.

Provide a `*.pins` file for the pin-mapping to be loaded with `config-pin`.

In order to make things compile, each hardware subdirectory requires the
following files with these exact names:

   * `beagleg-pin-mapping.h`: mapping of GPIO pins to logical pins (e.g. `MOTOR_1_STEP`).
     As an example, see the BUMPS [beagleg-pin-mapping.h](./BUMPS/beagleg-pin-mapping.h)

   * `pru-io-routines.hp`: a file containing a set of PRU subroutines to set
      certain values. You can write this file yourself or just use the generic
      version provided in this directory:

     ```bash
     cd hardware  # Where this README.md is; subdirectory of the beagleg/ toplevel dir
     cp template-pru-io-routines.hp MyCapeName/pru-io-routines.hp
     ```

You can enable compilation for your new cape by setting the variable
`BEAGLEG_HARDWARE_TARGET` in the toplevel Makefile to your cape name:

     BEAGLEG_HARDWARE_TARGET=MyCapeName

Please check out the existing subdirectories to get an idea. If you added a new board, consider sending a patch.

[BeagleBone-Black]: http://beagleboard.org/BLACK
[BUMPS]: http://github.com/hzeller/bumps
[CRAMPS]: http://reprap.org/wiki/CRAMPS
[PocketBeagle]: https://beagleboard.org/pocket
[beaglebone-universal-io]: https://github.com/cdsteinkuehler/beaglebone-universal-io
