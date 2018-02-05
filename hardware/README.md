Hardware support
================

While BeagleG currently is tied to the BeagleBone Black **am335x** CPU family utilizing its neat
PRU, it can support various hardware that wire up the outputs to stepmotor drivers.

There are various [BeagleBone-Black] capes that provide such stepper motor drivers for 3D printers
and CNC machines.

This directory contains sub-directories with the name of the particular hardware. Each directory
contains the necessary hardware description used by BeagleG. You need to enable the hardware
you intend to use in the toplevel Makefile.

   * [BUMPS/](./BUMPS) The [BUMPS] board was initially developed for BeagleG
     before there were any other boards available.
   * [CRAMPS/](./CRAMPS) The [CRAMPS] board by Charles Steinkuehler is a
     popular cape.
   * [Pockegotion](./Pockegotion) Work in progress for a cape for the
     [PocketBeagle].

(If you have access to other boards and run them with BeagleG, consider adding the support and
send a pull request)

## Load cape device tree

*The following was true Kernel 3.8.x type distributions, this is
simplified for 4.x. TODO: update documentation for 4.x*

In each hardware subdirectgory directory, named after the cape, there is a device tree overlay
file that you need to install for your hardware (or just use the one that comes with your board).

We can enable your cape, you can use the
[start-devicetree-overlay.sh](./start-devicetree-overlay.sh) script in this directory:

    sudo ./start-devicetree-overlay.sh BUMPS/BeagleG-BUMPS.dts

(pass the DTS file of the board you are using as parameter)

This initializes the pinmux now, but we have to do this every time after boot. Also,
we'd like to have the cape installed as early as possible in the boot process
to properly set all the output values to safe values.

There is a script for that.

First make sure that you have the `bb-customizations`
package installed:

    sudo apt-get install bb-customizations

.. Now run this script

    sudo ./install-devicetree-overlay.sh BUMPS/BeagleG-BUMPS.dts

In general, what this script is doing is to add the overlay name to the `cape_enable`
lines in `/boot/uEnv.txt` file to let the kernel know to enable that cape at boot time.

The kernel looks for the firmware in /lib/firmware - since at boot time the
root-fs is not mounted yet, just the init-rd ramdisk, we need to make sure
to have it in the initial ram-filesystem. The script issues an update-initramfs
that does that (with help from scripts from the bb-customizations package).

## Adding support for new hardware

To add support for a new cape, you need to create a subdirectory with the name of the
cape you want to add.

The directory should contain at least a README or README.md describing the board and
provide references where it can be found.

You should provide a `*.dts` device tree overlay file for easy install (or provide a link where
it can be found). If you have a `*.dts` file, it will work with the `start-devicetree-overlay.sh`
and `install-devicetree-overlay.sh` scripts in this directory.

In order to make things compile, each hardware subdirectory requires the following files with
these exact names:

   * `beagleg-pin-mapping.h`: mapping of GPIO pins to logical pins (e.g. `MOTOR_1_STEP`).
     As an example, see the BUMPS [beagleg-pin-mapping.h](./BUMPS/beagleg-pin-mapping.h)

   * `pru-io-routines.hp`: a file containing a set of PRU subroutines to set certain
     values. You can write this file yourself or just use the generic version provided
     in this directory:

     ```bash
     cd hardware  # Where this README.md is; subdirectory of the beagleg/ toplevel dir
     cp template-pru-io-routines.hp MyCapeName/pru-io-routines.hp
     ```

You can enable compilation for your new cape by setting the variable `BEAGLEG_HARDWARE_TARGET`
in the toplevel Makefile to your cape name:

     BEAGLEG_HARDWARE_TARGET=MyCapeName

Please check out the existing subdirectories to get an idea. If you added a new board,
consider sending a patch.

[BeagleBone-Black]: http://beagleboard.org/BLACK
[BUMPS]: http://github.com/hzeller/bumps
[CRAMPS]: http://reprap.org/wiki/CRAMPS
[PocketBeagle]: https://beagleboard.org/pocket