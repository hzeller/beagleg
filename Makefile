# (c) 2013 h.zeller@acm.org
# This is free software. License: GNU 3.0

# Stuff based on the environment. This assumes stuff to be compiled on the
# beaglebone and the am335x_pru_package checked out.
# https://github.com/beagleboard/am335x_pru_package

# Change here for which hardware you are compiling. See hardware/ directory.
# Currently supported BUMPS, CRAMPS, and VGEN5
export BEAGLEG_HARDWARE_TARGET?=BUMPS

# In case you cross compile this on a different architecture, uncomment this
# and set the prefix of the compiler binary.
# Or simply set the environment variable.
#export CROSS_COMPILE?=arm-arago-linux-gnueabi-

# Tuning options for ARM CPU. Unset this in an environment variable if not
# compiled on the Beaglebone but on a different system.
export ARM_COMPILE_FLAGS?=-mtune=cortex-a8 -march=armv7-a

all : FORCE
	$(MAKE) -e -C src ../machine-control ../gcode-print-stats

test clean:
	$(MAKE) -C src $@

FORCE:
.PHONY: FORCE
