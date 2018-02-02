# (c) 2013 h.zeller@acm.org
# This is free software. License: GNU 3.0

# Stuff based on the environment. This assumes stuff to be compiled on the
# beaglebone and the am335x_pru_package checked out.
# https://github.com/beagleboard/am335x_pru_package

# --
# The following variables can be overwritten by environment variables
# of the same name.
# --

# Change here for which hardware you are compiling. See hardware/ directory.
# Currently supported BUMPS, CRAMPS, and VGEN5
export BEAGLEG_HARDWARE_TARGET?=BUMPS

# Disable PWM timers. See README.md.
#export CONFIG_FLAGS?=-D_DISABLE_PWM_TIMERS

# In case you cross compile this on a different architecture, uncomment this
# and set the prefix of the compiler binary.
#export CROSS_COMPILE?=arm-arago-linux-gnueabi-

# Tuning options for ARM CPU. Unset this in an environment variable if not
# compiled on the Beaglebone but on a different system.
export ARM_COMPILE_FLAGS?=-mtune=cortex-a8 -march=armv7-a

PREFIX?=/usr/local
BINDIR=$(PREFIX)/bin

all machine-control:
	$(MAKE) -e -C src all

install: machine-control
	mkdir -p $(BINDIR)
	install -m 755 machine-control $(BINDIR)

clean test valgrind-test:
	$(MAKE) -C src/common $@
	$(MAKE) -C src/gcode-parser $@
	$(MAKE) -C src $@

dist-clean:
	$(MAKE) -C src dist-clean
