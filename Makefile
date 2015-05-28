# (c) 2013 h.zeller@acm.org
# This is free software. License: GNU 3.0

# Stuff based on the environment. This assumes stuff to be compiled on the
# beaglebone and the am335x_pru_package checked out.
# https://github.com/beagleboard/am335x_pru_package

# Change here for which hardware you are compiling. See hardware/ directory.
# Currently supported BUMPS and CRAMPS
HARDWARE_TARGET=BUMPS

# In case you cross compile this on a different architecture, uncomment this
# and set the prefix
#CROSS_COMPILE?=arm-arago-linux-gnueabi-

# Tuning options for ARM CPU. Unset this in an environment variable if compiled
# on a different system.
ARM_COMPILE_FLAGS?=-mtune=cortex-a8 -march=armv7-a

# Location of am335x package https://github.com/beagleboard/am335x_pru_package
# We check this out in a local git submodule.
AM335_BASE=am335x_pru_package
PASM=$(AM335_BASE)/pru_sw/utils/pasm
LIBDIR_APP_LOADER?=$(AM335_BASE)/pru_sw/app_loader/lib
INCDIR_APP_LOADER?=$(AM335_BASE)/pru_sw/app_loader/include
CAPE_INCLUDE=hardware/$(HARDWARE_TARGET)

CFLAGS+= -Wall -I$(INCDIR_APP_LOADER) -I$(CAPE_INCLUDE) -std=c99 -D_XOPEN_SOURCE=500 -O3 $(ARM_COMPILE_FLAGS)
LDFLAGS+=-lpthread -lm
PRUSS_LIBS=-Wl,-rpath=$(LIBDIR_APP_LOADER) -L$(LIBDIR_APP_LOADER) -lprussdrv

# Assembled binary from *.p file.
PRU_BIN=motor-interface-pru_bin.h

GCODE_OBJECTS=gcode-parser.o gcode-machine-control.o determine-print-stats.o \
              generic-gpio.o arc-gen.o
OBJECTS=motor-operations.o sim-firmware.o pru-motion-queue.o $(GCODE_OBJECTS)
MAIN_OBJECTS=machine-control.o gcode-print-stats.o
TARGETS=machine-control gcode-print-stats
TEST_BINARIES=gcode-machine-control_test gcode-parser_test

all : $(TARGETS) $(TEST_BINARIES)

gcode-print-stats: gcode-print-stats.o $(GCODE_OBJECTS)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

machine-control: machine-control.o $(OBJECTS)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $^ $(PRUSS_LIBS) $(LDFLAGS)

test: $(TEST_BINARIES)
	for test_bin in $(TEST_BINARIES) ; do ./$$test_bin ; done

%_test: %_test.c $(OBJECTS)
	$(CROSS_COMPILE)gcc $(CFLAGS) -o $@ $< $(OBJECTS) $(PRUSS_LIBS) $(LDFLAGS)

%.o: %.c
	$(CROSS_COMPILE)gcc $(CFLAGS) -c -o $@ $<

%_bin.h : %.p $(PASM)
	$(PASM) -I$(CAPE_INCLUDE) -V3 -c $<

$(PASM):
	make -C $(AM335_BASE)

pru-motion-queue.o : motor-interface-constants.h $(PRU_BIN)
motor-operations.o : motor-interface-constants.h
sim-firmware.o : motor-interface-constants.h

# test dependencies.
gcode-machine-control_test.c: gcode-machine-control.c

$(PRU_BIN) : motor-interface-constants.h \
             $(CAPE_INCLUDE)/beagleg-pin-mapping.h $(CAPE_INCLUDE)/pru-io-routines.hp

clean:
	rm -rf $(TARGETS) $(MAIN_OBJECTS) $(OBJECTS) $(PRU_BIN) $(TEST_BINARIES)
