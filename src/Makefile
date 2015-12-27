# (c) 2013 h.zeller@acm.org
# This is free software. License: GNU 3.0

# Stuff based on the environment. This assumes stuff to be compiled on the
# beaglebone and the am335x_pru_package checked out.
# https://github.com/beagleboard/am335x_pru_package

# Change here for which hardware you are compiling. See hardware/ directory.
# Currently supported BUMPS, CRAMPS, and VGEN5
BEAGLEG_HARDWARE_TARGET?=BUMPS

# In case you cross compile this on a different architecture, uncomment this
# and set the prefix. Or simply set the environment variable.
#CROSS_COMPILE?=arm-arago-linux-gnueabi-

# Tuning options for ARM CPU. Unset this in an environment variable if compiled
# on a different system.
ARM_COMPILE_FLAGS?=-mtune=cortex-a8 -march=armv7-a

# Location of am335x package https://github.com/beagleboard/am335x_pru_package
# We check this out in a local git submodule.
AM335_BASE=../am335x_pru_package
PASM=$(AM335_BASE)/pru_sw/utils/pasm
LIBDIR_APP_LOADER?=$(AM335_BASE)/pru_sw/app_loader/lib
INCDIR_APP_LOADER?=$(AM335_BASE)/pru_sw/app_loader/include
CAPE_INCLUDE=../hardware/$(BEAGLEG_HARDWARE_TARGET)

# where the google test source has been unpacked.
GTEST_SOURCE=/usr/src/gtest

CXX=g++
CFLAGS+= -Wall -I$(INCDIR_APP_LOADER) -I$(CAPE_INCLUDE) -D_XOPEN_SOURCE=500 -g -O0  $(ARM_COMPILE_FLAGS)

# Using old < c++11 standard as not all embedded devices have recent compilers.
# Sigh.
CXXFLAGS+=$(CFLAGS) -std=c++03

LDFLAGS+=-lpthread -lm
PRUSS_LIBS=$(LIBDIR_APP_LOADER)/libprussdrv.a

# Assembled binary from *.p file.
PRU_BIN=motor-interface-pru_bin.h

GCODE_OBJECTS=gcode-parser.o gcode-machine-control.o determine-print-stats.o \
              generic-gpio.o arc-gen.o pwm-timer.o logging.o
OBJECTS=motor-operations.o sim-firmware.o pru-motion-queue.o $(GCODE_OBJECTS)
MAIN_OBJECTS=machine-control.o gcode-print-stats.o

TARGETS=../machine-control ../gcode-print-stats
UNITTEST_BINARIES=gcode-machine-control_test gcode-parser_test

all : $(TARGETS)

../gcode-print-stats: gcode-print-stats.o $(GCODE_OBJECTS)
	$(CROSS_COMPILE)$(CXX) $(CFLAGS) -o $@ $^ $(LDFLAGS)

../machine-control: machine-control.o $(OBJECTS)
	$(CROSS_COMPILE)$(CXX) $(CFLAGS) -o $@ $^ $(PRUSS_LIBS) $(LDFLAGS)

test: $(UNITTEST_BINARIES)
	for test_bin in $(UNITTEST_BINARIES) ; do ./$$test_bin ; done

valgrind-test: $(UNITTEST_BINARIES)
	for test_bin in $(UNITTEST_BINARIES) ; do valgrind ./$$test_bin ; done

$(PRU_BIN) : motor-interface-constants.h \
             $(CAPE_INCLUDE)/beagleg-pin-mapping.h \
	     $(CAPE_INCLUDE)/pru-io-routines.hp

gtest/libgtest.a:
	mkdir -p gtest ; cd gtest ; cmake $(GTEST_SOURCE)
	make -C gtest

%_test: %_test.cc $(OBJECTS) gtest/libgtest.a
	$(CROSS_COMPILE)$(CXX) $(CXXFLAGS) -o $@ $< $(OBJECTS) $(PRUSS_LIBS) $(LDFLAGS) gtest/libgtest.a

%.o: %.cc
	$(CROSS_COMPILE)$(CXX) $(CXXFLAGS)  -c  $< -o $@
	@$(CROSS_COMPILE)$(CXX) $(CXXFLAGS) -MM $< > $*.d

%_bin.h : %.p $(PASM)
	$(PASM) -I$(CAPE_INCLUDE) -V3 -c $<

$(PASM):
	make -C $(AM335_BASE)

# Explicit dependencies
pru-motion-queue.o : $(PRU_BIN)

# Auto generated dependencies
-include $(OBJECTS:.o=.d)

clean:
	rm -rf $(TARGETS) $(MAIN_OBJECTS) $(OBJECTS) $(PRU_BIN) $(UNITTEST_BINARIES) $(OBJECTS:.o=.d) $(MAIN_OBJECTS:.o=.d)
