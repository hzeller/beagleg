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

# Where the google test source has been unpacked. If this is via debian
# package, that is probably in /usr/src.
# You also can check out from head in https://github.com/google/googletest and
# point to the googletest/ and googlemock/ directories.
GTEST_SOURCE?=/usr/src/gtest
GMOCK_SOURCE?=/usr/src/gmock

# In particular, if this is a checked out version of google test/mock, this
# makes sure that we see the right includes.
GTEST_INCLUDE=-I$(GTEST_SOURCE)/include -I$(GMOCK_SOURCE)/include

# Optmization flags. Typically, that should be -O3 for regluar use, but for
# debugging and development "-g -O0" can be more helpful. Overridable by
# environment variable.
BEAGLEG_OPT_CFLAGS?=-O3

CFLAGS+=-Wall -I$(INCDIR_APP_LOADER) -I$(CAPE_INCLUDE) -D_XOPEN_SOURCE=500 $(ARM_COMPILE_FLAGS) $(BEAGLEG_OPT_CFLAGS) -DCAPE_NAME='"$(BEAGLEG_HARDWARE_TARGET)"'

# We export the following variables so that the googletest library will get
# the same variables when compiling.
# We use c++11, but it looks like that even the latest
# bone-debian-7.11-lxde-4gb-armhf-2016-06-16-4gb image has an ancient 4.6.3
# compiler that still referred to that standard as c++0x
export CXXFLAGS+=-std=c++0x $(CFLAGS)
export CXX?=g++

LDFLAGS+=-lpthread -lm
PRUSS_LIBS=$(LIBDIR_APP_LOADER)/libprussdrv.a

# Assembled binary from *.p file.
PRU_BIN=motor-interface-pru_bin.h

GCODE_OBJECTS=gcode-parser.o gcode-machine-control.o determine-print-stats.o \
              generic-gpio.o arc-gen.o pwm-timer.o logging.o config-parser.o \
	      string-util.o machine-control-config.o hardware-mapping.o \
	      spindle-control.o planner.o simple-lexer.o adc.o \
	      gcode-parser-config.o fd-mux.o linebuf-reader.o
OBJECTS=motor-operations.o sim-firmware.o pru-motion-queue.o uio-pruss-interface.o $(GCODE_OBJECTS)
MAIN_OBJECTS=machine-control.o gcode-print-stats.o gcode2ps.o
TEST_FRAMEWORK_OBJECTS=gtest-all.o gmock-all.o

TARGETS=../machine-control ../gcode-print-stats gcode2ps
UNITTEST_BINARIES=gcode-machine-control_test gcode-parser_test config-parser_test string-util_test machine-control-config_test arc-gen_test planner_test simple-lexer_test pru-motion-queue_test linebuf-reader_test

DEPENDENCY_RULES=$(OBJECTS:=.d) $(UNITTEST_BINARIES:=.o.d) $(MAIN_OBJECTS:=.d)

all : $(TARGETS)

../gcode-print-stats: gcode-print-stats.o $(GCODE_OBJECTS)
	$(CROSS_COMPILE)$(CXX) -o $@ $^ $(LDFLAGS)

../machine-control: machine-control.o $(OBJECTS)
	$(CROSS_COMPILE)$(CXX) -o $@ $^ $(PRUSS_LIBS) $(LDFLAGS)

# While this is developed and does not have a final name yet, let's not make
# it a toplevel tool in ../
gcode2ps: gcode2ps.o $(GCODE_OBJECTS)
	$(CROSS_COMPILE)$(CXX) -o $@ $^ $(LDFLAGS)

test-html: test-out/test.html

test-out/test.html: gcode2ps test-create-html.sh testdata/*.gcode
	./test-create-html.sh testdata/*.gcode

test: $(UNITTEST_BINARIES)
	for test_bin in $(UNITTEST_BINARIES) ; do echo ; echo $$test_bin; ./$$test_bin || exit 1 ; done

valgrind-test: $(UNITTEST_BINARIES)
	for test_bin in $(UNITTEST_BINARIES) ; do valgrind --track-origins=yes --leak-check=full --error-exitcode=1 -q ./$$test_bin || exit 1; done

$(PRU_BIN) : motor-interface-constants.h \
             $(CAPE_INCLUDE)/beagleg-pin-mapping.h \
	     $(CAPE_INCLUDE)/pru-io-routines.hp

%_test: %_test.o $(OBJECTS) $(TEST_FRAMEWORK_OBJECTS) compiler-flags
	$(CROSS_COMPILE)$(CXX) -o $@ $< $(OBJECTS) $(PRUSS_LIBS) $(LDFLAGS) $(TEST_FRAMEWORK_OBJECTS)

%.o: %.cc compiler-flags
	$(CROSS_COMPILE)$(CXX) $(CXXFLAGS)  -c  $< -o $@
	@$(CROSS_COMPILE)$(CXX) $(CXXFLAGS) -MM $< > $@.d

%_test.o: %_test.cc compiler-flags
	$(CROSS_COMPILE)$(CXX)  $(GTEST_INCLUDE) $(CXXFLAGS) -c  $< -o $@
	@$(CROSS_COMPILE)$(CXX) $(GTEST_INCLUDE) $(CXXFLAGS) -MM $< > $@.d

%_bin.h : %.p $(PASM)
	$(PASM) -I$(CAPE_INCLUDE) -V3 -c $<

$(PASM):
	make -C $(AM335_BASE)

# Useful to convert gcode2ps output.
%.png : %.ps
	gs -q -r144 -dGraphicsAlphaBits=4 -dTextAlphaBits=4 -dEPSCrop -dBATCH -dNOPAUSE -sDEVICE=png16m -sOutputFile=$@ $<

# Explicit dependencies
uio-pruss-interface.o : $(PRU_BIN)

# Auto generated dependencies
-include $(DEPENDENCY_RULES)

# Test and Mock framework.
gtest-all.o : $(GTEST_SOURCE)/src/gtest-all.cc compiler-flags
	$(CROSS_COMPILE)$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE) -I$(GTEST_SOURCE) -I$(GTEST_SOURCE)/include -c  $< -o $@

gmock-all.o : $(GMOCK_SOURCE)/src/gmock-all.cc compiler-flags
	$(CROSS_COMPILE)$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE) -I$(GMOCK_SOURCE) -I$(GMOCK_SOURCE)/include -c  $< -o $@

clean:
	rm -rf $(TARGETS) $(MAIN_OBJECTS) $(OBJECTS) $(PRU_BIN) $(UNITTEST_BINARIES) $(UNITTEST_BINARIES:=.o) $(DEPENDENCY_RULES) $(TEST_FRAMEWORK_OBJECTS)

compiler-flags: FORCE
	@echo '$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE)' | cmp -s - $@ || echo '$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE)' > $@

.PHONY: FORCE
