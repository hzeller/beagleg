# (c) 2013 h.zeller@acm.org
# This is free software. License: GNU 3.0

# Stuff based on the environment. This assumes stuff to be compiled on the
# beaglebone and the am335x_pru_package checked out.
# https://github.com/beagleboard/am335x_pru_package

# Change here for which hardware you are compiling. See hardware/ directory.
# Currently supported BUMPS, CRAMPS, and VGEN5
BEAGLEG_HARDWARE_TARGET?=BUMPS

# Additional macros, for example to disable the pwm support simply add
# -D_DISABLE_PWM_TIMERS.
CONFIG_FLAGS?=

# In case you cross compile this on a different architecture, uncomment this
# and set the prefix. Or simply set the environment variable.
#CROSS_COMPILE?=arm-arago-linux-gnueabi-

# Tuning options for ARM CPU. Unset this in an environment variable if compiled
# on a different system.
ARM_COMPILE_FLAGS?=-mtune=cortex-a8 -march=armv7-a

# Allow to c onfigure e.g. BEAGLEG_HARDWARE_TARGET in a configuration file.
-include config.mk

# Location of am335x package https://github.com/beagleboard/am335x_pru_package
# We check this out in a local git submodule.
AM335_BASE=../am335x_pru_package
PASM=$(AM335_BASE)/pru_sw/utils/pasm
LIBDIR_APP_LOADER?=$(AM335_BASE)/pru_sw/app_loader/lib
INCDIR_APP_LOADER?=$(AM335_BASE)/pru_sw/app_loader/include
CAPE_INCLUDE=../hardware/$(BEAGLEG_HARDWARE_TARGET)

# Location of testing framework
GTEST_INCLUDE=$(shell pkg-config --cflags-only-I gtest gmock 2>/dev/null || echo "-I/usr/include")
GTEST_LIBS=$(shell pkg-config --libs gtest gmock 2>/dev/null || echo "-lgtest -lgmock")

# Optmization flags. Typically, that should be -O3 for regluar use, but for
# debugging and development "-g -O0" can be more helpful. Overridable by
# environment variable.
BEAGLEG_OPT_CFLAGS?=-O3

GIT_VERSION=$(shell git log -n1 --date=short --format="%cd (commit=%h)" 2>/dev/null || echo "[unknown version - compile from git]")

CFLAGS+=-Wall -Wextra -W -Wno-unused-parameter -I$(shell pwd) -I$(INCDIR_APP_LOADER) -I$(CAPE_INCLUDE) -D_XOPEN_SOURCE=500 $(ARM_COMPILE_FLAGS) $(BEAGLEG_OPT_CFLAGS) -DCAPE_NAME='"$(BEAGLEG_HARDWARE_TARGET)"' -DBEAGLEG_VERSION='"$(GIT_VERSION)"'

# Expected to have at least c++17 on all systems these days.
CXXFLAGS+=-std=c++17 $(CFLAGS) $(CONFIG_FLAGS)
CXX?=g++

LDFLAGS+=-lpthread -lm
PRUSS_LIBS=$(LIBDIR_APP_LOADER)/libprussdrv.a
COMMON_LIBS=common/libbeaglegbase.a gcode-parser/libgcodeparser.a
SUBDIRS=common gcode-parser

# Assembled binary from *.p file.
PRU_BIN=motor-interface-pru_bin.h


GCODE_OBJECTS=gcode-machine-control.o determine-print-stats.o \
              generic-gpio.o pwm-timer.o config-parser.o \
	      machine-control-config.o hardware-mapping.o \
	      spindle-control.o planner.o adc.o
OBJECTS=motion-queue-motor-operations.o sim-firmware.o sim-audio-out.o pru-motion-queue.o uio-pruss-interface.o $(GCODE_OBJECTS)
MAIN_OBJECTS=machine-control.o gcode-print-stats.o gcode2ps.o

TARGETS=../machine-control ../gcode-print-stats gcode2ps
UNITTEST_BINARIES=gcode-machine-control_test config-parser_test machine-control-config_test planner_test motion-queue-motor-operations_test pru-motion-queue_test

DEPENDENCY_RULES=$(OBJECTS:=.d) $(UNITTEST_BINARIES:=.o.d) $(MAIN_OBJECTS:=.d) hershey.o.d

all : $(TARGETS)

common/libbeaglegbase.a: FORCE
	$(MAKE) -C common

gcode-parser/libgcodeparser.a: FORCE
	$(MAKE) -C gcode-parser

../gcode-print-stats: gcode-print-stats.o $(GCODE_OBJECTS) $(COMMON_LIBS)
	$(CROSS_COMPILE)$(CXX) -o $@ $^ $(COMMON_LIBS) $(LDFLAGS)

../machine-control: machine-control.o $(OBJECTS) $(COMMON_LIBS)
	$(CROSS_COMPILE)$(CXX) -o $@ $^ $(COMMON_LIBS) $(PRUSS_LIBS) $(LDFLAGS)

# While this is developed and does not have a final name yet, let's not make
# it a toplevel tool in ../
gcode2ps: gcode2ps.o hershey.o $(GCODE_OBJECTS) $(COMMON_LIBS)
	$(CROSS_COMPILE)$(CXX) -o $@ $^ $(LDFLAGS)

test-html: test-out/test.html

test-out/test.html: gcode2ps test-create-html.sh testdata/*.gcode
	./test-create-html.sh testdata/*.gcode

test-binaries: $(UNITTEST_BINARIES)
	for d in $(SUBDIRS) ; do $(MAKE) -C $$d test-binaries || exit 1; done

local-tests: test-binaries
	for test_bin in $(UNITTEST_BINARIES) ; do echo ; echo $$test_bin; ./$$test_bin || exit 1 ; done

local-valgrind-tests: test-binaries
	for test_bin in $(UNITTEST_BINARIES) ; do valgrind --track-origins=yes --leak-check=full --error-exitcode=1 -q ./$$test_bin || exit 1; done

test: local-tests
	for d in $(SUBDIRS) ; do $(MAKE) -C $$d test || exit 1; done

beagleg.coverage: FORCE
	LDFLAGS="-fprofile-arcs -ftest-coverage" BEAGLEG_OPT_CFLAGS="-O0 -g -fprofile-arcs -ftest-coverage" $(MAKE) test
	lcov --no-external --exclude "*_test.cc" --capture --directory . --output-file beagleg.coverage

coverage-html: beagleg.coverage
	genhtml --output-directory coverage beagleg.coverage
	@echo "Output in $(shell realpath coverage)/index.html"

valgrind-test: local-valgrind-tests
	for d in $(SUBDIRS) ; do $(MAKE) -C $$d valgrind-test || exit 1; done

$(PRU_BIN) : motor-interface-constants.h \
             $(CAPE_INCLUDE)/beagleg-pin-mapping.h \
	     $(CAPE_INCLUDE)/pru-io-routines.hp

%_test: %_test.o $(OBJECTS) $(COMMON_LIBS) compiler-flags
	$(CROSS_COMPILE)$(CXX) -o $@ $< $(OBJECTS) $(COMMON_LIBS) $(PRUSS_LIBS) $(GTEST_LIBS) $(LDFLAGS)

%.o: %.cc compiler-flags
	$(CROSS_COMPILE)$(CXX) $(CXXFLAGS)  -c  $< -o $@
	@$(CROSS_COMPILE)$(CXX) $(CXXFLAGS) -MM $< > $@.d

%_test.o: %_test.cc compiler-flags
	$(CROSS_COMPILE)$(CXX)  $(GTEST_INCLUDE) $(CXXFLAGS) -c  $< -o $@
	@$(CROSS_COMPILE)$(CXX) $(GTEST_INCLUDE) $(CXXFLAGS) -MM $< > $@.d

%_bin.h : %.p $(PASM) compiler-flags
	$(PASM) -I$(CAPE_INCLUDE) -V3 -c $<

$(PASM):
	make -C $(AM335_BASE)

# Useful to convert gcode2ps output.
%.png : %.ps
	gs -q -dGraphicsAlphaBits=4 -dTextAlphaBits=4 -dEPSCrop -dBATCH -dNOPAUSE -sDEVICE=png16m -sOutputFile=$@ $<

# Explicit dependencies
uio-pruss-interface.o : $(PRU_BIN)

# Auto generated dependencies
-include $(DEPENDENCY_RULES)

clean:
	rm -rf $(TARGETS) $(MAIN_OBJECTS) $(OBJECTS) $(PRU_BIN) $(UNITTEST_BINARIES) $(UNITTEST_BINARIES:=.o) $(DEPENDENCY_RULES) $(TEST_FRAMEWORK_OBJECTS) hershey.o *.gcda *.gcov *.gcno *.cc.gcov.html *.h.gcov.html *.func.html
	$(MAKE) -C common clean
	$(MAKE) -C gcode-parser clean

dist-clean: clean
	make -C $(AM335_BASE) clean

compiler-flags: FORCE
	@echo '$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE)' | cmp -s - $@ || echo '$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE)' > $@

.PHONY: FORCE

.SECONDARY: $(UNITTEST_BINARIES:=.o)
