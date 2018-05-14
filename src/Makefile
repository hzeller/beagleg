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

GIT_VERSION=$(shell git log -n1 --date=short --format="%cd (commit=%h)" 2>/dev/null || echo "[unknown version - compile from git]")

CFLAGS+=-Wall -I. -I$(INCDIR_APP_LOADER) -I$(CAPE_INCLUDE) -D_XOPEN_SOURCE=500 $(ARM_COMPILE_FLAGS) $(BEAGLEG_OPT_CFLAGS) -DCAPE_NAME='"$(BEAGLEG_HARDWARE_TARGET)"' -DBEAGLEG_VERSION='"$(GIT_VERSION)"'

# We use c++11 and require at least gcc-4.7.
# Note: older beaglebone wheezy images have even older compilers, but you can
# install a 4.7 easily:
#   sudo apt-get install g++-4.7
#   CXX=g++-4.7 make
CXXFLAGS+=-std=c++11 $(CFLAGS) $(CONFIG_FLAGS)
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
OBJECTS=motor-operations.o sim-firmware.o pru-motion-queue.o uio-pruss-interface.o $(GCODE_OBJECTS)
MAIN_OBJECTS=machine-control.o gcode-print-stats.o gcode2ps.o
TEST_FRAMEWORK_OBJECTS=gtest-all.o gmock-all.o

TARGETS=../machine-control ../gcode-print-stats gcode2ps
UNITTEST_BINARIES=gcode-machine-control_test config-parser_test machine-control-config_test planner_test motor-operations_test pru-motion-queue_test

DEPENDENCY_RULES=$(OBJECTS:=.d) $(UNITTEST_BINARIES:=.o.d) $(MAIN_OBJECTS:=.d)

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

local-tests: $(UNITTEST_BINARIES)
	for test_bin in $(UNITTEST_BINARIES) ; do echo ; echo $$test_bin; ./$$test_bin || exit 1 ; done

local-valgrind-tests: $(UNITTEST_BINARIES)
	for test_bin in $(UNITTEST_BINARIES) ; do valgrind --track-origins=yes --leak-check=full --error-exitcode=1 -q ./$$test_bin || exit 1; done

test: local-tests
	for d in $(SUBDIRS) ; do $(MAKE) -C $$d test ; done

coverage:
	LDFLAGS="-fprofile-arcs -ftest-coverage" BEAGLEG_OPT_CFLAGS="-O0 -g -fprofile-arcs -ftest-coverage" make test
	gcovr -r . --html --html-details -e ".*_test.cc" -p -o coverage.html

valgrind-test: local-valgrind-tests
	for d in $(SUBDIRS) ; do $(MAKE) -C $$d valgrind-test ; done

$(PRU_BIN) : motor-interface-constants.h \
             $(CAPE_INCLUDE)/beagleg-pin-mapping.h \
	     $(CAPE_INCLUDE)/pru-io-routines.hp

%_test: %_test.o $(OBJECTS) $(TEST_FRAMEWORK_OBJECTS) $(COMMON_LIBS) compiler-flags
	$(CROSS_COMPILE)$(CXX) -o $@ $< $(OBJECTS) $(COMMON_LIBS) $(PRUSS_LIBS) $(LDFLAGS) $(TEST_FRAMEWORK_OBJECTS)

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
	gs -q -dGraphicsAlphaBits=4 -dTextAlphaBits=4 -dEPSCrop -dBATCH -dNOPAUSE -sDEVICE=png16m -sOutputFile=$@ $<

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
	rm -rf $(TARGETS) $(MAIN_OBJECTS) $(OBJECTS) $(PRU_BIN) $(UNITTEST_BINARIES) $(UNITTEST_BINARIES:=.o) $(DEPENDENCY_RULES) $(TEST_FRAMEWORK_OBJECTS) *.gcda *.gcov *.gcno *.cc.html *.h.html
	$(MAKE) -C common clean
	$(MAKE) -C gcode-parser clean

dist-clean: clean
	make -C $(AM335_BASE) clean

compiler-flags: FORCE
	@echo '$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE)' | cmp -s - $@ || echo '$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE)' > $@

.PHONY: FORCE
