# (c) 2013 h.zeller@acm.org
# This is free software. License: GNU 3.0

# Location of testing framework
GTEST_INCLUDE=$(shell pkg-config --cflags-only-I gtest gmock 2>/dev/null || echo "-I/usr/include")
GTEST_LIBS=$(shell pkg-config --libs gtest gmock 2>/dev/null || echo "-lgtest -lgmock")

# Optmization flags. Typically, that should be -O3 for regluar use, but for
# debugging and development "-g -O0" can be more helpful. Overridable by
# environment variable.
BEAGLEG_OPT_CFLAGS?=-O3

CFLAGS+=-Wall -Wextra -W -Wno-unused-result -Wno-unused-parameter -D_XOPEN_SOURCE=500 $(ARM_COMPILE_FLAGS) $(BEAGLEG_OPT_CFLAGS) -I$(shell pwd)/..

CXXFLAGS+=-std=c++17 $(CFLAGS)
CXX?=g++

LDFLAGS+=-lpthread -lm

# Assembled binary from *.p file.
PRU_BIN=motor-interface-pru_bin.h

OBJECTS=logging.o string-util.o fd-mux.o linebuf-reader.o
GENLIB=libbeaglegbase.a

UNITTEST_BINARIES=string-util_test linebuf-reader_test container_test

DEPENDENCY_RULES=$(OBJECTS:=.d) $(UNITTEST_BINARIES:=.o.d) $(MAIN_OBJECTS:=.d)

all : $(GENLIB)

$(GENLIB): $(OBJECTS)
	$(AR) rcs $@ $^

test-binaries: $(UNITTEST_BINARIES)

test: test-binaries
	for test_bin in $(UNITTEST_BINARIES) ; do echo ; echo $$test_bin; ./$$test_bin || exit 1 ; done

valgrind-test: test-binaries
	for test_bin in $(UNITTEST_BINARIES) ; do valgrind --track-origins=yes --leak-check=full --error-exitcode=1 -q ./$$test_bin || exit 1; done


%_test: %_test.o $(GENLIB) compiler-flags
	$(CROSS_COMPILE)$(CXX) -o $@ $< $(GENLIB) $(GTEST_LIBS) $(LDFLAGS)

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

# Auto generated dependencies
-include $(DEPENDENCY_RULES)

clean:
	rm -rf $(GENLIB) $(MAIN_OBJECTS) $(OBJECTS) $(UNITTEST_BINARIES) $(UNITTEST_BINARIES:=.o) $(DEPENDENCY_RULES) *.gcda *.gcov *.gcno *.cc.html *.h.html

compiler-flags: FORCE
	@echo '$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE)' | cmp -s - $@ || echo '$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE)' > $@

.PHONY: FORCE

.SECONDARY: $(UNITTEST_BINARIES:=.o)
