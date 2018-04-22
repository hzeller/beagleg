# (c) 2013 h.zeller@acm.org
# This is free software. License: GNU 3.0

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

CFLAGS+=-Wall -D_XOPEN_SOURCE=500 $(ARM_COMPILE_FLAGS) $(BEAGLEG_OPT_CFLAGS) -I../

# We use c++11, but it looks like that even the latest
# bone-debian-7.11-lxde-4gb-armhf-2016-06-16-4gb image has an ancient 4.6.3
# compiler that still referred to that standard as c++0x
CXXFLAGS+=-std=c++0x $(CFLAGS)
CXX?=g++

LDFLAGS+=-lpthread -lm

COMMON_LIBS=../common/libbeaglegbase.a

OBJECTS=gcode-parser.o arc-gen.o simple-lexer.o gcode-parser-config.o
GENLIB=libgcodeparser.a

UNITTEST_BINARIES=gcode-parser_test arc-gen_test
TEST_FRAMEWORK_OBJECTS=gtest-all.o gmock-all.o

DEPENDENCY_RULES=$(OBJECTS:=.d) $(UNITTEST_BINARIES:=.o.d) $(MAIN_OBJECTS:=.d)

all : $(GENLIB)

$(GENLIB): $(OBJECTS)
	$(AR) rcs $@ $^

../common/libbeaglegbase.a:
	$(MAKE) -C ../common

test: $(UNITTEST_BINARIES)
	for test_bin in $(UNITTEST_BINARIES) ; do echo ; echo $$test_bin; ./$$test_bin || exit 1 ; done

valgrind-test: $(UNITTEST_BINARIES)
	for test_bin in $(UNITTEST_BINARIES) ; do valgrind --track-origins=yes --leak-check=full --error-exitcode=1 -q ./$$test_bin || exit 1; done


%_test: %_test.o $(GENLIB) $(COMMON_LIBS) $(TEST_FRAMEWORK_OBJECTS) compiler-flags
	$(CROSS_COMPILE)$(CXX) -o $@ $< $(GENLIB) $(COMMON_LIBS) $(LDFLAGS) $(TEST_FRAMEWORK_OBJECTS)

%.o: %.cc compiler-flags
	$(CROSS_COMPILE)$(CXX) $(CXXFLAGS)  -c  $< -o $@
	@$(CROSS_COMPILE)$(CXX) $(CXXFLAGS) -MM $< > $@.d

%_test.o: %_test.cc compiler-flags
	$(CROSS_COMPILE)$(CXX)  $(GTEST_INCLUDE) $(CXXFLAGS) -c  $< -o $@
	@$(CROSS_COMPILE)$(CXX) $(GTEST_INCLUDE) $(CXXFLAGS) -MM $< > $@.d

# Auto generated dependencies
-include $(DEPENDENCY_RULES)

# Test and Mock framework.
gtest-all.o : $(GTEST_SOURCE)/src/gtest-all.cc compiler-flags
	$(CROSS_COMPILE)$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE) -I$(GTEST_SOURCE) -I$(GTEST_SOURCE)/include -c  $< -o $@

gmock-all.o : $(GMOCK_SOURCE)/src/gmock-all.cc compiler-flags
	$(CROSS_COMPILE)$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE) -I$(GMOCK_SOURCE) -I$(GMOCK_SOURCE)/include -c  $< -o $@

clean:
	rm -rf $(GENLIB) $(OBJECTS) $(UNITTEST_BINARIES) $(UNITTEST_BINARIES:=.o) $(DEPENDENCY_RULES) $(TEST_FRAMEWORK_OBJECTS) *.gcda *.gcov *.gcno *.cc.html *.h.html

compiler-flags: FORCE
	@echo '$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE)' | cmp -s - $@ || echo '$(CXX) $(CXXFLAGS) $(GTEST_INCLUDE)' > $@

.PHONY: FORCE
