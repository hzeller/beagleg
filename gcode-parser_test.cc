/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Test for gcode parser.
 */
#include "gcode-parser.h"

#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <strings.h>

// 'home' position of our simulated machine. Arbitrary values.
#define HOME_X 123
#define HOME_Y 456
#define HOME_Z 789
#define PROBE_POSITION 42

static int global_error_count = 0;
static void ExpectEqual(int line_no, int a, int b) {
  if (a != b) {
    fprintf(stderr, "Line %d: Expected %d but got %d\n", line_no, b, a);
    global_error_count++;
  }
}

#define EXPECT_EQUAL(a, b) ExpectEqual(__LINE__, (a), (b))

// In our mock implementation, we keep counters for each call.
enum {
  CALL_gcode_start,
  CALL_gcode_finished,
  CALL_go_home,
  CALL_probe_axis,
  CALL_motors_enable,
  CALL_coordinated_move,
  CALL_rapid_move,
  CALL_unprocessed,
  NUM_COUNTED_CALLS,
};

class ParseTester : public GCodeParser::Events {
public:
  ParseTester() {
    bzero(call_count, sizeof(call_count));
    GCodeParser::Config config;
    // some arbitrary machine origins to see that they are honored.
    config.machine_origin[AXIS_X] = HOME_X;
    config.machine_origin[AXIS_Y] = HOME_Y;
    config.machine_origin[AXIS_Z] = HOME_Z;
    parser_ = new GCodeParser(config, this);
  }
  virtual ~ParseTester() { delete parser_; }

  // Main function to test.
  void TestParseLine(const char *block) {
    parser_->ParseLine(block, NULL);
  }

  virtual void gcode_start()     { Count(CALL_gcode_start); }
  virtual void gcode_finished()  { Count(CALL_gcode_finished); }
  virtual void inform_origin_offset(const float *offset) {
    memcpy(parser_offset, offset, sizeof(AxesRegister));
  }
  virtual void go_home(AxisBitmap_t axis_bitmap) { Count(CALL_go_home); }
  virtual bool probe_axis(float feed_mm_p_sec, enum GCodeParserAxis axis,
                          float *probed_position) {
    Count(CALL_probe_axis);
    *probed_position = PROBE_POSITION;
    return true;
  }

  virtual void motors_enable(bool enable) { Count(CALL_motors_enable); }
  virtual bool coordinated_move(float feed_mm_p_sec, const float *axes) {
    Count(CALL_coordinated_move);
    StoreAbsPos(axes);
    return true;
  }
  virtual bool rapid_move(float feed_mm_p_sec, const float *axes) {
    Count(CALL_rapid_move);
    StoreAbsPos(axes);
    return true;
  }
  virtual const char *unprocessed(char letter, float value, const char *line) {
    Count(CALL_unprocessed);
    return NULL;
  }

  // Not interested.
  virtual void set_speed_factor(float factor) {}
  virtual void set_fanspeed(float value) {}
  virtual void set_temperature(float degrees_c) {}

public:
  // public counters.
  int call_count[NUM_COUNTED_CALLS];
  AxesRegister abs_pos;         // last coordinates we got from a move.
  AxesRegister parser_offset;   // current offset in the parser

private:
  void Count(int what) { call_count[what]++; }

  void StoreAbsPos(const float axes[]) {
    memcpy(abs_pos, axes, sizeof(AxesRegister));
  }

  GCodeParser *parser_;
};

class TestBracket {
public:
  TestBracket(const char *func)
    : funcname_(func), errors_before_(global_error_count) {
    printf(" %s\n", funcname_);
  }
  ~TestBracket() {
    if (errors_before_ == global_error_count) {
      printf(" DONE %s  (PASS)\n", funcname_);
    } else {
      printf(" DONE %s  (FAIL, %d errors)\n", funcname_,
             global_error_count - errors_before_);
    }
  }

private:
  const char *const funcname_;
  const int errors_before_;
};

#define START_TEST() TestBracket _testbracket_(__func__)

static void TEST_axes_letter_conversion() {
  START_TEST();
  // one way ..
  assert('X' == gcodep_axis2letter(AXIS_X));
  assert('A' == gcodep_axis2letter(AXIS_A));
  // .. and back
  assert(AXIS_X == gcodep_letter2axis('X'));
  assert(AXIS_X == gcodep_letter2axis('x'));
  assert(AXIS_Z == gcodep_letter2axis('z'));
  assert(AXIS_A == gcodep_letter2axis('A'));
  assert(AXIS_A == gcodep_letter2axis('a'));
}

static void TEST_simple_move() {
  START_TEST();
  ParseTester counter;

  counter.TestParseLine("G1 X100");
  assert(counter.call_count[CALL_coordinated_move] == 1);

  // If we move one axis, the others are still at the origin.
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 100);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Z], HOME_Z);

  counter.TestParseLine("G1 X10 Y10 Z-20");
  assert(counter.call_count[CALL_coordinated_move] == 2);
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 10);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y + 10);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Z], HOME_Z - 20);
}

static void TEST_absolute_relative() {
  START_TEST();
  ParseTester counter;

  counter.TestParseLine("G90");   // absolute mode.

  counter.TestParseLine("G1 X10 Y11 Z12");
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 10);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y + 11);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Z], HOME_Z + 12);

  // Another absolute position.
  counter.TestParseLine("G1 X20 Y21 Z22");
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 20);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y + 21);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Z], HOME_Z + 22);

  counter.TestParseLine("G91");   // Now, go in relative mode.

  counter.TestParseLine("G1 X5 Y6 Z7");
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 20 + 5);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y + 21 + 6);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Z], HOME_Z + 22 + 7);
}

static void TEST_set_origin_G92() {
  START_TEST();
  ParseTester counter;
  counter.TestParseLine("G1 X100 Y100");  // Some position.

  EXPECT_EQUAL(counter.parser_offset[AXIS_X], HOME_X);
  EXPECT_EQUAL(counter.parser_offset[AXIS_Y], HOME_Y);
  EXPECT_EQUAL(counter.parser_offset[AXIS_Z], HOME_Z);

  counter.TestParseLine("G92 X5 Y7");     // Tool left bottom of it.

  // New offset within machine cube.
  EXPECT_EQUAL(counter.parser_offset[AXIS_X], HOME_X + 100 - 5);
  EXPECT_EQUAL(counter.parser_offset[AXIS_Y], HOME_Y + 100 - 7);

  counter.TestParseLine("G1 X12 Y17");    // Move relative to that

  // Final position.
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 100 - 5 + 12);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y + 100 - 7 + 17);

  counter.TestParseLine("G92 X3 Y4");  // Set new origin relative to here
  counter.TestParseLine("G1 X1 Y1");

  // Now, we are relative to _that_
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 100 - 5 + 12 - 3 + 1);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y + 100 - 7 + 17 - 4 + 1);

  counter.TestParseLine("G92.2");      // Suspend.
  counter.TestParseLine("G1 X1 Y1");
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 1);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y + 1);

  // Only set one axis now, that way we see that the new axis is modified
  // and the original axis is pulled out of the G92 store again.
  counter.TestParseLine("G92 X0");     // Set again. Modify Current G92.
  counter.TestParseLine("G1 X1 Y1");
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 1 + 1);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y + 100 - 7 + 17 - 4 + 1);

  counter.TestParseLine("G92.2");      // Suspend.
  counter.TestParseLine("G1 X1 Y1");
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 1);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y + 1);

  counter.TestParseLine("G92.3");      // Restore.
  counter.TestParseLine("G1 X7 Y8");
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 1 + 7);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y + 100 - 7 + 17 - 4 + 8);

  counter.TestParseLine("G92.1");      // Reset. Back relative to machine.
  counter.TestParseLine("G1 X1 Y1");
  EXPECT_EQUAL(counter.abs_pos[AXIS_X], HOME_X + 1);
  EXPECT_EQUAL(counter.abs_pos[AXIS_Y], HOME_Y + 1);

  // Later, once that is implemented: test with a G54. It is not clear yet
  // if G92 works relative to the active coordinate system (G54, G55..), or
  // if it is an absolute offset at the time it was established.
}

static void TEST_probe_axis() {
  START_TEST();
  ParseTester counter;
  counter.TestParseLine("G30 Z3");  // 3: thickness of our probe

  EXPECT_EQUAL(counter.parser_offset[AXIS_Z], PROBE_POSITION - 3);

  counter.TestParseLine("G1 Z1");  // Move Z axis hovering 1 over it
  EXPECT_EQUAL(counter.abs_pos[AXIS_Z], PROBE_POSITION - 3 + 1);
}

int main() {
  printf("RUN (%s)\n", __FILE__);

  TEST_axes_letter_conversion();
  TEST_simple_move();
  TEST_absolute_relative();
  TEST_set_origin_G92();
  TEST_probe_axis();

  if (global_error_count == 0) {
    printf("PASS (%s)\n", __FILE__);
    return 0;
  } else {
    printf("FAILED (%s), %d errors\n", __FILE__, global_error_count);
    return 1;
  }
}
