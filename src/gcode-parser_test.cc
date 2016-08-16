/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Test for gcode parser.
 */
#include "gcode-parser.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>

#include <gtest/gtest.h>

// 'home' position of our simulated machine. Arbitrary values.
#define HOME_X 123
#define HOME_Y 456
#define HOME_Z 789
#define PROBE_POSITION 42

namespace {
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

class ParseTester : public GCodeParser::EventReceiver {
public:
  ParseTester() {
    bzero(call_count, sizeof(call_count));
    GCodeParser::Config config;
    // some arbitrary machine origins to see that they are honored.
    config.machine_origin[AXIS_X] = HOME_X;
    config.machine_origin[AXIS_Y] = HOME_Y;
    config.machine_origin[AXIS_Z] = HOME_Z;
    parser_ = new GCodeParser(config, this, false);
  }
  virtual ~ParseTester() { delete parser_; }

  // Main function to test.
  void TestParseLine(const char *block) {
    parser_->ParseLine(block, stderr);
  }

  virtual void gcode_start()     { Count(CALL_gcode_start); }
  virtual void gcode_finished(bool)  { Count(CALL_gcode_finished); }
  virtual void inform_origin_offset(const AxesRegister &offset) {
    parser_offset = offset;
  }
  virtual void go_home(AxisBitmap_t axis_bitmap) { Count(CALL_go_home); }
  virtual bool probe_axis(float feed_mm_p_sec, enum GCodeParserAxis axis,
                          float *probed_position) {
    Count(CALL_probe_axis);
    *probed_position = PROBE_POSITION;
    return true;
  }

  virtual void motors_enable(bool enable) { Count(CALL_motors_enable); }
  virtual bool coordinated_move(float feed_mm_p_sec, const AxesRegister &axes) {
    Count(CALL_coordinated_move);
    abs_pos = axes;
    return true;
  }
  virtual bool rapid_move(float feed_mm_p_sec, const AxesRegister &axes) {
    Count(CALL_rapid_move);
    abs_pos = axes;
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
  virtual void wait_temperature() {}
  virtual void dwell(float ms) {}

public:
  // public counters.
  int call_count[NUM_COUNTED_CALLS];
  AxesRegister abs_pos;         // last coordinates we got from a move.
  AxesRegister parser_offset;   // current offset in the parser

private:
  void Count(int what) { call_count[what]++; }

  GCodeParser *parser_;
};

}  // namespace

TEST(GCodeParserTest, axes_letter_conversion) {
  // one way ..
  EXPECT_EQ('X', gcodep_axis2letter(AXIS_X));
  EXPECT_EQ('A', gcodep_axis2letter(AXIS_A));

  // invalid value.
  EXPECT_EQ('?', gcodep_axis2letter(static_cast<GCodeParserAxis>(42)));

  // .. and back
  EXPECT_EQ(AXIS_X, gcodep_letter2axis('X'));
  EXPECT_EQ(AXIS_X, gcodep_letter2axis('x'));
  EXPECT_EQ(AXIS_Z, gcodep_letter2axis('z'));
  EXPECT_EQ(AXIS_A, gcodep_letter2axis('A'));
  EXPECT_EQ(AXIS_A, gcodep_letter2axis('a'));

  // invalid valud.
  EXPECT_EQ(GCODE_NUM_AXES, gcodep_letter2axis('%'));
}

TEST(GCodeParserTest, simple_move) {
  ParseTester counter;

  counter.TestParseLine("G1 X100");
  EXPECT_EQ(1, counter.call_count[CALL_coordinated_move]);

  // If we move one axis, the others are still at the origin.
  EXPECT_EQ(HOME_X + 100, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y,       counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z,       counter.abs_pos[AXIS_Z]);

  counter.TestParseLine("G1 X10 Y10 Z-20");
  EXPECT_EQ(2, counter.call_count[CALL_coordinated_move]);
  EXPECT_EQ(HOME_X + 10, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 10,  counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z - 20,  counter.abs_pos[AXIS_Z]);
}

TEST(GCodeParserTest, ParsingCompactNumbers) {
  // Coordinates are valid without spaces in-between
  ParseTester counter;

  counter.TestParseLine("G1Y0010Z0011X0012");
  EXPECT_EQ(1, counter.call_count[CALL_coordinated_move]);

  EXPECT_EQ(HOME_X + 12, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 10, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 11, counter.abs_pos[AXIS_Z]);
}

TEST(GCodeParserTest, ParsingComments) {
  ParseTester counter;

  counter.TestParseLine("G1 X10 (This is some comment) Y11 Z12 ; end of line");
  EXPECT_EQ(1, counter.call_count[CALL_coordinated_move]);

  EXPECT_EQ(HOME_X + 10, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 11, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 12, counter.abs_pos[AXIS_Z]);
}

TEST(GCodeParserTest, VariousNumbersOfLeadingZero) {
  ParseTester counter;

  counter.TestParseLine("G1X.1Y0.2Z00000.4");
  EXPECT_EQ(1, counter.call_count[CALL_coordinated_move]);

  EXPECT_NEAR(HOME_X + 0.1, counter.abs_pos[AXIS_X], 1e-4);
  EXPECT_NEAR(HOME_Y + 0.2, counter.abs_pos[AXIS_Y], 1e-4);
  EXPECT_NEAR(HOME_Z + 0.4, counter.abs_pos[AXIS_Z], 1e-4);
}

TEST(GCodeParserTest, SpacesAroundNumbers) {
  ParseTester counter;

  counter.TestParseLine("G\t1 X 2 \tY\t4 Z \t5");
  EXPECT_EQ(1, counter.call_count[CALL_coordinated_move]);

  EXPECT_EQ(HOME_X + 2, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 4, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 5, counter.abs_pos[AXIS_Z]);
}

TEST(GCodeParserTest, SquishedTogetherNumbers) {
  ParseTester counter;

  // Provoke a situation in which we have 0x<something> to
  // test for accidental hex parsing.
  counter.TestParseLine("G1Y0X17Z23");
  EXPECT_EQ(1, counter.call_count[CALL_coordinated_move]);

  EXPECT_EQ(HOME_X + 17, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 0, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 23, counter.abs_pos[AXIS_Z]);
}

TEST(GCodeParserTest, InvalidNumbers) {
  {
    ParseTester counter;
    counter.TestParseLine("G1X--17");
    EXPECT_EQ(0, counter.call_count[CALL_coordinated_move]);
  }
  {
    ParseTester counter;
    counter.TestParseLine("G1X..1");
    EXPECT_EQ(0, counter.call_count[CALL_coordinated_move]);
  }
}

TEST(GCodeParserTest, absolute_relative) {
  ParseTester counter;

  counter.TestParseLine("G90");   // absolute mode.

  counter.TestParseLine("G1 X10 Y11 Z12");
  EXPECT_EQ(HOME_X + 10, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 11, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 12, counter.abs_pos[AXIS_Z]);

  // Another absolute position.
  counter.TestParseLine("G1 X20 Y21 Z22");
  EXPECT_EQ(HOME_X + 20, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 21, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 22, counter.abs_pos[AXIS_Z]);

  counter.TestParseLine("G91");   // Now, go in relative mode.

  counter.TestParseLine("G1 X5 Y6 Z7");
  EXPECT_EQ(HOME_X + 20 + 5, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 21 + 6, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 22 + 7, counter.abs_pos[AXIS_Z]);

  // Still in relative mode. So even if we set a G92, all movements
  // still are relative
  counter.TestParseLine("G92 X0 Y0");

  counter.TestParseLine("G1 X17 Y11");
  EXPECT_EQ(HOME_X + 20 + 5 + 17, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 21 + 6 + 11, counter.abs_pos[AXIS_Y]);
}

TEST(GCodeParserTest, set_origin_G92) {
  ParseTester counter;
  counter.TestParseLine("G1 X100 Y100");  // Some position.

  EXPECT_EQ(HOME_X, counter.parser_offset[AXIS_X]);
  EXPECT_EQ(HOME_Y, counter.parser_offset[AXIS_Y]);
  EXPECT_EQ(HOME_Z, counter.parser_offset[AXIS_Z]);

  counter.TestParseLine("G92 X5 Y7");     // Tool left bottom of it.

  // New offset within machine cube.
  EXPECT_EQ(HOME_X + 100 - 5, counter.parser_offset[AXIS_X]);
  EXPECT_EQ(HOME_Y + 100 - 7, counter.parser_offset[AXIS_Y]);

  counter.TestParseLine("G1 X12 Y17");    // Move relative to that

  // Final position.
  EXPECT_EQ(HOME_X + 100 - 5 + 12, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 100 - 7 + 17, counter.abs_pos[AXIS_Y]);

  counter.TestParseLine("G92 X3 Y4");  // Set new origin relative to here
  counter.TestParseLine("G1 X1 Y1");

  // Now, we are relative to _that_
  EXPECT_EQ(HOME_X + 100 - 5 + 12 - 3 + 1, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 100 - 7 + 17 - 4 + 1, counter.abs_pos[AXIS_Y]);

  counter.TestParseLine("G92.2");      // Suspend.
  counter.TestParseLine("G1 X1 Y1");
  EXPECT_EQ(HOME_X + 1, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 1, counter.abs_pos[AXIS_Y]);

  // Only set one axis now, that way we see that the new axis is modified
  // and the original axis is pulled out of the G92 store again.
  counter.TestParseLine("G92 X0");     // Set again. Modify Current G92.
  counter.TestParseLine("G1 X1 Y1");
  EXPECT_EQ(HOME_X + 1 + 1, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 100 - 7 + 17 - 4 + 1, counter.abs_pos[AXIS_Y]);

  counter.TestParseLine("G92.2");      // Suspend.
  counter.TestParseLine("G1 X1 Y1");
  EXPECT_EQ(HOME_X + 1, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 1, counter.abs_pos[AXIS_Y]);

  counter.TestParseLine("G92.3");      // Restore.
  counter.TestParseLine("G1 X7 Y8");
  EXPECT_EQ(HOME_X + 1 + 7, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 100 - 7 + 17 - 4 + 8, counter.abs_pos[AXIS_Y]);

  counter.TestParseLine("G92.1");      // Reset. Back relative to machine.
  counter.TestParseLine("G1 X1 Y1");
  EXPECT_EQ(HOME_X + 1, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 1, counter.abs_pos[AXIS_Y]);

  // Later, once that is implemented: test with a G54. It is not clear yet
  // if G92 works relative to the active coordinate system (G54, G55..), or
  // if it is an absolute offset at the time it was established.
}

TEST(GCodeParserTest, probe_axis) {
  ParseTester counter;
  counter.TestParseLine("G30 Z3");  // 3: thickness of our probe

  EXPECT_EQ(PROBE_POSITION - 3, counter.parser_offset[AXIS_Z]);

  counter.TestParseLine("G1 Z1");  // Move Z axis hovering 1 over it
  EXPECT_EQ(PROBE_POSITION - 3 + 1, counter.abs_pos[AXIS_Z]);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
