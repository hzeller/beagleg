/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Test for gcode parser.
 */
#include "gcode-parser.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <math.h>

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
  NUM_COUNTED_CALLS
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
    memset(parameters_, 0, sizeof(parameters_));
    config.num_parameters = sizeof(parameters_) / sizeof(float);
    config.parameters = parameters_;
    parser_ = new GCodeParser(config, this, false);
    EXPECT_EQ(0, parser_->error_count());
  }
  virtual ~ParseTester() { delete parser_; }

  float get_parameter(int num) {
    assert(num >= 0 && num < (int)(sizeof(parameters_)/sizeof(float)));
    return parameters_[num];
  }

  // Main function to test. Returns 'false' if parsing failed.
  bool TestParseLine(const char *block) {
    int errors_before = parser_->error_count();
    parser_->ParseLine(block, stderr);
    return parser_->error_count() == errors_before;
  }

  virtual void gcode_start(GCodeParser *)     { Count(CALL_gcode_start); }
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
  float parameters_[5400];

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

  EXPECT_TRUE(counter.TestParseLine("G1Y0010Z0011X0012"));
  EXPECT_EQ(1, counter.call_count[CALL_coordinated_move]);

  EXPECT_EQ(HOME_X + 12, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 10, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 11, counter.abs_pos[AXIS_Z]);
}

TEST(GCodeParserTest, ParsingComments) {
  ParseTester counter;

  EXPECT_TRUE(counter.TestParseLine(
                "G1 X10 (This is some comment) Y11 Z12 ; end of line"));
  EXPECT_EQ(1, counter.call_count[CALL_coordinated_move]);

  EXPECT_EQ(HOME_X + 10, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 11, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 12, counter.abs_pos[AXIS_Z]);
}

TEST(GCodeParserTest, VariousNumbersOfLeadingZero) {
  ParseTester counter;

  EXPECT_TRUE(counter.TestParseLine("G1X.1Y0.2Z00000.4"));
  EXPECT_EQ(1, counter.call_count[CALL_coordinated_move]);

  EXPECT_NEAR(HOME_X + 0.1, counter.abs_pos[AXIS_X], 1e-4);
  EXPECT_NEAR(HOME_Y + 0.2, counter.abs_pos[AXIS_Y], 1e-4);
  EXPECT_NEAR(HOME_Z + 0.4, counter.abs_pos[AXIS_Z], 1e-4);
}

TEST(GCodeParserTest, SpacesAroundNumbers) {
  ParseTester counter;

  EXPECT_TRUE(counter.TestParseLine("G\t1 X 2 \tY\t4 Z \t5"));
  EXPECT_EQ(1, counter.call_count[CALL_coordinated_move]);

  EXPECT_EQ(HOME_X + 2, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 4, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 5, counter.abs_pos[AXIS_Z]);
}

TEST(GCodeParserTest, SquishedTogetherNumbers) {
  ParseTester counter;

  // Provoke a situation in which we have 0x<something> to
  // test for accidental hex parsing.
  EXPECT_TRUE(counter.TestParseLine("G1Y0X17Z23"));
  EXPECT_EQ(1, counter.call_count[CALL_coordinated_move]);

  EXPECT_EQ(HOME_X + 17, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 0, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 23, counter.abs_pos[AXIS_Z]);
}

TEST(GCodeParserTest, InvalidNumbers) {
  {
    ParseTester counter;
    EXPECT_FALSE(counter.TestParseLine("G1X--17"));
    EXPECT_EQ(0, counter.call_count[CALL_coordinated_move]);
  }
  {
    ParseTester counter;
    EXPECT_FALSE(counter.TestParseLine("G1X..1"));
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

TEST(GCodeParserTest, parameters) {
  ParseTester counter;

  // without comments
  EXPECT_TRUE(counter.TestParseLine("#1=25"));
  EXPECT_EQ(25, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("G1 X#1"));
  EXPECT_EQ(HOME_X + 25, counter.abs_pos[AXIS_X]);

  // with comments before
  EXPECT_TRUE(counter.TestParseLine("(set param) #1=50"));
  EXPECT_EQ(50, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("G1 X#1"));
  EXPECT_EQ(HOME_X + 50, counter.abs_pos[AXIS_X]);

  // with comments after
  EXPECT_TRUE(counter.TestParseLine("#1=75 (set param)"));
  EXPECT_EQ(75, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("G1 X#1"));
  EXPECT_EQ(HOME_X + 75, counter.abs_pos[AXIS_X]);

  // test inline set
  EXPECT_TRUE(counter.TestParseLine("G1 #1=100 X#1"));
  EXPECT_EQ(100, counter.get_parameter(1));
  EXPECT_EQ(HOME_X + 100, counter.abs_pos[AXIS_X]);

  // test multiple parameters
  EXPECT_TRUE(counter.TestParseLine("#1=25"));
  EXPECT_TRUE(counter.TestParseLine("#2=50"));
  EXPECT_TRUE(counter.TestParseLine("#3=75"));
  EXPECT_EQ(25, counter.get_parameter(1));
  EXPECT_EQ(50, counter.get_parameter(2));
  EXPECT_EQ(75, counter.get_parameter(3));
  EXPECT_TRUE(counter.TestParseLine("G1 X#1 Y#2 Z#3"));
  EXPECT_EQ(HOME_X + 25, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 50, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 75, counter.abs_pos[AXIS_Z]);

  EXPECT_TRUE(counter.TestParseLine("G1 #1=100 #2=200 #3=300 X#1 Y#2 Z#3"));
  EXPECT_EQ(HOME_X + 100, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y + 200, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z + 300, counter.abs_pos[AXIS_Z]);

  // test invalid parameter parsing
  EXPECT_FALSE(counter.TestParseLine("#"));          // expected value after '#'
  EXPECT_FALSE(counter.TestParseLine("#G1 X20"));    // unknown unary
  EXPECT_FALSE(counter.TestParseLine("#5400=100"));  // unsupported parameter number
  EXPECT_FALSE(counter.TestParseLine("#1="));        // expected value after '#1='
  EXPECT_FALSE(counter.TestParseLine("#1=G1 X10"));  // expected value after '#1='
  EXPECT_NE(HOME_X + 10, counter.abs_pos[AXIS_X]);

  // Parameter #0 is special: we can read it, but we can't set it.
  EXPECT_TRUE(counter.TestParseLine("G1 X#0"));
  EXPECT_EQ(HOME_X + 0, counter.abs_pos[AXIS_X]);
  EXPECT_FALSE(counter.TestParseLine("#0=42"));      // writing unsupported parameter number
  EXPECT_EQ(0, counter.get_parameter(0));

  // test indexed parameters
  EXPECT_TRUE(counter.TestParseLine("#1=25"));
  EXPECT_EQ(25, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#2=1"));
  EXPECT_EQ(1, counter.get_parameter(2));

  EXPECT_TRUE(counter.TestParseLine("G1 X##2"));
  EXPECT_EQ(HOME_X + 25, counter.abs_pos[AXIS_X]);
}

TEST(GCodeParserTest, set_system_origin) {
  ParseTester counter;

  // set the G54 coordinate system to 100,100,0
  counter.TestParseLine("G10 L2 P1 X100 Y100 Z0");

  // make sure we are at the home position with no offset
  counter.TestParseLine("G1 X0 Y0 Z0");
  EXPECT_EQ(HOME_X, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z, counter.abs_pos[AXIS_Z]);
  EXPECT_EQ(HOME_X, counter.parser_offset[AXIS_X]);
  EXPECT_EQ(HOME_Y, counter.parser_offset[AXIS_Y]);
  EXPECT_EQ(HOME_Z, counter.parser_offset[AXIS_Z]);

  // change to the G54 coordinate system
  counter.TestParseLine("G54");
  EXPECT_EQ(1, counter.get_parameter(5220));

  // the machine is not expected to move
  EXPECT_EQ(HOME_X, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z, counter.abs_pos[AXIS_Z]);
  // the offset should be set
  EXPECT_EQ(100, counter.parser_offset[AXIS_X]);
  EXPECT_EQ(100, counter.parser_offset[AXIS_Y]);
  EXPECT_EQ(0, counter.parser_offset[AXIS_Z]);

  // test the remaing GCodes
  EXPECT_TRUE(counter.TestParseLine("G55"));
  EXPECT_EQ(2, counter.get_parameter(5220));

  EXPECT_TRUE(counter.TestParseLine("G56"));
  EXPECT_EQ(3, counter.get_parameter(5220));

  EXPECT_TRUE(counter.TestParseLine("G57"));
  EXPECT_EQ(4, counter.get_parameter(5220));

  EXPECT_TRUE(counter.TestParseLine("G58"));
  EXPECT_EQ(5, counter.get_parameter(5220));

  EXPECT_TRUE(counter.TestParseLine("G59"));
  EXPECT_EQ(6, counter.get_parameter(5220));

  EXPECT_TRUE(counter.TestParseLine("G59.1"));
  EXPECT_EQ(7, counter.get_parameter(5220));

  EXPECT_TRUE(counter.TestParseLine("G59.2"));
  EXPECT_EQ(8, counter.get_parameter(5220));

  EXPECT_TRUE(counter.TestParseLine("G59.3"));
  EXPECT_EQ(9, counter.get_parameter(5220));

  // this one should fail as we attempt to choose an invalid coordinate system.
  EXPECT_FALSE(counter.TestParseLine("G59.4"));
  EXPECT_EQ(9, counter.get_parameter(5220));     // kept at prev. coord system.

  // set the G56 coordinate system to 25,50,10
  EXPECT_TRUE(counter.TestParseLine("G10 L2 P3 X25 Y50 Z10"));
  EXPECT_TRUE(counter.TestParseLine("G56"));
  // the machine is still not expected to move
  EXPECT_EQ(HOME_X, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(HOME_Y, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(HOME_Z, counter.abs_pos[AXIS_Z]);
  // the offset should be set
  EXPECT_EQ(25, counter.parser_offset[AXIS_X]);
  EXPECT_EQ(50, counter.parser_offset[AXIS_Y]);
  EXPECT_EQ(10, counter.parser_offset[AXIS_Z]);
  // move to the new origin
  counter.TestParseLine("G1 X0 Y0 Z0");
  EXPECT_EQ(25, counter.abs_pos[AXIS_X]);
  EXPECT_EQ(50, counter.abs_pos[AXIS_Y]);
  EXPECT_EQ(10, counter.abs_pos[AXIS_Z]);
}

TEST(GCodeParserTest, expressions) {
  ParseTester counter;

  // binary operations

  // addition
  EXPECT_TRUE(counter.TestParseLine("#1=[100 + 20]"));
  EXPECT_EQ(100 + 20, counter.get_parameter(1));

  // subtraction
  EXPECT_TRUE(counter.TestParseLine("#1=[100 - 20]"));
  EXPECT_EQ(100 - 20, counter.get_parameter(1));

  // division
  EXPECT_TRUE(counter.TestParseLine("#1=[100 / 20]"));
  EXPECT_EQ(100 / 20, counter.get_parameter(1));

  // multiplication
  EXPECT_TRUE(counter.TestParseLine("#1=[100 * 20]"));
  EXPECT_EQ(100 * 20, counter.get_parameter(1));

  // power
  EXPECT_TRUE(counter.TestParseLine("#1=[2 ** 3]"));
  EXPECT_EQ(powf(2, 3), counter.get_parameter(1));

  // modulus
  EXPECT_TRUE(counter.TestParseLine("#1=[5 MOD 2]"));
  EXPECT_EQ(fabs(fmodf(5, 2)), counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=[-5 MOD 2]"));
  EXPECT_EQ(fabs(fmodf(5, 2)), counter.get_parameter(1));

  // logical and
  EXPECT_TRUE(counter.TestParseLine("#1=[1 AND 1]"));
  EXPECT_EQ(1, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=[1 AND 0]"));
  EXPECT_EQ(0, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=[0 AND 1]"));
  EXPECT_EQ(0, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=[0 AND 0]"));
  EXPECT_EQ(0, counter.get_parameter(1));

  // logical or
  EXPECT_TRUE(counter.TestParseLine("#1=[1 OR 1]"));
  EXPECT_EQ(1, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=[1 OR 0]"));
  EXPECT_EQ(1, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=[0 OR 1]"));
  EXPECT_EQ(1, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=[0 OR 0]"));
  EXPECT_EQ(0, counter.get_parameter(1));

  // logical xor
  EXPECT_TRUE(counter.TestParseLine("#1=[1 XOR 1]"));
  EXPECT_EQ(0, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=[1 XOR 0]"));
  EXPECT_EQ(1, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=[0 XOR 1]"));
  EXPECT_EQ(1, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=[0 XOR 0]"));
  EXPECT_EQ(0, counter.get_parameter(1));

  // precedence
  EXPECT_TRUE(counter.TestParseLine("#1=[2.0 / 3 * 1.5 - 5.5 / 11.0]"));
  EXPECT_EQ(((2.0 / 3) * 1.5) - (5.5 / 11.0), counter.get_parameter(1));

  // unary operations

  // absolute value
  EXPECT_TRUE(counter.TestParseLine("#1=abs[-100]"));
  EXPECT_EQ(fabsf(-100.0f), counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=abs[100]"));
  EXPECT_EQ(fabsf(100.0f), counter.get_parameter(1));

  // arc cosine
  EXPECT_TRUE(counter.TestParseLine("#1=acos[.5]"));
  EXPECT_FLOAT_EQ((acosf(.5f) * 180.0f) / M_PI, counter.get_parameter(1));
  EXPECT_EQ(60, counter.get_parameter(1));

  // arc sine
  EXPECT_TRUE(counter.TestParseLine("#1=asin[.5]"));
  EXPECT_FLOAT_EQ((asinf(.5f) * 180.0f) / M_PI, counter.get_parameter(1));
  EXPECT_EQ(30, counter.get_parameter(1));

  // arc tangent
  EXPECT_TRUE(counter.TestParseLine("#1=atan[4]/[4]"));
  EXPECT_FLOAT_EQ((atan2f(4.0f, 4.0f) * 180.0f) / M_PI,
                  counter.get_parameter(1));
  EXPECT_EQ(45, counter.get_parameter(1));

  // cosine
  EXPECT_TRUE(counter.TestParseLine("#1=cos[60]"));
  EXPECT_EQ(cosf((60 * M_PI) / 180.0f), counter.get_parameter(1));

  // e raised to the given power
  EXPECT_TRUE(counter.TestParseLine("#1=exp[4]"));
  EXPECT_EQ(expf(4), counter.get_parameter(1));

  // round down
  EXPECT_TRUE(counter.TestParseLine("#1=fix[4.5]"));
  EXPECT_EQ(floorf(4.5), counter.get_parameter(1));

  // round up
  EXPECT_TRUE(counter.TestParseLine("#1=fup[4.5]"));
  EXPECT_EQ(ceilf(4.5), counter.get_parameter(1));

  // natural logarithm
  EXPECT_TRUE(counter.TestParseLine("#1=ln[10]"));
  EXPECT_EQ(logf(10), counter.get_parameter(1));

  // round to nearest whole number
  EXPECT_TRUE(counter.TestParseLine("#1=round[4.4]"));
  EXPECT_EQ(4, counter.get_parameter(1));
  EXPECT_TRUE(counter.TestParseLine("#1=round[4.5]"));
  EXPECT_EQ(5, counter.get_parameter(1));

  // sine
  EXPECT_TRUE(counter.TestParseLine("#1=sin[30]"));
  EXPECT_EQ(sinf((30 * M_PI) / 180.0f), counter.get_parameter(1));

  // square root
  EXPECT_TRUE(counter.TestParseLine("#1=sqrt[4]"));
  EXPECT_EQ(sqrt(4), counter.get_parameter(1));

  // sine
  EXPECT_TRUE(counter.TestParseLine("#1=tan[30]"));
  EXPECT_EQ(tanf((30 * M_PI) / 180.0f), counter.get_parameter(1));
}

TEST(GCodeParserTest, precedence) {
  ParseTester counter;

  EXPECT_TRUE(counter.TestParseLine("#1=[1 + 1]"));
  EXPECT_EQ(1 + 1, counter.get_parameter(1));

  EXPECT_TRUE(counter.TestParseLine("#1=[6 + 15 / 3]"));
  EXPECT_EQ(6 + 15 / 3, counter.get_parameter(1));

  EXPECT_TRUE(counter.TestParseLine("#1=[[6 + 15] / 3]"));
  EXPECT_EQ((6 + 15) / 3, counter.get_parameter(1));

  EXPECT_TRUE(counter.TestParseLine("#1=[3 * 2**3]"));
  EXPECT_EQ(3 * 8, counter.get_parameter(1));
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
