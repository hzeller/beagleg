/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2016 Henner Zeller <h.zeller@acm.org>
 *
 * This file is part of BeagleG. http://github.com/hzeller/beagleg
 *
 * BeagleG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BeagleG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "gcode-parser.h"

#include <math.h>
#include <iostream>
#include <gtest/gtest.h>

// Going around the circle for start-points with this step.
#define TEST_ARC_STEP 2*M_PI/255

// The area around zero angle, we look at with higher scrutiny as
// this is where a circle can flip/flop between tiny and full depending
// on some rounded bits somewhere.
// So for this, we test a small HIRES_TEST_CIRCLE_SEGMENT around zero with
// the same number of steps we otherwise would test a full circle.
#define HIRES_TEST_CIRCLE_SEGMENT (2*M_PI/1e12)
#define HIRES_TEST_ARC_STEP (TEST_ARC_STEP/1e12)

class TestArcAccumulator : public GCodeParser::EventReceiver {
public:
  TestArcAccumulator(const AxesRegister &start) : last_(start), total_len_(0) {}

  void gcode_start(GCodeParser *parser) final {}
  void go_home(AxisBitmap_t axis_bitmap) final {}
  void set_speed_factor(float factor) final {}
  void set_fanspeed(float value) final {}
  void set_temperature(float degrees_c) final {}
  void wait_temperature() final {}
  void dwell(float time_ms) final {}
  void motors_enable(bool enable) final {}
  bool coordinated_move(float feed_mm_p_sec, const AxesRegister &pos) final {
    total_len_ += hypotf(pos[AXIS_X] - last_[AXIS_X],
                         pos[AXIS_Y] - last_[AXIS_Y]);
    last_ = pos;
    return true;
  }

  float total_len() const { return total_len_; }

  bool rapid_move(float feed_mm_p_sec,
                  const AxesRegister &absolute_pos) final { return true; }
  const char *unprocessed(char letter, float value,
                                  const char *rest_of_line) final {
    return nullptr;
  }
private:
  AxesRegister last_;
  float total_len_;
};

static void testHalfTurnAnyStartPosition(bool clockwise) {
  // We start anywhere and do a half turn, testing that we're generating
  // exactly an arc of the specified length.
  for (float start_angle = 0; start_angle < 2*M_PI; start_angle+=TEST_ARC_STEP) {
    AxesRegister start, center, target;

    // Generating an arc starting at (1,0) ccw (-1,0) - so half a turn.

    start[AXIS_X] = cosf(start_angle);  // Where we start
    start[AXIS_Y] = sinf(start_angle);

    center[AXIS_X] = 0;
    center[AXIS_Y] = 0;

    target[AXIS_X] = cosf(start_angle + M_PI);
    target[AXIS_Y] = sinf(start_angle + M_PI);

    TestArcAccumulator collect(start);
    collect.arc_move(100, AXIS_Z, clockwise, start, center, target);

    // We did a half-turn, so we expect the total length to be roughly PI
    EXPECT_NEAR(collect.total_len(), M_PI, 0.002);
  }
}

TEST(ArcGenerator, HalfTurnAnyStartPosition_CW) {
  testHalfTurnAnyStartPosition(true);
}

TEST(ArcGenerator, HalfTurnAnyStartPosition_CCW) {
  testHalfTurnAnyStartPosition(false);
}

// Starting from a position, generate arcs of varying length and determine
// if they have the length as expected.
static void testArcLength(bool clockwise, double start, double step, double end) {
  // Varying arc-length.
  for (double turn_angle = start; turn_angle <= end-step; turn_angle+=step) {
    AxesRegister start, center, target;

    start[AXIS_X] = 1.0;
    start[AXIS_Y] = 0.0;

    center[AXIS_X] = 0;
    center[AXIS_Y] = 0;

    target[AXIS_X] = cos(turn_angle);
    target[AXIS_Y] = sin(turn_angle);

    TestArcAccumulator collect(start);
    collect.arc_move(100, AXIS_Z, clockwise, start, center, target);

    const float expected_len = clockwise ? 2*M_PI-turn_angle : turn_angle;
    EXPECT_NEAR(collect.total_len(), expected_len, 0.003);
  }
}

TEST(ArcGenerator, ArcLength_CW_360) {
  testArcLength(true, TEST_ARC_STEP, TEST_ARC_STEP, 2*M_PI);
}

TEST(ArcGenerator, ArcLength_CW_HiRes_StartCircle) {
  testArcLength(true,
                2*M_PI - HIRES_TEST_CIRCLE_SEGMENT,
                HIRES_TEST_ARC_STEP,
                2*M_PI);
}

TEST(ArcGenerator, ArcLength_CW_HiRes_FullCircle) {
  testArcLength(true,
                HIRES_TEST_ARC_STEP,
                HIRES_TEST_ARC_STEP,
                HIRES_TEST_CIRCLE_SEGMENT);
}

TEST(ArcGenerator, ArcLength_CCW_360) {
  testArcLength(false, TEST_ARC_STEP, TEST_ARC_STEP, 2*M_PI);
}

TEST(ArcGenerator, ArcLength_CCW_HiRes_StartCircle) {
  testArcLength(false,
                HIRES_TEST_ARC_STEP,
                HIRES_TEST_ARC_STEP,
                HIRES_TEST_CIRCLE_SEGMENT);
}

TEST(ArcGenerator, ArcLength_CCW_HiRes_FullCircle) {
  testArcLength(false,
                2*M_PI - HIRES_TEST_CIRCLE_SEGMENT,
                HIRES_TEST_ARC_STEP,
                2*M_PI);
}

// Test special case: if start/end position are the same, then we expect
// a full turn.
static void testFullTurn(bool clockwise) {
  AxesRegister start, center, target;

  start[AXIS_X] = 1.0;
  start[AXIS_Y] = 0.0;

  center[AXIS_X] = 0;
  center[AXIS_Y] = 0;

  target[AXIS_X] = 1.0;
  target[AXIS_Y] = 0.0;

  TestArcAccumulator collect(start);
  collect.arc_move(100, AXIS_Z, clockwise, start, center, target);

  EXPECT_NEAR(collect.total_len(), 2*M_PI, 0.003);
}

TEST(ArcGenerator, FullTurn_CW) {
  testFullTurn(true);
}

TEST(ArcGenerator, FullTurn_CCW) {
  testFullTurn(false);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
