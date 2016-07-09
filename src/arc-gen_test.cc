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

#include "arc-gen.h"

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

struct TestArcAccumulator {
  TestArcAccumulator() : total_len(0) {}
  AxesRegister last;
  float total_len;
};
static void TestArcCallback(void *user_data, const AxesRegister& pos) {
  TestArcAccumulator *acc = (TestArcAccumulator*) user_data;
  acc->total_len += hypotf(pos[AXIS_X] - acc->last[AXIS_X],
                           pos[AXIS_Y] - acc->last[AXIS_Y]);
  acc->last = pos;
}

static void testHalfTurnAnyStartPosition(bool clockwise) {
  // We start anywhere and do a half turn, testing that we're generating
  // exactly an arc of the specified length.
  for (float start_angle = 0; start_angle < 2*M_PI; start_angle+=TEST_ARC_STEP) {
    TestArcAccumulator collect;
    AxesRegister start, offset, target;

    // Generating an arc starting at (1,0) ccw (-1,0) - so half a turn.
    
    start[AXIS_X] = cosf(start_angle);  // Where we start
    start[AXIS_Y] = sinf(start_angle);

    offset[AXIS_X] = -start[AXIS_X]; // Center as seen from start (I, J)
    offset[AXIS_Y] = -start[AXIS_Y];

    target[AXIS_X] = cosf(start_angle + M_PI);
    target[AXIS_Y] = sinf(start_angle + M_PI);

    collect.last = start;
    arc_gen(AXIS_Z, clockwise, &start, offset, target,
            &TestArcCallback, &collect);

    // We did a half-turn, so we expect the total length to be roughly PI
    EXPECT_NEAR(collect.total_len, M_PI, 0.002);
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
    TestArcAccumulator collect;
    AxesRegister start, offset, target;

    start[AXIS_X] = 1.0;
    start[AXIS_Y] = 0.0;

    offset[AXIS_X] = -1.0;
    offset[AXIS_Y] = 0;

    target[AXIS_X] = cos(turn_angle);
    target[AXIS_Y] = sin(turn_angle);

    collect.last = start;
    arc_gen(AXIS_Z, clockwise, &start, offset, target,
            &TestArcCallback, &collect);

    const float expected_len = clockwise ? 2*M_PI-turn_angle : turn_angle;
    EXPECT_NEAR(collect.total_len, expected_len, 0.003);
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
  TestArcAccumulator collect;
  AxesRegister start, offset, target;

  start[AXIS_X] = 1.0;
  start[AXIS_Y] = 0.0;

  offset[AXIS_X] = -1.0;
  offset[AXIS_Y] = 0;

  target[AXIS_X] = 1.0;
  target[AXIS_Y] = 0.0;

  collect.last = start;
  arc_gen(AXIS_Z, clockwise, &start, offset, target,
          &TestArcCallback, &collect);

  EXPECT_NEAR(collect.total_len, 2*M_PI, 0.003);
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
