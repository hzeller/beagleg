/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 */
#include "planner.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <gtest/gtest.h>

#include "gcode-parser.h"
#include "hardware-mapping.h"
#include "logging.h"
#include "motor-operations.h"
#include "gcode-machine-control.h"

// We have a bug in which we generate extremely short segments that
// are very fast.
// TODO: This test should work with this set to 0 (and then
// the ifdef's removed).
#define EXPECT_SHORT_SEGMENT_GLITCH 1

// Using different steps/mm speeds results in problems right now.
// TODO: This should work being set to 1
#define USE_DIFFERENT_STEP_SPEEDS 0

// How much we allow adjacent segments to differ in euclidian speed.
// This means essentially, we allow 8% joining-speed mismatch.
// TODO: this is too high! This should probably be more in the 0.001 range.
#define SPEED_COMPARISON_EQUAL_FUDGE_FRACTION 0.08

// Set up config that they are the same for all the tests.
static void InitTestConfig(struct MachineControlConfig *c) {
  float steps_per_mm = 1000;
  for (int i = 0; i <= AXIS_Z; ++i) {
    c->steps_per_mm[i] = steps_per_mm;
#if USE_DIFFERENT_STEP_SPEEDS
    // We do different steps/mm to detect problems when going between
    // euclidian space and step-space.
    steps_per_mm *= 4;
#endif
    c->acceleration[i] = 100;  // mm/s^2
    c->max_feedrate[i] = 10000;
  }
  c->threshold_angle = 0;
  c->require_homing = false;
}

class FakeMotorOperations : public MotorOperations {
public:
  FakeMotorOperations(const MachineControlConfig &config)
    : config_(config) {}

  virtual void Enqueue(const LinearSegmentSteps &segment_in) {
    LinearSegmentSteps segment = segment_in;
    // Let's convert the velocities into Euclidian space again.
    const float hypotenuse = FixEuclidSpeed(&segment);
#if 1
    // Somewhat verbose, but useful :). All these values are calculated
    // back into actual distances and speeds from steps.
    fprintf(stderr, "  Receiving: (%6.1f, %6.1f, %6.1f); Euclid space: "
            "len: %5.1f ; v: %7.1f -> %7.1f\n",
            segment.steps[AXIS_X] / config_.steps_per_mm[AXIS_X],
            segment.steps[AXIS_Y] / config_.steps_per_mm[AXIS_Y],
            segment.steps[AXIS_Z] / config_.steps_per_mm[AXIS_Z],
            hypotenuse,
            segment.v0, segment.v1);
#endif

    // We recognize what we call 'glitch' at segments that are very
    // short (a few steps), and have some very high speeds
    if (abs(segment.steps[AXIS_X]) < 2 &&
        abs(segment.steps[AXIS_Y]) < 2 &&
        abs(segment.steps[AXIS_Z]) < 2) {
      fprintf(stderr, "^^^^^^^^ steps:(%d, %d, %d): GLITCH!\n",
              segment.steps[AXIS_X], segment.steps[AXIS_Y],
              segment.steps[AXIS_Z]);
#if EXPECT_SHORT_SEGMENT_GLITCH
      // When we expect it, we ignore it by not adding to collected segments.
      return;
#endif
    }

    collected_.push_back(segment);
  }

  virtual void MotorEnable(bool on)  {}
  virtual void WaitQueueEmpty() {}

  const std::vector<LinearSegmentSteps> &segments() { return collected_; }

private:
  // Convert speeds in segments back to speed in euklidian space to have
  // something useful to relate to.
  // Out of convenience, return back the length of the segment in euclid space
  float FixEuclidSpeed(LinearSegmentSteps *seg) {
    // Real world coordinates
    const float dx = seg->steps[AXIS_X] / config_.steps_per_mm[AXIS_X];
    const float dy = seg->steps[AXIS_Y] / config_.steps_per_mm[AXIS_Y];
    const float dz = seg->steps[AXIS_Z] / config_.steps_per_mm[AXIS_Z];
    const float hypotenuse = sqrtf(dx*dx + dy*dy + dz*dz);
    int defining_axis = AXIS_X;
    for (int i = AXIS_Y; i < AXIS_Z; ++i) {
      if (abs(seg->steps[i]) > abs(seg->steps[defining_axis]))
        defining_axis = i;
    }

    // The hypotenuse is faster than each of the sides, so its speed is
    // proportionally larger.
    const float defining_side_len
      = fabsf(seg->steps[defining_axis] / config_.steps_per_mm[defining_axis]);
    const float correction = hypotenuse / defining_side_len;

    // Go from steps/sec back to to mm/sec and apply correction to be back in
    // euclid space.
    seg->v0 = (seg->v0 / config_.steps_per_mm[defining_axis]) * correction;
    seg->v1 = (seg->v1 / config_.steps_per_mm[defining_axis]) * correction;

    return hypotenuse;
  }

  const MachineControlConfig &config_;
  // We keep this public for easier testing
  std::vector<LinearSegmentSteps> collected_;
};

class PlannerHarness {
public:
  PlannerHarness(float threshold_angle = 0)
    : motor_ops_(config_), finished_(false) {
    InitTestConfig(&config_);
    config_.threshold_angle = threshold_angle;
    simulated_hardware_.AddMotorMapping(AXIS_X, 1, false);
    simulated_hardware_.AddMotorMapping(AXIS_Y, 2, false);
    simulated_hardware_.AddMotorMapping(AXIS_Z, 3, false);
    planner_ = new Planner(&config_, &simulated_hardware_, &motor_ops_);
  }
  ~PlannerHarness() { delete planner_; }

  void Enqueue(const AxesRegister &target, float feed) {
    assert(!finished_);   // Can only call if segments() has not been called.
    //fprintf(stderr, "NewPos: (%.1f, %.1f)\n", target[AXIS_X], target[AXIS_Y]);
    planner_->Enqueue(target, feed);
  }

  const std::vector<LinearSegmentSteps> &segments() {
    if (!finished_) {
      planner_->BringPathToHalt();
      finished_ = true;
    }
    return motor_ops_.segments();
  }

private:
  MachineControlConfig config_;
  FakeMotorOperations motor_ops_;
  HardwareMapping simulated_hardware_;
  bool finished_;
  Planner *planner_;
};

// Conditions that we expect in all moves.
static void VerifyCommonExpectations(
      const std::vector<LinearSegmentSteps> &segments) {
  ASSERT_GT(segments.size(), 1) << "Expected more than one segment";

  // Some basic assumption: something is moving.
  EXPECT_GT(segments[0].v1, 0);

  // At the begin and end of our travel, we have zero speed.
  EXPECT_EQ(0, segments[0].v0);
  EXPECT_EQ(0, segments[segments.size()-1].v1);

  // The joining speeds between segments match.
  for (size_t i = 0; i < segments.size()-1; ++i) {
    // Let's determine the speed in euclidian space.
#if 1
    EXPECT_NEAR(segments[i].v1, segments[i+1].v0,
                segments[i].v1 * SPEED_COMPARISON_EQUAL_FUDGE_FRACTION)
      << "Joining speed between " << i << " and " << (i+1);
#endif
  }
}

TEST(PlannerTest, SimpleMove_NeverReachingFullSpeed) {
  PlannerHarness plantest;

  AxesRegister pos;
  pos[AXIS_X] = 100;
  pos[AXIS_Y] = 100;
  plantest.Enqueue(pos, 1000);  // never reach speed in short move

  // We expect two segments: accelerating to maximum speed we can
  // reach, then decelerating.
  EXPECT_EQ(2, plantest.segments().size());

  VerifyCommonExpectations(plantest.segments());
}

TEST(PlannerTest, SimpleMove_ReachesFullSpeed) {
  PlannerHarness plantest;

  AxesRegister pos;
  pos[AXIS_X] = 100;
  pos[AXIS_Y] = 100;
  plantest.Enqueue(pos, 10);

  // We expect three segments: accelerating, plateau and decelerating.
  EXPECT_EQ(3, plantest.segments().size());

  VerifyCommonExpectations(plantest.segments());
}

static std::vector<LinearSegmentSteps> DoAngleMove(float threshold_angle,
                                                   float start_angle,
                                                   float delta_angle) {
  const float kFeedrate = 3000.0f;  // Never reached. We go from accel to decel.
  fprintf(stderr, "DoAngleMove(%.1f, %.1f, %.1f)\n",
          threshold_angle, start_angle, delta_angle);
  PlannerHarness plantest(threshold_angle);
  const float kSegmentLen = 100;

  float radangle = 2 * M_PI * start_angle / 360;
  AxesRegister pos;
  pos[AXIS_X] = kSegmentLen * cos(radangle) + pos[AXIS_X];
  pos[AXIS_Y] = kSegmentLen * sin(radangle) + pos[AXIS_Y];
  plantest.Enqueue(pos, kFeedrate);

  radangle += 2 * M_PI * delta_angle / 360;
  pos[AXIS_X] = kSegmentLen * cos(radangle) + pos[AXIS_X];
  pos[AXIS_Y] = kSegmentLen * sin(radangle) + pos[AXIS_Y];
  plantest.Enqueue(pos, kFeedrate);
  std::vector<LinearSegmentSteps> segments = plantest.segments();
  VerifyCommonExpectations(segments);
  return segments;
}

TEST(PlannerTest, CornerMove_90Degrees) {
  const float kThresholdAngle = 5.0f;
  std::vector<LinearSegmentSteps> segments = DoAngleMove(kThresholdAngle, 0, 90);
  ASSERT_EQ(4, segments.size());

  // This is a 90 degree move, we expect to slow down all the way to zero
  // in the elbow.
  EXPECT_EQ(0, segments[1].v1);
  EXPECT_EQ(segments[1].v1, segments[2].v0);
}

void testShallowAngleAllStartingPoints(float threshold, float testing_angle) {
  // Essentially, we go around the circle as starting segments.
  for (float angle = 0; angle < 360; angle += threshold/2) {
    std::vector<LinearSegmentSteps> segments =
      DoAngleMove(threshold, angle, testing_angle);
#if EXPECT_SHORT_SEGMENT_GLITCH
    // With the short segment glitch, we sometimes get three segments.
    ASSERT_GE(segments.size(), 2);
#else
    // We essentially expect two segments, plowing through in the middle
    // with full speed
    ASSERT_EQ(2, segments.size()) << "For start-angle " << angle;
#endif

    // A shallow move just plows through the middle, so we expect a larger
    // speed than zero
    EXPECT_GT(segments[0].v1, 0);

    // Still, the join-speed at the elbow is the same.
    EXPECT_NEAR(segments[0].v1, segments[1].v0,
                segments[0].v1 * SPEED_COMPARISON_EQUAL_FUDGE_FRACTION)
      << "At angle " << angle;
  }
}

TEST(PlannerTest, CornerMove_Shallow_PositiveAngle) {
  const float kThresholdAngle = 5.0f;
  const float kTestingAngle = 0.7 * kThresholdAngle;
  testShallowAngleAllStartingPoints(kThresholdAngle, kTestingAngle);
}

TEST(PlannerTest, CornerMove_Shallow_NegativeAngle) {
  const float kThresholdAngle = 5.0f;
  const float kTestingAngle = -0.7 * kThresholdAngle;
  testShallowAngleAllStartingPoints(kThresholdAngle, kTestingAngle);
}

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
