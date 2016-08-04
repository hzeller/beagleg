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
#define USE_DIFFERENT_STEP_SPEEDS 1

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

GCodeParserAxis GetDefiningAxis(const LinearSegmentSteps &seg) {
  GCodeParserAxis defining = AXIS_X;
  if (abs(seg.steps[AXIS_Y]) > abs(seg.steps[defining])) defining = AXIS_Y;
  if (abs(seg.steps[AXIS_Z]) > abs(seg.steps[defining])) defining = AXIS_Z;
  return defining;
}

class FakeMotorOperations : public MotorOperations {
public:
  FakeMotorOperations(const MachineControlConfig &config)
    : config_(config) {}

  virtual void Enqueue(const LinearSegmentSteps &segment) {
    // Let's convert the velocities into Euclidian space again.
    LinearSegmentSteps euclidian_speeds = segment;
    const float hypotenuse = FixEuclidSpeed(&euclidian_speeds);
#if 1
    // Somewhat verbose, but useful :). All these values are calculated
    // back into actual distances and speeds from steps.
    fprintf(stderr, "  Receiving: (%6.1f, %6.1f, %6.1f); Euclid space: "
            "len: %5.1f ; v: %7.1f -> %7.1f\n",
            euclidian_speeds.steps[AXIS_X] / config_.steps_per_mm[AXIS_X],
            euclidian_speeds.steps[AXIS_Y] / config_.steps_per_mm[AXIS_Y],
            euclidian_speeds.steps[AXIS_Z] / config_.steps_per_mm[AXIS_Z],
            hypotenuse,
            euclidian_speeds.v0, euclidian_speeds.v1);
#endif

    // We recognize what we call 'glitch' at segments that are very
    // short (a few steps), and have some very high speeds
    if (abs(euclidian_speeds.steps[AXIS_X]) < 2 &&
        abs(euclidian_speeds.steps[AXIS_Y]) < 2 &&
        abs(euclidian_speeds.steps[AXIS_Z]) < 2) {
      fprintf(stderr, "^^^^^^^^ steps:(%d, %d, %d): GLITCH!\n",
              euclidian_speeds.steps[AXIS_X], euclidian_speeds.steps[AXIS_Y],
              euclidian_speeds.steps[AXIS_Z]);
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
  // If angle or config is not set, assumes default. Takes ownership of
  // config.
  PlannerHarness(float threshold_angle = 0,
                 MachineControlConfig *config = NULL)
    : config_(config ? config : new MachineControlConfig()),
      motor_ops_(*config_), finished_(false) {
    if (!config) {
      InitTestConfig(config_);
      config_->threshold_angle = threshold_angle;
    }
    simulated_hardware_.AddMotorMapping(AXIS_X, 1, false);
    simulated_hardware_.AddMotorMapping(AXIS_Y, 2, false);
    simulated_hardware_.AddMotorMapping(AXIS_Z, 3, false);
    planner_ = new Planner(config_, &simulated_hardware_, &motor_ops_);
  }
  ~PlannerHarness() {
    delete planner_;
    delete config_;
  }

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
  MachineControlConfig *config_;
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
    EXPECT_EQ(segments[i].v1, segments[i+1].v0)
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

// When we move axes, they should try to reach the speed the user requested
// unless there is maximum speed an axis can do (very typical in CNC machines
// in which the Z axis is much slower than X or Y).
//
// When we do a diagonal move and request a super-high speed then we will be
// limited by the max feedrates on each of these axes: The slowest axis
// determines how fast we overall can go: once the slowest axis reaches its
// configured maximum speed _in its direction_, we can't go any faster.
//
// Say our X-axis is limited to 1000mm/s and the Z axis to 10mm/s. If we
// do a diagonal 1000mm move in X direction and 20mm in Z direction, then
// we effectively limit the X direction to 500mm/s because the Z axis uses
// its sweet time.
//
// This is what we're testing here. However it seems like we are limiting the
// overall speed to whatever the defining axis is. Which in our case would be:
// the whole 1000mm segment would move with 10mm/s if Z is the dominant axis
// (which they can - they often have many more steps/mm).
TEST(PlannerTest, SimpleMove_AxisSpeedLimitClampsOverallSpeed) {
  const float kClampFeedrate = 10.0;
  const float kFastFactor = 17;   // Faster axis is faster by this much.
  GCodeParserAxis kSlowAxis = AXIS_X;  // most interesting if it is not defining.
  MachineControlConfig *config = new MachineControlConfig();
  InitTestConfig(config);
  config->max_feedrate[AXIS_X] = (AXIS_X == kSlowAxis)
    ? kClampFeedrate : kClampFeedrate * kFastFactor;
  config->max_feedrate[AXIS_Y] = (AXIS_Y == kSlowAxis)
    ? kClampFeedrate : kClampFeedrate * kFastFactor;
  PlannerHarness plantest(0, config);

  // Let's do a diagonal move.
  // First: AXIS_X shall be dominant
  AxesRegister pos;
  pos[AXIS_X] = 1000;
  pos[AXIS_Y] = 1000;
  // We request an extremely high speed, but expect it to be clamped to whatever
  // that axis can do.
  plantest.Enqueue(pos, 100000);

  EXPECT_EQ(3, plantest.segments().size());  // accel - move - decel

  // We accelerate and decelerate, the middle section is constant speed.
  const LinearSegmentSteps &constant_speed_section = plantest.segments()[1];
  EXPECT_EQ(constant_speed_section.v0, constant_speed_section.v1);

  // The axis with the slowest allowed feedrate should now about max out
  // its declared speed.
  GCodeParserAxis defining = GetDefiningAxis(constant_speed_section);
  // Get the step speed of the axis we're interested in.
  float step_speed_of_interest = constant_speed_section.v0
    * constant_speed_section.steps[kSlowAxis]
    / constant_speed_section.steps[defining];
  fprintf(stderr, "Defining axis: %c speed: defining %.1f ; "
          "slow axis %c speed %.1f\n",
          gcodep_axis2letter(defining), constant_speed_section.v0,
          gcodep_axis2letter(kSlowAxis), step_speed_of_interest);
  // This is failing. Instead of limiting the speed to the maximum the axis can
  // do, it looks like we clamp the euklidian speed to what the defining
  // axis can do.
  //
  // TODO: this is failing right now. It seems the speed is clamped, but we
  // see that it is kFastFactor faster clamped. That would indicate that the
  // clamping is applied to whatever the defining axis does.
  EXPECT_FLOAT_EQ(kClampFeedrate * config->steps_per_mm[kSlowAxis],
                  step_speed_of_interest);
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
    EXPECT_EQ(segments[0].v1, segments[1].v0)
      << "At angle " << angle;
  }
}

// these tests don't work currently.
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
