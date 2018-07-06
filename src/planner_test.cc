/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 */
#include "planner.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#include <gtest/gtest.h>

#include "gcode-parser/gcode-parser.h"
#include "common/logging.h"

#include "gcode-machine-control.h"
#include "hardware-mapping.h"
#include "motor-operations.h"

// Using different steps/mm speeds results in problems right now.
// TODO: This should work being set to 1
#define SPEED_STEP_FACTOR 4

// Set up config that they are the same for all the tests.
static void InitTestConfig(struct MachineControlConfig *c) {
  float steps_per_mm = 1000;
  for (int i = 0; i <= AXIS_Z; ++i) {
    const GCodeParserAxis axis = (GCodeParserAxis) i;
    c->steps_per_mm[axis] = steps_per_mm;
    // We do different steps/mm to detect problems when going between
    // euclidian space and step-space.
    steps_per_mm *= SPEED_STEP_FACTOR;
    c->acceleration[axis] = 100;  // mm/s^2
    c->max_feedrate[axis] = 10000;
  }
  c->threshold_angle = 0;
  c->speed_tune_angle = 0;
  c->require_homing = false;
}

class FakeMotorOperations : public MotorOperations {
public:
  FakeMotorOperations(const MachineControlConfig &config)
    : config_(config) {}

  void Enqueue(const LinearSegmentSteps &segment) final {
#if 0
    // Prepare for printing.

    // Let's convert the velocities into Euclidian space again.
    LinearSegmentSteps euclidian_speeds = segment;
    const float hypotenuse = FixEuclidSpeed(&euclidian_speeds);
    // We recognize what we call a 'rounding_glitch' for segments that
    // are very short (a few steps). These are move steps from the planner
    // that are created due to rounding of the accel/decel steps. They
    // should have speeds equal to the last v0.
    bool rounding_glitch = false;
    if (abs(segment.steps[AXIS_X]) < 2 &&
        abs(segment.steps[AXIS_Y]) < 2 &&
        abs(segment.steps[AXIS_Z]) < 2 &&
        segment.v0 == segment.v1)
      rounding_glitch = true;

    // Somewhat verbose, but useful :).
    // The values are calculated back back into actual distances from steps.
    // Speeds are in steps/sec to avoid displaying incorrect speeds for the
    // 'rounding_glitch'.
    fprintf(stderr, "  Receiving: (%6.1f, %6.1f, %6.1f); Euclid space: "
            "len: %5.1f ; v: %8.1f -> %8.1f %s %s",
            euclidian_speeds.steps[AXIS_X] / config_.steps_per_mm[AXIS_X],
            euclidian_speeds.steps[AXIS_Y] / config_.steps_per_mm[AXIS_Y],
            euclidian_speeds.steps[AXIS_Z] / config_.steps_per_mm[AXIS_Z],
            hypotenuse,
            segment.v0, segment.v1,
            (segment.v0 < segment.v1) ? "accel" :
            (segment.v0 > segment.v1) ? "decel" : "move",
            rounding_glitch ? "rounding GLITCH!" : "");
    if (!rounding_glitch)
      fprintf(stderr, " ; v: %7.1f -> %7.1f",
              euclidian_speeds.v0, euclidian_speeds.v1);
    fprintf(stderr, "\n");
#endif

    collected_.push_back(segment);
  }

  void MotorEnable(bool on) final {}
  void WaitQueueEmpty() final {}
  bool GetPhysicalStatus(PhysicalStatus *status) final { return false; }
  void SetExternalPosition(int axis, int steps) final {}

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
    GCodeParserAxis defining_axis = AXIS_X;
    for (int i = AXIS_Y; i < AXIS_Z; ++i) {
      const GCodeParserAxis a = (GCodeParserAxis) i;
      if (abs(seg->steps[a]) > abs(seg->steps[defining_axis]))
        defining_axis = a;
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
                 float speed_tune_angle = 0,
                 MachineControlConfig *config = NULL)
    : config_(config ? config : new MachineControlConfig()),
      motor_ops_(*config_), finished_(false) {
    if (!config) {
      InitTestConfig(config_);
      config_->threshold_angle = threshold_angle;
      config_->speed_tune_angle = speed_tune_angle;
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
  ASSERT_GT((int)segments.size(), 1) << "Expected more than one segment";

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
  EXPECT_EQ(2, (int)plantest.segments().size());

  VerifyCommonExpectations(plantest.segments());
}

TEST(PlannerTest, SimpleMove_ReachesFullSpeed) {
  PlannerHarness plantest;

  AxesRegister pos;
  pos[AXIS_X] = 100;
  pos[AXIS_Y] = 100;
  plantest.Enqueue(pos, 10);

  // We expect three segments: accelerating, plateau and decelerating.
  EXPECT_EQ(3, (int)plantest.segments().size());

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

// We set up two axis, one that can be very fast and one that is much slower.
// Also, each of these can be a defining axis.
//
// When we do a diagnoal move we expect that the overall speed is limited so
// that the slow axis is maxed out, i.e. we expect that the slow axis reaches
// its clamped speed.
static void parametrizedAxisClamping(GCodeParserAxis defining_axis,
                                     GCodeParserAxis slowAxis) {
  const float kClampFeedrate = 10.0;
  const float kFastFactor = 17;   // Faster axis is faster by this much.
  MachineControlConfig *config = new MachineControlConfig();
  InitTestConfig(config);
  config->max_feedrate[AXIS_X] = (AXIS_X == slowAxis)
    ? kClampFeedrate : kClampFeedrate * kFastFactor;
  config->max_feedrate[AXIS_Y] = (AXIS_Y == slowAxis)
    ? kClampFeedrate : kClampFeedrate * kFastFactor;

  // We want to force one of these to be the defining axis. Since both will
  // travel the same distance, we can achieve that by having one of the axis
  // reuire more steps/mm.
  config->steps_per_mm[AXIS_X] = 1000;
  config->steps_per_mm[AXIS_Y] = 1000;
  config->steps_per_mm[defining_axis] *= 12.345;

  PlannerHarness plantest(0, 0, config);

  // Let's do a diagonal move.
  // First: AXIS_X shall be dominant
  AxesRegister pos;
  pos[AXIS_X] = 1000;
  pos[AXIS_Y] = 3000;
  // We request an extremely high speed, but expect it to be clamped to whatever
  // that axis can do.
  plantest.Enqueue(pos, 100000);

  EXPECT_EQ(3, (int)plantest.segments().size());  // accel - move - decel

  // We accelerate and decelerate, the middle section is constant speed.
  const LinearSegmentSteps &constant_speed_section = plantest.segments()[1];
  EXPECT_EQ(constant_speed_section.v0, constant_speed_section.v1);

  // Get the step speed of the axis we're interested in.
  float step_speed_of_interest = constant_speed_section.v0
    * constant_speed_section.steps[slowAxis]
    / constant_speed_section.steps[defining_axis];
  fprintf(stderr, "Defining axis: %c speed: defining %.1f ; "
          "slow axis %c speed %.1f\n",
          gcodep_axis2letter(defining_axis), constant_speed_section.v0,
          gcodep_axis2letter(slowAxis), step_speed_of_interest);

  // We have reached the maximum speed we can do. This is, because one of
  // our axes reached its maximum speed it can do. So we expect that axis
  // (the slow axis) to go at exactly the speed it is to be clamped to.
  EXPECT_FLOAT_EQ(kClampFeedrate * config->steps_per_mm[slowAxis],
                  step_speed_of_interest);
}

// We test all combinations of defining axis and which shall be the slow axis.
TEST(PlannerTest, SimpleMove_AxisSpeedLimitClampsOverallSpeed_XX) {
  parametrizedAxisClamping(AXIS_X, AXIS_X);
}

TEST(PlannerTest, SimpleMove_AxisSpeedLimitClampsOverallSpeed_XY) {
  parametrizedAxisClamping(AXIS_X, AXIS_Y);
}

TEST(PlannerTest, SimpleMove_AxisSpeedLimitClampsOverallSpeed_YX) {
  parametrizedAxisClamping(AXIS_Y, AXIS_X);
}

TEST(PlannerTest, SimpleMove_AxisSpeedLimitClampsOverallSpeed_YY) {
  parametrizedAxisClamping(AXIS_Y, AXIS_Y);
}

static std::vector<LinearSegmentSteps> DoAngleMove(float threshold_angle,
                                                   float speed_tune_angle,
                                                   float start_angle,
                                                   float delta_angle) {
  const float kFeedrate = 3000.0f;  // Never reached. We go from accel to decel.
#if 0
  fprintf(stderr, "DoAngleMove(%.1f, %.1f, %.1f)\n",
          threshold_angle, start_angle, delta_angle);
#endif
  PlannerHarness plantest(threshold_angle, speed_tune_angle);
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
  const float kSpeedTuneAngle = 0.0f;
  std::vector<LinearSegmentSteps> segments =
    DoAngleMove(kThresholdAngle, kSpeedTuneAngle, 0, 90);
  ASSERT_EQ(4, (int)segments.size());

  // This is a 90 degree move, we expect to slow down all the way to zero
  // in the elbow.
  EXPECT_EQ(0, segments[1].v1);
  EXPECT_EQ(segments[1].v1, segments[2].v0);
}

void testShallowAngleAllStartingPoints(float threshold, float testing_angle) {
  const float kSpeedTuneAngle = 0.0f;
  // Essentially, we go around the circle as starting segments.
  for (float angle = 0; angle < 360; angle += threshold/2) {
    std::vector<LinearSegmentSteps> segments =
      DoAngleMove(threshold, kSpeedTuneAngle, angle, testing_angle);

    // Depending on the two move angles we expect 2 to 4 segments.
    // 2 segments (first move euclid speed is faster than the second)
    //   1- accel of the first move to the angle
    //   2- decel of the second move
    //      Note that the planner will output a GLITCH about the second move
    //      being too short for full decel.
    // 3 segments (first move euclid speed is slower than the second)
    //   1- accel of the first move to the angle
    //   2- accel of the second move
    //   3- decel of the second move
    // 4 segments (first move euclid speed is slower than the second)
    //   1- accel of the first move to the angle
    //   2- accel of the second move
    //   3- a 1 step move segment due to rounding of the accel/decel
    //   4- decel of the second move
    ASSERT_GE((int)segments.size(), 2);

    // A shallow move just plows through the middle, so we expect all the
    // joint speeds to be larger than zero.
    for (size_t i = 0; i < segments.size(); ++i) {
      if (i > 0) {
        EXPECT_GT(segments[i].v0, 0) << "At angle " << angle;
      }
      if (i < segments.size()-1) {
        EXPECT_GT(segments[i].v1, 0) << "At angle " << angle;
      }
    }
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
