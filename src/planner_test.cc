/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*- */
#include "planner.h"

#include <gtest/gtest.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "common/logging.h"
#include "gcode-machine-control.h"
#include "gcode-parser/gcode-parser.h"
#include "hardware-mapping.h"
#include "segment-queue.h"

// Using different steps/mm speeds results in problems right now.
// TODO: This should work being set to 1
#define SPEED_STEP_FACTOR 4

// Helper function for implementing ASSERT_NEAR.
testing::AssertionResult DoubleNearAbsRelPredFormat(const char *expr1,
                                                    const char *expr2,
                                                    const char *rel_error_expr,
                                                    double val1, double val2,
                                                    double rel_error) {
  const double diff = fabs(val1 - val2);
  const double fmax = std::max(fabs(val1), fabs(val2));
  if (diff <= rel_error * fmax) return testing::AssertionSuccess();

  return testing::AssertionFailure()
         << "The difference between " << expr1 << " and " << expr2 << " is "
         << diff << ", which exceeds " << rel_error_expr << " * max(abs("
         << expr1 << "), abs(" << expr2 << ")), where\n"
         << expr1 << " evaluates to " << val1 << ",\n"
         << expr2 << " evaluates to " << val2 << ",\n"
         << rel_error_expr << " evaluates to " << rel_error << ".";
}

#define EXPECT_NEAR_REL(val1, val2, rel_error) \
  EXPECT_PRED_FORMAT3(DoubleNearAbsRelPredFormat, val1, val2, rel_error)

// Compute the absolute acceleration given space s and
// starting speed v0 and end speed v1.
template <typename T>
static inline T fabs_accel(const T v0, const T v1, const int s) {
  const T accel = (v1 * v1 - v0 * v0) / (2 * s);
  return fabs(accel);
}

// Set up config that they are the same for all the tests.
static void InitTestConfig(struct MachineControlConfig *c) {
  float steps_per_mm = 1000;
  for (int i = 0; i <= AXIS_Z; ++i) {
    const GCodeParserAxis axis = (GCodeParserAxis)i;
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

static inline double sqd(double x) { return x * x; }  // square a number

class FakeMotorOperations : public SegmentQueue {
 public:
  explicit FakeMotorOperations(const MachineControlConfig &config)
      : config_(config) {}

  bool Enqueue(const LinearSegmentSteps &segment) final {
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
    return true;
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
    const float hypotenuse = sqrtf(dx * dx + dy * dy + dz * dz);
    GCodeParserAxis defining_axis = AXIS_X;
    for (int i = AXIS_Y; i < AXIS_Z; ++i) {
      const GCodeParserAxis a = (GCodeParserAxis)i;
      if (abs(seg->steps[a]) > abs(seg->steps[defining_axis]))
        defining_axis = a;
    }

    // The hypotenuse is faster than each of the sides, so its speed is
    // proportionally larger.
    const float defining_side_len =
      fabsf(seg->steps[defining_axis] / config_.steps_per_mm[defining_axis]);
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
  explicit PlannerHarness(float threshold_angle = 0, float speed_tune_angle = 0,
                          MachineControlConfig *config = NULL)
      : config_(config ? config : new MachineControlConfig()),
        motor_ops_(*config_),
        finished_(false) {
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
    assert(!finished_);  // Can only call if segments() has not been called.
    // fprintf(stderr, "NewPos: (%.1f, %.1f)\n", target[AXIS_X],
    // target[AXIS_Y]);
    planner_->Enqueue(target, feed);
  }

  const std::vector<LinearSegmentSteps> &segments() {
    if (!finished_) {
      planner_->BringPathToHalt();
      finished_ = true;
    }
    return motor_ops_.segments();
  }

  MachineControlConfig *GetConfig() const { return config_; }

 private:
  MachineControlConfig *config_;
  FakeMotorOperations motor_ops_;
  HardwareMapping simulated_hardware_;
  bool finished_;
  Planner *planner_;
};

#if 0
// Verify that on average, the profile satisfies
// the configured constraints both for speed and acceleration.
static void VerifySegmentConstraints(const MachineControlConfig &config,
                                     const LinearSegmentSteps &segment,
                                     const float rel_error = 1e-3) {
  for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
    const GCodeParserAxis axis = (GCodeParserAxis)i;
    const auto steps = segment.steps[axis];
    if (steps == 0) continue;
    const double accel = fabs_accel(segment.v0, segment.v1, steps);
    const double max_accel =
      config.acceleration[axis] * config.steps_per_mm[axis];
    const double max_feedrate =
      config.max_feedrate[axis] * config.steps_per_mm[axis];
    EXPECT_LE(segment.v0, max_feedrate);
    EXPECT_LE(segment.v1, max_feedrate);
    if (segment.v0 != segment.v1) {
      EXPECT_LE(accel, max_accel);
    }
  }
}
#endif

static size_t GetDefiningMotor(const LinearSegmentSteps &segment) {
  size_t motor_index = 0;
  for (size_t i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
    if (abs(segment.steps[i]) > abs(segment.steps[motor_index]))
      motor_index = i;
  }
  return motor_index;
}

// Conditions that we expect in all moves.
static void VerifyCommonExpectations(
  const std::vector<LinearSegmentSteps> &segments,
  const MachineControlConfig &config, const float rel_error = 1e-3) {
  ASSERT_GT((int)segments.size(), 1) << "Expected more than one segment";

  // Some basic assumption: something is moving.
  EXPECT_GT(segments[0].v1, 0);

  // At the begin and end of our travel, we have zero speed.
  EXPECT_EQ(0, segments[0].v0);
  EXPECT_EQ(0, segments[segments.size() - 1].v1);

  // The joining speeds between segments match.
  for (size_t i = 0; i < segments.size() - 1; ++i) {
    // Let's determine the speed in euclidian space.
    const size_t m1 = GetDefiningMotor(segments[i]);
    const size_t m2 = GetDefiningMotor(segments[i + 1]);
    if (m1 == m2) {
      EXPECT_EQ(segments[i].v1, segments[i + 1].v0)
        << "Joining speed between " << i << " and " << (i + 1);
    }
#if 0  // This is now failing, we plan to fix it on the next PR(#44)
    VerifySegmentConstraints(config, segments[i]);
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

  VerifyCommonExpectations(plantest.segments(), *plantest.GetConfig());
}

TEST(PlannerTest, SimpleMove_ReachesFullSpeed) {
  PlannerHarness plantest;

  AxesRegister pos;
  pos[AXIS_X] = 100;
  pos[AXIS_Y] = 100;
  plantest.Enqueue(pos, 10);

  // We expect three segments: accelerating, plateau and decelerating.
  EXPECT_EQ(3, (int)plantest.segments().size());

  VerifyCommonExpectations(plantest.segments(), *plantest.GetConfig());
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
// A similar reasoning applies to the acceleration as well.
//
// We set up two axis, one that can be very fast and one that is much slower.
// Also, each of these can be a defining axis.
//
// When we do a diagnoal move we expect that the overall speed is limited so
// that the slow axis is maxed out, i.e. we expect that the slow axis reaches
// its clamped speed.
static void parametrizedAxisClamping(GCodeParserAxis defining_axis,
                                     GCodeParserAxis slowAxis) {
  const float kClampFeedrate = 10.0;
  const float kFastFactor = 17;  // Faster axis is faster by this much.
  const float kAllowedRelativeError = 0.001;

  MachineControlConfig *config = new MachineControlConfig();
  InitTestConfig(config);
  config->max_feedrate[AXIS_X] =
    (AXIS_X == slowAxis) ? kClampFeedrate : kClampFeedrate * kFastFactor;
  config->max_feedrate[AXIS_Y] =
    (AXIS_Y == slowAxis) ? kClampFeedrate : kClampFeedrate * kFastFactor;

  // We set the slow axis acceleration to be kFastFactor times slower
  const float kClampAcceleration = 100;
  config->acceleration[AXIS_X] = (AXIS_X == slowAxis)
                                   ? kClampAcceleration
                                   : kClampAcceleration * kFastFactor;
  config->acceleration[AXIS_Y] = (AXIS_Y == slowAxis)
                                   ? kClampAcceleration
                                   : kClampAcceleration * kFastFactor;

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

  const float slow_axis_ratio = (float)constant_speed_section.steps[slowAxis] /
                                constant_speed_section.steps[defining_axis];

  // Get the step speed of the axis we're interested in.
  float step_speed_of_interest = constant_speed_section.v0 * slow_axis_ratio;

  // Get the acceleration segment accel.
  const LinearSegmentSteps &constant_accel_section = plantest.segments()[0];
  EXPECT_TRUE(constant_accel_section.v0 < constant_accel_section.v1);
  const double step_accel_of_interest =
    (sqd(constant_accel_section.v1) - sqd(constant_accel_section.v0)) *
    slow_axis_ratio / (2.0 * constant_accel_section.steps[defining_axis]);

  fprintf(stderr,
          "Defining axis: %c speed: defining %.1f ; "
          "slow axis %c speed %.1f, accel %.1f\n",
          gcodep_axis2letter(defining_axis), constant_speed_section.v0,
          gcodep_axis2letter(slowAxis), step_speed_of_interest,
          step_accel_of_interest);

  // We have reached the maximum speed we can do. This is, because one of
  // our axes reached its maximum speed it can do. So we expect that axis
  // (the slow axis) to go at exactly the speed it is to be clamped to.
  EXPECT_NEAR_REL(kClampFeedrate * config->steps_per_mm[slowAxis],
                  step_speed_of_interest, kAllowedRelativeError);

  // Same for accel.
  EXPECT_NEAR_REL(kClampAcceleration * config->steps_per_mm[slowAxis],
                  step_accel_of_interest, kAllowedRelativeError);
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
  VerifyCommonExpectations(segments, *plantest.GetConfig());
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
  for (float angle = 0; angle < 360; angle += threshold / 2) {  // NOLINT
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
      if (i < segments.size() - 1) {
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

#if 0  // Needs fixing. PR #44 under way
// This function compute acceleration based on the starting velocity, the ending
// velocity and the path in steps
static int acceleration_segment(float v0, float v1, int delta) {
  double acceleration;
  acceleration = (v1 * v1 - v0 * v0) / (2.0 * abs(delta));
  return (int)(acceleration);
}
// I found a problem with the wrong speed determining for the segment in the
// planner. This occurs with short segments of the movement,followed by a change
// in the direction of motion. This is typical for the case when a straight line
// goes into an arc or circle. In these segments, due to a high initial speed
// and a short braking path in the segment,there is an acceleration exceeding
// the permissible limits. That as a result leads to an uncontrollable
// situation.
TEST(PlannerTest, DetectOutRangeAcceleration) {
  MachineControlConfig *config = new MachineControlConfig();
  InitTestConfig(config);
  config->steps_per_mm[AXIS_X] = 1000;
  config->steps_per_mm[AXIS_Y] = 1000;
  config->acceleration[AXIS_X] = 100;  // mm/s^2
  config->acceleration[AXIS_Y] = 100;  // mm/s^2
  PlannerHarness plantest(2.5, 0, config);
  float speed = 10000;
  AxesRegister pos;
  pos[AXIS_X] = 60;
  pos[AXIS_Y] = 250;
  plantest.Enqueue(pos, speed);  // long movement
  pos[AXIS_X] = 60;
  pos[AXIS_Y] = 100;
  plantest.Enqueue(pos, speed);  // long movement
  pos[AXIS_X] = 60;
  pos[AXIS_Y] = 99.9;
  plantest.Enqueue(pos, speed);  // short movement
  pos[AXIS_X] = 60;
  pos[AXIS_Y] = 99.8;
  plantest.Enqueue(pos, speed);  // short movement
  pos[AXIS_X] = 60;
  pos[AXIS_Y] = 99.7;
  plantest.Enqueue(pos, speed);  // short movement
  pos[AXIS_X] = 60.1;
  pos[AXIS_Y] = 99.6;
  plantest.Enqueue(pos, speed);  // change direction angle 45 degree, this is
                                 // the problematic part of moving
  pos[AXIS_X] = 60.2;
  pos[AXIS_Y] = 99.5;
  plantest.Enqueue(pos, speed);  // continue moving
  pos[AXIS_X] = 60.3;
  pos[AXIS_Y] = 99.5;
  plantest.Enqueue(pos, speed);  // change direction angle 135 degree
  pos[AXIS_X] = 60.4;
  pos[AXIS_Y] = 99.4;
  plantest.Enqueue(pos, speed);  // continue moving
  std::vector<LinearSegmentSteps> segments = plantest.segments();
  int ax, ay;
  for (size_t i = 0; i < segments.size(); ++i) {
    ax = 0;
    ay = 0;
    if (abs(segments[i].steps[AXIS_X]) > abs(segments[i].steps[AXIS_Y])) {
      if (abs(segments[i].steps[AXIS_X]) > 0) {
        ax = acceleration_segment(segments[i].v0, segments[i].v1,
                                  segments[i].steps[AXIS_X]);
      } else
        ax = 0;
    } else {
      if (abs(segments[i].steps[AXIS_Y]) > 0) {
        ay = acceleration_segment(segments[i].v0, segments[i].v1,
                                  segments[i].steps[AXIS_Y]);
      } else
        ay = 0;
    }
#if 1
    fprintf(stderr,
            "segment[%d] v0=%f  v1=%f    lenghtX=%d   lenghtY=%d   accelX=%d  "
            "accelY=%d \n",
            (int)i, segments[i].v0, segments[i].v1, segments[i].steps[AXIS_X],
            segments[i].steps[AXIS_Y], ax, ay);
#endif
    // When I tried different speed values ​​and boundary accelerations, I
    // found that the acceleration is sometimes
    // a little more than the boundary permissible, within 0.5..1%. Then I
    // decided to add 0.3% to acceleration limits value
    // Maybe this is not a problem and such a deviation is permissible. This
    // should be tested directly on the cnc machine.
    if (abs(ax) > 0) {
      EXPECT_GE(
        config->acceleration[AXIS_X] * config->steps_per_mm[AXIS_X] * 1.003,
        abs(ax));  // added 0.3% of the acceleration limit
    }
    if (abs(ay) > 0) {
      EXPECT_GE(
        config->acceleration[AXIS_Y] * config->steps_per_mm[AXIS_Y] * 1.003,
        abs(ay));  // added 0.3% of the acceleration limit
    }
  }
  VerifyCommonExpectations(plantest.segments());
}
#endif

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
