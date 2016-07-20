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

#define END_SENTINEL 0x42

// Set up config that they are the same for all the tests.
static void InitTestConfig(struct MachineControlConfig *c) {
  for (int i = 0; i <= AXIS_Z; ++i) {
    // We do different steps/mm to detect miscaclulations of speeds.
    c->steps_per_mm[i] = 100 + i * 10;  // step/mm
    c->acceleration[i] = 1000;  // mm/s^2
    c->max_feedrate[i] = 10000;
  }
  c->threshold_angle = 0;
  c->require_homing = false;
}

class FakeMotorOperations : public MotorOperations {
public:
  virtual void Enqueue(const LinearSegmentSteps &segment) {
    collected_.push_back(segment);
  }

  virtual void MotorEnable(bool on)  {}
  virtual void WaitQueueEmpty() {}

  const std::vector<LinearSegmentSteps> &segments() { return collected_; }

private:
  // We keep this public for easier testing
  std::vector<LinearSegmentSteps> collected_;
};

class PlannerHarness {
public:
  PlannerHarness(float threshold_angle = 0) : finished_(false) {
    InitTestConfig(&config_);
    config_.threshold_angle = threshold_angle;
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
  FakeMotorOperations motor_ops_;
  HardwareMapping simulated_hardware_;
  MachineControlConfig config_;
  bool finished_;
  Planner *planner_;
};

TEST(PlannerTest, SimpleMove) {
  PlannerHarness plantest;

  AxesRegister pos;
  pos[AXIS_X] = 100;
  pos[AXIS_Y] = 100;
  plantest.Enqueue(pos, 100);

  // We expect two segments: accelerating to full speed, then decelerating
  EXPECT_EQ(2, plantest.segments().size());

  // At the begin and end of our travel, we have zero speed.
  EXPECT_EQ(0, plantest.segments()[0].v0);
  EXPECT_EQ(0, plantest.segments()[1].v1);

  // End of the first segment matches speed of begginning of next
  // segment.
  EXPECT_EQ(plantest.segments()[0].v1, plantest.segments()[1].v0);

  // TODO: calculate expected middle speed according to the expected step-moves
  // and compare. For now let's just make sure that it is actually larger than
  /// zero.
  EXPECT_GT(plantest.segments()[0].v1, 0);
}

// TODO(Hartley): also supply start angle.
static std::vector<LinearSegmentSteps> DoAngleMove(float threshold_angle,
                                                   float angle) {
  PlannerHarness plantest(threshold_angle);
  const float kSegmentLen = 100;
  AxesRegister pos;
  pos[AXIS_X] = kSegmentLen; pos[AXIS_Y] = 0;
  plantest.Enqueue(pos, 100);

  const float radangle = 2 * M_PI * angle / 360;
  pos[AXIS_X] = cos(radangle) * kSegmentLen  + pos[AXIS_X];
  pos[AXIS_Y] = sin(radangle) * kSegmentLen  + pos[AXIS_Y];
  plantest.Enqueue(pos, 100);
  return plantest.segments();
}

TEST(PlannerTest, CornerMove_90Degrees) {
  std::vector<LinearSegmentSteps> segments = DoAngleMove(5, 90);
  ASSERT_EQ(4, segments.size());

  // Zero speed at beginning and end.
  EXPECT_EQ(0, segments[0].v0);
  EXPECT_EQ(0, segments[segments.size()-1].v1);

  // This is a 90 degree move, we expect to slow down all the way to zero
  // in the elbow
  EXPECT_EQ(0, segments[1].v1);
  EXPECT_EQ(segments[1].v1, segments[2].v0);
}

#if 0  // this is not working as expected yet.
TEST(PlannerTest, CornerMove_Shallow) {
  std::vector<LinearSegmentSteps> segments = DoAngleMove(5, 4);
  EXPECT_EQ(2, segments.size());   // ?? we only get one

  // Zero speed at beginning and end.
  EXPECT_EQ(0, segments[0].v0);
  EXPECT_EQ(0, segments[segments.size()-1].v1);

  // A shallow move just plows through the middle, so we expect a larger
  // speed than zero
  EXPECT_GT(segments[0].v1, 0);

  // Still, the join-speed at the elbow is the same.
  EXPECT_EQ(segments[1].v1, segments[2].v0);
}
#endif

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
