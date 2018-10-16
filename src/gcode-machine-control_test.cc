/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Test for gcode machine control.
 *
 * We simulate inputs coming from the GCode parser and expect that
 * we get a certain sequence of MotorMovment commands.
 *
 * Very simple right now, I should probably build some testing
 * framework.
 */
#include "gcode-machine-control.h"

#include <stdio.h>
#include <string.h>

#include <gtest/gtest.h>

#include "gcode-parser/gcode-parser.h"
#include "common/logging.h"

#include "hardware-mapping.h"
#include "motor-operations.h"

#define END_SENTINEL 0x42

// Set up config that they are the same for all the tests.
static void init_test_config(struct MachineControlConfig *c,
                             HardwareMapping *hmap) {
  for (int i = 0; i <= AXIS_Z; ++i) {
    const GCodeParserAxis axis = (GCodeParserAxis) i;
    c->steps_per_mm[axis] = 100;  // step/mm
    c->acceleration[axis] = 1000;  // mm/s^2
    c->max_feedrate[axis] = (i+1) * 1000;
  }
  c->threshold_angle = 0;
  c->speed_tune_angle = 0;
  c->require_homing = false;
}

namespace {
class MockMotorOps : public MotorOperations {
public:
  MockMotorOps(const LinearSegmentSteps *expected)
    : expect_(expected), current_(expected), errors_(0) {}

  ~MockMotorOps() {
    EXPECT_EQ(0, errors_);
    // Did we walk through all states ?
    EXPECT_EQ(END_SENTINEL, current_->aux_bits);  // reached end ?
  }

  bool Enqueue(const LinearSegmentSteps &param) final {
    const int number = (int)(current_ - expect_);
    EXPECT_NE(END_SENTINEL, current_->aux_bits);
    ExpectEq(current_, param, number);
    ++current_;
    return true;
  }

  void MotorEnable(bool on) final {}
  void WaitQueueEmpty() final {}
  bool GetPhysicalStatus(PhysicalStatus *status) final { return false; }
  void SetExternalPosition(int axis, int steps) final { }

private:
  // Helpers to compare and print MotorMovements.
  static void PrintMovement(const char *msg, const struct LinearSegmentSteps *m) {
    printf("%s: v0=%.1f -> v1=%.1f {%d, %d, %d}", msg, m->v0, m->v1,
           m->steps[0], m->steps[1], m->steps[2]);
  }

  void ExpectEq(const struct LinearSegmentSteps *expected,
                const struct LinearSegmentSteps &reality,
                int info_segment_number) {
    if (expected->v0 != reality.v0 ||
        expected->v1 != reality.v1 ||
        expected->steps[0] != reality.steps[0] ||
        expected->steps[1] != reality.steps[1] ||
        expected->steps[2] != reality.steps[2]) {
      printf("segment #%d: ", info_segment_number);
      PrintMovement("expected", expected);
      PrintMovement("; but was", &reality);
      printf("\n");
      ++errors_;
    }
  }

  const LinearSegmentSteps *const expect_;
  const LinearSegmentSteps *current_;

  int errors_;
};
}

class Harness {
 public:
  // Initialize harness with the expected sequence of motor movements
  // and return the callback struct to receive simulated gcode calls.
  Harness(const LinearSegmentSteps *expected) : expect_motor_ops_(expected) {
    struct MachineControlConfig config;
    init_test_config(&config, &hardware_);
    machine_control = GCodeMachineControl::Create(config, &expect_motor_ops_,
                                                  &hardware_,
                                                  NULL,   // spindle
                                                  NULL);  // msg-stream
    assert(machine_control != NULL);
  }

  ~Harness() {
    delete machine_control;
  }

  GCodeParser::EventReceiver *gcode_emit() {
    return machine_control->ParseEventReceiver();
  }

  MockMotorOps expect_motor_ops_;
  HardwareMapping hardware_;
  GCodeMachineControl *machine_control;
};

TEST(GCodeMachineControlTest, initial_feedrate_not_set) {
  static const struct LinearSegmentSteps expected[] = {
    { 0.0, 0.0, END_SENTINEL, {}},
  };

  // Move to pos 100, do not set the feedrate.
  AxesRegister coordinates;
  coordinates[AXIS_A] = 100;
  {
    Harness harness(expected);
    EXPECT_FALSE(harness.gcode_emit()->coordinated_move(-1, coordinates));
    // ... but after we have set the feedrate once
    EXPECT_TRUE(harness.gcode_emit()->coordinated_move(42, coordinates));
    // ... we remember it next time.
    EXPECT_TRUE(harness.gcode_emit()->coordinated_move(-1, coordinates));
  }

  // G0 feedrate is always initialized derived from the configuration.
  {
    Harness harness(expected);
    EXPECT_TRUE(harness.gcode_emit()->rapid_move(-1, coordinates));
  }
}

// If we get two line segments that are straight and don't change speed, we
// expect no slow-down.
TEST(GCodeMachineControlTest, straight_segments_same_speed) {
  // TODO(hzeller): use mock functionality for this to compare.
  static const struct LinearSegmentSteps expected[] = {
    { /*v0*/     0.0, /*v1*/ 10000.0, 0, /*steps*/ { 500}},  // accel
    { /*v0*/ 10000.0, /*v1*/ 10000.0, 0, /*steps*/ {9500}},  // 1st move @100mm/s
    { /*v0*/ 10000.0, /*v1*/ 10000.0, 0, /*steps*/ {9500}},  // 2nd move @100mm/s
    { /*v0*/ 10000.0, /*v1*/     0.0, 0, /*steps*/ { 500}},  // decel back to 0
    { 0.0, 0.0, END_SENTINEL, {}},
  };
  Harness harness(expected);

  // Move to pos 100, then 200, first with speed 100, then speed 50
  AxesRegister coordinates;
  coordinates[AXIS_X] = 100;
  harness.gcode_emit()->coordinated_move(100, coordinates);  // 100mm/s

  // second half, same speed
  coordinates[AXIS_X] = 200;
  harness.gcode_emit()->coordinated_move(100, coordinates);  // also 100mm/s

  harness.gcode_emit()->motors_enable(false);  // finish movement.
}

// Since the speed limit of Y doubles the X speed limit, and the steps ratio is
// 1:12 we expect the speed of the defining axis (Y) to be 6/5 of the X speed
// limit. The same concept in the same way is extended to acceleration.
TEST(GCodeMachineControlTest, speed_clamping) {
  // Total steps x = 200 000  total steps y = 240 000
  static const struct LinearSegmentSteps expected[] = {
    { /*v0*/     0.0, /*v1*/ 120000.0, 0, /*steps*/ { 60000, 72000}},  // accel
    { /*v0*/120000.0, /*v1*/ 120000.0, 0, /*steps*/ { 80000, 96000}},  // move
    { /*v0*/120000.0, /*v1*/      0.0, 0, /*steps*/ { 60000, 72000}},  // decel back to 0
    { 0.0, 0.0, END_SENTINEL, {}},
  };
  Harness harness(expected);

  AxesRegister coordinates;
  coordinates[AXIS_X] = 2000;
  coordinates[AXIS_Y] = 2400;

  // We set a huge feedrate value to see if it's correctly clamped.
  harness.gcode_emit()->coordinated_move(1e10, coordinates);

  harness.gcode_emit()->motors_enable(false);  // finish movement.
}


// If we have two line segments in a straight line and one is slower than the
// other, slow down the first segment at the end to the travel speed of the
// next segment.
TEST(GCodeMachineControlTest, straight_segments_speed_change) {
  static const struct LinearSegmentSteps expected[] = {
    { /*v0*/     0.0, /*v1*/ 10000.0, 0, /*steps*/ { 500}},  // accel
    { /*v0*/ 10000.0, /*v1*/ 10000.0, 0, /*steps*/ {9125}},  // move @100mm/s
    { /*v0*/ 10000.0, /*v1*/  5000.0, 0, /*steps*/ { 375}},  // slow: match next speed
    { /*v0*/  5000.0, /*v1*/  5000.0, 0, /*steps*/ {9875}},  // move @50mm/s
    { /*v0*/  5000.0, /*v1*/     0.0, 0, /*steps*/ { 125}},  // final slow to zero
    { 0.0, 0.0, END_SENTINEL, {}},
  };
  Harness harness(expected);

  // Move to pos 100, then 200, first with speed 100, then speed 50
  AxesRegister coordinates;
  coordinates[AXIS_X] = 100;
  harness.gcode_emit()->coordinated_move(100, coordinates); // 100mm/s

  // second half, less speed.
  coordinates[AXIS_X] = 200;
  harness.gcode_emit()->coordinated_move(50, coordinates);  // 50mm/s

  harness.gcode_emit()->motors_enable(false);  // finish movement.
}

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
