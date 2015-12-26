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

#include "motor-operations.h"
#include "gcode-parser.h"

#include <stdio.h>
#include <string.h>

#include <gtest/gtest.h>

#define END_SENTINEL 0x42

// Set up config that they are the same for all the tests.
static void init_test_config(struct MachineControlConfig *c) {
  for (int i = 0; i < GCODE_NUM_AXES; ++i)
    c->steps_per_mm[i] = 100;  // step/mm
  for (int i = 0; i < GCODE_NUM_AXES; ++i)
    c->acceleration[i] = 1000;  // mm/s^2
  for (int i = 0; i < GCODE_NUM_AXES; ++i)
    c->max_feedrate[i] = 10000;
  c->axis_mapping   = "XYZEABCU";
  c->threshold_angle = 0;
}

namespace {
class MockMotorOps : public MotorOperations {
public:
  MockMotorOps(const MotorMovement *expected)
    : expect_(expected), current_(expected), errors_(0) {}

  ~MockMotorOps() {
    EXPECT_EQ(0, errors_);
    // Did we walk through all states ?
    EXPECT_EQ(END_SENTINEL, current_->aux_bits);  // reached end ?
  }

  virtual int Enqueue(const MotorMovement &param, FILE *err_stream) {
    const int number = (int)(current_ - expect_);
    EXPECT_NE(END_SENTINEL, current_->aux_bits);
    ExpectEq(current_, param, number);
    ++current_;
    return 0;
  }

  virtual void MotorEnable(bool on) {}
  virtual void WaitQueueEmpty(){}

private:
  // Helpers to compare and print MotorMovements.
  static void PrintMovement(const char *msg, const struct MotorMovement *m) {
    printf("%s: v0=%.1f -> v1=%.1f {%d, %d, %d}", msg, m->v0, m->v1,
           m->steps[0], m->steps[1], m->steps[2]);
  }

  void ExpectEq(const struct MotorMovement *expected,
                const struct MotorMovement &reality,
                int info_segment_number) {
    if (memcmp(expected, &reality, sizeof(*expected)) != 0) {
      printf("segment #%d: ", info_segment_number);
      PrintMovement("expected", expected);
      PrintMovement("; but was", &reality);
      printf("\n");
      ++errors_;
    }
  }

  const MotorMovement *const expect_;
  const MotorMovement *current_;

  int errors_;
};
}

class Harness {
 public:
  // Initialize harness with the expected sequence of motor movements
  // and return the callback struct to receive simulated gcode calls.
  Harness(const MotorMovement *expected)
    : expect_motor_ops(expected) {
    init_test_config(&config);
    machine_control = GCodeMachineControl::Create(config, &expect_motor_ops, NULL);
  }

  ~Harness() {
    delete machine_control;
  }

  GCodeParser::EventReceiver *gcode_emit() {
    return machine_control->ParseEventReceiver();
  }

  struct MachineControlConfig config;
  MockMotorOps expect_motor_ops;
  GCodeMachineControl *machine_control;
};

// If we get two line segments that are straight and don't change speed, we
// expect no slow-down.
TEST(GCodeMachineControlTest, straight_segments_same_speed) {
  // TODO(hzeller): use mock functionality for this to compare.
  static const struct MotorMovement expected[] = {
    { /*v0*/     0.0, /*v1*/ 10000.0, 0, /*steps*/ { 500}},  // accel
    { /*v0*/ 10000.0, /*v1*/ 10000.0, 0, /*steps*/ {9500}},  // 1st move @100mm/s
    { /*v0*/ 10000.0, /*v1*/ 10000.0, 0, /*steps*/ {9500}},  // 2nd move @100mm/s
    { /*v0*/ 10000.0, /*v1*/     0.0, 0, /*steps*/ { 500}},  // decel back to 0
    { 0.0, 0.0, END_SENTINEL},
  };
  Harness harness(expected);

  // Move to pos 100, then 200, first with speed 100, then speed 50
  AxesRegister coordinates;
  coordinates[0] = 100;
  harness.gcode_emit()->coordinated_move(100, coordinates);  // 100mm/s

  // second half, same speed
  coordinates[0] = 200;
  harness.gcode_emit()->coordinated_move(100, coordinates);  // also 100mm/s

  harness.gcode_emit()->motors_enable(false);  // finish movement.
}

// If we have two line segments in a straight line and one is slower than the
// other, slow down the first segment at the end to the travel speed of the
// next segment.
TEST(GCodeMachineControlTest, straight_segments_speed_change) {
  static const struct MotorMovement expected[] = {
    { /*v0*/     0.0, /*v1*/ 10000.0, 0, /*steps*/ { 500}},  // accel
    { /*v0*/ 10000.0, /*v1*/ 10000.0, 0, /*steps*/ {9125}},  // move @100mm/s
    { /*v0*/ 10000.0, /*v1*/  5000.0, 0, /*steps*/ { 375}},  // slow: match next speed
    { /*v0*/  5000.0, /*v1*/  5000.0, 0, /*steps*/ {9875}},  // move @50mm/s
    { /*v0*/  5000.0, /*v1*/     0.0, 0, /*steps*/ { 125}},  // final slow to zero
    { 0.0, 0.0, END_SENTINEL},
  };
  Harness harness(expected);

  // Move to pos 100, then 200, first with speed 100, then speed 50
  AxesRegister coordinates;
  coordinates[0] = 100;
  harness.gcode_emit()->coordinated_move(100, coordinates); // 100mm/s

  // second half, less speed.
  coordinates[0] = 200;
  harness.gcode_emit()->coordinated_move(50, coordinates);  // 50mm/s

  harness.gcode_emit()->motors_enable(false);  // finish movement.
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
