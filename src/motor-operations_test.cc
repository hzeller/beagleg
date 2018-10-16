/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Test for pru motion queue.
 *
 * We simulate the pru hardware and test if the absolute position is correctly
 * retrieved.
 *
 */
#include "motion-queue.h"

#include <stdio.h>
#include <string.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "common/container.h"
#include "common/logging.h"
#include "hardware-mapping.h"
#include "motor-operations.h"

class MockMotionQueue : public MotionQueue {
public:
  MockMotionQueue() : remaining_loops_(0), queue_size_(0) {}

  bool Enqueue(MotionSegment *segment) {
    remaining_loops_ = segment->loops_accel
      + segment->loops_travel + segment->loops_decel;
    queue_size_++;
    return true;
  }

  void WaitQueueEmpty() {};
  void MotorEnable(bool on) {};
  void Shutdown(bool flush_queue) {};
  int GetPendingElements(uint32_t *head_item_progress) {
      if (head_item_progress)
        *head_item_progress = remaining_loops_;
      return queue_size_;
  }

  void SimRun(const uint32_t executed_loops, const unsigned int buffer_size) {
    assert(buffer_size <= queue_size_);
    if (buffer_size == queue_size_)
      assert(remaining_loops_ >= executed_loops);
    remaining_loops_ = executed_loops;
    queue_size_ = buffer_size;
  }

private:
  uint32_t remaining_loops_;
  unsigned int queue_size_;
};

// Check that on init, the initial position is 0.
TEST(RealtimePosition, init_pos) {
  HardwareMapping hw;
  MockMotionQueue motion_backend = MockMotionQueue();
  MotionQueueMotorOperations motor_operations(&hw, &motion_backend);

  PhysicalStatus status;
  motor_operations.GetPhysicalStatus(&status);
  const int expected[BEAGLEG_NUM_MOTORS] = {0, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_THAT(expected, ::testing::ContainerEq(status.pos_steps));
}

// Check that the steps are correctly evaluated from the shadow queue
// with the correct sign.
TEST(RealtimePosition, back_and_forth) {
  HardwareMapping hw;
  MockMotionQueue motion_backend = MockMotionQueue();
  MotionQueueMotorOperations motor_operations(&hw, &motion_backend);

  // Enqueue a segment
  const LinearSegmentSteps kSegment1 = {
    0 /* v0 */, 0 /* v1 */, 0 /* aux */,
    {1000, 0, 0, 0, 0, 0, 0, 0} /* steps */
  };
  // Enqueue a segment
  const LinearSegmentSteps kSegment2 = {
    0 /* v0 */, 0 /* v1 */, 0 /* aux */,
    {-1000, 0, 0, 0, 0, 0, 0, 0} /* steps */
  };

  PhysicalStatus status;
  motor_operations.Enqueue(kSegment1);

  motion_backend.SimRun(0, 0);
  motor_operations.GetPhysicalStatus(&status);

  {
    const int expected[BEAGLEG_NUM_MOTORS] =
      {1000, 0, 0, 0, 0, 0, 0, 0};
    EXPECT_THAT(expected, ::testing::ContainerEq(status.pos_steps));
  }


  motor_operations.Enqueue(kSegment2);
  motor_operations.Enqueue(kSegment1);

  motion_backend.SimRun(0, 0);
  motor_operations.GetPhysicalStatus(&status);

  {
    const int expected[BEAGLEG_NUM_MOTORS] =
      {1000, 0, 0, 0, 0, 0, 0, 0};
    EXPECT_THAT(expected, ::testing::ContainerEq(status.pos_steps));
  }
}

// Enqueue an empty element. We need to be sure that also empty elements
// are taken into account into the shadow queue since they are sent to the
// motion queue.
TEST(RealtimePosition, empty_element) {
  HardwareMapping hw;
  MockMotionQueue motion_backend = MockMotionQueue();
  MotionQueueMotorOperations motor_operations(&hw, &motion_backend);

  // Enqueue a segment
  const LinearSegmentSteps kSegment1 = {
    0 /* v0 */, 0 /* v1 */, 0xff /* aux */,
    {0, 0, 0, 0, 0, 0, 0, 0} /* steps */
  };
  const LinearSegmentSteps kSegment2 = {
    0 /* v0 */, 0 /* v1 */, 0 /* aux */,
    {100, 0, 0, 0, 0, 0, 0, 0} /* steps */
  };

  motor_operations.Enqueue(kSegment2);
  motor_operations.Enqueue(kSegment1);

  motion_backend.SimRun(0, 2);
  PhysicalStatus status;
  motor_operations.GetPhysicalStatus(&status);
  {
    const int expected[BEAGLEG_NUM_MOTORS] =
      {100, 0, 0, 0, 0, 0, 0, 0};
    EXPECT_THAT(expected, ::testing::ContainerEq(status.pos_steps));
  }
}

// Enqueue a bunch of LinearSegmentSteps and retrieve the position.
TEST(RealtimePosition, sample_pos) {
  HardwareMapping hw;
  MockMotionQueue motion_backend = MockMotionQueue();
  MotionQueueMotorOperations motor_operations(&hw, &motion_backend);

  // Enqueue a segment
  const LinearSegmentSteps kSegment1 = {
    0 /* v0 */, 0 /* v1 */, 0 /* aux */,
    {10, -20, 30, -40, 0, -60, 70, -80} /* steps */
  };
  const LinearSegmentSteps kSegment2 = {
    0 /* v0 */, 0 /* v1 */, 0 /* aux */,
    {17, +42, 12, -90, 30, +91, 113, -1000} /* steps */
  };
  motor_operations.Enqueue(kSegment1);
  motor_operations.Enqueue(kSegment2);


  motion_backend.SimRun(20, 2);
  PhysicalStatus status;
  motor_operations.GetPhysicalStatus(&status);
  {
    const int expected[BEAGLEG_NUM_MOTORS] =
      {8, -17, 26, -35, 0, -52, 61, -70};
    EXPECT_THAT(expected, ::testing::ContainerEq(status.pos_steps));
  }

  motor_operations.Enqueue(kSegment1);
  motion_backend.SimRun(0, 1);
  motor_operations.GetPhysicalStatus(&status);

  {
    const int expected[BEAGLEG_NUM_MOTORS] =
      {37, 2, 72, -170, 30, -29, 253, -1160};
    EXPECT_THAT(expected, ::testing::ContainerEq(status.pos_steps));
  }
}

// Check that SimRun(0, x) produce the same absolute position as
// SimRun(MAX_SEGMENT_STEPS, x + 1).
TEST(RealtimePosition, zero_loops_edge) {
  HardwareMapping hw;
  MockMotionQueue motion_backend = MockMotionQueue();
  MotionQueueMotorOperations motor_operations(&hw, &motion_backend);

  // Enqueue a segment
  LinearSegmentSteps segment = {
    0 /* v0 */, 0 /* v1 */, 0 /* aux */,
    {10, 20, 30, 40, 50, 60, 70, 80} /* steps */
  };
  const int expected[BEAGLEG_NUM_MOTORS] = {10, 20, 30, 40, 50, 60, 70, 80};

  motor_operations.Enqueue(segment);
  motor_operations.Enqueue(segment);
  motion_backend.SimRun(0, 2);

  PhysicalStatus status;
  motor_operations.GetPhysicalStatus(&status);
  EXPECT_THAT(expected, ::testing::ContainerEq(status.pos_steps));

  motion_backend.SimRun(160, 1);

  motor_operations.GetPhysicalStatus(&status);
  EXPECT_THAT(expected, ::testing::ContainerEq(status.pos_steps));
}

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
