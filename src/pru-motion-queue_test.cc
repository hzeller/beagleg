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

#include "container.h"
#include "logging.h"
#include "motor-operations.h"
#include "pru-hardware-interface.h"
#include "hardware-mapping.h"
#include "motor-interface-constants.h"

// PRU-side mock implementation of the ring buffer.
struct MockPRUCommunication {
  struct QueueStatus status;
  struct MotionSegment ring_buffer[QUEUE_LEN];
} __attribute__((packed));

class MockPRUInterface : public PruHardwareInterface {
public:
  MockPRUInterface() { mmap = NULL; }
  ~MockPRUInterface() { free(mmap); }

  bool Init() { return true; }
  bool StartExecution() { return true; }
  unsigned WaitEvent() { return 1; }
  bool Shutdown() { return true; }

  bool AllocateSharedMem(void **pru_mmap, const size_t size) {
    mmap = (struct MockPRUCommunication *) malloc(size);
    *pru_mmap = (void *) mmap;
    return true;
  }

  void SimRun(unsigned int execution_index, const uint32_t loops_left) {
    mmap->status.index = execution_index % QUEUE_LEN;
    mmap->status.counter = loops_left;
    // For now we set as always empty (executed) the state of each segment
    for (int i = 0; i < QUEUE_LEN; ++i) {
      mmap->ring_buffer[i].state = STATE_EMPTY;
    }
  }

private:
  struct MockPRUCommunication *mmap;
};

// Check that on init, the initial position is 0.
TEST(RealtimePosition, init_pos) {
  MotorsRegister absolute_pos_loops;
  MockPRUInterface *pru_interface = new MockPRUInterface();
  HardwareMapping *hmap = new HardwareMapping();
  PRUMotionQueue motion_backend(hmap, (PruHardwareInterface*) pru_interface);

  motion_backend.GetMotorsLoops(&absolute_pos_loops);

  const MotorsRegister expected = {0, 0, 0, 0, 0, 0, 0, 0};
  EXPECT_THAT(expected, ::testing::ContainerEq(absolute_pos_loops));

  delete pru_interface;
  delete hmap;
}

// Check that SimRun(1, 0) produce the same absolute position as
// SimRun(2, MAX_SEGMENT_LOOPS).
TEST(RealtimePosition, zero_loops_edge) {
  //motor 0 +, motor 1 -, motor 2 -, 150 loops. fractions: /1 /3 /5
  static struct MotionSegment segment = {
    STATE_FILLED /*state*/, 0x00 | 1 << 1 | 1 << 2 /*direction bits*/,
    0 /*loops accel*/, 150u /*loops travel*/, 0 /*loops decel*/,
    0 /*aux*/, 0 /*accel_series_index*/, 0 /*hires_accel_cycles*/,
    0 /*travel_delay_cycles*/,
    0xFFFFFFFF, 0x55555555, 0x33333333, 0, 0, 0, 0, 0/*fractions*/,
  };
  MotorsRegister absolute_pos_loops;
  MockPRUInterface *pru_interface = new MockPRUInterface();
  HardwareMapping *hmap = new HardwareMapping();
  PRUMotionQueue motion_backend(hmap, (PruHardwareInterface*) pru_interface);

  motion_backend.Enqueue(&segment);
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  pru_interface->SimRun(0, 0);

  motion_backend.GetMotorsLoops(&absolute_pos_loops);

  const MotorsRegister expected = {150, -50, -30, 0, 0, 0, 0, 0};
  EXPECT_THAT(expected, ::testing::ContainerEq(absolute_pos_loops));

  pru_interface->SimRun(1, 150);
  motion_backend.GetMotorsLoops(&absolute_pos_loops);

  EXPECT_THAT(expected, ::testing::ContainerEq(absolute_pos_loops));

  delete pru_interface;
  delete hmap;
}

// execution index and counter changing.
TEST(RealtimePosition, hybrid_execution) {
  //motor 0 +, motor 1 -, motor 2 -, 150 loops. fractions: /1 /3 /5
  static struct MotionSegment segment1 = {
    STATE_FILLED /*state*/, 0x00 | 1 << 1 | 1 << 2 /*direction bits*/,
    0 /*loops accel*/, 150u /*loops travel*/, 0 /*loops decel*/,
    0 /*aux*/, 0 /*accel_series_index*/, 0 /*hires_accel_cycles*/,
    0 /*travel_delay_cycles*/,
    0xFFFFFFFF, 0x55555555, 0x33333333, 0, 0, 0, 0, 0/*fractions*/,
  };

  //motor 0 -, motor 1 +, motor 2 -, 15 loops. fractions: /1 /3 /5
  static struct MotionSegment segment2 = {
    STATE_FILLED /*state*/, 0x00 | 1 << 0 | 1 << 2 /*direction bits*/,
    0 /*loops accel*/, 15u /*loops travel*/, 0 /*loops decel*/,
    0 /*aux*/, 0 /*accel_series_index*/, 0 /*hires_accel_cycles*/,
    0 /*travel_delay_cycles*/,
    0xFFFFFFFF, 0x55555555, 0x33333333, 0, 0, 0, 0, 0/*fractions*/,
  };

  MotorsRegister absolute_pos_loops;
  MotorsRegister expected;
  MockPRUInterface *pru_interface = new MockPRUInterface();
  HardwareMapping *hmap = new HardwareMapping();

  PRUMotionQueue motion_backend(hmap, (PruHardwareInterface*) pru_interface);

  motion_backend.Enqueue(&segment1);
  pru_interface->SimRun(0, 75);
  motion_backend.GetMotorsLoops(&absolute_pos_loops);

  expected = {75, -25, -15, 0, 0, 0, 0, 0};
  EXPECT_THAT(expected, ::testing::ContainerEq(absolute_pos_loops));

  motion_backend.Enqueue(&segment2);

  pru_interface->SimRun(1, 7);
  motion_backend.GetMotorsLoops(&absolute_pos_loops);

  expected = {142, -47, -32, 0, 0, 0, 0, 0};
  EXPECT_THAT(expected, ::testing::ContainerEq(absolute_pos_loops));

  pru_interface->SimRun(1, 0);
  motion_backend.GetMotorsLoops(&absolute_pos_loops);

  expected = {135, -45, -33, 0, 0, 0, 0, 0};
  EXPECT_THAT(expected, ::testing::ContainerEq(absolute_pos_loops));

  delete pru_interface;
  delete hmap;
}

// Test edge cases when the ring buffer is completely full, the it empties and
// then full again.
TEST(RealtimePosition, queue_full) {
  //motor 0 +, motor 1 -, motor 2 -, 150 loops. fractions: /1 /3 /5
  static struct MotionSegment segment_forward = {
    STATE_FILLED /*state*/, 0x00 | 1 << 1 | 1 << 2 /*direction bits*/,
    0 /*loops accel*/, 150u /*loops travel*/, 0 /*loops decel*/,
    0 /*aux*/, 0 /*accel_series_index*/, 0 /*hires_accel_cycles*/,
    0 /*travel_delay_cycles*/,
    0xFFFFFFFF, 0x55555555, 0x33333333, 0, 0, 0, 0, 0/*fractions*/,
  };

  //motor 0 -, motor 1 +, motor 2 +, 150 loops. fractions: /1 /3 /5
  static struct MotionSegment segment_reverse = {
    STATE_FILLED /*state*/, 0x00 | 1 << 0 /*direction bits*/,
    0 /*loops accel*/, 150u /*loops travel*/, 0 /*loops decel*/,
    0 /*aux*/, 0 /*accel_series_index*/, 0 /*hires_accel_cycles*/,
    0 /*travel_delay_cycles*/,
    0xFFFFFFFF, 0x55555555, 0x33333333, 0, 0, 0, 0, 0/*fractions*/,
  };

  MotorsRegister absolute_pos_loops;
  MotorsRegister expected;
  MockPRUInterface *pru_interface = new MockPRUInterface();
  HardwareMapping *hmap = new HardwareMapping();

  PRUMotionQueue motion_backend(hmap, (PruHardwareInterface*) pru_interface);

  for (int i = 0; i < QUEUE_LEN; ++i) {
    motion_backend.Enqueue(&segment_forward);
    segment_forward.state = STATE_FILLED;
  }

  pru_interface->SimRun(QUEUE_LEN - 1, 0);
  motion_backend.GetMotorsLoops(&absolute_pos_loops);

  expected = {2400, -800, -480, 0, 0, 0, 0, 0};
  EXPECT_THAT(expected, ::testing::ContainerEq(absolute_pos_loops));

  for (int i = 0; i < 3; ++i) {
    motion_backend.Enqueue(&segment_reverse);
    segment_reverse.state = STATE_FILLED;
  }

  pru_interface->SimRun(2, 0);
  motion_backend.GetMotorsLoops(&absolute_pos_loops);

  expected = {1950, -650, -390, 0, 0, 0, 0, 0};
  EXPECT_THAT(expected, ::testing::ContainerEq(absolute_pos_loops));

  // Simulate a stop, the pru awaits for the next slot to be filled.
  pru_interface->SimRun(3, 0);

  for (int i = 0; i < QUEUE_LEN; ++i) {
    motion_backend.Enqueue(&segment_reverse);
    segment_reverse.state = STATE_FILLED;
  }

  pru_interface->SimRun(2, 0);
  motion_backend.GetMotorsLoops(&absolute_pos_loops);

  expected = {-450, 150, 90, 0, 0, 0, 0, 0};
  EXPECT_THAT(expected, ::testing::ContainerEq(absolute_pos_loops));

  delete pru_interface;
  delete hmap;
}

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
