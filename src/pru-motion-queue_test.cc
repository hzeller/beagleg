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

#include "logging.h"
#include "motor-operations.h"
#include "pru-hardware-interface.h"
#include "hardware-mapping.h"
#include "motor-interface-constants.h"

class MockPruInterface : public PruHardwareInterface {
public:
  MockPruInterface() { mmap = NULL; }
  bool Init() { return true; }
  bool AllocateSharedMem(void **pru_mmap, const size_t size) {
    mmap = new MotionSegment[size]();
    *pru_mmap = (void *) mmap;
  }
  bool StartExecution() { return true; }
  unsigned WaitEvent() { return 1; }
  bool Shutdown() { return true; }
  ~MockPruInterface() { delete mmap; }
  void SimRun(unsigned int execution_index, const uint32_t loops_left) {
    assert(loops_left <= *mock_pru_queue_[0]);
    assert(executed_slots < mock_pru_queue_.size());
    for (unsigned i = 0; i < executed_slots; ++i) {
      mock_pru_queue_.pop_front();
    }
    *mock_pru_queue_[0] = loops_left;
  }
private:
  MotionSegment *mmap;
};

bool MockPruInterface::Init() {
  return true;
}
  // void Enqueue(MotionSegment *segment);
  // void WaitQueueEmpty();
  // void MotorEnable(bool on);
  // void Shutdown(bool flush_queue);

// Check that the initial position is 0
TEST(RealtimePosition, init_pos) {
  //PruHardwareInterface *pru_interface = new MockPruInterface();
  HardwareMapping *hmap = new HardwareMapping();

  //PRUMotionQueue motion_backend(hmap, pru_interface);
  motion_backend.GetMotorLoops();

  //motor_operations.GetCurrentMotorPos(motor_position_steps);
  EXPECT_EQ (0, motor_position_steps[0]);

  //delete pru_interface;
  delete hmap;
}

// Enqueue a low-loops segment.
TEST(RealtimePosition, small_segment) {
  static const struct MotionSegment small_segment = {
    STATE_FILLED /*state*/, 0x00 | 1 << 1 | 1 << 2 /*direction bits*/,
    0 /*loops accel*/, 10u /*loops travel*/, 0 /*loops decel*/,
    0 /*aux*/, 0 /*accel_series_index*/, 0x0F /*travel_delay_cycles*/,
    0xFFFFFFFF, 0x00FFFFF, 0x000FFFF, 0, 0, 0, 0, 0/*fractions*/,
  };
  PruHardwareInterface *pru_interface = new MockPruInterface();
  HardwareMapping *hmap = new HardwareMapping();

  PRUMotionQueue motion_backend(hmap, pru_interface);

  motion_backend.Enqueue(small_segment)

  motion_backend.GetMotorLoops();

  motor_operations.GetCurrentMotorPos(motor_position_steps);
  EXPECT_EQ (0, motor_position_steps[0]);

  delete pru_interface;
  delete hmap;
}

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
