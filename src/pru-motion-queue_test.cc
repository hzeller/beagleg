/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * Test for pru motion queue.
 *
 * We simulate the pru hardware and test if the absolute position is correctly
 * retrieved.
 *
 */
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <stdio.h>
#include <string.h>

#include "common/container.h"
#include "common/logging.h"
#include "hardware-mapping.h"
#include "motion-queue.h"
#include "motor-interface-constants.h"
#include "pru-hardware-interface.h"
#include "segment-queue.h"

using ::testing::NiceMock;

// PRU-side mock implementation of the ring buffer.
struct MockPRUCommunication {
  internal::QueueStatus status;
  MotionSegment ring_buffer[QUEUE_LEN];
} __attribute__((packed));

class MockPRUInterface : public PruHardwareInterface {
 public:
  MockPRUInterface() : execution_index_(QUEUE_LEN - 1) {
    mmap = NULL;
    ON_CALL(*this, Init).WillByDefault([]() { return true; });
    ON_CALL(*this, Shutdown).WillByDefault([]() { return true; });
  }
  ~MockPRUInterface() override { free(mmap); }

  bool StartExecution() final { return true; }
  unsigned WaitEvent() final { return 1; }
  MOCK_METHOD(bool, Init, (), ());
  MOCK_METHOD(bool, Shutdown, (), ());

  bool AllocateSharedMem(void **pru_mmap, const size_t size) final {
    if (mmap != NULL) return true;
    mmap = (struct MockPRUCommunication *)malloc(size);
    *pru_mmap = (void *)mmap;
    memset(*pru_mmap, 0x00, size);
    return true;
  }

  void SimRun(int num_exec, const uint32_t loops_left,
              bool last_not_executed = true) {
    // Simulate the execution of num_exec motion segments
    for (int i = 0; i < num_exec; ++i) {
      execution_index_ = (execution_index_ + 1) % QUEUE_LEN;
      assert(mmap->ring_buffer[execution_index_].state != STATE_EMPTY);
      mmap->ring_buffer[execution_index_].state = STATE_EMPTY;
    }
    if (last_not_executed || loops_left) {
      mmap->ring_buffer[execution_index_].state = STATE_FILLED;
    }
    mmap->status.index = execution_index_;
    mmap->status.counter = loops_left;
  }

 private:
  struct MockPRUCommunication *mmap;
  unsigned int execution_index_;
};

TEST(PruMotionQueue, status_init) {
  MotorsRegister absolute_pos_loops;
  NiceMock<MockPRUInterface> pru_interface;
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface *)&pru_interface);
  EXPECT_EQ(motion_backend.GetPendingElements(NULL), 0);
}

TEST(PruMotionQueue, single_exec) {
  MotorsRegister absolute_pos_loops;
  NiceMock<MockPRUInterface> pru_interface;
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface *)&pru_interface);

  struct MotionSegment segment = {};
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  pru_interface.SimRun(1, 0);
  EXPECT_EQ(motion_backend.GetPendingElements(NULL), 1);
}

TEST(PruMotionQueue, full_exec) {
  MotorsRegister absolute_pos_loops;
  NiceMock<MockPRUInterface> pru_interface;
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface *)&pru_interface);

  struct MotionSegment segment = {};
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  pru_interface.SimRun(1, 0, false);
  EXPECT_EQ(motion_backend.GetPendingElements(NULL), 0);
}

TEST(PruMotionQueue, single_exec_some_loops) {
  MotorsRegister absolute_pos_loops;
  NiceMock<MockPRUInterface> pru_interface;
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface *)&pru_interface);

  struct MotionSegment segment = {};
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  pru_interface.SimRun(2, 10);
  EXPECT_EQ(motion_backend.GetPendingElements(NULL), 1);
}

TEST(PruMotionQueue, one_round_queue) {
  MotorsRegister absolute_pos_loops;
  NiceMock<MockPRUInterface> pru_interface;
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface *)&pru_interface);

  struct MotionSegment segment = {};
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  pru_interface.SimRun(2, 0);
  EXPECT_EQ(motion_backend.GetPendingElements(NULL), 1);

  for (int i = 0; i < QUEUE_LEN - 1; ++i) {
    segment.state = STATE_FILLED;
    motion_backend.Enqueue(&segment);
  }
  EXPECT_EQ(motion_backend.GetPendingElements(NULL), QUEUE_LEN);
}

// Check the PRU is reset and no elements are pending.
TEST(PruMotionQueue, clear_queue) {
  MotorsRegister absolute_pos_loops;
  NiceMock<MockPRUInterface> pru_interface;
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface *)&pru_interface);

  struct MotionSegment segment = {};
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  pru_interface.SimRun(2, 10);
  EXPECT_EQ(motion_backend.GetPendingElements(NULL), 1);

  // Start recording mocks.
  ASSERT_TRUE(testing::Mock::VerifyAndClearExpectations(&pru_interface));
  {
    testing::InSequence seq;
    EXPECT_CALL(pru_interface, Shutdown())
      .Times(1)
      .WillRepeatedly(testing::Return(true));
    EXPECT_CALL(pru_interface, Init()).Times(1).WillOnce(testing::Return(true));
  }
  EXPECT_TRUE(motion_backend.Clear());
  EXPECT_EQ(motion_backend.GetPendingElements(NULL), 0);
}

TEST(PruMotionQueue, exec_index_lt_queue_pos) {
  MotorsRegister absolute_pos_loops;
  NiceMock<MockPRUInterface> pru_interface;
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface *)&pru_interface);

  struct MotionSegment segment = {};
  for (int i = 0; i < QUEUE_LEN; ++i) {
    segment.state = STATE_FILLED;
    motion_backend.Enqueue(&segment);
  }

  pru_interface.SimRun(QUEUE_LEN, 0);
  EXPECT_EQ(motion_backend.GetPendingElements(NULL), 1);

  for (int i = 0; i < 1; ++i) {
    segment.state = STATE_FILLED;
    motion_backend.Enqueue(&segment);
  }

  EXPECT_EQ(motion_backend.GetPendingElements(NULL), 2);
}

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
