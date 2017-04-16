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
  MockPRUInterface() : execution_index_(QUEUE_LEN - 1) { mmap = NULL; }
  ~MockPRUInterface() { free(mmap); }

  bool Init() { return true; }
  bool StartExecution() { return true; }
  unsigned WaitEvent() { return 1; }
  bool Shutdown() { return true; }

  bool AllocateSharedMem(void **pru_mmap, const size_t size) {
    mmap = (struct MockPRUCommunication *) malloc(size);
    *pru_mmap = (void *) mmap;
    bzero(*pru_mmap, size);
    return true;
  }

  void SimRun(int num_exec, const uint32_t loops_left,
              bool last_not_executed = true) {
    // Simulate the execution of num_exec motion segments
    for (int i=0; i < num_exec; ++i) {
      execution_index_ = (execution_index_ + 1) % QUEUE_LEN;
      assert(mmap->ring_buffer[execution_index_].state != STATE_EMPTY);
      mmap->ring_buffer[execution_index_].state = STATE_EMPTY;
    }
    if (last_not_executed || loops_left )
      mmap->ring_buffer[execution_index_].state = STATE_FILLED;
    mmap->status.index = execution_index_;
    mmap->status.counter = loops_left;
  }

private:
  struct MockPRUCommunication *mmap;
  unsigned int execution_index_;
};

static void check_buffer_value(PRUMotionQueue *motion_backend,
                               const unsigned size) {
  unsigned int buffer;
  motion_backend->GetExecutionProgress(NULL, &buffer);
  EXPECT_EQ(buffer, size);
}

TEST(PruMotionQueue, status_init) {
  MotorsRegister absolute_pos_loops;
  MockPRUInterface pru_interface = MockPRUInterface();
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface*) &pru_interface);
  check_buffer_value(&motion_backend, 0);
}

TEST(PruMotionQueue, single_exec) {
  MotorsRegister absolute_pos_loops;
  MockPRUInterface pru_interface = MockPRUInterface();
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface*) &pru_interface);

  static struct MotionSegment segment = {0};
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  pru_interface.SimRun(1, 0);
  check_buffer_value(&motion_backend, 1);
}

TEST(PruMotionQueue, full_exec) {
  MotorsRegister absolute_pos_loops;
  MockPRUInterface pru_interface = MockPRUInterface();
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface*) &pru_interface);

  static struct MotionSegment segment = {0};
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  pru_interface.SimRun(1, 0, false);
  check_buffer_value(&motion_backend, 0);
}

TEST(PruMotionQueue, single_exec_some_loops) {
  MotorsRegister absolute_pos_loops;
  MockPRUInterface pru_interface = MockPRUInterface();
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface*) &pru_interface);

  static struct MotionSegment segment = {0};
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  pru_interface.SimRun(2, 10);
  check_buffer_value(&motion_backend, 1);
}

TEST(PruMotionQueue, one_round_queue) {
  MotorsRegister absolute_pos_loops;
  MockPRUInterface pru_interface = MockPRUInterface();
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface*) &pru_interface);

  static struct MotionSegment segment = {0};
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  segment.state = STATE_FILLED;
  motion_backend.Enqueue(&segment);
  pru_interface.SimRun(2, 0);
  check_buffer_value(&motion_backend, 1);

  for (int i = 0; i < QUEUE_LEN - 1; ++i) {
    segment.state = STATE_FILLED;
    motion_backend.Enqueue(&segment);
  }
  check_buffer_value(&motion_backend, QUEUE_LEN);
}

TEST(PruMotionQueue, exec_index_lt_queue_pos) {
  MotorsRegister absolute_pos_loops;
  MockPRUInterface pru_interface = MockPRUInterface();
  HardwareMapping hmap = HardwareMapping();
  PRUMotionQueue motion_backend(&hmap, (PruHardwareInterface*) &pru_interface);

  static struct MotionSegment segment = {0};
  for (int i = 0; i < QUEUE_LEN; ++i) {
    segment.state = STATE_FILLED;
    motion_backend.Enqueue(&segment);
  }

  pru_interface.SimRun(QUEUE_LEN, 0);
  check_buffer_value(&motion_backend, 1);

  for (int i = 0; i < 1; ++i) {
    segment.state = STATE_FILLED;
    motion_backend.Enqueue(&segment);
  }

  check_buffer_value(&motion_backend, 2);
}

int main(int argc, char *argv[]) {
  Log_init("/dev/stderr");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
