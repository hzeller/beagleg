/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013-2020 Henner Zeller <h.zeller@acm.org>
 *
 * This file is part of BeagleG. http://github.com/hzeller/beagleg
 *
 * BeagleG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BeagleG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "spi-segment-queue.h"

#include <string.h>

#include "spi.h"

// The protocol elements. Right now just pretend.
namespace beagleg {
struct SpiMotionSegment {
  uint32_t count_steps;
};

struct QueueStatus {   // This should be like PhysicalStatus
  uint32_t counter : 24; // remaining number of cycles to be performed
  uint32_t index : 8;    // represent the executing slot [0 to QUEUE_LEN - 1]
};
}  // namespace beagleg

// How many bytes there are in one record.
static constexpr int BYTES_PER_FIFO_RECORD = sizeof(beagleg::SpiMotionSegment);

// This needs to be in-sync with FPGA impl.
enum Command {
  CMD_NO_OP      = 0x00,  // Just send one byte to receive fifo free
  CMD_STATUS     = 0x01, // Send 5 bytes, receive fifo free + status word
  CMD_WRITE_FIFO = 0x02,
  CMD_READ_FIFO  = 0x03,
};

class BeagleGSPIProtocol {
public:
  BeagleGSPIProtocol(SPIHost *channel) : spi_channel_(channel) {}

  // Get number of free slots.
  int GetFreeSlots() {
    const char cmd = CMD_NO_OP;
    char val;
    spi_channel_->TransferBuffer(&cmd, &val, 1);
    return val;
  }

  // Get queue status. Also return number of free slots.
  int GetQueueStatus(beagleg::QueueStatus *status) {
    const size_t tx_len = 1 + sizeof(beagleg::QueueStatus);
    char tx_buffer[tx_len] = {};
    char rx_buffer[tx_len];
    tx_buffer[0] = CMD_STATUS;
    spi_channel_->TransferBuffer(tx_buffer, rx_buffer, tx_len);
    memcpy(status, rx_buffer + 1, sizeof(*status));
    return rx_buffer[0];
  }

  // Attempts to send motion segments. Only sends amount possible.
  // Returns number of segments sent.
  int SendMotionSegments(const beagleg::SpiMotionSegment *segments, int count) {
    const char command = CMD_WRITE_FIFO;
    uint8_t free_slots;
    if (!spi_channel_->TransferBuffer(&command, &free_slots, 1, false))
      return -1;

    if (free_slots < count) {
#if 0
      fprintf(stderr, "Available fifo space of %d < requested %d\n",
              free_slots, count);
#endif
      return 0;
    }

    const int segment_tx_count = std::min((int)free_slots, count);
    const int segment_byte_len = segment_tx_count * sizeof(beagleg::SpiMotionSegment);
    char rx_buffer[segment_byte_len];   // Let's see what it sends back
#if 0
    fprintf(stderr, "Sending actual data; %d elements = %d bytes\n",
            segment_tx_count, segment_byte_len);
#endif
    if (!spi_channel_->TransferBuffer(segments, rx_buffer, segment_byte_len))
      return -1;
    return segment_tx_count;
  }

private:
  SPIHost *const spi_channel_;
};

SPISegmentQueue::SPISegmentQueue(SPIHost *spi,
                                 StepGeneratorModuleSim *module_sim)
  : protocol_handler_(new BeagleGSPIProtocol(spi)),
    module_sim_(module_sim) {
}

SPISegmentQueue::~SPISegmentQueue() {
  delete protocol_handler_;
}

bool SPISegmentQueue::Enqueue(const LinearSegmentSteps &segment) {
  const int motor_steps = segment.steps[0];  // only looking at one motor
  // 0: don't send these, as  probably only other motors were active
  // < 0: don't send, as they are interpreted as unsigned right now
  if (motor_steps <= 0) return true;

  fprintf(stderr, "got Enqueue(%d start=%.1f end=%.1f)\n",
          motor_steps, segment.v0, segment.v1);
  beagleg::SpiMotionSegment seg;
  seg.count_steps = motor_steps;  // just sending motor one for now
  while ((protocol_handler_->SendMotionSegments(&seg, 1)) != 1) {
    module_sim_->Cycle(100);  // Keep the clocks going.
  }
  return true;
}

void SPISegmentQueue::MotorEnable(bool on) { /* TODO: implement */ }

void SPISegmentQueue::WaitQueueEmpty() {
  static constexpr int kCycles = 1000;
  fprintf(stderr, "Wait queue empty ...\n");
  int count = 0;
  // TODO: this only works until the last slot is put into the step
  // generator, so we still haven't generated all the steps at that point.
  while (protocol_handler_->GetFreeSlots() < 16) {
    module_sim_->Cycle(kCycles);
    ++count;
  }
  fprintf(stderr, "In WaitQueueEmpty(): called %d * %d cycles\n",
          count, kCycles);
}

bool SPISegmentQueue::GetPhysicalStatus(PhysicalStatus *status) {
  return false;  // TODO: implement
}
void SPISegmentQueue::SetExternalPosition(int axis, int pos) {
  // TODO: implement.
}
