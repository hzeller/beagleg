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
    fprintf(stderr, "Writing first byte to read free slots, and but keep CS low to continue transaction...\n");
    const char command = CMD_WRITE_FIFO;
    uint8_t free_slots;
    if (!spi_channel_->TransferBuffer(&command, &free_slots, 1, false))
      return -1;

    if (free_slots < count) {
      fprintf(stderr, "Available fifo space of %d < requested %d\n",
              free_slots, count);
    }

    const int segment_tx_count = std::min((int)free_slots, count);
    const int segment_byte_len = segment_tx_count * sizeof(beagleg::SpiMotionSegment);
    char rx_buffer[segment_byte_len];   // Let's see what it sends back
    fprintf(stderr, "Sending actual data; %d elements = %d bytes\n",
            segment_tx_count, segment_byte_len);
    if (!spi_channel_->TransferBuffer(segments, rx_buffer, segment_byte_len))
      return -1;
    return segment_tx_count;
  }

private:
  SPIHost *const spi_channel_;
};

SPISegmentQueue::SPISegmentQueue(SPIHost *spi)
  : protocol_handler_(new BeagleGSPIProtocol(spi)) {
}

SPISegmentQueue::~SPISegmentQueue() {
  delete protocol_handler_;
}

bool SPISegmentQueue::Enqueue(const LinearSegmentSteps &segment) {
  fprintf(stderr, "got Enqueue(%d start=%.1f end=%.1f)\n",
          segment.steps[0], segment.v0, segment.v1);
  beagleg::SpiMotionSegment seg;
  seg.count_steps = segment.steps[0];  // just sending motor one for now
  while ((protocol_handler_->SendMotionSegments(&seg, 1)) != 1) {
    /* keep waiting. This will also wiggle the clock enough for eventually
     * having sent the steps.
     */
  }
  return true;
}

void SPISegmentQueue::MotorEnable(bool on) { /* TODO: implement */ }
void SPISegmentQueue::WaitQueueEmpty() { /* TODO: implement */ }
bool SPISegmentQueue::GetPhysicalStatus(PhysicalStatus *status) {
  return false;  // TODO: implement
}
void SPISegmentQueue::SetExternalPosition(int axis, int pos) {
  // TODO: implement.
}
