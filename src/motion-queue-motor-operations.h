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
#ifndef MOTION_QUEUE_MOTOR_OPERATIONS_H
#define MOTION_QUEUE_MOTOR_OPERATIONS_H

#include <deque>

#include "hardware-mapping.h"
#include "motion-queue.h"
#include "segment-queue.h"

class MotionQueueMotorOperations : public SegmentQueue {
public:
  // Initialize motor operations, sending planned results into the motion backend.
  MotionQueueMotorOperations(HardwareMapping *hw, MotionQueue *backend);
  ~MotionQueueMotorOperations() override;

  bool Enqueue(const LinearSegmentSteps &segment) final;
  void MotorEnable(bool on) final;
  void WaitQueueEmpty() final;
  bool GetPhysicalStatus(PhysicalStatus *status) final;
  void SetExternalPosition(int axis, int pos) final;

private:
  bool EnqueueInternal(const LinearSegmentSteps &param,
                       int defining_axis_steps);

  HardwareMapping *const hardware_mapping_;
  MotionQueue *backend_;

  struct HistorySegment;
  std::deque<struct HistorySegment> *shadow_queue_;
};

#endif // MOTION_QUEUE_MOTOR_OPERATIONS_H
