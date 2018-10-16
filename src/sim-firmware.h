/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2014 Henner Zeller <h.zeller@acm.org>
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

#include "motion-queue.h"

#include <stdio.h>

class SimFirmwareQueue : public MotionQueue {
public:
  SimFirmwareQueue(FILE *out, int relevant_motors = MOTION_MOTOR_COUNT);
  ~SimFirmwareQueue() override;

  bool Enqueue(MotionSegment *segment) final;
  void WaitQueueEmpty() final {}
  void MotorEnable(bool on) final {}
  void Shutdown(bool flush_queue) final {}
  int GetPendingElements(uint32_t *head_item_progress) final {
    if (head_item_progress)
      *head_item_progress = 0;
    return 1;
  }

private:
  class Averager;

  FILE *const out_;
  const int relevant_motors_;
  Averager *const averager_;
};
