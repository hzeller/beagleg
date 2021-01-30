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

SPISegmentQueue::SPISegmentQueue() {
}

bool SPISegmentQueue::Enqueue(const LinearSegmentSteps &segment) {
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
