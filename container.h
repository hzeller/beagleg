/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2015 Henner Zeller <h.zeller@acm.org>
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

#ifndef _BEAGLEG_CONTAINER_H_
#define _BEAGLEG_CONTAINER_H_

#include <strings.h>
#include <string.h>

template <typename T, int N>
class FixedArray {
public:
  FixedArray() {
    clear();
  }
  FixedArray(const FixedArray<T,N> &other) { CopyFrom(other); }

  FixedArray<T,N> &operator= (const FixedArray<T,N> &other) {
    CopyFrom(other); return *this;
  }
  T &operator[] (int i) { return data_[i]; }
  const T & operator[] (int i) const { return data_[i]; }

  size_t size() const { return N; }

  void clear() { bzero(data_, sizeof(data_)); }
  void CopyFrom(const FixedArray<T,N> &other) {
    memcpy(data_, other.data_, sizeof(data_));
  }

private:
  T data_[N];
};

#endif  // _BEAGLEG_CONTAINER_H_
