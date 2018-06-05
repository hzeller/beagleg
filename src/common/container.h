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
#include <assert.h>
#include <initializer_list>

// Fixed array of POD types (that can be zeroed with bzero()).
// Allows to have the index be a specific type (typically an enum instad of int).
// Use for compile-defined small arrays, such as for axes and motors.
template <typename T, int N, typename IDX = int>
class FixedArray {
public:
  typedef T* iterator;
  typedef const T* const_iterator;

  FixedArray() { zero(); }
  FixedArray(const std::initializer_list<T> &in_list) {
    zero();
    auto it = in_list.begin();
    for (int i = 0; i < N && it != in_list.end(); ++i, ++it) {
      data_[i] = *it;
    }
  }
  FixedArray(const FixedArray<T,N,IDX> &other) { CopyFrom(other); }

  FixedArray<T,N,IDX> &operator= (const FixedArray<T,N,IDX> &other) {
    CopyFrom(other); return *this;
  }
  T &operator[] (IDX i) { assert((int)i < N); return data_[(int)i]; }
  const T & operator[] (IDX i) const {
    assert((int)i < N);
    return data_[(int)i];
  }
  bool operator== (const FixedArray<T,N,IDX> &other) const {
    return memcmp(data_, other.data_, sizeof(data_)) == 0;
  }

  size_t size() const { return N; }

  void zero() { bzero(data_, sizeof(data_)); }

  iterator begin() { return data_; }
  iterator end() { return data_ + N; }

  const_iterator begin() const { return data_; }
  const_iterator end() const { return data_ + N; }

private:
  void CopyFrom(const FixedArray<T,N,IDX> &other) {
    if (data_ != other.data_)
      memcpy(data_, other.data_, sizeof(data_));
  }

  T data_[N];
};

// A simple fixed size, compile-time allocated deque.
template <typename T, int CAPACITY>
class RingDeque {
public:
  RingDeque() : write_pos_(0), read_pos_(0) {
    // intentionally not initializing memory to better see if users do.
  }

  size_t size() const {
    return (write_pos_ + CAPACITY - read_pos_) % CAPACITY;
  }

  // Add a new element and return pointer to it.
  // Element is not initialized.
  T* append() {
    assert(size() < CAPACITY - 1);
    T *result = buffer_ + write_pos_;
    write_pos_ = (write_pos_ + 1) % CAPACITY;
    return result;
  }

  // Return the content relative to the read position.
  T* operator [] (size_t pos) {
    assert(size() > pos);
    return &buffer_[(read_pos_ + pos) % CAPACITY];
  }

  // Return last inserted position
  T* back() {
    assert(size() > 0);
    return &buffer_[(write_pos_ + CAPACITY - 1) % CAPACITY];
  }

  void pop_front() {
    assert(size() > 0);
    read_pos_ = (read_pos_ + 1) % CAPACITY;
  }

  void pop_back() {
    assert(size() > 0);
    write_pos_ = (write_pos_ + CAPACITY - 1) % CAPACITY;
  }

private:
  unsigned write_pos_;
  unsigned read_pos_;
  T buffer_[CAPACITY];
};


// This class provides a way to iterate over enumeration values. Assumes enum
// values to be contiguous.
template<typename T, T first_val, T after_last_val>
class EnumIterable {
public:
  class const_iterator {
  public:
    const_iterator(T value) : val_(value) {}
    T operator* () const { return val_; }
    void operator++ () { val_ = static_cast<T>(val_ + 1); }
    bool operator!= (const const_iterator& other) { return other.val_ != val_; }

  private:
    T val_;
  };

  const_iterator begin() const { return const_iterator(first_val); }
  const_iterator end() const { return const_iterator(after_last_val); }
};
#endif  // _BEAGLEG_CONTAINER_H_
