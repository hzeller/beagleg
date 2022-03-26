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

#include <assert.h>
#include <string.h>
#include <strings.h>

#include <initializer_list>
#include <memory>

// Fixed array of POD types (that can be zeroed with bzero()).
// Allows to have the index be a specific type (typically an enum instad of
// int). Use for compile-defined small arrays, such as for axes and motors.
template <typename T, int N, typename IDX = int>
class FixedArray {
 public:
  typedef T *iterator;
  typedef const T *const_iterator;

  FixedArray() { zero(); }
  FixedArray(const std::initializer_list<T> &in_list) {
    zero();
    auto it = in_list.begin();
    for (int i = 0; i < N && it != in_list.end(); ++i, ++it) {
      data_[i] = *it;
    }
  }
  FixedArray(const FixedArray<T, N, IDX> &other) { CopyFrom(other); }

  FixedArray<T, N, IDX> &operator=(const FixedArray<T, N, IDX> &other) {
    if (&other == this) return *this;
    CopyFrom(other);
    return *this;
  }
  T &operator[](IDX i) {
    assert((int)i < N);
    return data_[(int)i];
  }
  const T &operator[](IDX i) const {
    assert((int)i < N);
    return data_[(int)i];
  }
  bool operator==(const FixedArray<T, N, IDX> &other) const {
    return memcmp(data_, other.data_, sizeof(data_)) == 0;
  }

  constexpr size_t size() const { return N; }

  void zero() { bzero(data_, sizeof(data_)); }

  iterator begin() { return data_; }
  iterator end() { return data_ + N; }

  const_iterator begin() const { return data_; }
  const_iterator end() const { return data_ + N; }

 private:
  void CopyFrom(const FixedArray<T, N, IDX> &other) {
    if (data_ != other.data_) memcpy(data_, other.data_, sizeof(data_));
  }

  T data_[N];
};

template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&...args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template <typename T, size_t CAPACITY>
struct FixedStoragePolicy {
  static_assert(CAPACITY > 1, "Capacity needs to be > 1");
  // intentionally not initializing memory to better see if users do.
  T buffer[CAPACITY];
  using type = T;
  T *GetBuffer() { return &buffer[0]; }
  static constexpr size_t storage_size = CAPACITY;
};

template <typename T>
struct DynamicStoragePolicy {
  explicit DynamicStoragePolicy(const size_t size)
      : storage_size(size), buffer(std::unique_ptr<T[]>(new T[size])) {
    assert(size > 1 && "Capacity needs to be > 1");
  }

  using type = T;
  size_t storage_size;
  std::unique_ptr<T[]> buffer;
  T *GetBuffer() { return buffer.get(); }
};

template <typename StoragePolicy>
class RingDeque : private StoragePolicy {
 public:
  size_t size() const {
    return (write_pos_ + StoragePolicy::storage_size - read_pos_) %
           StoragePolicy::storage_size;
  }
  bool empty() const { return write_pos_ == read_pos_; }
  constexpr size_t capacity() const { return StoragePolicy::storage_size - 1; }
  using T = typename StoragePolicy::type;
  using StoragePolicy::StoragePolicy;

  // Add a new element and return pointer to it.
  // Element is not initialized.
  T *append() {
    assert(size() < capacity());
    T *result = StoragePolicy::GetBuffer() + write_pos_;
    write_pos_ = (write_pos_ + 1) % StoragePolicy::storage_size;
    return result;
  }

  // Return the content relative to the read position.
  T *operator[](size_t pos) {
    assert(size() > pos);
    return &StoragePolicy::GetBuffer()[(read_pos_ + pos) %
                                       StoragePolicy::storage_size];
  }

  // Return last inserted position
  T *back() {
    assert(!empty());
    const auto storage_size = StoragePolicy::storage_size;
    return &StoragePolicy::GetBuffer()[(write_pos_ + storage_size - 1) %
                                       storage_size];
  }

  void pop_front() {
    assert(!empty());
    read_pos_ = (read_pos_ + 1) % StoragePolicy::storage_size;
  }

  void pop_back() {
    assert(!empty());
    const auto storage_size = StoragePolicy::storage_size;
    write_pos_ = (write_pos_ + storage_size - 1) % storage_size;
  }

 private:
  unsigned write_pos_ = 0;
  unsigned read_pos_ = 0;
};

// A simple fixed size, compile-time allocated deque.
// Any positiv CAPACITY parameter above 1 should work. Choosing a power of
// two will typically generate faster code as modulo ops can become bit-AND ops.
//
// For a cheap empty() and size() implementation, the actual capacity() is one
// less than the chosen CAPACITY template parameter.
template <typename T, size_t CAPACITY>
using FixedRingDeque = RingDeque<FixedStoragePolicy<T, CAPACITY>>;

// Identical to the FixedRingDeque but runtime-allocated.
template <typename T>
using DynamicRingDeque = RingDeque<DynamicStoragePolicy<T>>;

// This class provides a way to iterate over enumeration values. Assumes enum
// values to be contiguous.
template <typename T, T first_val, T after_last_val>
class EnumIterable {
 public:
  class const_iterator {
   public:
    explicit const_iterator(T value) : val_(value) {}
    T operator*() const { return val_; }
    void operator++() { val_ = static_cast<T>(val_ + 1); }
    bool operator!=(const const_iterator &other) { return other.val_ != val_; }

   private:
    T val_;
  };

  const_iterator begin() const { return const_iterator(first_val); }
  const_iterator end() const { return const_iterator(after_last_val); }
};
#endif  // _BEAGLEG_CONTAINER_H_
