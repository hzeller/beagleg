/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2022 Henner Zeller <h.zeller@acm.org>
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

#include "common/container.h"

#include <gtest/gtest.h>

TEST(FixedArray, BasicOp) {
  FixedArray<int, 3> buffer;
  EXPECT_EQ(buffer.size(), 3u);  // one less than CAPACITY
#if __cplusplus > 201400L
  static_assert(buffer.size() == 3);  // >= c++14: compile-time constexpr
#endif
  buffer[0] = 1;
  buffer[1] = 2;
  buffer[2] = 3;

  int count = 0;
  int sum = 0;
  for (int val : buffer) {
    count++;
    sum += val;
  }
  EXPECT_EQ(count, 3);
  EXPECT_EQ(sum, 1 + 2 + 3);
}

// Wrapper to have a uniform template signature.
template <typename T, int CAPACITY>
struct DynamicRingDequeWrapper : public DynamicRingDeque<T> {
  explicit DynamicRingDequeWrapper() : DynamicRingDeque<T>(CAPACITY) {}
};
template <typename T, int CAPACITY>
struct RingDequesBuilder {
  using Dynamic = DynamicRingDequeWrapper<T, CAPACITY>;
  using Fixed = FixedRingDeque<T, CAPACITY>;
};

using RingDeques = RingDequesBuilder<int, 4>;

using RingDequeTypes = testing::Types<RingDeques::Dynamic, RingDeques::Fixed>;

template <typename T>
struct RingDequeTest : public testing::Test {
  using RingDequeType = T;
};

TYPED_TEST_SUITE(RingDequeTest, RingDequeTypes);

TYPED_TEST(RingDequeTest, BasicOp) {
  using RingDeque = typename TestFixture::RingDequeType;
  auto buffer = RingDeque();

  EXPECT_TRUE(buffer.empty());
  EXPECT_EQ(buffer.size(), 0u);

  *buffer.append() = 42;
  EXPECT_EQ(buffer.size(), 1u);
  EXPECT_FALSE(buffer.empty());
  EXPECT_EQ(*buffer.back(), 42);

  *buffer.append() = 1;
  EXPECT_EQ(buffer.size(), 2u);
  EXPECT_EQ(*buffer.back(), 1);
  buffer.pop_back();
  EXPECT_EQ(*buffer.back(), 42);
  buffer.pop_front();
  EXPECT_TRUE(buffer.empty());
}

TYPED_TEST(RingDequeTest, Wrapping) {
  using RingDeque = typename TestFixture::RingDequeType;
  auto buffer = RingDeque();

  EXPECT_EQ(buffer.capacity(), 3u);  // one less than CAPACITY
#if __cplusplus > 201400L
  if constexpr (std::is_same<RingDeque, RingDeques::Fixed>::value) {
    static_assert(buffer.capacity() == 3);  // >= c++14: compile-time constexpr
  }
#endif

  // Advance the internal positions so that we force wrapping.
  *buffer.append() = 42;
  *buffer.append() = 42;
  buffer.pop_front();
  buffer.pop_front();

  *buffer.append() = 1;
  *buffer.append() = 2;
  *buffer.append() = 3;
  EXPECT_EQ(buffer.size(), buffer.capacity());

  EXPECT_EQ(*buffer[0], 1);
  EXPECT_EQ(*buffer[1], 2);
  EXPECT_EQ(*buffer[2], 3);
}

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
