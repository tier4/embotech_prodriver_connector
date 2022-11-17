// Copyright 2022 The Autoware Foundation.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "embotech_prodriver_connector/embotech_prodriver_connector_utils.hpp"

// gtest
#include <gtest/gtest.h>

#include <limits>

// Copied from grid_map::PolygonIterator
TEST(Utils, TimeConversion)
{
  for (const auto original_time :
       {rclcpp::Time(10, 20), rclcpp::Time(10 * 1e9 + 20),
        rclcpp::Time(std::numeric_limits<int32_t>::max() - 1, 999999999)}) {
    const auto ptcl_time = embotech_prodriver_connector::fromRosTime(original_time);
    const auto ros_time = embotech_prodriver_connector::toRosTime(ptcl_time);
    EXPECT_NEAR(ros_time.seconds(), original_time.seconds(), 1e-6);
    EXPECT_NEAR(ros_time.nanoseconds(), original_time.nanoseconds(), 1e6);
  }
}
