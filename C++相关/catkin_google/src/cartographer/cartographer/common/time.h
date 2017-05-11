/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_TIME_H_
#define CARTOGRAPHER_COMMON_TIME_H_

#include <chrono>
#include <ostream>
#include <ratio>

#include "cartographer/common/port.h"

namespace cartographer {
namespace common {

constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
/**
* @brief 代表世界时间段持续时间和时间戳，它们是自从Epoch以来的64位整数，它是从UTC开始的1月1日1开始的100纳秒.
*/
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;

// Convenience functions to create common::Durations.
/**
* @brief 方便功能创建common ::持续时间.
*/
Duration FromSeconds(double seconds);
Duration FromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
/**
* @brief 返回给定的持续时间（以秒为单位）.
*/
double ToSeconds(Duration duration);

// Creates a time from a Universal Time Scale.
/**
* @brief 从世界时间尺度创建一个时间.
*/
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
/**
* @brief 输出给定时间的通用时间标度时间戳.
*/
int64 ToUniversal(Time time);

// For logging and unit tests, outputs the timestamp integer.
/**
* @brief 对于日志记录和单元测试，输出时间戳整数.
*/
std::ostream& operator<<(std::ostream& os, Time time);

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TIME_H_
