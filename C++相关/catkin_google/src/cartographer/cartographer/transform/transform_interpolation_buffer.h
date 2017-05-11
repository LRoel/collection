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

#ifndef CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
#define CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_

#include <deque>
#include <memory>

#include "cartographer/common/time.h"
#include "cartographer/mapping/proto/trajectory.pb.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
namespace transform {

// A time-ordered buffer of transforms that supports interpolated lookups.
/**
* @brief 支持内插查找的时间有序的变换缓冲区.
*/
class TransformInterpolationBuffer {
 public:
  static std::unique_ptr<TransformInterpolationBuffer> FromTrajectory(
      const mapping::proto::Trajectory& trajectory);

  // Adds a new transform to the buffer and removes the oldest transform if the
  // buffer size limit is exceeded.
  /**
  * @brief 向缓冲区添加一个新的变换，如果超过缓冲区大小限制，则删除最旧的变换.
  */
  void Push(common::Time time, const transform::Rigid3d& transform);

  // Returns true if an interpolated transfrom can be computed at 'time'.
  /**
  * @brief 如果可以在'时间'计算内插变换，则返回true.
  */
  bool Has(common::Time time) const;

  // Returns an interpolated transform at 'time'. CHECK()s that a transform at
  // 'time' is available.
  /**
  * @brief 返回“时间”的内插变换.CHECK（）表示在'时间'转换是可用的.
  */
  transform::Rigid3d Lookup(common::Time time) const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  /**
  * @brief 返回缓冲区中最早变换的时间戳，如果缓冲区为空则返回0.
  */
  common::Time earliest_time() const;

  // Returns the timestamp of the earliest transform in the buffer or 0 if the
  // buffer is empty.
  /**
  * @brief 返回缓冲区中最早变换的时间戳，如果缓冲区为空则返回0.
  */
  common::Time latest_time() const;

  // Returns true if the buffer is empty.
  /**
  * @brief 如果缓冲区为空，则返回true.
  */
  bool empty() const;

 private:
  struct TimestampedTransform {
    common::Time time;
    transform::Rigid3d transform;
  };

  std::deque<TimestampedTransform> deque_;
};

}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_INTERPOLATION_BUFFER_H_
