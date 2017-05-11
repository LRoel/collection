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

#ifndef CARTOGRAPHER_IO_POINTS_BATCH_H_
#define CARTOGRAPHER_IO_POINTS_BATCH_H_

#include <array>
#include <cstdint>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/time.h"

namespace cartographer {
namespace io {

// A point's color.
/**
* @brief 点的颜色.
*/
using Color = std::array<uint8_t, 3>;

// A number of points, captured around the same 'time' and by a
// sensor at the same 'origin'.
/**
* @brief 在同一个“时间”周围捕捉到的点数和传感器在同一个“起点”.
*/
struct PointsBatch {
  PointsBatch() {
    origin = Eigen::Vector3f::Zero();
    trajectory_index = 0;
  }

  // Time at which this batch has been acquired.
  /**
  * @brief 此批次获得的时间.
  */
  common::Time time;

  // Origin of the data, i.e. the location of the sensor in the world at
  // 'time'.
  /**
  * @brief 数据的来源，i.e.传感器在世界上的“时间”位置.
  */
  Eigen::Vector3f origin;

  // Sensor that generated this data's 'frame_id' or empty if this information
  // is unknown.
  /**
  * @brief 如果此信息未知，则生成此数据的“frame_id”的传感器或为空.
  */
  string frame_id;

  // Trajectory index that produced this point.
  /**
  * @brief 产生这一点的轨迹指数.
  */
  int trajectory_index;

  // Geometry of the points in a metric frame.
  /**
  * @brief 公制框架中的点的几何.
  */
  std::vector<Eigen::Vector3f> points;

  // Intensities are optional and may be unspecified. The meaning of these
  // intensity values varies by device. For example, the VLP16 provides values
  // in the range [0, 100] for non-specular return values and values up to 255
  // for specular returns. On the other hand, Hokuyo lasers provide a 16-bit
  // value that rarely peaks above 4096.
  /**
  * @brief 强度是可选的，可能是未指定的.这些强度值的含义因设备而异.例如，VLP16为非镜面返回值的范围[0,100]提供值，镜面返回值最多为255.另一方面，Hokuyo激光器提供16位的值，很难高于4096.
  */
  std::vector<float> intensities;

  // Colors are optional. If set, they are RGB values.
  /**
  * @brief 颜色是可选的.如果设置，它们是RGB值.
  */
  std::vector<Color> colors;
};

// Removes the indices in 'to_remove' from 'batch'.
/**
* @brief 从'批处理'中删除'to_remove'中的索引.
*/
void RemovePoints(std::vector<int> to_remove, PointsBatch* batch);

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_POINTS_BATCH_H_
