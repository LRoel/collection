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

#ifndef CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_
#define CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_

#include <iterator>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"

namespace cartographer {
namespace sensor {

// A compressed representation of a point cloud consisting of a collection of
// points (Vector3f).
/**
* @brief 由点集合（Vector3f）组成的点云的压缩表示.
*/
// Internally, points are grouped by blocks. Each block encodes a bit of meta
// data (number of points in block, coordinates of the block) and encodes each
// point with a fixed bit rate in relation to the block.
/**
* @brief 在内部，点按块分组.每个块对一个元数据（块的点数，块的坐标）进行编码，并且以相对于块的固定比特率对每个点进行编码.
*/
class CompressedPointCloud {
 public:
  class ConstIterator;

  CompressedPointCloud() : num_points_(0) {}
  explicit CompressedPointCloud(const PointCloud& point_cloud);

  // Returns a compressed point cloud and further returns a mapping 'new_to_old'
  // from the compressed indices to the original indices, i.e., conceptually
  // compressed[i] = point_cloud[new_to_old[i]].
  /**
  * @brief 返回压缩点云，并进一步从压缩索引返回映射“new_to_old”到原始索引i.e.，概念压缩[i] = point_cloud [new_to_old [i]].
  */
  static CompressedPointCloud CompressAndReturnOrder(
      const PointCloud& point_cloud, std::vector<int>* new_to_old);

  // Returns decompressed point cloud.
  /**
  * @brief 返回解压缩点云.
  */
  PointCloud Decompress() const;

  bool empty() const;
  size_t size() const;
  ConstIterator begin() const;
  ConstIterator end() const;

  proto::CompressedPointCloud ToProto() const;

 private:
  CompressedPointCloud(const std::vector<int32>& point_data, size_t num_points);

  const std::vector<int32> point_data_;
  const size_t num_points_;
};

// Forward iterator for compressed point clouds.
/**
* @brief 压缩点云的前向迭代器.
*/
class CompressedPointCloud::ConstIterator
    : public std::iterator<std::forward_iterator_tag, Eigen::Vector3f> {
 public:
  // Creates begin iterator.
  /**
  * @brief 创建开始迭代器.
  */
  explicit ConstIterator(const CompressedPointCloud* compressed_point_cloud);

  // Creates end iterator.
  /**
  * @brief 创建结束迭代器.
  */
  static ConstIterator EndIterator(
      const CompressedPointCloud* compressed_point_cloud);

  Eigen::Vector3f operator*() const;
  ConstIterator& operator++();
  bool operator!=(const ConstIterator& it) const;

 private:
  // Reads next point from buffer. Also handles reading the meta data of the
  // next block, if the current block is depleted.
  /**
  * @brief 从缓冲区读取下一个点.如果当前块耗尽，还可以处理读取下一个块的元数据.
  */
  void ReadNextPoint();

  const CompressedPointCloud* compressed_point_cloud_;
  size_t remaining_points_;
  int32 remaining_points_in_current_block_;
  Eigen::Vector3f current_point_;
  Eigen::Vector3i current_block_coordinates_;
  std::vector<int32>::const_iterator input_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COMPRESSED_POINT_CLOUD_H_
