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

#ifndef CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_
#define CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h"

namespace cartographer {
namespace sensor {

// Returns a voxel filtered copy of 'point_cloud' where 'size' is the length
// a voxel edge.
/**
* @brief 返回体素过滤的“point_cloud”副本，其中“size”是体元边缘的长度.
*/
PointCloud VoxelFiltered(const PointCloud& point_cloud, float size);

// Voxel filter for point clouds. For each voxel, the assembled point cloud
// contains the first point that fell into it from any of the inserted point
// clouds.
/**
* @brief 点云的体素过滤器.对于每个体素，组合的点云包含从任何插入的点云落入其中的第一个点.
*/
class VoxelFilter {
 public:
  // 'size' is the length of a voxel edge.
  /**
  * @brief “大小”是体素边缘的长度.
  */
  explicit VoxelFilter(float size);

  VoxelFilter(const VoxelFilter&) = delete;
  VoxelFilter& operator=(const VoxelFilter&) = delete;

  // Inserts a point cloud into the voxel filter.
  /**
  * @brief 将点云插入到体素过滤器中.
  */
  void InsertPointCloud(const PointCloud& point_cloud);

  // Returns the filtered point cloud representing the occupied voxels.
  /**
  * @brief 返回表示占用体素的过滤点云.
  */
  const PointCloud& point_cloud() const;

 private:
  mapping_3d::HybridGridBase<uint8> voxels_;
  PointCloud point_cloud_;
};

proto::AdaptiveVoxelFilterOptions CreateAdaptiveVoxelFilterOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

class AdaptiveVoxelFilter {
 public:
  explicit AdaptiveVoxelFilter(
      const proto::AdaptiveVoxelFilterOptions& options);

  AdaptiveVoxelFilter(const AdaptiveVoxelFilter&) = delete;
  AdaptiveVoxelFilter& operator=(const AdaptiveVoxelFilter&) = delete;

  PointCloud Filter(const PointCloud& point_cloud) const;

 private:
  const proto::AdaptiveVoxelFilterOptions options_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_VOXEL_FILTER_H_
