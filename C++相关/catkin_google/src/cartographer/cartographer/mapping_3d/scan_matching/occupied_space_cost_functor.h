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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTOR_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTOR_H_

#include "Eigen/Core"
#include "cartographer/mapping_3d/hybrid_grid.h"
#include "cartographer/mapping_3d/scan_matching/interpolated_grid.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

// Computes the cost of inserting occupied space described by the point cloud
// into the map. The cost increases with the amount of free space that would be
// replaced by occupied space.
/**
* @brief 计算将点云描述的占用空间插入到地图中的成本.成本随着被占用空间取代的可用空间量的增加而增加.
*/
class OccupiedSpaceCostFunctor {
 public:
  // Creates an OccupiedSpaceCostFunctor using the specified grid, 'rotation' to
  // add to all poses, and point cloud.
  /**
  * @brief 使用指定的网格创建一个OccupiedSpaceCostFunctor，“旋转”添加到所有姿势和点云.
  */
  OccupiedSpaceCostFunctor(const double scaling_factor,
                           const sensor::PointCloud& point_cloud,
                           const HybridGrid& hybrid_grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        interpolated_grid_(hybrid_grid) {}

  OccupiedSpaceCostFunctor(const OccupiedSpaceCostFunctor&) = delete;
  OccupiedSpaceCostFunctor& operator=(const OccupiedSpaceCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const translation, const T* const rotation,
                  T* const residual) const {
    const transform::Rigid3<T> transform(
        Eigen::Map<const Eigen::Matrix<T, 3, 1>>(translation),
        Eigen::Quaternion<T>(rotation[0], rotation[1], rotation[2],
                             rotation[3]));
    return Evaluate(transform, residual);
  }

  template <typename T>
  bool Evaluate(const transform::Rigid3<T>& transform,
                T* const residual) const {
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      const Eigen::Matrix<T, 3, 1> world =
          transform * point_cloud_[i].cast<T>();
      const T probability =
          interpolated_grid_.GetProbability(world[0], world[1], world[2]);
      residual[i] = scaling_factor_ * (1. - probability);
    }
    return true;
  }

 private:
  const double scaling_factor_;
  const sensor::PointCloud& point_cloud_;
  const InterpolatedGrid interpolated_grid_;
};

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_OCCUPIED_SPACE_COST_FUNCTOR_H_
