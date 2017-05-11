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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_PRECOMPUTATION_GRID_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_PRECOMPUTATION_GRID_H_

#include "cartographer/mapping_3d/hybrid_grid.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

class PrecomputationGrid : public HybridGridBase<uint8> {
 public:
  PrecomputationGrid(const float resolution, const Eigen::Vector3f& origin)
      : HybridGridBase<uint8>(resolution, origin) {}

  // Maps values from [0, 255] to [kMinProbability, kMaxProbability].
  /**
  * @brief 将值从[0，255]映射到[kMinProbability，kMaxProbability].
  */
  static float ToProbability(float value) {
    return mapping::kMinProbability +
           value *
               ((mapping::kMaxProbability - mapping::kMinProbability) / 255.f);
  }
};

// Converts a HybridGrid to a PrecomputationGrid representing the same data,
// but only using 8 bit instead of 2 x 16 bit.
/**
* @brief 将HybridGrid转换为表示相同数据的PrecomputationGrid，但仅使用8位而不是2 x 16位.
*/
PrecomputationGrid ConvertToPrecomputationGrid(const HybridGrid& hybrid_grid);

// Returns a grid of the same resolution containing the maximum value of
// original voxels in 'grid'. This maximum is over the 8 voxels that have
// any combination of index components optionally increased by 'shift'.
/**
* @brief 返回包含“grid”中原始体素的最大值的相同分辨率的网格.这个最大值超过8个体素，它们具有任意组合的索引成分，任选地通过'shift'.
*/
// If 'shift' is 2 ** (depth - 1), where depth 0 is the original grid, and this
// is using the precomputed grid of one depth before, this results in
// precomputation grids analogous to the 2D case.
/**
* @brief 如果'shift'是2 **（depth-1），其中深度0是原始网格，并且这是使用一个深度的预计算网格，则会导致类似于2D情况的预计算网格.
*/
PrecomputationGrid PrecomputeGrid(const PrecomputationGrid& grid,
                                  bool half_resolution,
                                  const Eigen::Array3i& shift);

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_PRECOMPUTATION_GRID_H_
