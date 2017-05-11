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

#ifndef CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_
#define CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/io/points_processor.h"
#include "cartographer/mapping_3d/hybrid_grid.h"

namespace cartographer {
namespace io {

// Voxel filters the data and only passes on points that we believe are on
// non-moving objects.
/**
* @brief 体素过滤数据，只传递我们相信在非移动物体上的点.
*/
class OutlierRemovingPointsProcessor : public PointsProcessor {
 public:
  constexpr static const char* kConfigurationFileActionName =
      "voxel_filter_and_remove_moving_objects";

  OutlierRemovingPointsProcessor(double voxel_size, PointsProcessor* next);

  static std::unique_ptr<OutlierRemovingPointsProcessor> FromDictionary(
      common::LuaParameterDictionary* dictionary, PointsProcessor* next);

  ~OutlierRemovingPointsProcessor() override {}

  OutlierRemovingPointsProcessor(const OutlierRemovingPointsProcessor&) =
      delete;
  OutlierRemovingPointsProcessor& operator=(
      const OutlierRemovingPointsProcessor&) = delete;

  void Process(std::unique_ptr<PointsBatch> batch) override;
  FlushResult Flush() override;

 private:
  // To reduce memory consumption by not having to keep all rays in memory, we
  // filter outliers in three phases each going over all data: First we compute
  // all voxels containing any hits, then we compute the rays passing through
  // each of these voxels, and finally we output all hits in voxels that are
  // considered obstructed.
  /**
  * @brief 为了通过不必将所有光线保持在内存中来减少内存消耗，我们将过滤异常值分为三个阶段，每个过程遍历所有数据：首先，我们计算包含任何命中的所有体素，然后计算通过每个这些体素的射线，最后我们.
  */
  struct VoxelData {
    int hits = 0;
    int rays = 0;
  };
  enum class State {
    kPhase1,
    kPhase2,
    kPhase3,
  };

  // First phase counts the number of hits per voxel.
  /**
  * @brief 第一阶段计算每个体素的命中次数.
  */
  void ProcessInPhaseOne(const PointsBatch& batch);

  // Second phase counts how many rays pass through each voxel. This is only
  // done for voxels that contain hits. This is to reduce memory consumption by
  // not adding data to free voxels.
  /**
  * @brief 第二阶段计数多少光线通过每个体素.这仅适用于包含点击的体素.这是为了通过不向自由体素中添加数据来减少内存消耗.
  */
  void ProcessInPhaseTwo(const PointsBatch& batch);

  // Third phase produces the output containing all inliers. We consider each
  // hit an inlier if it is inside a voxel that has a sufficiently high
  // hit-to-ray ratio.
  /**
  * @brief 第三阶段产生包含所有内部值的输出.我们认为，如果它在一个具有足够高的命中与射线比的体素内，则每个命中都是一个非常高的.
  */
  void ProcessInPhaseThree(std::unique_ptr<PointsBatch> batch);

  const double voxel_size_;
  PointsProcessor* const next_;
  State state_;
  mapping_3d::HybridGridBase<VoxelData> voxels_;
};

}  // namespace io
}  // namespace cartographer

#endif  // CARTOGRAPHER_IO_OUTLIER_REMOVING_POINTS_PROCESSOR_H_
