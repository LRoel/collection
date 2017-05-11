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

// This is an implementation of the algorithm described in "Real-Time
// Correlative Scan Matching" by Olson.
/**
* @brief 这是Olson的“Real-Time Correlative Scan Matching”中描述的算法的实现.
*/
//
// It is similar to the RealTimeCorrelativeScanMatcher but has a different
// trade-off: Scan matching is faster because more effort is put into the
// precomputation done for a given map. However, this map is immutable after
// construction.
/**
* @brief 它类似于RealTimeCorrelativeScanMatcher，但具有不同的权衡：扫描匹配更快，因为对于给定的映射完成的预计算更多的努力.但是，这张地图在施工后是不可变的.
*/

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/port.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/scan_matching/correlative_scan_matcher.h"
#include "cartographer/mapping_2d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h"
#include "cartographer/sensor/point_cloud.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

proto::FastCorrelativeScanMatcherOptions
CreateFastCorrelativeScanMatcherOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// A precomputed grid that contains in each cell (x0, y0) the maximum
// probability in the width x width area defined by x0 <= x < x0 + width and
// y0 <= y < y0.
/**
* @brief 在每个单元格（x0，y0）中包含由x0 <= x <x0 + width和y0 <= y <y0定义的宽度x宽度区域的最大概率的预计算网格.
*/
class PrecomputationGrid {
 public:
  PrecomputationGrid(const ProbabilityGrid& probability_grid,
                     const CellLimits& limits, int width,
                     std::vector<float>* reusable_intermediate_grid);

  // Returns a value between 0 and 255 to represent probabilities between
  // kMinProbability and kMaxProbability.
  /**
  * @brief 返回0到255之间的值，以表示kMinProbability和kMaxProbability之间的概率.
  */
  int GetValue(const Eigen::Array2i& xy_index) const {
    const Eigen::Array2i local_xy_index = xy_index - offset_;
    // The static_cast<unsigned> is for performance to check with 2 comparisons
    // xy_index.x() < offset_.x() || xy_index.y() < offset_.y() ||
    // local_xy_index.x() >= wide_limits_.num_x_cells ||
    // local_xy_index.y() >= wide_limits_.num_y_cells
    // instead of using 4 comparisons.
    /**
    * @brief static_cast <unsigned>用于通过2对比xy_index来检查性能.x（）<offset.x（）|| .y（）<offset.y（）|| .x（）> = wide_limits_.num_x_cells || .y（）> = wide_limits_.num_y_cells而不是使用4个比较.
    */
    if (static_cast<unsigned>(local_xy_index.x()) >=
            static_cast<unsigned>(wide_limits_.num_x_cells) ||
        static_cast<unsigned>(local_xy_index.y()) >=
            static_cast<unsigned>(wide_limits_.num_y_cells)) {
      return 0;
    }
    const int stride = wide_limits_.num_x_cells;
    return cells_[local_xy_index.x() + local_xy_index.y() * stride];
  }

  // Maps values from [0, 255] to [kMinProbability, kMaxProbability].
  /**
  * @brief 将值从[0，255]映射到[kMinProbability，kMaxProbability].
  */
  static float ToProbability(float value) {
    return mapping::kMinProbability +
           value *
               ((mapping::kMaxProbability - mapping::kMinProbability) / 255.f);
  }

 private:
  uint8 ComputeCellValue(float probability) const;

  // Offset of the precomputation grid in relation to the 'probability_grid'
  // including the additional 'width' - 1 cells.
  /**
  * @brief 预计算网格相对于'probability_grid'的偏移包括额外的'width' -  1个单元格.
  */
  const Eigen::Array2i offset_;

  // Size of the precomputation grid.
  /**
  * @brief 预计算网格的大小.
  */
  const CellLimits wide_limits_;

  // Probabilites mapped to 0 to 255.
  /**
  * @brief Probabilit映射到0到25​​5.
  */
  std::vector<uint8> cells_;
};

class PrecomputationGridStack;

// An implementation of "Real-Time Correlative Scan Matching" by Olson.
/**
* @brief 实施Olson的“实时相关扫描匹配”.
*/
class FastCorrelativeScanMatcher {
 public:
  FastCorrelativeScanMatcher(
      const ProbabilityGrid& probability_grid,
      const proto::FastCorrelativeScanMatcherOptions& options);
  ~FastCorrelativeScanMatcher();

  FastCorrelativeScanMatcher(const FastCorrelativeScanMatcher&) = delete;
  FastCorrelativeScanMatcher& operator=(const FastCorrelativeScanMatcher&) =
      delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate'. If a score above 'min_score' (excluding equality)
  // is possible, true is returned, and 'score' and 'pose_estimate' are updated
  // with the result.
  /**
  * @brief 在'probability_grid'中给出'initial_pose_estimate'来对齐'point_cloud'.如果超过'min_score'（不包括平等）的分数是可能的，则返回true，并且使用结果更新'score'和'pose_estimate'.
  */
  bool Match(const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud, float min_score,
             float* score, transform::Rigid2d* pose_estimate) const;

  // Aligns 'point_cloud' within the full 'probability_grid', i.e., not
  // restricted to the configured search window. If a score above 'min_score'
  // (excluding equality) is possible, true is returned, and 'score' and
  // 'pose_estimate' are updated with the result.
  /**
  * @brief 在完整的'probability_grid'内对齐'point_cloud'，i.e.，不限于配置的搜索窗口.如果超过'min_score'（不包括平等）的分数是可能的，则返回true，并且使用结果更新'score'和'pose_estimate'.
  */
  bool MatchFullSubmap(const sensor::PointCloud& point_cloud, float min_score,
                       float* score, transform::Rigid2d* pose_estimate) const;

 private:
  // The actual implementation of the scan matcher, called by Match() and
  // MatchFullSubmap() with appropriate 'initial_pose_estimate' and
  // 'search_parameters'.
  /**
  * @brief 由Match（）和MatchFullSubmap（）调用的扫描匹配器的实际实现与适当的'initial_pose_estimate'和'search_parameters'.
  */
  bool MatchWithSearchParameters(
      SearchParameters search_parameters,
      const transform::Rigid2d& initial_pose_estimate,
      const sensor::PointCloud& point_cloud, float min_score, float* score,
      transform::Rigid2d* pose_estimate) const;
  std::vector<Candidate> ComputeLowestResolutionCandidates(
      const std::vector<DiscreteScan>& discrete_scans,
      const SearchParameters& search_parameters) const;
  std::vector<Candidate> GenerateLowestResolutionCandidates(
      const SearchParameters& search_parameters) const;
  void ScoreCandidates(const PrecomputationGrid& precomputation_grid,
                       const std::vector<DiscreteScan>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate>* const candidates) const;
  Candidate BranchAndBound(const std::vector<DiscreteScan>& discrete_scans,
                           const SearchParameters& search_parameters,
                           const std::vector<Candidate>& candidates,
                           int candidate_depth, float min_score) const;

  const proto::FastCorrelativeScanMatcherOptions options_;
  MapLimits limits_;
  std::unique_ptr<PrecomputationGridStack> precomputation_grid_stack_;
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_CORRELATIVE_SCAN_MATCHER_H_
