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

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_

#include <memory>
#include <vector>

#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping_2d/probability_grid.h"
#include "cartographer/mapping_2d/scan_matching/proto/ceres_scan_matcher_options.pb.h"
#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

proto::CeresScanMatcherOptions CreateCeresScanMatcherOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// Align scans with an existing map using Ceres.
/**
* @brief 使用Ceres与现有的地图对齐扫描.
*/
class CeresScanMatcher {
 public:
  explicit CeresScanMatcher(const proto::CeresScanMatcherOptions& options);
  virtual ~CeresScanMatcher();

  CeresScanMatcher(const CeresScanMatcher&) = delete;
  CeresScanMatcher& operator=(const CeresScanMatcher&) = delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate' and returns 'pose_estimate', 'covariance', and
  // the solver 'summary'.
  /**
  * @brief 给定一个'initial_pose_estimate'，在'probability_grid'中对齐'point_cloud'并返回'pose_estimate'，'协方差'和求解器'summary'.
  */
  void Match(const transform::Rigid2d& previous_pose,
             const transform::Rigid2d& initial_pose_estimate,
             const sensor::PointCloud& point_cloud,
             const ProbabilityGrid& probability_grid,
             transform::Rigid2d* pose_estimate,
             kalman_filter::Pose2DCovariance* covariance,
             ceres::Solver::Summary* summary) const;

 private:
  const proto::CeresScanMatcherOptions options_;
  ceres::Solver::Options ceres_solver_options_;
};

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
