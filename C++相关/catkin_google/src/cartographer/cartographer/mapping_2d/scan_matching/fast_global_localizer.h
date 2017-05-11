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

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_GLOBAL_LOCALIZER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_GLOBAL_LOCALIZER_H_

#include <vector>

#include "Eigen/Geometry"
#include "cartographer/mapping_2d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/sensor/voxel_filter.h"

namespace cartographer {
namespace mapping_2d {
namespace scan_matching {

// Perform global localization against the provided 'matchers'. The 'cutoff'
// specifies the minimum correlation that will be accepted.
/**
* @brief 对所提供的“匹配者”进行全球本地化.“截止”指定将被接受的最小相关性.
*/
// This function does not take ownership of the pointers passed in
// 'matchers'; they are passed as a vector of raw pointers to give maximum
// flexibility to callers.
/**
* @brief 该函数不占用“匹配器”中传递的指针的所有权;.
*/
//
// Returns true in the case of successful localization. The output parameters
// should not be trusted if the function returns false. The 'cutoff' and
// 'best_score' are in the range [0.0, 1.0].
/**
* @brief 在本地化成功的情况下返回true.如果函数返回false，输出参数不应该被信任.'cutoff'和'best_score'在[0.0，1.0].
*/
bool PerformGlobalLocalization(
    float cutoff, const cartographer::sensor::AdaptiveVoxelFilter& voxel_filter,
    const std::vector<
        cartographer::mapping_2d::scan_matching::FastCorrelativeScanMatcher*>&
        matchers,
    const cartographer::sensor::PointCloud& point_cloud,
    transform::Rigid2d* best_pose_estimate, float* best_score);

}  // namespace scan_matching
}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_FAST_GLOBAL_LOCALIZER_H_
