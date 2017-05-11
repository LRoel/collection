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

#ifndef CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
#define CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_

#include <array>
#include <deque>
#include <functional>
#include <limits>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/histogram.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/trajectory_connectivity.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h"
#include "cartographer/mapping_3d/scan_matching/fast_correlative_scan_matcher.h"
#include "cartographer/mapping_3d/sparse_pose_graph/optimization_problem.h"
#include "cartographer/mapping_3d/submaps.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/voxel_filter.h"

namespace cartographer {
namespace mapping_3d {
namespace sparse_pose_graph {

// Asynchronously computes constraints.
/**
* @brief 异步计算约束.
*/
//
// Intermingle an arbitrary number of calls to MaybeAddConstraint() or
// MaybeAddGlobalConstraint, then call WhenDone(). After all computations are
// done the 'callback' will be called with the result and another
// MaybeAdd(Global)Constraint()/WhenDone() cycle can follow.
/**
* @brief 混合任意数量的调用MaybeAddConstraint（）或MaybeAddGlobalConstraint，然后调用WhenDone（）.所有的计算完成后，将使用结果调用“回调”，并且可以跟随另一个MaybeAdd（全局）约束（）/ WhenDone（）循环）.
*/
//
// This class is thread-safe.
/**
* @brief 这个类是线程安全的.
*/
class ConstraintBuilder {
 public:
  using Constraint = mapping::SparsePoseGraph::Constraint;
  using Result = std::vector<Constraint>;

  ConstraintBuilder(
      const mapping::sparse_pose_graph::proto::ConstraintBuilderOptions&
          options,
      common::ThreadPool* thread_pool);
  ~ConstraintBuilder();

  ConstraintBuilder(const ConstraintBuilder&) = delete;
  ConstraintBuilder& operator=(const ConstraintBuilder&) = delete;

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_index', and the 'range_data_3d.returns' in 'trajectory_nodes' for
  // 'scan_index'. The 'initial_relative_pose' is relative to the 'submap'.
  /**
  * @brief 在“submap_index”标识的'submap'和'range_data_3d.为'scan_index'返回''.'initial_relative_pose'是相对于'submap'.
  */
  //
  // The pointees of 'submap' and 'range_data_3d.returns' must stay valid until
  // all computations are finished.
  /**
  * @brief “submap”和“range_data_3d”的所有者.返回“必须保持有效，直到所有计算完成.
  */
  void MaybeAddConstraint(
      int submap_index, const Submap* submap, int scan_index,
      const std::vector<mapping::TrajectoryNode>& trajectory_nodes,
      const transform::Rigid3d& initial_relative_pose);

  // Schedules exploring a new constraint between 'submap' identified by
  // 'submap_index' and the 'range_data_3d.returns' in 'trajectory_nodes' for
  // 'scan_index'. This performs full-submap matching.
  /**
  * @brief 在“submap_index”和“range_data_3d”标识的'submap'之间探索新约束的时间表.为'scan_index'返回''.这执行完全子映射匹配.
  */
  //
  // The scan at 'scan_index' should be from trajectory 'scan_trajectory', and
  // the 'submap' should be from 'submap_trajectory'. The
  // 'trajectory_connectivity' is updated if the full-submap match succeeds.
  /**
  * @brief “scan_index”上的扫描应来自轨迹'scan_trajectory'，'submap'应该来自'submap_trajectory'.如果完全子映射匹配成功，则会更新'tracks_connectivity'.
  */
  //
  // The pointees of 'submap' and 'range_data_3d.returns' must stay valid until
  // all computations are finished.
  /**
  * @brief “submap”和“range_data_3d”的所有者.返回“必须保持有效，直到所有计算完成.
  */
  void MaybeAddGlobalConstraint(
      int submap_index, const Submap* submap, int scan_index,
      const mapping::Submaps* scan_trajectory,
      const mapping::Submaps* submap_trajectory,
      mapping::TrajectoryConnectivity* trajectory_connectivity,
      const std::vector<mapping::TrajectoryNode>& trajectory_nodes);

  // Must be called after all computations related to 'scan_index' are added.
  /**
  * @brief 在添加与“scan_index”相关的所有计算后必须调用.
  */
  void NotifyEndOfScan(int scan_index);

  // Registers the 'callback' to be called with the results, after all
  // computations triggered by MaybeAddConstraint() have finished.
  /**
  * @brief 在由MaybeAddConstraint（）触发的所有计算完成后，使用结果注册要调用的“回调”.
  */
  void WhenDone(std::function<void(const Result&)> callback);

  // Returns the number of consecutive finished scans.
  /**
  * @brief 返回连续完成扫描的次数.
  */
  int GetNumFinishedScans();

 private:
  struct SubmapScanMatcher {
    const HybridGrid* hybrid_grid;
    std::unique_ptr<scan_matching::FastCorrelativeScanMatcher>
        fast_correlative_scan_matcher;
  };

  // Either schedules the 'work_item', or if needed, schedules the scan matcher
  // construction and queues the 'work_item'.
  /**
  * @brief 调度“work_item”，或者如果需要，安排扫描匹配器构造并排队“work_item”.
  */
  void ScheduleSubmapScanMatcherConstructionAndQueueWorkItem(
      int submap_index,
      const std::vector<mapping::TrajectoryNode>& submap_nodes,
      const HybridGrid* submap, std::function<void()> work_item)
      REQUIRES(mutex_);

  // Constructs the scan matcher for a 'submap', then schedules its work items.
  /**
  * @brief 为“子图”构建扫描匹配器，然后调度其工作项.
  */
  void ConstructSubmapScanMatcher(
      int submap_index,
      const std::vector<mapping::TrajectoryNode>& submap_nodes,
      const HybridGrid* submap) EXCLUDES(mutex_);

  // Returns the scan matcher for a submap, which has to exist.
  /**
  * @brief 返回必须存在的子映射的扫描匹配器.
  */
  const SubmapScanMatcher* GetSubmapScanMatcher(int submap_index)
      EXCLUDES(mutex_);

  // Runs in a background thread and does computations for an additional
  // constraint, assuming 'submap' and 'point_cloud' do not change anymore.
  /**
  * @brief 在后台线程中运行，并对额外的约束进行计算，假设“submap”和“point_cloud”不再更改.
  */
  // If 'match_full_submap' is true, and global localization succeeds, will
  // connect 'scan_trajectory' and 'submap_trajectory' in
  // 'trajectory_connectivity'.
  /**
  * @brief 如果'match_full_submap'为真，并且全局定位成功，将在“轨迹连通性”中连接“扫描轨迹”和“子图轨迹”.
  */
  // As output, it may create a new Constraint in 'constraint'.
  /**
  * @brief 作为输出，它可能在“约束”中创建一个新的约束.
  */
  void ComputeConstraint(
      int submap_index, const Submap* submap, int scan_index,
      const mapping::Submaps* scan_trajectory,
      const mapping::Submaps* submap_trajectory, bool match_full_submap,
      mapping::TrajectoryConnectivity* trajectory_connectivity,
      const sensor::CompressedPointCloud* const compressed_point_cloud,
      const transform::Rigid3d& initial_relative_pose,
      std::unique_ptr<Constraint>* constraint) EXCLUDES(mutex_);

  // Decrements the 'pending_computations_' count. If all computations are done,
  // runs the 'when_done_' callback and resets the state.
  /**
  * @brief 减少“pending_computations_”计数.如果所有的计算完成，运行'when_done_'回调并重置状态.
  */
  void FinishComputation(int computation_index) EXCLUDES(mutex_);

  const mapping::sparse_pose_graph::proto::ConstraintBuilderOptions options_;
  common::ThreadPool* thread_pool_;
  common::Mutex mutex_;

  // 'callback' set by WhenDone().
  /**
  * @brief “回调”由WhenDone（）设置.
  */
  std::unique_ptr<std::function<void(const Result&)>> when_done_
      GUARDED_BY(mutex_);

  // Index of the scan in reaction to which computations are currently
  // added. This is always the highest scan index seen so far, even when older
  // scans are matched against a new submap.
  /**
  * @brief 当前添加计算的反应的扫描索引.这是迄今为止看到的最高扫描索引，即使较旧的扫描与新的子图匹配.
  */
  int current_computation_ GUARDED_BY(mutex_) = 0;

  // For each added scan, maps to the number of pending computations that were
  // added for it.
  /**
  * @brief 对于每个添加的扫描，映射到为其添加的待处理计算的数量.
  */
  std::map<int, int> pending_computations_ GUARDED_BY(mutex_);

  // Constraints currently being computed in the background. A deque is used to
  // keep pointers valid when adding more entries.
  /**
  * @brief 目前在后台计算的限制.在添加更多条目时，使用deque来保持指针有效.
  */
  std::deque<std::unique_ptr<Constraint>> constraints_ GUARDED_BY(mutex_);

  // Map of already constructed scan matchers by 'submap_index'.
  /**
  * @brief 已经构建的扫描匹配器的地图由“submap_index”.
  */
  std::map<int, SubmapScanMatcher> submap_scan_matchers_ GUARDED_BY(mutex_);

  // Map by 'submap_index' of scan matchers under construction, and the work
  // to do once construction is done.
  /**
  * @brief 正在建设的扫描对象的“submap_index”映射，一旦完成，即可完成工作.
  */
  std::map<int, std::vector<std::function<void()>>> submap_queued_work_items_
      GUARDED_BY(mutex_);

  common::FixedRatioSampler sampler_;
  const sensor::AdaptiveVoxelFilter adaptive_voxel_filter_;
  scan_matching::CeresScanMatcher ceres_scan_matcher_;

  // Histogram of scan matcher scores.
  /**
  * @brief 直方图扫描匹配分数.
  */
  common::Histogram score_histogram_ GUARDED_BY(mutex_);
};

}  // namespace sparse_pose_graph
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SPARSE_POSE_GRAPH_CONSTRAINT_BUILDER_H_
