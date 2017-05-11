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

// Implements the loop closure method called Sparse Pose Adjustment (SPA) from
// Konolige, Kurt, et al. "Efficient sparse pose adjustment for 2d mapping."
// Intelligent Robots and Systems (IROS), 2010 IEEE/RSJ International Conference
// on (pp. 22--29). IEEE, 2010.
/**
* @brief 实现从Konolige，Kurt等人的称为稀疏姿态调整（SPA）的循环闭合方法.“2D映射的高效稀疏姿态调整.“智能机器人与系统（IROS），2010 IEEE / RSJ International Conference on（pp.22-29）.IEEE，2010.
*/
//
// It is extended for submapping:
// Each scan has been matched against one or more submaps (adding a constraint
// for each match), both poses of scans and of submaps are to be optimized.
/**
* @brief 它被扩展用于子映射：每个扫描已经与一个或多个子映射匹配（为每个匹配添加约束），扫描和子图的两个姿势都将被优化.
*/
// All constraints are between a submap i and a scan j.
/**
* @brief 所有约束都在子图i和扫描j之间.
*/

#ifndef CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_H_

#include <deque>
#include <functional>
#include <limits>
#include <map>
#include <unordered_map>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/mutex.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/pose_tracker.h"
#include "cartographer/mapping/proto/scan_matching_progress.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/trajectory_connectivity.h"
#include "cartographer/mapping_2d/sparse_pose_graph/constraint_builder.h"
#include "cartographer/mapping_2d/sparse_pose_graph/optimization_problem.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
    namespace mapping_2d {

// Implements the SPA loop closure method.
/**
* @brief 实现SPA闭环方法.
*/
        class SparsePoseGraph : public mapping::SparsePoseGraph {
        public:
            SparsePoseGraph(
                    const mapping::proto::SparsePoseGraphOptions& options,
                    common::ThreadPool* thread_pool,
                    std::deque<mapping::TrajectoryNode::ConstantData>* constant_node_data);
            ~SparsePoseGraph() override;

            SparsePoseGraph(const SparsePoseGraph&) = delete;
            SparsePoseGraph& operator=(const SparsePoseGraph&) = delete;

            // Adds a new 'range_data_in_pose' observation at 'time', and a 'pose'
            // that will later be optimized. The 'tracking_to_pose' is remembered so
            // that the optimized pose can be embedded into 3D. The 'pose' was determined
            // by scan matching against the 'matching_submap' and the scan was inserted
            // into the 'insertion_submaps'.
            /**
			* @brief 在“时间”上添加一个新的“range_data_in_pose”观察结果，以及稍后将优化的“姿态”.“tracking_to_pose”被记住，以便将优化的姿势嵌入到3D中.'pose'是通过与'matching_submap'的扫描匹配确定的，扫描被插入到'insertion_submaps'.
			*/
            void AddScan(common::Time time, const transform::Rigid3d& tracking_to_pose,
                         const sensor::RangeData& range_data_in_pose,
                         const transform::Rigid2d& pose,
                         const kalman_filter::Pose2DCovariance& pose_covariance,
                         const mapping::Submaps* submaps,
                         const mapping::Submap* matching_submap,
                         const std::vector<const mapping::Submap*>& insertion_submaps)
            EXCLUDES(mutex_);

	        /**
	         * @brief Add GPS and Scan Data
	         */
	         void AddScanGPS(common::Time time, const transform::Rigid3d& tracking_to_pose,
	                         const sensor::RangeData& range_data_in_pose,
	                         const transform::Rigid2d& pose,
	                         const kalman_filter::Pose2DCovariance& pose_covariance,
	                         const mapping::Submaps* submaps,
	                         const mapping::Submap* matching_submap,
	                         const std::vector<const mapping::Submap*>& insertion_submaps,
				             const Eigen::Vector3f &gps_pos,
			                 const Eigen::Vector3f &gps_var)
	        EXCLUDES(mutex_);

            // Adds new IMU data to be used in the optimization.
            /**
			* @brief 添加要在优化中使用的新IMU数据.
			*/
            void AddImuData(const mapping::Submaps* trajectory, common::Time time,
                            const Eigen::Vector3d& linear_acceleration,
                            const Eigen::Vector3d& angular_velocity);

            void RunFinalOptimization() override;
            bool HasNewOptimizedPoses() override;
            mapping::proto::ScanMatchingProgress GetScanMatchingProgress() override;
            std::vector<std::vector<const mapping::Submaps*>> GetConnectedTrajectories()
            override;
            std::vector<transform::Rigid3d> GetSubmapTransforms(
                    const mapping::Submaps& submaps) EXCLUDES(mutex_) override;
            transform::Rigid3d GetLocalToGlobalTransform(const mapping::Submaps& submaps)
            EXCLUDES(mutex_) override;
            std::vector<mapping::TrajectoryNode> GetTrajectoryNodes() override
            EXCLUDES(mutex_);
            std::vector<Constraint> constraints() override;

        private:
            struct SubmapState {
                const mapping::Submap* submap = nullptr;

                // Indices of the scans that were inserted into this map together with
                // constraints for them. They are not to be matched again when this submap
                // becomes 'finished'.
                /**
				* @brief 插入到该地图中的扫描指标及其约束.当这个子图变成“完成”时，它们不会被再次匹配.
				*/
                std::set<int> scan_indices;

                // Whether in the current state of the background thread this submap is
                // finished. When this transitions to true, all scans are tried to match
                // against this submap. Likewise, all new scans are matched against submaps
                // which are finished.
                /**
				* @brief 是否在当前状态的后台线程中完成此子图.当它转换为true时，会尝试将所有扫描与该子图匹配.同样，所有新的扫描都与完成的子图匹配.
				*/
                bool finished = false;

                // The trajectory to which this SubmapState belongs.
                /**
				* @brief 该SubmapState所属的轨迹.
				*/
                const mapping::Submaps* trajectory = nullptr;
            };

            // Handles a new work item.
            /**
			* @brief 处理一个新的工作项.
			*/
            void AddWorkItem(std::function<void()> work_item) REQUIRES(mutex_);

            int GetSubmapIndex(const mapping::Submap* submap) const REQUIRES(mutex_) {
                const auto iterator = submap_indices_.find(submap);
                CHECK(iterator != submap_indices_.end());
                return iterator->second;
            }

            // Grows 'submap_transforms_' to have an entry for every element of 'submaps'.
            /**
			* @brief 生长'submap_transforms_'为“子图”的每个元素都有一个条目.
			*/
            void GrowSubmapTransformsAsNeeded(
                    const std::vector<const mapping::Submap*>& submaps) REQUIRES(mutex_);

            // Adds constraints for a scan, and starts scan matching in the background.
            /**
			* @brief 添加扫描的约束，并在后台启动扫描匹配.
			*/
            void ComputeConstraintsForScan(
                    int scan_index, const mapping::Submap* matching_submap,
                    std::vector<const mapping::Submap*> insertion_submaps,
                    const mapping::Submap* finished_submap, const transform::Rigid2d& pose,
                    const kalman_filter::Pose2DCovariance& covariance) REQUIRES(mutex_);

	        /**
	         * @brief compute GPS constraints
	         */
	         void ComputeConstraintsForGPS(
			        int scan_index, const mapping::Submap* matching_submap,
			        std::vector<const mapping::Submap*> insertion_submaps,
			        const mapping::Submap* finished_submap, const transform::Rigid2d& pose,
			        const kalman_filter::Pose2DCovariance& covariance) REQUIRES(mutex_);

            // Computes constraints for a scan and submap pair.
            /**
			* @brief 计算扫描和子图对的约束.
			*/
            void ComputeConstraint(const int scan_index, const int submap_index)
            REQUIRES(mutex_);

            // Adds constraints for older scans whenever a new submap is finished.
            /**
			* @brief 每当新的子图完成时，为旧的扫描添加约束.
			*/
            void ComputeConstraintsForOldScans(const mapping::Submap* submap)
            REQUIRES(mutex_);

            // Registers the callback to run the optimization once all constraints have
            // been computed, that will also do all work that queue up in 'scan_queue_'.
            /**
			* @brief 注册回调，一旦计算所有约束，就运行优化，这也将在'scan_queue_'中排队的所有工作.
			*/
            void HandleScanQueue() REQUIRES(mutex_);

            // Waits until we caught up (i.e. nothing is waiting to be scheduled), and
            // all computations have finished.
            /**
			* @brief 等待，直到我们赶上（我.e.没有什么正在等待安排），所有的计算都已经完成了.
			*/
            void WaitForAllComputations() EXCLUDES(mutex_);

            // Runs the optimization. Callers have to make sure, that there is only one
            // optimization being run at a time.
            /**
			* @brief 运行优化.来电者必须确保，一次只运行一个优化.
			*/
            void RunOptimization() EXCLUDES(mutex_);

            // Adds extrapolated transforms, so that there are transforms for all submaps.
            /**
			* @brief 添加外推变换，以便所有子映射都有变换.
			*/
            std::vector<transform::Rigid3d> ExtrapolateSubmapTransforms(
                    const std::vector<transform::Rigid2d>& submap_transforms,
                    const mapping::Submaps& submaps) const REQUIRES(mutex_);

            const mapping::proto::SparsePoseGraphOptions options_;
            common::Mutex mutex_;

            // If it exists, further scans must be added to this queue, and will be
            // considered later.
            /**
			* @brief 如果存在，则必须在此队列中添加进一步的扫描，稍后再考虑.
			*/
            std::unique_ptr<std::deque<std::function<void()>>> scan_queue_
            GUARDED_BY(mutex_);

            // How our various trajectories are related.
            /**
			* @brief 我们的各种轨迹如何相关.
			*/
            mapping::TrajectoryConnectivity trajectory_connectivity_ GUARDED_BY(mutex_);

            // We globally localize a fraction of the scans from each trajectory.
            /**
			* @brief 我们在全球范围内定位每个轨迹的一小部分扫描.
			*/
            std::unordered_map<const mapping::Submaps*,
                    std::unique_ptr<common::FixedRatioSampler>>
                    global_localization_samplers_ GUARDED_BY(mutex_);

            // Number of scans added since last loop closure.
            /**
			* @brief 自上次循环关闭以来添加的扫描数.
			*/
            int num_scans_since_last_loop_closure_ GUARDED_BY(mutex_) = 0;

            // Whether the optimization has to be run before more data is added.
            /**
			* @brief 在添加更多数据之前是否必须运行优化.
			*/
            bool run_loop_closure_ GUARDED_BY(mutex_) = false;

            // Current optimization problem.
            /**
			* @brief 当前优化问题.
			*/
            sparse_pose_graph::OptimizationProblem optimization_problem_;
            sparse_pose_graph::ConstraintBuilder constraint_builder_ GUARDED_BY(mutex_);
            std::vector<Constraint> constraints_;
            std::vector<transform::Rigid2d> submap_transforms_;  // (map <- submap)

            // Submaps get assigned an index and state as soon as they are seen, even
            // before they take part in the background computations.
            /**
			* @brief 即使在参与背景计算之前，子图也会被分配到一个索引和状态.
			*/
            std::map<const mapping::Submap*, int> submap_indices_ GUARDED_BY(mutex_);
            std::vector<SubmapState> submap_states_ GUARDED_BY(mutex_);

            // Whether to return true on the next call to HasNewOptimizedPoses().
            /**
			* @brief 是否在下次调用HasNewOptimizedPoses（）时返回true.
			*/
            bool has_new_optimized_poses_ GUARDED_BY(mutex_) = false;

            // Connectivity structure of our trajectories.
            /**
			* @brief 我们的轨迹的连接结构.
			*/
            std::vector<std::vector<const mapping::Submaps*>> connected_components_;
            // Trajectory to connected component ID.
            /**
			* @brief 轨迹连接组件ID.
			*/
            std::map<const mapping::Submaps*, size_t> reverse_connected_components_;

            // Data that are currently being shown.
            /**
			* @brief 当前正在显示的数据.
			*/
            //
            // Deque to keep references valid for the background computation when adding
            // new data.
            /**
			* @brief Deque在添加新数据时保持对后台计算有效的引用.
			*/
            std::deque<mapping::TrajectoryNode::ConstantData>* constant_node_data_;
            std::vector<mapping::TrajectoryNode> trajectory_nodes_ GUARDED_BY(mutex_);

            // Current submap transforms used for displaying data.
            /**
			* @brief 用于显示数据的当前子图变换.
			*/
            std::vector<transform::Rigid2d> optimized_submap_transforms_
            GUARDED_BY(mutex_);
        };

    }  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_SPARSE_POSE_GRAPH_H_
