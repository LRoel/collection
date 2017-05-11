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

#ifndef CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_
#define CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_

#include <vector>

#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/mapping/proto/scan_matching_progress.pb.h"
#include "cartographer/mapping/proto/sparse_pose_graph.pb.h"
#include "cartographer/mapping/proto/sparse_pose_graph_options.pb.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/transform/rigid_transform.h"

namespace cartographer {
	namespace mapping {

		proto::SparsePoseGraphOptions CreateSparsePoseGraphOptions(
				common::LuaParameterDictionary* const parameter_dictionary);

// Splits TrajectoryNodes by ID.
/**
* @brief 按ID分割轨迹节点.
*/
		std::vector<std::vector<TrajectoryNode>> SplitTrajectoryNodes(
				const std::vector<TrajectoryNode>& trajectory_nodes);

		class SparsePoseGraph {
		public:
			// A "constraint" as in the paper by Konolige, Kurt, et al. "Efficient sparse
			// pose adjustment for 2d mapping." Intelligent Robots and Systems (IROS),
			// 2010 IEEE/RSJ International Conference on (pp. 22--29). IEEE, 2010.
			/**
			* @brief Konolige，Kurt等人的文章中的“约束”.“2D映射的高效稀疏姿态调整.“智能机器人与系统（IROS），2010 IEEE / RSJ International Conference on（pp.22-29）.IEEE，2010.
			*/
			struct Constraint {
				struct Pose {
					transform::Rigid3d zbar_ij;
					Eigen::Matrix<double, 6, 6> sqrt_Lambda_ij;
				};

				struct GPSPose {
					transform::Rigid3d zbar_ij;
					Eigen::Matrix<double, 6, 6> sqrt_Lambda_ij;
				};

				// Submap index.
				/**
				* @brief 子图索引.
				*/
				int i;

				// Scan index.
				/**
				* @brief 扫描索引.
				*/
				int j;

				// Pose of the scan 'j' relative to submap 'i'.
				/**
				* @brief 扫描'j'相对于子图'i'的姿势.
				*/
				Pose pose;

				GPSPose gpspose;

				// Differentiates between intra-submap (where scan 'j' was inserted into
				// submap 'i') and inter-submap constraints (where scan 'j' was not inserted
				// into submap 'i').
				/**
				* @brief 区分地图（其中扫描'j'被插入子地图'i'）和子地图限制（其中扫描'j'未被插入子地图'i'）.
				*/
				enum Tag { INTRA_SUBMAP, INTER_SUBMAP } tag;
			};

			SparsePoseGraph() {}
			virtual ~SparsePoseGraph() {}

			SparsePoseGraph(const SparsePoseGraph&) = delete;
			SparsePoseGraph& operator=(const SparsePoseGraph&) = delete;

			// Computes optimized poses.
			/**
			* @brief 计算优化姿势.
			*/
			virtual void RunFinalOptimization() = 0;

			// Will once return true whenever new optimized poses are available.
			/**
			* @brief 当新的优化姿势可用时，将一旦返回true.
			*/
			virtual bool HasNewOptimizedPoses() = 0;

			// Returns the scan matching progress.
			/**
			* @brief 返回扫描匹配进度.
			*/
			virtual proto::ScanMatchingProgress GetScanMatchingProgress() = 0;

			// Get the current trajectory clusters.
			/**
			* @brief 获取当前的轨迹集群.
			*/
			virtual std::vector<std::vector<const Submaps*>>
			GetConnectedTrajectories() = 0;

			// Returns the current optimized transforms for the given 'submaps'.
			/**
			* @brief 返回给定“子图”的当前优化变换.
			*/
			virtual std::vector<transform::Rigid3d> GetSubmapTransforms(
					const Submaps& submaps) = 0;

			// Returns the transform converting data in the local map frame (i.e. the
			// continuous, non-loop-closed frame) into the global map frame (i.e. the
			// discontinuous, loop-closed frame).
			/**
			* @brief 返回本地地图帧中的变换转换数据（i.e.连续的，非循环闭合的帧）到全局映射帧（即，.e.不连续的，闭环的框架）.
			*/
			virtual transform::Rigid3d GetLocalToGlobalTransform(
					const Submaps& submaps) = 0;

			// Returns the current optimized trajectory.
			/**
			* @brief 返回当前优化的轨迹.
			*/
			virtual std::vector<TrajectoryNode> GetTrajectoryNodes() = 0;

			// Returns the collection of constraints.
			/**
			* @brief 返回约束的集合.
			*/
			virtual std::vector<Constraint> constraints() = 0;

			// Serializes the constraints and the computed trajectory.
			/**
			* @brief 序列化约束和计算轨迹.
			*/
			//
			// TODO(whess): Support multiple trajectories.
			/**
			* @brief TODO（whess）：支持多个轨迹.
			*/
			proto::SparsePoseGraph ToProto();
		};

	}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_SPARSE_POSE_GRAPH_H_
