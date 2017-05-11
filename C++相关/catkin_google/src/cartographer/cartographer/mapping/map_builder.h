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

#ifndef CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
#define CARTOGRAPHER_MAPPING_MAP_BUILDER_H_

#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/map_builder_options.pb.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/mapping/sparse_pose_graph.h"
#include "cartographer/mapping/submaps.h"
#include "cartographer/mapping/trajectory_builder.h"
#include "cartographer/mapping/trajectory_node.h"
#include "cartographer/mapping_2d/sparse_pose_graph.h"
#include "cartographer/mapping_3d/sparse_pose_graph.h"
#include "cartographer/sensor/collator.h"

namespace cartographer {
namespace mapping {

proto::MapBuilderOptions CreateMapBuilderOptions(
    common::LuaParameterDictionary* const parameter_dictionary);

// Wires up the complete SLAM stack with TrajectoryBuilders (for local submaps)
// and a SparsePoseGraph for loop closure.
/**
* @brief 使用TrajectoryBuilders（用于本地子映射）和SparsePoseGraph连接完整的SLAM堆栈以进行循环闭包.
*/
class MapBuilder {
 public:
  MapBuilder(const proto::MapBuilderOptions& options,
             std::deque<mapping::TrajectoryNode::ConstantData>* constant_data);
  ~MapBuilder();

  MapBuilder(const MapBuilder&) = delete;
  MapBuilder& operator=(const MapBuilder&) = delete;

  // Create a new trajectory and return its index.
  /**
  * @brief 创建一个新的轨迹并返回其索引.
  */
  int AddTrajectoryBuilder(
      const std::unordered_set<string>& expected_sensor_ids);

  // Returns the TrajectoryBuilder corresponding to the specified
  // 'trajectory_id'.
  /**
  * @brief 返回对应于指定的“tracks_id”的TrajectoryBuilder.
  */
  mapping::TrajectoryBuilder* GetTrajectoryBuilder(int trajectory_id) const;

  // Marks the TrajectoryBuilder corresponding to 'trajectory_id' as finished,
  // i.e. no further sensor data is expected.
  /**
  * @brief 将与“track_id”相对应的“轨迹绘图”标记为已完成，.e.不需要进一步的传感器数据.
  */
  void FinishTrajectory(int trajectory_id);

  // Must only be called if at least one unfinished trajectory exists. Returns
  // the ID of the trajectory that needs more data before the MapBuilder is
  // unblocked.
  /**
  * @brief 只有至少存在一个未完成的轨迹才能被调用.在MapBuilder解除封锁之前，返回需要更多数据的轨迹的ID.
  */
  int GetBlockingTrajectoryId() const;

  // Returns the trajectory ID for 'trajectory'.
  /**
  * @brief 返回“轨迹”的轨迹ID.
  */
  int GetTrajectoryId(const mapping::Submaps* trajectory) const;

  // Returns the trajectory connectivity.
  /**
  * @brief 返回轨迹连通性.
  */
  proto::TrajectoryConnectivity GetTrajectoryConnectivity();

  // Fills the SubmapQuery::Response corresponding to 'submap_index' from
  // 'trajectory_id'. Returns an error string on failure, or an empty string on
  // success.
  /**
  * @brief 填充SubmapQuery :: Response对应“submap_index”from“rails_id”.在失败时返回一个错误字符串，或成功返回一个空字符串.
  */
  string SubmapToProto(int trajectory_id, int submap_index,
                       proto::SubmapQuery::Response* response);

  int num_trajectory_builders() const;

  mapping::SparsePoseGraph* sparse_pose_graph();

 private:
  const proto::MapBuilderOptions options_;
  common::ThreadPool thread_pool_;

  std::unique_ptr<mapping_2d::SparsePoseGraph> sparse_pose_graph_2d_;
  std::unique_ptr<mapping_3d::SparsePoseGraph> sparse_pose_graph_3d_;
  mapping::SparsePoseGraph* sparse_pose_graph_;

  sensor::Collator sensor_collator_;
  std::vector<std::unique_ptr<mapping::TrajectoryBuilder>> trajectory_builders_;
  std::unordered_map<const mapping::Submaps*, int> trajectory_ids_;
};

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_MAP_BUILDER_H_
