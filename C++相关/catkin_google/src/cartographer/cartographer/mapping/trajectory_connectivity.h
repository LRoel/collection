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

#ifndef CARTOGRAPHER_MAPPING_TRAJECTORY_CONNECTIVITY_H_
#define CARTOGRAPHER_MAPPING_TRAJECTORY_CONNECTIVITY_H_

#include <map>
#include <unordered_map>

#include "cartographer/common/mutex.h"
#include "cartographer/mapping/proto/trajectory_connectivity.pb.h"
#include "cartographer/mapping/submaps.h"

namespace cartographer {
namespace mapping {

// A class that tracks the connectivity structure between trajectories.
/**
* @brief 跟踪轨迹之间的连接结构的类.
*/
//
// Connectivity includes both the count ("How many times have I _directly_
// connected trajectories i and j?") and the transitive connectivity.
/**
* @brief 连通性包括计数（“我有直接连接的轨迹i和j？”）和传递连接.
*/
//
// This class is thread-safe.
/**
* @brief 这个类是线程安全的.
*/
class TrajectoryConnectivity {
 public:
  TrajectoryConnectivity();

  TrajectoryConnectivity(const TrajectoryConnectivity&) = delete;
  TrajectoryConnectivity& operator=(const TrajectoryConnectivity&) = delete;

  // Add a trajectory which is initially connected to nothing.
  /**
  * @brief 添加一个最初连接到的轨迹.
  */
  void Add(const Submaps* trajectory) EXCLUDES(lock_);

  // Connect two trajectories. If either trajectory is untracked, it will be
  // tracked. This function is invariant to the order of its arguments. Repeated
  // calls to Connect increment the connectivity count.
  /**
  * @brief 连接两个轨迹.如果任一轨迹未被追踪，它将被追踪.这个函数对于它的参数的顺序是不变的.对Connect的重复呼叫会增加连接数.
  */
  void Connect(const Submaps* trajectory_a, const Submaps* trajectory_b)
      EXCLUDES(lock_);

  // Determines if two trajectories have been (transitively) connected. If
  // either trajectory is not being tracked, returns false. This function is
  // invariant to the order of its arguments.
  /**
  * @brief 确定两个轨迹是否已经（过渡地）连接.如果没有跟踪轨迹，则返回false.这个函数对于它的参数的顺序是不变的.
  */
  bool TransitivelyConnected(const Submaps* trajectory_a,
                             const Submaps* trajectory_b) EXCLUDES(lock_);

  // Return the number of _direct_ connections between trajectory_a and
  // trajectory_b. If either trajectory is not being tracked, returns 0. This
  // function is invariant to the order of its arguments.
  /**
  * @brief 返回轨迹和轨迹之间的直接连接数b.如果没有跟踪轨迹，则返回0.这个函数对于它的参数的顺序是不变的.
  */
  int ConnectionCount(const Submaps* trajectory_a, const Submaps* trajectory_b)
      EXCLUDES(lock_);

  // The trajectories, grouped by connectivity.
  /**
  * @brief 轨迹，按连通性分组.
  */
  std::vector<std::vector<const Submaps*>> ConnectedComponents()
      EXCLUDES(lock_);

 private:
  // Find the representative and compresses the path to it.
  /**
  * @brief 找到代表并压缩它的路径.
  */
  const Submaps* FindSet(const Submaps* trajectory) REQUIRES(lock_);
  void Union(const Submaps* trajectory_a, const Submaps* trajectory_b)
      REQUIRES(lock_);

  common::Mutex lock_;
  // Tracks transitive connectivity using a disjoint set forest, i.e. each
  // entry points towards the representative for the given trajectory.
  /**
  * @brief 跟踪使用不相交的森林的传输连接，i.e.每个入口指向给定轨迹的代表.
  */
  std::map<const Submaps*, const Submaps*> forest_ GUARDED_BY(lock_);
  // Tracks the number of direct connections between a pair of trajectories.
  /**
  * @brief 跟踪一对轨迹之间的直接连接数.
  */
  std::map<std::pair<const Submaps*, const Submaps*>, int> connection_map_
      GUARDED_BY(lock_);
};

// Returns a proto encoding connected components according to
// 'trajectory_indices'.
/**
* @brief 根据“tracks_indices”返回原始编码连接的组件.
*/
proto::TrajectoryConnectivity ToProto(
    std::vector<std::vector<const Submaps*>> connected_components,
    std::unordered_map<const mapping::Submaps*, int> trajectory_indices);

// Returns the connected component containing 'trajectory_index'.
/**
* @brief 返回连接的组件，包含'tracks_index'.
*/
proto::TrajectoryConnectivity::ConnectedComponent FindConnectedComponent(
    const cartographer::mapping::proto::TrajectoryConnectivity&
        trajectory_connectivity,
    int trajectory_id);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_TRAJECTORY_CONNECTIVITY_H_
