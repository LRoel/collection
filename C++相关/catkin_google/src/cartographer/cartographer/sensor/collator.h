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

#ifndef CARTOGRAPHER_SENSOR_COLLATOR_H_
#define CARTOGRAPHER_SENSOR_COLLATOR_H_

#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "cartographer/sensor/data.h"
#include "cartographer/sensor/ordered_multi_queue.h"

namespace cartographer {
namespace sensor {

class Collator {
 public:
  using Callback = std::function<void(const string&, std::unique_ptr<Data>)>;

  Collator() {}

  Collator(const Collator&) = delete;
  Collator& operator=(const Collator&) = delete;

  // Adds a trajectory to produce sorted sensor output for. Calls 'callback'
  // for each collated sensor data.
  /**
  * @brief 添加轨迹以生成排序的传感器输出.对每个整理的传感器数据调用“回调”.
  */
  void AddTrajectory(int trajectory_id,
                     const std::unordered_set<string>& expected_sensor_ids,
                     Callback callback);

  // Marks 'trajectory_id' as finished.
  /**
  * @brief 标记“track_id”完成.
  */
  void FinishTrajectory(int trajectory_id);

  // Adds 'data' for 'trajectory_id' to be collated. 'data' must contain valid
  // sensor data. Sensor packets with matching 'sensor_id' must be added in time
  // order.
  /**
  * @brief 添加'tracking_id'的数据进行整理.“数据”必须包含有效的传感器数据.必须按照时间顺序添加具有匹配“sensor_id”的传感器数据包.
  */
  void AddSensorData(int trajectory_id, const string& sensor_id,
                     std::unique_ptr<Data> data);

  // Dispatches all queued sensor packets. May only be called once.
  /**
  * @brief 调度所有排队的传感器数据包.只能叫一次.
  */
  // AddSensorData may not be called after Flush.
  /**
  * @brief Flush后可能不会调用AddSensorData.
  */
  void Flush();

  // Must only be called if at least one unfinished trajectory exists. Returns
  // the ID of the trajectory that needs more data before the Collator is
  // unblocked.
  /**
  * @brief 只有至少存在一个未完成的轨迹才能被调用.在解除封锁程序解除封锁之前，返回需要更多数据的轨迹的ID.
  */
  int GetBlockingTrajectoryId() const;

 private:
  // Queue keys are a pair of trajectory ID and sensor identifier.
  /**
  * @brief 队列密钥是一对轨迹ID和传感器标识符.
  */
  OrderedMultiQueue queue_;

  // Map of trajectory ID to all associated QueueKeys.
  /**
  * @brief 将轨迹ID映射到所有关联的QueueKey.
  */
  std::unordered_map<int, std::vector<QueueKey>> queue_keys_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_COLLATOR_H_
