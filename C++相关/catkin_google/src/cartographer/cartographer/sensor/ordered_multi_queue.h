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

#ifndef CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_
#define CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "cartographer/common/blocking_queue.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/sensor/data.h"

namespace cartographer {
namespace sensor {

struct QueueKey {
  int trajectory_id;
  string sensor_id;

  bool operator<(const QueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) <
           std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
};

// Maintains multiple queues of sorted sensor data and dispatches it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
/**
* @brief 维护排序的传感器数据的多个队列，并按合并排序顺序进行调度.在所有队列中分派下一个时间排序值之前，它将等待看到每个未完成队列的至少一个值.
*/
//
// This class is thread-compatible.
/**
* @brief 这个类是线程兼容的.
*/
class OrderedMultiQueue {
 public:
  using Callback = std::function<void(std::unique_ptr<Data>)>;

  OrderedMultiQueue();
  ~OrderedMultiQueue();

  // Adds a new queue with key 'queue_key' which must not already exist.
  /**
  * @brief 添加一个新的队列，键盘'queue_key'不能存在.
  */
  // 'callback' will be called whenever data from this queue can be dispatched.
  /**
  * @brief 只要可以调度此队列的数据，就会调用“回调”.
  */
  void AddQueue(const QueueKey& queue_key, Callback callback);

  // Marks a queue as finished, i.e. no further data can be added. The queue
  // will be removed once the last piece of data from it has been dispatched.
  /**
  * @brief 标记队列完成，我.e.不能添加进一步的数据.一旦发送了最后一条数据，队列将被删除.
  */
  void MarkQueueAsFinished(const QueueKey& queue_key);

  // Adds 'data' to a queue with the given 'queue_key'. Data must be added
  // sorted per queue.
  /**
  * @brief 使用给定的“queue_key”将数据添加到队列中.每个队列必须添加数据.
  */
  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);

  // Dispatches all remaining values in sorted order and removes the underlying
  // queues.
  /**
  * @brief 按排序顺序调度所有剩余的值，并删除底层队列.
  */
  void Flush();

  // Must only be called if at least one unfinished queue exists. Returns the
  // key of a queue that needs more data before the OrderedMultiQueue can
  // dispatch data.
  /**
  * @brief 只有至少有一个未完成的队列存在才能被调用.在OrderedMultiQueue可以调度数据之前，返回需要更多数据的队列的密钥.
  */
  QueueKey GetBlocker() const;

 private:
  struct Queue {
    common::BlockingQueue<std::unique_ptr<Data>> queue;
    Callback callback;
    bool finished = false;
  };

  void Dispatch();
  void CannotMakeProgress(const QueueKey& queue_key);
  common::Time GetCommonStartTime(int trajectory_id);

  // Used to verify that values are dispatched in sorted order.
  /**
  * @brief 用于验证按排序顺序调度值.
  */
  common::Time last_dispatched_time_ = common::Time::min();

  std::map<int, common::Time> common_start_time_per_trajectory_;
  std::map<QueueKey, Queue> queues_;
  QueueKey blocker_;
};

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_ORDERED_MULTI_QUEUE_H_
