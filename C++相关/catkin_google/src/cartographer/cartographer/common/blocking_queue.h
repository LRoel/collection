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

#ifndef CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
#define CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_

#include <cstddef>
#include <deque>
#include <memory>

#include "cartographer/common/mutex.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

// A thread-safe blocking queue that is useful for producer/consumer patterns.
/**
* @brief 线程安全阻止队列，对生产者/消费者模式有用.
*/
// 'T' must be movable.
/**
* @brief 'T'必须是可移动的.
*/
template <typename T>
class BlockingQueue {
 public:
  static constexpr size_t kInfiniteQueueSize = 0;

  // Constructs a blocking queue with infinite queue size.
  /**
  * @brief 构造具有无限队列大小的阻塞队列.
  */
  BlockingQueue() : BlockingQueue(kInfiniteQueueSize) {}

  BlockingQueue(const BlockingQueue&) = delete;
  BlockingQueue& operator=(const BlockingQueue&) = delete;

  // Constructs a blocking queue with a size of 'queue_size'.
  /**
  * @brief 构造一个大小为“queue_size”的阻塞队列.
  */
  explicit BlockingQueue(const size_t queue_size) : queue_size_(queue_size) {}

  // Pushes a value onto the queue. Blocks if the queue is full.
  /**
  * @brief 将值推入队列.如果队列已满，则阻止.
  */
  void Push(T t) {
    MutexLocker lock(&mutex_);
    lock.Await([this]() REQUIRES(mutex_) { return QueueNotFullCondition(); });
    deque_.push_back(std::move(t));
  }

  // Like push, but returns false if 'timeout' is reached.
  /**
  * @brief 喜欢push，但如果达到“超时”，则返回false.
  */
  bool PushWithTimeout(T t, const common::Duration timeout) {
    MutexLocker lock(&mutex_);
    if (!lock.AwaitWithTimeout(
            [this]() REQUIRES(mutex_) { return QueueNotFullCondition(); },
            timeout)) {
      return false;
    }
    deque_.push_back(std::move(t));
    return true;
  }

  // Pops the next value from the queue. Blocks until a value is available.
  /**
  * @brief 从队列中弹出下一个值.直到值可用为止.
  */
  T Pop() {
    MutexLocker lock(&mutex_);
    lock.Await([this]() REQUIRES(mutex_) { return QueueNotEmptyCondition(); });

    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Like Pop, but can timeout. Returns nullptr in this case.
  /**
  * @brief 喜欢流行，但可以超时.在这种情况下返回nullptr.
  */
  T PopWithTimeout(const common::Duration timeout) {
    MutexLocker lock(&mutex_);
    if (!lock.AwaitWithTimeout(
            [this]() REQUIRES(mutex_) { return QueueNotEmptyCondition(); },
            timeout)) {
      return nullptr;
    }
    T t = std::move(deque_.front());
    deque_.pop_front();
    return t;
  }

  // Returns the next value in the queue or nullptr if the queue is empty.
  /**
  * @brief 如果队列为空，则返回队列中的下一个值或nullptr.
  */
  // Maintains ownership. This assumes a member function get() that returns
  // a pointer to the given type R.
  /**
  * @brief 保持所有权.这假定一个成员函数get（）返回一个指向给定类型R的指针.
  */
  template <typename R>
  const R* Peek() {
    MutexLocker lock(&mutex_);
    if (deque_.empty()) {
      return nullptr;
    }
    return deque_.front().get();
  }

  // Returns the number of items currently in the queue.
  /**
  * @brief 返回当前队列中的项目数.
  */
  size_t Size() {
    MutexLocker lock(&mutex_);
    return deque_.size();
  }

 private:
  // Returns true iff the queue is not empty.
  /**
  * @brief 如果队列不为空则返回true.
  */
  bool QueueNotEmptyCondition() REQUIRES(mutex_) { return !deque_.empty(); }

  // Returns true iff the queue is not full.
  /**
  * @brief 如果队列未满，则返回true.
  */
  bool QueueNotFullCondition() REQUIRES(mutex_) {
    return queue_size_ == kInfiniteQueueSize || deque_.size() < queue_size_;
  }

  Mutex mutex_;
  const size_t queue_size_ GUARDED_BY(mutex_);
  std::deque<T> deque_ GUARDED_BY(mutex_);
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_BLOCKING_QUEUE_H_
