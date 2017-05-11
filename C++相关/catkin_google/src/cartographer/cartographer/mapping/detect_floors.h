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

#ifndef CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_
#define CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_

#include "cartographer/mapping/proto/trajectory.pb.h"

#include "cartographer/common/interval.h"
#include "cartographer/common/time.h"

namespace cartographer {
namespace mapping {

struct Floor {
  // The spans of time we spent on this floor. Since we might have walked up and
  // down many times in this place, there can be many spans of time we spent on
  // a particular floor.
  /**
  * @brief 我们在这个楼层度过的时间跨度.由于我们可能在这个地方多次走过，所以在一个特定的地板上花费很多时间.
  */
  std::vector<common::Interval<common::Time>> timespans;

  // The median z-value of this floor.
  /**
  * @brief 这个楼层的中值z值.
  */
  double z;
};

// Uses a heuristic which looks at z-values of the poses to detect individual
// floors of a building. This requires that floors are *mostly* on the same
// z-height and that level changes happen *relatively* abrubtly, e.g. by taking
// the stairs.
/**
* @brief 使用一种启发式的方法来查看姿态的z值来检测建筑物的各个楼层.这要求楼层*大部分是*在相同的z高度上，并且水平的变化*相对地*相当地，.G.通过楼梯.
*/
std::vector<Floor> DetectFloors(const proto::Trajectory& trajectory);

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_DETECT_FLOORS_H_
