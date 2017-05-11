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

#ifndef CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
#define CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/math.h"
#include "cartographer/common/port.h"
#include "cartographer/mapping/probability_values.h"
#include "cartographer/mapping_2d/map_limits.h"
#include "cartographer/mapping_2d/proto/probability_grid.pb.h"
#include "cartographer/mapping_2d/xy_index.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping_2d {

// Represents a 2D grid of probabilities.
/**
* @brief 代表2D概率网格.
*/
class ProbabilityGrid {
 public:
  explicit ProbabilityGrid(const MapLimits& limits)
      : limits_(limits),
        cells_(limits_.cell_limits().num_x_cells *
                   limits_.cell_limits().num_y_cells,
               mapping::kUnknownProbabilityValue),
        max_x_(0),
        max_y_(0),
        min_x_(limits_.cell_limits().num_x_cells - 1),
        min_y_(limits_.cell_limits().num_y_cells - 1) {}

  explicit ProbabilityGrid(const proto::ProbabilityGrid& proto)
      : limits_(proto.limits()),
        cells_(),
        update_indices_(proto.update_indices().begin(),
                        proto.update_indices().end()),
        max_x_(proto.max_x()),
        max_y_(proto.max_y()),
        min_x_(proto.min_x()),
        min_y_(proto.min_y()) {
    cells_.reserve(proto.cells_size());
    for (const auto cell : proto.cells()) {
      CHECK_LE(cell, std::numeric_limits<uint16>::max());
      cells_.push_back(cell);
    }
  }

  // Returns the limits of this ProbabilityGrid.
  /**
  * @brief 返回此ProbabilityGrid的限制.
  */
  const MapLimits& limits() const { return limits_; }

  // Starts the next update sequence.
  /**
  * @brief 启动下一个更新序列.
  */
  void StartUpdate() {
    while (!update_indices_.empty()) {
      DCHECK_GE(cells_[update_indices_.back()], mapping::kUpdateMarker);
      cells_[update_indices_.back()] -= mapping::kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Sets the probability of the cell at 'xy_index' to the given 'probability'.
  /**
  * @brief 将“xy_index”处的单元格的概率设置为给定的“概率”.
  */
  // Only allowed if the cell was unknown before.
  /**
  * @brief 只有在细胞未知之前才允许.
  */
  void SetProbability(const Eigen::Array2i& xy_index, const float probability) {
    uint16& cell = cells_[GetIndexOfCell(xy_index)];
    CHECK_EQ(cell, mapping::kUnknownProbabilityValue);
    cell = mapping::ProbabilityToValue(probability);
    UpdateBounds(xy_index);
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'xy_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // StartUpdate() is called. Returns true if the cell was updated.
  /**
  * @brief 如果单元格尚未更新，则将“调查ComputeLookupTableToApplyOdds（）”指定为“xy_index”上单元格的概率时指定的“赔率”.在调用StartUpdate（）之前，将忽略同一单元格的多个更新.如果单元格已更新，则返回true.
  */
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  /**
  * @brief 如果这是对指定单元格的ApplyOdds（）的第一个调用，它的值将被设置为对应于'odds'的概率.
  */
  bool ApplyLookupTable(const Eigen::Array2i& xy_index,
                        const std::vector<uint16>& table) {
    DCHECK_EQ(table.size(), mapping::kUpdateMarker);
    const int cell_index = GetIndexOfCell(xy_index);
    uint16& cell = cells_[cell_index];
    if (cell >= mapping::kUpdateMarker) {
      return false;
    }
    update_indices_.push_back(cell_index);
    cell = table[cell];
    DCHECK_GE(cell, mapping::kUpdateMarker);
    UpdateBounds(xy_index);
    return true;
  }

  // Returns the probability of the cell with 'xy_index'.
  /**
  * @brief 使用'xy_index'返回单元格的概率.
  */
  float GetProbability(const Eigen::Array2i& xy_index) const {
    if (limits_.Contains(xy_index)) {
      return mapping::ValueToProbability(cells_[GetIndexOfCell(xy_index)]);
    }
    return mapping::kMinProbability;
  }

  // Returns the probability of the cell containing the point ('x', 'y').
  /**
  * @brief 返回包含点（'x'，'y'）的单元格的概率.
  */
  float GetProbability(const double x, const double y) const {
    return GetProbability(limits_.GetXYIndexOfCellContainingPoint(x, y));
  }

  // Returns true if the probability at the specified index is known.
  /**
  * @brief 如果指定索引的概率已知，则返回true.
  */
  bool IsKnown(const Eigen::Array2i& xy_index) const {
    return limits_.Contains(xy_index) && cells_[GetIndexOfCell(xy_index)] !=
                                             mapping::kUnknownProbabilityValue;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  /**
  * @brief 填入'偏移'和'限制'以定义其中包含所有已知单元格的子区域.
  */
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const {
    *offset = Eigen::Array2i(min_x_, min_y_);
    *limits = CellLimits(std::max(max_x_, min_x_) - min_x_ + 1,
                         std::max(max_y_, min_y_) - min_y_ + 1);
  }

  // Grows the map as necessary to include 'x' and 'y'. This changes the meaning
  // of these coordinates going forward. This method must be called immediately
  // after 'StartUpdate', before any calls to 'ApplyLookupTable'.
  /**
  * @brief 根据需要增加地图以包含“x”和“y”.这改变了这些坐标的意义.在“StartUpdate”之后，必须立即调用此方法，然后调用“ApplyLookupTable”.
  */
  void GrowLimits(const double x, const double y) {
    CHECK(update_indices_.empty());
    while (!limits_.Contains(limits_.GetXYIndexOfCellContainingPoint(x, y))) {
      const int x_offset = limits_.cell_limits().num_x_cells / 2;
      const int y_offset = limits_.cell_limits().num_y_cells / 2;
      const MapLimits new_limits(
          limits_.resolution(),
          limits_.max() +
              limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
          CellLimits(2 * limits_.cell_limits().num_x_cells,
                     2 * limits_.cell_limits().num_y_cells));
      const int stride = new_limits.cell_limits().num_x_cells;
      const int offset = x_offset + stride * y_offset;
      const int new_size = new_limits.cell_limits().num_x_cells *
                           new_limits.cell_limits().num_y_cells;
      std::vector<uint16> new_cells(new_size,
                                    mapping::kUnknownProbabilityValue);
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
          new_cells[offset + j + i * stride] =
              cells_[j + i * limits_.cell_limits().num_x_cells];
        }
      }
      cells_ = new_cells;
      limits_ = new_limits;
      min_x_ += x_offset;
      min_y_ += y_offset;
      max_x_ += x_offset;
      max_y_ += y_offset;
    }
  }

  proto::ProbabilityGrid ToProto() {
    proto::ProbabilityGrid result;
    *result.mutable_limits() = cartographer::mapping_2d::ToProto(limits_);
    result.mutable_cells()->Reserve(cells_.size());
    for (const auto cell : cells_) {
      result.mutable_cells()->Add(cell);
    }
    result.mutable_update_indices()->Reserve(update_indices_.size());
    for (const auto update : update_indices_) {
      result.mutable_update_indices()->Add(update);
    }
    result.set_max_x(max_x_);
    result.set_max_y(max_y_);
    result.set_min_x(min_x_);
    result.set_min_y(min_y_);
    return result;
  }

 private:
  // Returns the index of the cell at 'xy_index'.
  /**
  * @brief 返回'xy_index'处的单元格索引.
  */
  int GetIndexOfCell(const Eigen::Array2i& xy_index) const {
    CHECK(limits_.Contains(xy_index)) << xy_index;
    return limits_.cell_limits().num_x_cells * xy_index.y() + xy_index.x();
  }

  void UpdateBounds(const Eigen::Array2i& xy_index) {
    min_x_ = std::min(min_x_, xy_index.x());
    min_y_ = std::min(min_y_, xy_index.y());
    max_x_ = std::max(max_x_, xy_index.x());
    max_y_ = std::max(max_y_, xy_index.y());
  }

  MapLimits limits_;
  std::vector<uint16> cells_;  // Highest bit is update marker.
  std::vector<int> update_indices_;

  // Minimum and maximum cell coordinates of known cells to efficiently compute
  // cropping limits.
  /**
  * @brief 已知单元格的最小和最大单元坐标有效地计算裁剪限制.
  */
  int max_x_;
  int max_y_;
  int min_x_;
  int min_y_;
};

}  // namespace mapping_2d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_2D_PROBABILITY_GRID_H_
