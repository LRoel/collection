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

#include "cartographer/mapping_2d/ray_casting.h"

namespace cartographer {
namespace mapping_2d {

namespace {

// Factor for subpixel accuracy of start and end point.
/**
* @brief 起始点和终点的子像素精度因子.
*/
constexpr int kSubpixelScale = 1000;

// We divide each pixel in kSubpixelScale x kSubpixelScale subpixels. 'begin'
// and 'end' are coordinates at subpixel precision. We compute all pixels in
// which some part of the line segment connecting 'begin' and 'end' lies.
/**
* @brief 我们划分kSubpixelScale x kSubpixelScale子像素中的每个像素.'begin'和'end'是以子像素精度为坐标.我们计算连接“开始”和“结束”的线段的某些部分的所有像素.
*/
void CastRay(const Eigen::Array2i& begin, const Eigen::Array2i& end,
             const std::function<void(const Eigen::Array2i&)>& visitor) {
  // For simplicity, we order 'begin' and 'end' by their x coordinate.
  /**
  * @brief 为了简单起见，我们用x坐标命令“开始”和“结束”.
  */
  if (begin.x() > end.x()) {
    CastRay(end, begin, visitor);
    return;
  }

  CHECK_GE(begin.x(), 0);
  CHECK_GE(begin.y(), 0);
  CHECK_GE(end.y(), 0);

  // Special case: We have to draw a vertical line in full pixels, as 'begin'
  // and 'end' have the same full pixel x coordinate.
  /**
  * @brief 特殊情况：我们必须以完整像素绘制垂直线，因为“​​开始”和“结束”具有相同的全像素x坐标.
  */
  if (begin.x() / kSubpixelScale == end.x() / kSubpixelScale) {
    Eigen::Array2i current(begin.x() / kSubpixelScale,
                           std::min(begin.y(), end.y()) / kSubpixelScale);
    const int end_y = std::max(begin.y(), end.y()) / kSubpixelScale;
    for (; current.y() <= end_y; ++current.y()) {
      visitor(current);
    }
    return;
  }

  const int64 dx = end.x() - begin.x();
  const int64 dy = end.y() - begin.y();
  const int64 denominator = 2 * kSubpixelScale * dx;

  // The current full pixel coordinates. We begin at 'begin'.
  /**
  * @brief 当前的全像素坐标.我们从'开始'开始.
  */
  Eigen::Array2i current = begin / kSubpixelScale;

  // To represent subpixel centers, we use a factor of 2 * 'kSubpixelScale' in
  // the denominator.
  /**
  * @brief 为了表示子像素中心，我们在分母中使用2 *'kSubpixelScale的因子.
  */
  // +-+-+-+ -- 1 = (2 * kSubpixelScale) / (2 * kSubpixelScale)
  // | | | |
  // +-+-+-+
  // | | | |
  // +-+-+-+ -- top edge of first subpixel = 2 / (2 * kSubpixelScale)
  // | | | | -- center of first subpixel = 1 / (2 * kSubpixelScale)
  // +-+-+-+ -- 0 = 0 / (2 * kSubpixelScale)

  // The center of the subpixel part of 'begin.y()' assuming the
  // 'denominator', i.e., sub_y / denominator is in (0, 1).
  /**
  * @brief +  -  +  -  +  -  +  -  1 =（2 * kSubpixelScale）/（2 * kSubpixelScale）+  -  +  -  +  -  +  - 第一个子像素的上边缘= 2 /（2 * kSubpixelScale）| .y（）'假设'分母'，i.e.，sub_y /分母在（0，1）.
  */
  int64 sub_y = (2 * (begin.y() % kSubpixelScale) + 1) * dx;

  // The distance from the from 'begin' to the right pixel border, to be divided
  // by 2 * 'kSubpixelScale'.
  /**
  * @brief 从'开始'到正确的像素边界的距离，被除以2 *'kSubpixelScale'.
  */
  const int first_pixel =
      2 * kSubpixelScale - 2 * (begin.x() % kSubpixelScale) - 1;
  // The same from the left pixel border to 'end'.
  /**
  * @brief 从左边的像素边框到'结束'.
  */
  const int last_pixel = 2 * (end.x() % kSubpixelScale) + 1;

  // The full pixel x coordinate of 'end'.
  /**
  * @brief 'end'的全像素x坐标.
  */
  const int end_x = std::max(begin.x(), end.x()) / kSubpixelScale;

  // Move from 'begin' to the next pixel border to the right.
  /**
  * @brief 从“开始”移动到右边的下一个像素边框.
  */
  sub_y += dy * first_pixel;
  if (dy > 0) {
    while (true) {
      visitor(current);
      while (sub_y > denominator) {
        sub_y -= denominator;
        ++current.y();
        visitor(current);
      }
      ++current.x();
      if (sub_y == denominator) {
        sub_y -= denominator;
        ++current.y();
      }
      if (current.x() == end_x) {
        break;
      }
      // Move from one pixel border to the next.
      /**
      * @brief 从一个像素边框移动到下一个边框.
      */
      sub_y += dy * 2 * kSubpixelScale;
    }
    // Move from the pixel border on the right to 'end'.
    /**
    * @brief 从右侧的像素边框移动到“结束”.
    */
    sub_y += dy * last_pixel;
    visitor(current);
    while (sub_y > denominator) {
      sub_y -= denominator;
      ++current.y();
      visitor(current);
    }
    CHECK_NE(sub_y, denominator);
    CHECK_EQ(current.y(), end.y() / kSubpixelScale);
    return;
  }

  // Same for lines non-ascending in y coordinates.
  /**
  * @brief 在y坐标中不上升的线条相同.
  */
  while (true) {
    visitor(current);
    while (sub_y < 0) {
      sub_y += denominator;
      --current.y();
      visitor(current);
    }
    ++current.x();
    if (sub_y == 0) {
      sub_y += denominator;
      --current.y();
    }
    if (current.x() == end_x) {
      break;
    }
    sub_y += dy * 2 * kSubpixelScale;
  }
  sub_y += dy * last_pixel;
  visitor(current);
  while (sub_y < 0) {
    sub_y += denominator;
    --current.y();
    visitor(current);
  }
  CHECK_NE(sub_y, 0);
  CHECK_EQ(current.y(), end.y() / kSubpixelScale);
}

}  // namespace

void CastRays(const sensor::RangeData& range_data, const MapLimits& limits,
              const std::function<void(const Eigen::Array2i&)>& hit_visitor,
              const std::function<void(const Eigen::Array2i&)>& miss_visitor) {
  const double superscaled_resolution = limits.resolution() / kSubpixelScale;
  const MapLimits superscaled_limits(
      superscaled_resolution, limits.max(),
      CellLimits(limits.cell_limits().num_x_cells * kSubpixelScale,
                 limits.cell_limits().num_y_cells * kSubpixelScale));
  const Eigen::Array2i begin =
      superscaled_limits.GetXYIndexOfCellContainingPoint(range_data.origin.x(),
                                                         range_data.origin.y());

  // Compute and add the end points.
  /**
  * @brief 计算并添加端点.
  */
  std::vector<Eigen::Array2i> ends;
  ends.reserve(range_data.returns.size());
  for (const Eigen::Vector3f& hit : range_data.returns) {
    ends.push_back(
        superscaled_limits.GetXYIndexOfCellContainingPoint(hit.x(), hit.y()));
    hit_visitor(ends.back() / kSubpixelScale);
  }

  // Now add the misses.
  /**
  * @brief 现在加入未命中.
  */
  for (const Eigen::Array2i& end : ends) {
    CastRay(begin, end, miss_visitor);
  }

  // Finally, compute and add empty rays based on misses in the scan.
  /**
  * @brief 最后，根据扫描中的漏洞计算和添加空的光线.
  */
  for (const Eigen::Vector3f& missing_echo : range_data.misses) {
    CastRay(begin,
            superscaled_limits.GetXYIndexOfCellContainingPoint(
                missing_echo.x(), missing_echo.y()),
            miss_visitor);
  }
}

}  // namespace mapping_2d
}  // namespace cartographer
