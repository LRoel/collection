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

#ifndef CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_GRID_H_
#define CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_GRID_H_

#include <cmath>

#include "cartographer/mapping_3d/hybrid_grid.h"

namespace cartographer {
namespace mapping_3d {
namespace scan_matching {

// Interpolates between HybridGrid probability voxels. We use the tricubic
// interpolation which interpolates the values and has vanishing derivative at
// these points.
/**
* @brief 在HybridGrid概率体素之间进行插值.我们使用三角插值来插值，并在这些点上消失衍生.
*/
//
// This class is templated to work with the autodiff that Ceres provides.
/**
* @brief 这个类是Ceres提供的autodiff的模板.
*/
// For this reason, it is also important that the interpolation scheme be
// continuously differentiable.
/**
* @brief 因此，插值方案可以连续地区分也是重要的.
*/
class InterpolatedGrid {
 public:
  explicit InterpolatedGrid(const HybridGrid& hybrid_grid)
      : hybrid_grid_(hybrid_grid) {}

  InterpolatedGrid(const InterpolatedGrid&) = delete;
  InterpolatedGrid& operator=(const InterpolatedGrid&) = delete;

  // Returns the interpolated probability at (x, y, z) of the HybridGrid
  // used to perform the interpolation.
  /**
  * @brief 返回用于执行插值的HybridGrid的（x，y，z）的内插概率.
  */
  //
  // This is a piecewise, continuously differentiable function. We use the
  // scalar part of Jet parameters to select our interval below. It is the
  // tensor product volume of piecewise cubic polynomials that interpolate
  // the values, and have vanishing derivative at the interval boundaries.
  /**
  * @brief 这是一个分段，持续的差异化功能.我们使用Jet参数的标量部分来选择我们的间隔.它是分段值的分段三次多项式的张量积，并且在间隔边界处具有消失的导数.
  */
  template <typename T>
  T GetProbability(const T& x, const T& y, const T& z) const {
    double x1, y1, z1, x2, y2, z2;
    ComputeInterpolationDataPoints(x, y, z, &x1, &y1, &z1, &x2, &y2, &z2);

    const Eigen::Array3i index1 =
        hybrid_grid_.GetCellIndex(Eigen::Vector3f(x1, y1, z1));
    const double q111 = hybrid_grid_.GetProbability(index1);
    const double q112 =
        hybrid_grid_.GetProbability(index1 + Eigen::Array3i(0, 0, 1));
    const double q121 =
        hybrid_grid_.GetProbability(index1 + Eigen::Array3i(0, 1, 0));
    const double q122 =
        hybrid_grid_.GetProbability(index1 + Eigen::Array3i(0, 1, 1));
    const double q211 =
        hybrid_grid_.GetProbability(index1 + Eigen::Array3i(1, 0, 0));
    const double q212 =
        hybrid_grid_.GetProbability(index1 + Eigen::Array3i(1, 0, 1));
    const double q221 =
        hybrid_grid_.GetProbability(index1 + Eigen::Array3i(1, 1, 0));
    const double q222 =
        hybrid_grid_.GetProbability(index1 + Eigen::Array3i(1, 1, 1));

    const T normalized_x = (x - x1) / (x2 - x1);
    const T normalized_y = (y - y1) / (y2 - y1);
    const T normalized_z = (z - z1) / (z2 - z1);

    // Compute pow(..., 2) and pow(..., 3). Using pow() here is very expensive.
    /**
    * @brief 计算电力（.，2）和pow（.，3）.使用pow（）这里是非常昂贵的.
    */
    const T normalized_xx = normalized_x * normalized_x;
    const T normalized_xxx = normalized_x * normalized_xx;
    const T normalized_yy = normalized_y * normalized_y;
    const T normalized_yyy = normalized_y * normalized_yy;
    const T normalized_zz = normalized_z * normalized_z;
    const T normalized_zzz = normalized_z * normalized_zz;

    // We first interpolate in z, then y, then x. All 7 times this uses the same
    // scheme: A * (2t^3 - 3t^2 + 1) + B * (-2t^3 + 3t^2).
    /**
    * @brief 我们首先在z内插，然后y，然后x.全部7次使用相同的方案：A *（2t ^ 3  -  3t ^ 2 + 1）+ B *（-2t ^ 3 + 3t ^ 2）.
    */
    // The first polynomial is 1 at t=0, 0 at t=1, the second polynomial is 0
    // at t=0, 1 at t=1. Both polynomials have derivative zero at t=0 and t=1.
    /**
    * @brief 第一个多项式在t = 0时为1，在t = 1时为0，在t = 0时第二个多项式为0，t = 1时第一个多项式为1.两个多项式在t = 0和t = 1时都具有微分零点.
    */
    const T q11 = (q111 - q112) * normalized_zzz * 2. +
                  (q112 - q111) * normalized_zz * 3. + q111;
    const T q12 = (q121 - q122) * normalized_zzz * 2. +
                  (q122 - q121) * normalized_zz * 3. + q121;
    const T q21 = (q211 - q212) * normalized_zzz * 2. +
                  (q212 - q211) * normalized_zz * 3. + q211;
    const T q22 = (q221 - q222) * normalized_zzz * 2. +
                  (q222 - q221) * normalized_zz * 3. + q221;
    const T q1 = (q11 - q12) * normalized_yyy * 2. +
                 (q12 - q11) * normalized_yy * 3. + q11;
    const T q2 = (q21 - q22) * normalized_yyy * 2. +
                 (q22 - q21) * normalized_yy * 3. + q21;
    return (q1 - q2) * normalized_xxx * 2. + (q2 - q1) * normalized_xx * 3. +
           q1;
  }

 private:
  template <typename T>
  void ComputeInterpolationDataPoints(const T& x, const T& y, const T& z,
                                      double* x1, double* y1, double* z1,
                                      double* x2, double* y2,
                                      double* z2) const {
    const Eigen::Vector3f lower = CenterOfLowerVoxel(x, y, z);
    *x1 = lower.x();
    *y1 = lower.y();
    *z1 = lower.z();
    *x2 = lower.x() + hybrid_grid_.resolution();
    *y2 = lower.y() + hybrid_grid_.resolution();
    *z2 = lower.z() + hybrid_grid_.resolution();
  }

  // Center of the next lower voxel, i.e., not necessarily the voxel containing
  // (x, y, z). For each dimension, the largest voxel index so that the
  // corresponding center is at most the given coordinate.
  /**
  * @brief 下一个下体素的中心，我.e.，不一定是含有（x，y，z）.对于每个维度，最大体素索引，使得对应的中心至多为给定的坐标.
  */
  Eigen::Vector3f CenterOfLowerVoxel(const double x, const double y,
                                     const double z) const {
    // Center of the cell containing (x, y, z).
    /**
    * @brief 含有（x，y，z）.
    */
    Eigen::Vector3f center = hybrid_grid_.GetCenterOfCell(
        hybrid_grid_.GetCellIndex(Eigen::Vector3f(x, y, z)));
    // Move to the next lower voxel center.
    /**
    * @brief 移动到下一个较低体素中心.
    */
    if (center.x() > x) {
      center.x() -= hybrid_grid_.resolution();
    }
    if (center.y() > y) {
      center.y() -= hybrid_grid_.resolution();
    }
    if (center.z() > z) {
      center.z() -= hybrid_grid_.resolution();
    }
    return center;
  }

  // Uses the scalar part of a Ceres Jet.
  /**
  * @brief 使用Ceres Jet的标量部分.
  */
  template <typename T>
  Eigen::Vector3f CenterOfLowerVoxel(const T& jet_x, const T& jet_y,
                                     const T& jet_z) const {
    return CenterOfLowerVoxel(jet_x.a, jet_y.a, jet_z.a);
  }

  const HybridGrid& hybrid_grid_;
};

}  // namespace scan_matching
}  // namespace mapping_3d
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_3D_SCAN_MATCHING_INTERPOLATED_GRID_H_
