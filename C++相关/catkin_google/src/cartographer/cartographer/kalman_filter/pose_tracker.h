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

#ifndef CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_
#define CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_

#include <deque>
#include <memory>

#include "Eigen/Cholesky"
#include "Eigen/Core"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/kalman_filter/gaussian_distribution.h"
#include "cartographer/kalman_filter/proto/pose_tracker_options.pb.h"
#include "cartographer/kalman_filter/unscented_kalman_filter.h"
#include "cartographer/mapping/imu_tracker.h"
#include "cartographer/mapping/odometry_state_tracker.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace kalman_filter {

typedef Eigen::Matrix3d Pose2DCovariance;
typedef Eigen::Matrix<double, 6, 6> PoseCovariance;

struct PoseAndCovariance {
  transform::Rigid3d pose;
  PoseCovariance covariance;
};

PoseAndCovariance operator*(const transform::Rigid3d& transform,
                            const PoseAndCovariance& pose_and_covariance);

// Projects a 3D pose covariance into a 2D pose covariance.
/**
* @brief 将3D姿态协方差投影到2D姿态协方差.
*/
Pose2DCovariance Project2D(const PoseCovariance& embedded_covariance);

// Embeds a 2D pose covariance into a 3D pose covariance.
/**
* @brief 将2D姿态协方差嵌入到3D姿态协方差中.
*/
PoseCovariance Embed3D(const Pose2DCovariance& embedded_covariance,
                       double position_variance, double orientation_variance);

PoseCovariance BuildPoseCovariance(double translational_variance,
                                   double rotational_variance);

// Deserializes the 'proto_matrix' into a PoseCovariance.
/**
* @brief 将'proto_matrix'反序列化为PoseCovariance.
*/
PoseCovariance PoseCovarianceFromProtoMatrix(
    const sensor::proto::Matrix& proto_matrix);

proto::PoseTrackerOptions CreatePoseTrackerOptions(
    common::LuaParameterDictionary* parameter_dictionary);

// A Kalman filter for a 3D state of position and orientation.
/**
* @brief 一种卡尔曼滤波器，用于3D状态的位置和方向.
*/
// Includes functions to update from IMU and laser scan matches.
/**
* @brief 包括从IMU和激光扫描匹配更新的功能.
*/
class PoseTracker {
 public:
  enum {
    kMapPositionX = 0,
    kMapPositionY,
    kMapPositionZ,
    kMapOrientationX,
    kMapOrientationY,
    kMapOrientationZ,
    kMapVelocityX,
    kMapVelocityY,
    kMapVelocityZ,
    kDimension  // We terminate loops with this.
  };

  enum class ModelFunction { k2D, k3D };

  using KalmanFilter = UnscentedKalmanFilter<double, kDimension>;
  using State = KalmanFilter::StateType;
  using StateCovariance = Eigen::Matrix<double, kDimension, kDimension>;
  using Distribution = GaussianDistribution<double, kDimension>;

  // Create a new Kalman filter starting at the origin with a standard normal
  // distribution at 'time'.
  /**
  * @brief 从原点创建一个新的卡尔曼滤波器，在“时间”处具有标准正态分布.
  */
  PoseTracker(const proto::PoseTrackerOptions& options,
              const ModelFunction& model_function, common::Time time);
  virtual ~PoseTracker();

  // Sets 'pose' and 'covariance' to the mean and covariance of the belief at
  // 'time' which must be >= to the current time. Must not be nullptr.
  /**
  * @brief 将“姿态”和“协方差”设置为“时间”上的信念的平均和协方差，其必须>当前时间.不能为空.
  */
  void GetPoseEstimateMeanAndCovariance(common::Time time,
                                        transform::Rigid3d* pose,
                                        PoseCovariance* covariance);

  // Updates from an IMU reading (in the IMU frame).
  /**
  * @brief IMU阅读的更新（IMU框架）.
  */
  void AddImuLinearAccelerationObservation(
      common::Time time, const Eigen::Vector3d& imu_linear_acceleration);
  void AddImuAngularVelocityObservation(
      common::Time time, const Eigen::Vector3d& imu_angular_velocity);

  // Updates from a pose estimate in the map frame.
  /**
  * @brief 从地图框架中的姿态估计更新.
  */
  void AddPoseObservation(common::Time time, const transform::Rigid3d& pose,
                          const PoseCovariance& covariance);

  // Updates from a pose estimate in the odometer's map-like frame.
  /**
  * @brief 在里程表的地图框架中进行姿态估计的更新.
  */
  void AddOdometerPoseObservation(common::Time time,
                                  const transform::Rigid3d& pose,
                                  const PoseCovariance& covariance);

  common::Time time() const { return time_; }

  // Returns the belief at the 'time' which must be >= to the current time.
  /**
  * @brief 返回在'时间'的信念，它必须>当前时间> =.
  */
  Distribution GetBelief(common::Time time);

  const mapping::OdometryStateTracker::OdometryStates& odometry_states() const;

 private:
  // Returns the distribution required to initialize the KalmanFilter.
  /**
  * @brief 返回初始化KalmanFilter所需的分发.
  */
  static Distribution KalmanFilterInit();

  // Build a model noise distribution (zero mean) for the given time delta.
  /**
  * @brief 建立给定时间增量的模型噪声分布（零均值）.
  */
  const Distribution BuildModelNoise(double delta_t) const;

  // Predict the state forward in time. This is a no-op if 'time' has not moved
  // forward.
  /**
  * @brief 及时预测状态.如果“时间”没有向前移动，这是无效的.
  */
  void Predict(common::Time time);

  // Computes a pose combining the given 'state' with the 'imu_tracker_'
  // orientation.
  /**
  * @brief 计算将给定的“状态”与“imu_tracker_”方向相结合的姿势.
  */
  transform::Rigid3d RigidFromState(const PoseTracker::State& state);

  const proto::PoseTrackerOptions options_;
  const ModelFunction model_function_;
  common::Time time_;
  KalmanFilter kalman_filter_;
  mapping::ImuTracker imu_tracker_;
  mapping::OdometryStateTracker odometry_state_tracker_;
};

}  // namespace kalman_filter
}  // namespace cartographer

#endif  // CARTOGRAPHER_KALMAN_FILTER_POSE_TRACKER_H_
