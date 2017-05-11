/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include <costmap_2d/range_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(costmap_2d::RangeLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace costmap_2d
{
  void RangeLayer::onInitialize()
  {
    ObstacleLayer::onInitialize();
    ros::NodeHandle private_nh("~/" + name_);
  }

  void RangeLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
  {
    range_dsrv_ = new dynamic_reconfigure::Server<costmap_2d::RangePluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::RangePluginConfig>::CallbackType cb = boost::bind(
            &RangeLayer::reconfigureCB, this, _1, _2);
    range_dsrv_->setCallback(cb);
  }

  void RangeLayer::reconfigureCB(costmap_2d::RangePluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
    footprint_clearing_enabled_ = config.footprint_clearing_enabled;
    max_obstacle_height_ = config.max_obstacle_height;
    combination_method_ = config.combination_method;
  }

  void RangeLayer::raytraceFreespace(const Observation& clearing_observation, double* min_x, double* min_y,
                                                double* max_x, double* max_y) {
    double ox = clearing_observation.origin_.x;
    double oy = clearing_observation.origin_.y;
    pcl::PointCloud<pcl::PointXYZ> cloud = *(clearing_observation.cloud_);

    // get the map coordinates of the origin of the sensor
    unsigned int x0, y0;
    if (!worldToMap(ox, oy, x0, y0)) {
      ROS_WARN_THROTTLE(
              1.0,
              "The origin for the sensor at (%.2f, %.2f) is out of map bounds. So, the costmap cannot raytrace for it.",
              ox, oy);
      return;
    }

    // we can pre-compute the enpoints of the map outside of the inner loop... we'll need these later
    double origin_x = origin_x_, origin_y = origin_y_;
    double map_end_x = origin_x + size_x_ * resolution_;
    double map_end_y = origin_y + size_y_ * resolution_;


    touch(ox, oy, min_x, min_y, max_x, max_y);

    // for each point in the cloud, we want to trace a line from the origin and clear obstacles along it
    for (unsigned int i = 0; i < cloud.points.size(); ++i) {
      double wx = cloud.points[i].x;
      double wy = cloud.points[i].y;

      // now we also need to make sure that the enpoint we're raytracing
      // to isn't off the costmap and scale if necessary
      double a = wx - ox;
      double b = wy - oy;

      // the minimum value to raytrace from is the origin
      if (wx < origin_x) {
        double t = (origin_x - ox) / a;
        wx = origin_x;
        wy = oy + b * t;
      }
      if (wy < origin_y) {
        double t = (origin_y - oy) / b;
        wx = ox + a * t;
        wy = origin_y;
      }

      // the maximum value to raytrace to is the end of the map
      if (wx > map_end_x) {
        double t = (map_end_x - ox) / a;
        wx = map_end_x - .001;
        wy = oy + b * t;
      }
      if (wy > map_end_y) {
        double t = (map_end_y - oy) / b;
        wx = ox + a * t;
        wy = map_end_y - .001;
      }

      // now that the vector is scaled correctly... we'll get the map coordinates of its endpoint
      unsigned int x1, y1;

      // check for legality just in case
      if (!worldToMap(wx, wy, x1, y1))
        continue;

      unsigned int cell_raytrace_range = cellDistance(clearing_observation.raytrace_range_);
      // unsigned int min_range_limit = cellDistance(raytrace_limit_);
      unsigned int min_range_limit = cellDistance(0.35);
      MarkCell marker(costmap_, FREE_SPACE);
      // and finally... we can execute our trace to clear obstacles along that line
      raytraceLine_limit(marker, x0, y0, x1, y1, cell_raytrace_range, min_range_limit);

      updateRaytraceBounds(ox, oy, wx, wy, clearing_observation.raytrace_range_, min_x, min_y, max_x, max_y);
    }
  }

}  // namespace costmap_2d
