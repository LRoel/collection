/*
 * function_switch.h
 *
 *  Created on: Dec 17, 2016
 *      Author: shengsong
 */

#ifndef INCLUDE_FUNCTION_SWITCH_FUNCTION_SWITCH_H_
#define INCLUDE_FUNCTION_SWITCH_FUNCTION_SWITCH_H_

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "tf/transform_listener.h"
#include "boost/thread.hpp"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

class FunctionSwitch
{
    ros::Subscriber switch_sub_;




public:
    FunctionSwitch();
    void switcher(const std_msgs::Int8::ConstPtr& msg);
    void acquirePose();
    void positionThread();

    tf::Pose current_pose_;
    tf::TransformListener listener_;
    tf::StampedTransform transform_;

    boost::mutex pose_mutex_;
    boost::thread position_thread_;

    ros::Publisher initial_pose_pub_;

};





#endif /* INCLUDE_FUNCTION_SWITCH_FUNCTION_SWITCH_H_ */
