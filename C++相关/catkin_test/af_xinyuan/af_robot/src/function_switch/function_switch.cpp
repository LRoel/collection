/*
 * function_switcher.cpp
 *
 *  Created on: Dec 17, 2016
 *      Author: shengsong
 */

#include "function_switch/function_switch.h"



FunctionSwitch::FunctionSwitch()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string switch_topic;
//    private_nh.param("switch_topic", switch_topic, (std::string)"/switch");
    nh.param("switch_topic", switch_topic, (std::string)"/switch");

    switch_sub_ = nh.subscribe(switch_topic, 100, &FunctionSwitch::switcher, this);

    try
    {
        listener_.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0));
    }
    catch (...)
    {
    }

    position_thread_ = boost::thread(&FunctionSwitch::positionThread, this);
    initial_pose_pub_ = nh.advertise <geometry_msgs::PoseWithCovarianceStamped> ("initial_pose", 100);
}

void FunctionSwitch::positionThread()
{
    ros::Rate r(10);

    while(ros::ok())
    {
        boost::mutex::scoped_lock lock(pose_mutex_);
        try
        {
            listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_);
            current_pose_.setOrigin(transform_.getOrigin());
            current_pose_.setRotation(transform_.getRotation());
        }
        catch (...)
        {
            ROS_WARN_ONCE("Transform from /map to /base_footprint is not running...");
        }
        lock.unlock();
        r.sleep();
    }
}

void FunctionSwitch::switcher(const std_msgs::Int8::ConstPtr& msg)
{
    switch(msg->data)
    {
        case 0:
            // 关闭导航
            system("pkill amcl");
            // 启动建图脚本
            system("~/catkin_ws/src/af_robot/launch/gmapping.sh");
        break;
    case -1:
	{
        // 记录位置
       boost::mutex::scoped_lock lock(pose_mutex_);
       geometry_msgs::PoseWithCovarianceStamped pose_msg;
       pose_msg.pose.pose.position.x = current_pose_.getOrigin().x();
       pose_msg.pose.pose.position.y = current_pose_.getOrigin().y();
       pose_msg.pose.pose.position.z = current_pose_.getOrigin().z();

       pose_msg.pose.pose.orientation.x = current_pose_.getRotation().getX();
       pose_msg.pose.pose.orientation.y = current_pose_.getRotation().getY();
       pose_msg.pose.pose.orientation.z = current_pose_.getRotation().getZ();
       pose_msg.pose.pose.orientation.w = current_pose_.getRotation().getW();

       system("pkill gmapping");
       // 导航
       system("~/catkin_ws/src/af_robot/launch/amcl.sh");
       // 设置初始位置
       initial_pose_pub_.publish(pose_msg);
	}
        break;
    default:
        // 清空
        break;
    }
}
