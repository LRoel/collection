/*
 * af_robot.h
 *
 *  Created on: 2016年8月18日
 *      Author: shengsong
 */

#ifndef INCLUDE_AF_ROBOT_AF_ROBOT_H_
#define INCLUDE_AF_ROBOT_AF_ROBOT_H_

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"

#include "af_msgs/Path.h"
#include "af_msgs/Vertex.h"
#include "af_msgs/RobotState.h"

#include "map_db.h"

#define REPORT_RATE 10.0

#define MAX_LINEAR_SPEED 0.3
#define MAX_ANGULAR_SPEED 0.5
#define MAX_STEERING_ANGLE 19

#define LOW_BATTERY 40
#define FULL_BATTERY 98

#define PLANNING 0x00
#define PATROLLING 0x01
#define NAVIGATING 0x02
#define IDLING 0x03
#define CONTROLLING 0x04 //TODO redundant to IDLING
#define CHARGING 0x05 //TODO redundant to NAVIGATING
#define MALFUNC 0x06 //TODO redundant for whole robot system

// ros 接收状态表
#define PATROLLING_R 0x01
#define NAVIGATING_R 0x02
#define IDLING_R 0x03 //< 遇到行人停止
#define CONTROLLING_R 0x04
#define STRING_LIGHT_R 0x05
#define PHOTO_TAKEN_R 0x06

// ros 发送状态表、android 接收状态表
#define PATROLLING_A 0x01
#define NAVIGATING_A 0x02
#define CONTROLLING_A 0x03
#define TOCHARGE_A 0x04 //< 去充电
#define CHARGING_A 0x05 //< 充电中
#define CHARGED_A 0x06 //< 充电完成
#define IDLING_A 0x07 //< 停止、待命
#define MALFUNC_A 0x08 //< 故障

#define CANCEL 0x00
#define INTERCOM_PAUSE 0x01
#define PEDESTRIAN_PAUSE 0x02
#define PHOTO_PAUSE 0x03

class AFRobot : public Map
{
    int current_state_;
    int current_id_;

    void androidCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);
    void cmdFdbkCallback(const std_msgs::Int8::ConstPtr& msg);
    void currentGoalCallback(const af_msgs::Vertex::ConstPtr& msg);
    void currentResultCallback(const af_msgs::Vertex::ConstPtr& msg);
    void stateCallback(const af_msgs::RobotState::ConstPtr& msg);
    void initStateCallback(const std_msgs::Bool::ConstPtr& msg);
    void cameraCallback(const std_msgs::String::ConstPtr& msg);
//    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void reportThread();

    int checksum(std_msgs::Int8MultiArray msg);
    int pauseMove(int event_index, bool pause_flag = true);

    ros::NodeHandle nh_;
    ros::Subscriber android_sub_;
    ros::Publisher android_pub_;
    ros::Subscriber cmd_fdbk_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher alpha_pub_;


    ros::Publisher vel_pub_;
    ros::Publisher disable_pub_;

    ros::Subscriber current_goal_sub_;
    ros::Subscriber current_result_sub_;

    ros::Subscriber state_sub_;
    ros::Subscriber init_state_sub_;

    ros::Subscriber camera_sub_;
    ros::Publisher camera_pub_;

//    ros::Publisher cancel_pub_;

    ros::Publisher continue_pub_;
    ros::Publisher pause_pub_;

    ros::Publisher marker_pub_;
    ros::Publisher text_pub_;
//    ros::Subscriber costmap_sub_;

    tf::TransformListener listener_;
    tf::StampedTransform transform_;

    double x_, y_;
    int battery_percentage_, inner_temperature_;
    bool malfunc_flag_, lifting_flag_, emergency_stop_flag_, call_security_flag_,
        front_collision_flag_, deviated_flag_;
    bool photo_flag_;

    boost::mutex report_m;
    boost::thread report_thread_;

    int last_patrol_id;
    int current_result_;
public:
    AFRobot();
};



#endif /* INCLUDE_AF_ROBOT_AF_ROBOT_H_ */
