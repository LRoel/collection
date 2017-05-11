/*
 * af_path.h
 *
 *  Created on: 2016年8月16日
 *      Author: shengsong
 */

#ifndef INCLUDE_AF_PATH_H_
#define INCLUDE_AF_PATH_H_

//TODO af_path af_robot 公用 ros内部

#define PLANNING 0x00
#define PATROLLING 0x01
#define NAVIGATING 0x02
#define IDLING 0x03
#define CONTROLLING 0x04 //TODO redundant to IDLING
#define CHARGING 0x05 //TODO redundant to NAVIGATING
#define MALFUNC 0x06 //TODO redundant for whole robot system

#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int8MultiArray.h"
#include "tf/transform_listener.h"

#include "af_path/af_path_planner.h"
#include "af_msgs/Path.h"

class AFPath : public AFPathPlanner
{
    ros::NodeHandle nh_;

    ros::Subscriber cmd_sub_;
//    ros::Publisher cmd_fdbk_pub_;
    ros::Publisher path_pub_;
//    ros::Subscriber path_fdbk_sub_;

    int size_;
    boost::shared_ptr< Vertex[] > vertex_;

    int current_state_;

    int clearPath();
    int getPatrolPath(int id);
    int getNavPath(int id);
    int pubPath();

    void cmdCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);
//    void pathFdbkCallback(const std_msgs::Int8::ConstPtr& msg);

    tf::TransformListener listener_;
    tf::StampedTransform transform_;

    double x_, y_;

public:
    AFPath(const std::string host = HOST, const std::string user = USER, const std::string pass = PASS, const std::string database = DB);

};


#endif /* INCLUDE_AF_PATH_H_ */
