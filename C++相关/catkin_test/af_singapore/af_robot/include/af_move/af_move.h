/*
 * af_move.h
 *
 *  Created on: 2016年8月18日
 *      Author: shengsong
 */

#ifndef INCLUDE_AF_MOVE_AF_MOVE_H_
#define INCLUDE_AF_MOVE_AF_MOVE_H_

#define REACH_DIST 2.5
#include <cmath>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "std_msgs/Int8.h"
#include "tf/transform_listener.h"

#include "af_msgs/Path.h"

#include "map_struct.h"
/**
 * brief
 */

class AFMove
{
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_; //< size为0则停止
    ros::Publisher path_fdbk_pub_; //< 完成反馈

    ros::Publisher goal_pub_; //<
    ros::Publisher cancel_pub_;
    ros::Subscriber result_sub_;
    ros::Subscriber feedback_sub_;

    ros::Publisher current_goal_pub_;
//    ros::Subscriber disable_sub_;
    ros::Publisher current_result_pub_;


    ros::Subscriber pause_sub_;
    ros::Subscriber continue_sub_;

    boost::shared_ptr< Vertex[] > vertex_;

    void pathCallback(const af_msgs::Path::ConstPtr& msg);
    void resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
    void feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg);
//    void disableCallback(const std_msgs::Int8::ConstPtr& msg);
    void pauseCallback(const std_msgs::Int8::ConstPtr& msg);
    void continueCallback(const std_msgs::Int8::ConstPtr& msg);

    int n_path_, current_goal_;
    int pubCurrentGoal();

    int last_goal_;
    ros::NodeHandle nh;
    int retry_times;

    boost::thread position_thread;
    void positionThread();
    double x_, y_;
    tf::Quaternion quaternion_;
    double heading_;
    tf::TransformListener listener_;
    tf::StampedTransform transform_;
public:
    AFMove();
};




#endif /* INCLUDE_AF_MOVE_AF_MOVE_H_ */
