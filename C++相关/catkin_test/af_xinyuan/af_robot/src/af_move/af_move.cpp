/*
 * af_move.cpp
 *
 *  Created on: 2016年8月11日
 *      Author: shengsong
 */

#include "af_move/af_move.h"

AFMove::AFMove()
:n_path_(0), current_goal_(0), last_goal_(0),
 x_(0), y_(0), heading_(0)
{
    nh = ros::NodeHandle("~");
    nh.param< int >("retry_times", retry_times, 50);
    path_sub_ = nh_.subscribe("path", 10, &AFMove::pathCallback, this);
    path_fdbk_pub_ = nh_.advertise< std_msgs::Int8 >("path_fdbk", 10);

    goal_pub_ = nh_.advertise< move_base_msgs::MoveBaseActionGoal >("/move_base/goal", 10);
    cancel_pub_ = nh_.advertise< actionlib_msgs::GoalID >("move_base/cancel", 10);
    result_sub_ = nh_.subscribe("/move_base/result", 10, &AFMove::resultCallback, this);
//    feedback_sub_ = nh_.subscribe("/move_base/feedback", 10, &AFMove::feedbackCallback, this);

    current_goal_pub_ = nh_.advertise< af_msgs::Vertex >("current_goal", 10);
//    disable_sub_ = nh_.subscribe("disable_move", 10, &AFMove::disableCallback, this);
    current_result_pub_ = nh_.advertise< af_msgs::Vertex >("current_result", 10);

    pause_sub_ = nh_.subscribe("pause", 10, &AFMove::pauseCallback, this);
    continue_sub_ = nh_.subscribe("continue", 10, &AFMove::continueCallback, this);
    vertex_ = boost::shared_ptr< Vertex[] >(new Vertex[0]);
    try
    {
        listener_.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_);
        x_ = transform_.getOrigin().x();
        y_ = transform_.getOrigin().y();
        quaternion_ = transform_.getRotation();
        heading_ = int(tf::getYaw(transform_.getRotation()) / std::asin(1) * 90);
    }
    catch (...)
    {
        x_ = 0;
        y_ = 0;
        heading_ = 0;
        quaternion_ = tf::Quaternion(0, 0, 0, 0);
    }
    position_thread = boost::thread(&AFMove::positionThread, this);
}

int AFMove::pubCurrentGoal()
{std::cout << "Pub current_goal_=" << current_goal_ << std::endl;
    if (current_goal_ <= 0)
        return -1;
    if (current_goal_ >= n_path_)
        current_goal_ = n_path_ -1;
    // TODO 容易越界
//        current_goal_ = n_path_;

    if (vertex_[current_goal_].id == vertex_[current_goal_-1].id)
    {
        move_base_msgs::MoveBaseActionGoal goal_msg;
        goal_msg.goal.target_pose.header.frame_id = "map";
        goal_msg.goal.target_pose.pose.position.x = vertex_[current_goal_].x;
        goal_msg.goal.target_pose.pose.position.y = vertex_[current_goal_].y;
        goal_msg.goal.target_pose.pose.orientation.x = quaternion_.getX();
        goal_msg.goal.target_pose.pose.orientation.y = quaternion_.getY();
        goal_msg.goal.target_pose.pose.orientation.z = quaternion_.getZ();
        goal_msg.goal.target_pose.pose.orientation.w = quaternion_.getW();
        goal_pub_.publish(goal_msg); // move_base/goal

    }
    else
    {
        // 给move_base发送点vertex_[current_goal_] current_goal_从1索引
        Vertex direction_v;
        direction_v.x = vertex_[current_goal_].x - vertex_[current_goal_-1].x;
        direction_v.y = vertex_[current_goal_].y - vertex_[current_goal_-1].y;
        double direction;
        if (direction_v.x == 0)
            direction = 0;
        else if (direction_v.x > 0)
            direction = std::atan((vertex_[current_goal_].y - vertex_[current_goal_-1].y) / (vertex_[current_goal_].x - vertex_[current_goal_-1].x));
        else
        {
            if (direction_v.y > 0)
                direction = std::atan((vertex_[current_goal_].y - vertex_[current_goal_-1].y) / (vertex_[current_goal_].x - vertex_[current_goal_-1].x)) + 2*std::asin(1);
            else
                direction = std::atan((vertex_[current_goal_].y - vertex_[current_goal_-1].y) / (vertex_[current_goal_].x - vertex_[current_goal_-1].x)) - 2*std::asin(1);
        }

        move_base_msgs::MoveBaseActionGoal goal_msg;
        goal_msg.goal.target_pose.header.frame_id = "map";
        goal_msg.goal.target_pose.pose.position.x = vertex_[current_goal_].x;
        goal_msg.goal.target_pose.pose.position.y = vertex_[current_goal_].y;
        goal_msg.goal.target_pose.pose.orientation.x = 0;
        goal_msg.goal.target_pose.pose.orientation.y = 0;
        goal_msg.goal.target_pose.pose.orientation.z = std::sin(direction/2);
        goal_msg.goal.target_pose.pose.orientation.w = std::cos(direction/2);
        goal_pub_.publish(goal_msg); // move_base/goal
    }
    std::cout << "Published goal: "<< current_goal_ << " vertex id:" << vertex_[current_goal_].id << std::endl;

    af_msgs::Vertex current_goal_msg;
    current_goal_msg.id = vertex_[current_goal_].id; // vertex_从0索引,0为路上第一个节点
    current_goal_msg.x = vertex_[current_goal_].x; // current_goal从1索引
    current_goal_msg.y = vertex_[current_goal_].y;
    current_goal_pub_.publish(current_goal_msg); // ~/current_goal

    return 0;
}

void AFMove::pathCallback(const af_msgs::Path::ConstPtr& msg)
{
    std::cout << "进入pathCallback。。。npath_="<<msg->size << std::endl;
    if (msg->size <= 0)
    {
        // msg->size <=0 时 为停止命令
        actionlib_msgs::GoalID cancel_msg;
        cancel_pub_.publish(cancel_msg);
    }
    else
    {
	std::cout << "收到巡逻路线！" << std::endl;
        //           > 0 时 为有效路径
        n_path_ = msg->size;
        vertex_ = boost::shared_ptr< Vertex[] >(new Vertex[n_path_]);
        for (int i=0; i<n_path_; i++)
        {
            vertex_[i].id = msg->vertex[i].id;
            vertex_[i].x = msg->vertex[i].x;
            vertex_[i].y = msg->vertex[i].y;
        }
        last_goal_ = 0;
        current_goal_ = 1;
        pubCurrentGoal(); // !!!使用move_base规划至巡逻路线起点
    }
}

//deprecated!!!!!!
/*
void AFMove::feedbackCallback(const move_base_msgs::MoveBaseActionFeedback::ConstPtr& msg) // 按距离
{
    double x = msg->feedback.base_position.pose.position.x;
    double y = msg->feedback.base_position.pose.position.y;
    double distance = std::sqrt((x - vertex_[current_goal_].x) * (x - vertex_[current_goal_].x) + (y - vertex_[current_goal_].y) * (y - vertex_[current_goal_].y));
    std::cout << "feedbackCallback" << std::endl;

    if (distance < REACH_DIST)
    {
        std::cout << "distance < REACH_DIST current_goal_="<<current_goal_<<std::endl;
        //发布到达当前位置
        if (last_goal_ == current_goal_)
            return;
        else
            last_goal_ = current_goal_;

        af_msgs::Vertex current_result_msg;
        current_result_msg.id = vertex_[current_goal_].id;
        current_result_msg.x = vertex_[current_goal_].x;
        current_result_msg.y = vertex_[current_goal_].y;
        current_result_pub_.publish(current_result_msg);

//        if (current_goal_ < n_path_ - 1)
//        {
            current_goal_ ++;
            pubCurrentGoal();
//        }
    }
}
*/

void AFMove::resultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
    std::cout << "resultCallback status="<< msg->status.status << std::endl;
    std_msgs::Int8 path_fdbk_msg;
    static int giveup_cnt = 0;
    if (msg->status.status == 4)
        giveup_cnt ++;
    else if (msg->status.status == 3)
        giveup_cnt = 0;
    switch (msg->status.status)
    {
    case 2:
        path_fdbk_msg.data = 2;
        path_fdbk_pub_.publish(path_fdbk_msg);
        break;
    case 3:
        std::cout << "result current_goal_=" << current_goal_  << std::endl;
//        if (current_goal_ == n_path_ - 1)
        if (current_goal_ >= n_path_)
        {
            path_fdbk_msg.data = 3;
            path_fdbk_pub_.publish(path_fdbk_msg);
        }
//        else
//        {
//            pubCurrentGoal();
//        }
//        else
//        {
//            path_fdbk_msg.data = 0;
//            path_fdbk_pub_.publish(path_fdbk_msg);
//        }

        break;
    case 4:
        std::cout << giveup_cnt << std::endl;
        if (giveup_cnt < retry_times)
            pubCurrentGoal();
        else
        {
            path_fdbk_msg.data = 4;
            path_fdbk_pub_.publish(path_fdbk_msg);
        }
        break;
    default:
        break;
    }
}

void AFMove::pauseCallback(const std_msgs::Int8::ConstPtr& msg)
{
    actionlib_msgs::GoalID cancel_msg;
    cancel_pub_.publish(cancel_msg);
}

void AFMove::continueCallback(const std_msgs::Int8::ConstPtr& msg)
{
    pubCurrentGoal();
}

void AFMove::positionThread()
{
    ros::Rate r(10);
    while(ros::ok())
    {
        try
        {
            listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_);
            x_ = transform_.getOrigin().x();
            y_ = transform_.getOrigin().y();
            quaternion_ = transform_.getRotation();
            heading_ = int(tf::getYaw(transform_.getRotation()) / std::asin(1) * 90);
        }
        catch (...)
        {
            ROS_WARN_ONCE("Transform from /map to /base_footprint is not running...");
            x_ = 0;
            y_ = 0;
            quaternion_ = tf::Quaternion(0, 0, 0, 0);
            heading_ = 0;
        }

        double distance = std::sqrt((x_ - vertex_[current_goal_].x) * (x_ - vertex_[current_goal_].x) + (y_ - vertex_[current_goal_].y) * (y_ - vertex_[current_goal_].y));
            //std::cout << heading_ << std::endl;

        if (distance < REACH_DIST)
        {
//            std::cout << "distance < REACH_DIST current_goal_="<<current_goal_<<std::endl;
            //发布到达当前位置
            if (last_goal_ == current_goal_) //可能冗余
                continue;
            else
                last_goal_ = current_goal_;

            af_msgs::Vertex current_result_msg;
            current_result_msg.id = vertex_[current_goal_].id;
            current_result_msg.x = vertex_[current_goal_].x;
            current_result_msg.y = vertex_[current_goal_].y;
            current_result_pub_.publish(current_result_msg);

            current_goal_ ++;
            pubCurrentGoal();
        }
    }
}
