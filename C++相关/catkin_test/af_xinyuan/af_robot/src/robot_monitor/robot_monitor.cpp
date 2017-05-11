/*
 * robot_monitor.cpp
 *
 *  Created on: Jan 4, 2017
 *      Author: shengsong
 */

#include <boost/thread.hpp>

#include "ros/ros.h"
#include "af_msgs/StatusCheck.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

class RobotMonitor
{
    ros::Publisher report_pub_;

    ros::Subscriber lidar_sub_, cams_sub_;
    af_msgs::StatusCheck * status_check_;

    boost::mutex check_mutex_;
    boost::thread report_thread_;

public:
    RobotMonitor();
    ~RobotMonitor();
    void lidarCheck(const sensor_msgs::LaserScan::ConstPtr& msg);
    void camsCheck(const nav_msgs::Odometry::ConstPtr& msg);
    void reportThread();
};

RobotMonitor::RobotMonitor()
{
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    std::string lidar_topic;
    std::string cams_topic;
    std::string status_topic;

    private_nh.param("lidar_topic", lidar_topic, (std::string)"/scan");
    private_nh.param("cams_topic", cams_topic, (std::string)"/visual_odom");
    private_nh.param("status_topic", status_topic, (std::string)"/status_check");

    lidar_sub_ = nh.subscribe(lidar_topic, 100, &RobotMonitor::lidarCheck, this);
    cams_sub_ = nh.subscribe(cams_topic, 100, &RobotMonitor::camsCheck, this);

    status_check_ = new af_msgs::StatusCheck();

    report_pub_ = nh.advertise<af_msgs::StatusCheck>(status_topic, 100);
    report_thread_ = boost::thread(&RobotMonitor::reportThread, this);
}

RobotMonitor::~RobotMonitor()
{
    if (status_check_ != NULL)
        delete status_check_;
}

void RobotMonitor::lidarCheck(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(check_mutex_);
    status_check_->lidar.resize(1);
    status_check_->lidar.assign(1,true);
}

void RobotMonitor::camsCheck(const nav_msgs::Odometry::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(check_mutex_);
    status_check_->camera.resize(1);
    status_check_->camera.assign(1, true);
}

void RobotMonitor::reportThread()
{
#ifndef REPORT_RATE
#define REPORT_RATE 10
#endif
    ros::Rate r(REPORT_RATE);
    while(ros::ok())
    {
        boost::mutex::scoped_lock lock(check_mutex_);
        report_pub_.publish(*status_check_);

        if (status_check_ != NULL)
            delete status_check_;
        status_check_ = new af_msgs::StatusCheck();
//        status_check_.lidar.resize(1);
//        status_check_.lidar.assign(1, false);
//
//        status_check_.camera.resize(1);
//        status_check_.camera.assign(1, false);

        lock.unlock();
        r.sleep();
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "robot_monitor");
    RobotMonitor monitor;
    ros::spin();
    return 0;
}

