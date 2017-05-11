/*
 * af_base.h
 *
 *  Created on: 2016年11月24日
 *      Author: shengsong
 */

#ifndef INCLUDE_AF_BASE_AF_BASE_H_
#define INCLUDE_AF_BASE_AF_BASE_H_

#include <boost/thread.hpp>

#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include "UartDriver.h"

#include "af_bringup/Robot_encode.h"
#include <geometry_msgs/Twist.h>


class AFBase
{
    ros::NodeHandle nh_;
    ros::Publisher base_pub_;
    ros::Subscriber base_sub_;

    ros::Subscriber alpha_sub_;

    int fd_;
    boost::thread read_thread;
    boost::mutex io_mutex;

    void writeCmd(short linear, short alpha);
    void writeCallback(const geometry_msgs::Twist & msg);
    void writeCallbackAlpha(const geometry_msgs::Twist & msg);

public:
    AFBase(std::string port="/dev/ttyUSB0", int baud_rate=115200, int data_bits=8, int stop_bits=1, int parity='N');
    ~AFBase();

    void readThread();
};


#endif /* INCLUDE_AF_BASE_AF_BASE_H_ */
