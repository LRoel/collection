/*
 * af_android.h
 *
 *  Created on: 2016年8月26日
 *      Author: shengsong
 */

#ifndef INCLUDE_AF_ANDROID_AF_ANDROID_H_
#define INCLUDE_AF_ANDROID_AF_ANDROID_H_

#include <boost/thread.hpp>


#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include "UartDriver.h"

class AFAndroid
{
    ros::NodeHandle nh_;
    ros::Publisher android_pub_;
    ros::Subscriber android_sub_;

    int fd_;
    boost::thread read_thread;
    boost::mutex io_mutex;

    void writeCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);

public:
    AFAndroid(std::string port="/dev/ttyUSB0", int baud_rate=115200, int data_bits=8, int stop_bits=1, int parity='N');
    ~AFAndroid();

    void readThread();
};




#endif /* INCLUDE_AF_ANDROID_AF_ANDROID_H_ */
