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

  unsigned char check_sum(unsigned char *buffer, int start, int end);
    void chargingCallback(const std_msgs::Int8MultiArray::ConstPtr& msg) {
      if (msg->data[0] == 0)
        return;

      unsigned char buffer[8] = {};

      buffer[0] = 0x59;
      buffer[1] = msg->data[1] & 0xFF;
      buffer[2] = msg->data[2] & 0xFF;
      buffer[7] = check_sum(buffer, 1, 6) & 0xFF;

      write(fd_, buffer, 8);
      ROS_INFO("电量电流信息下发到串口");
    }
  ros::Subscriber charging_sub_;

public:
    AFBase(std::string port="/dev/ttyUSB0", int baud_rate=115200, int data_bits=8, int stop_bits=1, int parity='N');
    ~AFBase();

    void readThread();
};



#endif /* INCLUDE_AF_BASE_AF_BASE_H_ */
