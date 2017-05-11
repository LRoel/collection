/*
 * af_base_node.cpp
 *
 *  Created on: 2016年11月24日
 *      Author: shengsong
 */

#include "af_base/af_base.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "af_base");
    ros::NodeHandle nh("~");
    std::string port;
    int baud_rate;
    int data_bits;
    int stop_bits;
    int parity;
    nh.param< std::string >("port", port, "/dev/ttyUSB.base");
    nh.param< int >("baud_rate", baud_rate, 115200);
    nh.param< int >("data_bits", data_bits, 8);
    nh.param< int >("stop_bits", stop_bits, 1);
    nh.param< int >("parity", parity, 'N');

    ROS_INFO("串口");
    std::cout << "串口:" << port << std::endl;

    AFBase af_base(port, baud_rate, data_bits, stop_bits, parity);

    ros::spin();
    return 0;
}
