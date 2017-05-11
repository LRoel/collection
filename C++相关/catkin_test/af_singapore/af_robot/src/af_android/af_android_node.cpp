/*
 * af_android_node.cpp
 *
 *  Created on: 2016年8月26日
 *      Author: shengsong
 */
#include "af_android/af_android.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "af_android");
    ros::NodeHandle nh("~");
    std::string port;
    int baud_rate;
    int data_bits;
    int stop_bits;
    int parity;
    nh.param< std::string >("port", port, "/dev/ttyUSB.android");
    nh.param< int >("baud_rate", baud_rate, 115200);
    nh.param< int >("data_bits", data_bits, 8);
    nh.param< int >("stop_bits", stop_bits, 1);
    nh.param< int >("parity", parity, 'N');

    ROS_INFO("串口");
    std::cout << "串口:" << port << std::endl;
//    try
//    {
        AFAndroid af_android(port, baud_rate, data_bits, stop_bits, parity);
//    }
//    catch (int e)
//    {
//        if (e == -1)
//            std::cout << "串口连接失败" << std::endl;
//        return -1;
//    }
    ros::spin();
    return 0;
}


