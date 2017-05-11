/*
 * af_robot_node.cpp
 *
 *  Created on: 2016年8月20日
 *      Author: shengsong
 */

#include "af_robot/af_robot.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "af_robot");
    AFRobot af_robot;
//    std::cout << tf::getYaw(tf::Quaternion(0,0,0,1)) <<std::endl;
    ros::spin();
    return 0;
}
