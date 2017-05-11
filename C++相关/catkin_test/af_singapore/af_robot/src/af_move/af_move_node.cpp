/*
 * af_move_node.cpp
 *
 *  Created on: 2016年8月18日
 *      Author: shengsong
 */
#include "af_move/af_move.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "af_move");
    ROS_INFO("af_move node initialized.");
    AFMove af_move;
    ros::spin();
    return 0;
}


