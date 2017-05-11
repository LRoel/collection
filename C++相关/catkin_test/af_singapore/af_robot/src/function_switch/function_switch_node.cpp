/*
 * function_switcher_node.cpp
 *
 *  Created on: Dec 17, 2016
 *      Author: shengsong
 */
#include "function_switch/function_switch.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "function_switch");
    FunctionSwitch fs;
    ros::spin();
    return 0;
}



