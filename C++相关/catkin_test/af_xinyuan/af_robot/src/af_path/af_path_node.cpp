/*
 * af_path_node.cpp
 *
 *  Created on: 2016年8月15日
 *      Author: shengsong
 */
#include "ros/ros.h"
#include "af_path/af_path.h"
#include "tf/transform_listener.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "af_path");
    ros::NodeHandle nh("~");
    std::string host, user, pass, db;
    nh.param< std::string >("host", host, HOST);
    nh.param< std::string >("user", user, USER);
    nh.param< std::string >("pass", pass, PASS);
    nh.param< std::string >("database", db, DB);

    AFPath af_path(host, user, pass, db);

//    ros::NodeHandle nh;
//
//     tf::TransformListener listener;
//
//     ros::Rate r(50);
//     while (ros::ok())
//     {
//       tf::StampedTransform transform;
//       try{
//         listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);
//       }
//       catch (tf::TransformException ex)
//       {
//         ROS_ERROR("%s", ex.what());
//         ros::Duration(1.0).sleep();
//       }
//
//       std::cout << transform.getOrigin().x() << ' ' << transform.getOrigin().y() << std::endl;
//       r.sleep();
//     }
    ros::spin();
    return 0;
}
