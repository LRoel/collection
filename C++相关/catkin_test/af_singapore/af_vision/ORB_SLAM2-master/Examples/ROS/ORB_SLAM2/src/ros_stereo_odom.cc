/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <tf/tf.h>
#include <math.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include "Converter.h"

#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/PoseStamped.h"
#include"../../../include/System.h"

#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <Eigen/Dense>

using namespace std;

ros::Time current_time, last_time;
class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    void PublishPose(cv::Mat Tcw);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;

    /******************* pub slam_odom topic ********************/
    ros::Publisher* pSlamPosPub;   

    tf::TransformBroadcaster odom_broadcaster;
private:
    float x_x_tmp = 0.0;
    float x_y_tmp = 0.0;
    float y_tmp = 0.0;
    float z_tmp = 0.0;
    double yaw_tmp = 0.0;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    double x_x = 0.1;
    double x_y = 0.1;
    double vx = 0.1;
    double vy = -0.1;
    double vth = 0.1;
    /******************* pub slam_odom topic  end ********************/
};


    /******************* pub slam_odom topic ********************/
 /******************* geometry_msgs::PoseStamped ********************/
/*
void ImageGrabber::PublishPose(cv::Mat Tcw)
{
    geometry_msgs::PoseStamped pose_slam;
    if(!Tcw.empty())
    {
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(1,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
    
        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

        pose_slam.pose.position.x = twc.at<float>(0);
        pose_slam.pose.position.y = twc.at<float>(2);
        pose_slam.pose.position.z = twc.at<float>(1);
        pose_slam.pose.orientation.x = q[0];
        pose_slam.pose.orientation.y = q[1];
        pose_slam.pose.orientation.z = q[2];
        pose_slam.pose.orientation.w = q[3];
        pose_slam.header.frame_id = "VSLAM";
        pose_slam.header.stamp = ros::Time::now();
        //cout << "PublishPose position.x = " << pose_slam.pose.position.x << endl;

        (pSlamPosPub)->publish(pose_slam);

    }
}
*/

 /******************* pub slam_odom topic  end ********************/



int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    if(argv[3])
    {
        igb.do_rectify = true;

        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "camera/right/image_raw", 1);


    /******************* pub slam_odom topic ********************/
    //ros::Publisher SlamPosPub = nh.advertise<geometry_msgs::PoseStamped>("ORB_SLAM/pose", 5);

    ros::Publisher SlamPosPub = nh.advertise<nav_msgs::Odometry>("ORB_SLAM/odom", 5);


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    /******************* pub slam_odom topic ********************/
    igb.pSlamPosPub = &(SlamPosPub);
    /******************* pub slam_odom topic  end ********************/

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");


    //SLAM.SaveMap("Slam_latest_Map.bin");
    SLAM.LoadMap("Slam_Map.bin");
    ros::shutdown();

    return 0;
}



 /******************* nav_msgs::Odometry ********************/

void ImageGrabber::PublishPose(cv::Mat Tcw)
{

    nav_msgs::Odometry pose_slam;

 /******************* pub slam_odom topic  end ********************/
    current_time = ros::Time::now();
    if(!Tcw.empty())
    {
    	cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);
        //geometry_msgs::Quaternion p = ORB_SLAM2::Converter::toQuaternion(Rwc);
        geometry_msgs::Quaternion p;
        p.x = q[0];
        p.y = q[1];
        p.z = q[2];
        p.w = q[3];

        float lx = twc.at<float>(2);
        float ly = -twc.at<float>(0);
        float lz = -twc.at<float>(1);

        //first, we'll publish the transform over tf
      /* 
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "visionOdom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = lx;
        odom_trans.transform.translation.y = ly;
        odom_trans.transform.translation.z = lz;
       */  
                /****Quaternion****/
     //   odom_trans.transform.rotation = p;
        //send the transform
     //   odom_broadcaster.sendTransform(odom_trans);

        pose_slam.header.stamp = ros::Time::now();
        pose_slam.header.frame_id = "visionOdom";

        //set the position   Point  and Quaternion
        pose_slam.pose.pose.position.x = lx;
        pose_slam.pose.pose.position.y = ly;
        pose_slam.pose.pose.position.z = lz;
        pose_slam.pose.pose.orientation = p;        

	//print
        ROS_WARN("x:%f",lx);
        ROS_WARN("y:%f",ly);
        ROS_WARN("z:%f",lz);

        x_x = (lx - x_x_tmp);
        x_y = (ly - x_y_tmp);
        
        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        //print time
        ROS_INFO("dt:%f",dt); 

        vx = sqrt(x_x * x_x + x_y * x_y)/dt;
        vy = 0;
        //double vz = (twc.at<float>(1) - z_tmp) / dt;
       // double delta_th = vth * dt;
         
        x_x_tmp = lx;
        x_y_tmp = ly;
        //z_tmp = twc.at<float>(1);

        yaw = tf::getYaw(p);
        vth = (yaw - yaw_tmp) / dt;
        yaw_tmp = yaw;

        //print
        ROS_INFO("yaw%f",yaw);

        //set the velocity

        pose_slam.child_frame_id = "base_link";
        pose_slam.twist.twist.linear.x = vx;
        pose_slam.twist.twist.linear.y = 0;
      //  pose_slam.twist.twist.linear.z = 0;
        pose_slam.twist.twist.angular.z = vth;

        (pSlamPosPub)->publish(pose_slam);
        last_time = ros::Time::now();
    }
}
    /******************* pub slam_odom topic  end ********************/



void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight, Tcw;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
        //mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    /*************** topic_slam_odom ***************/

        //cv::Mat Tcw= mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    
        last_time = ros::Time::now();
        PublishPose(Tcw);
    /******************* pub slam_odom topic  end ********************/

    }
    else
    {
        mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }

}


