#include <ros/ros.h>  
#include <tf/transform_broadcaster.h>  
#include <nav_msgs/Odometry.h>  
#include <af_bringup/Robot_encode.h>  
#include "ActualOdom.h"
#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <af_bringup/testConfig.h>

//goal:subscribe the car_speed, then send them 

double left_fix_;
double right_fix_;
double length_;


class SubscribeAndPublish  
{  
public:  
  SubscribeAndPublish()  
  { 
      
     thetaL = 0.0; 
     thetaR = 0.0;   	 
	 
     vx_ = 0.0;  
     vy_ = 0.0;  
     vth_ = 0.0;
     theta_ = 0.0;
     current_time_ = ros::Time::now();  
     last_time_ = ros::Time::now();  
    //Topic you want to publish  
    
    pub_ = n_.advertise<nav_msgs::Odometry>("odom", 100);  
  
    //Topic you want to subscribe  
    sub_ = n_.subscribe("robot_encode_val", 100, &SubscribeAndPublish::callback, this);  
  }  
  
  void callback(const af_bringup::Robot_encode::ConstPtr& input)
  {  
    //nav_msgs::Odometry output;  
    //.... do something with the input and generate the output... 
     
    current_time_ = ros::Time::now(); 

    thetaL = input->left_encode;
    thetaR = input->right_encode;
    theta_ = -1.0 * input->theta*PI/180.0;
    //thetaL = thetaL/32;
    //thetaR = thetaR/32;


    //printf("encoderValue_L = %d, encoderValue_R = %d \r\n",input->left_encode,input->right_encode);

    m_actualOdom.cal_motion(thetaL, thetaR, 0.0);
    m_actualOdom.sumTranslation();
    //m_actualOdom.printModel(); 
	
		printf("theta:%f \n",m_actualOdom.m_sumQ*180/PI);

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion base_quat = tf::createQuaternionMsgFromYaw(m_actualOdom.m_sumQ);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped base_trans;
    base_trans.header.stamp = current_time_;
    base_trans.header.frame_id = "odom";
    base_trans.child_frame_id = "base_footprint";

    base_trans.transform.translation.x = m_actualOdom.m_sumX;
    base_trans.transform.translation.y = m_actualOdom.m_sumY;
    base_trans.transform.translation.z = 0.0;
    base_trans.transform.rotation = base_quat;

    //send the transform
    odom_broadcaster.sendTransform(base_trans);


    //next, we'll publish the odometry message over ROS 
    double dt = (current_time_ - last_time_).toSec(); 
    dt = 0.01;
    vx_ = m_actualOdom.m_tranX/dt;  
    vy_ = m_actualOdom.m_tranY/dt;  
    vth_ = m_actualOdom.m_rotation/dt;

    nav_msgs::Odometry odom;  
    odom.header.stamp = current_time_;  
    odom.header.frame_id = "odom";  

    //set the position  
    odom.pose.pose.position.x = m_actualOdom.m_sumX;  
    odom.pose.pose.position.y = m_actualOdom.m_sumY;  
    odom.pose.pose.position.z = 0.0;  
    odom.pose.pose.orientation = base_quat;  
    odom.pose.covariance[0]  = 0.00001;
    odom.pose.covariance[7]  = 0.00001;
    odom.pose.covariance[14] = 99999;
    odom.pose.covariance[21] = 99999;
    odom.pose.covariance[28] = 99999;
    odom.pose.covariance[35] = 0.01;

    //set the velocity  
    odom.child_frame_id = "base_footprint"; 
    odom.twist.twist.linear.x = (thetaL+thetaR)*PulseToDistance/2.0/dt;  
    odom.twist.twist.linear.y = 0;  
    //odom.twist.twist.angular.z = vth_;  
    ROS_INFO("linear: %f, theta: %f",odom.twist.twist.linear.x ,theta_);
    if (fabs(odom.twist.twist.linear.x) <= 0.001) {
      linear_ = 0.0;
    }
    else
    {
      linear_ =  odom.twist.twist.linear.x;
    }
    odom.twist.twist.angular.z = linear_ * tan(theta_) / 0.581; 
    odom.twist.covariance = odom.pose.covariance;

    //publish the message  
    pub_.publish(odom);  

    last_time_ = current_time_;  
  
  }  
  
private:  
//   
  ros::NodeHandle n_;   
  ros::Publisher pub_;  
  ros::Subscriber sub_;  
  ros::Time current_time_, last_time_;  
  tf::TransformBroadcaster odom_broadcaster_;  
  
  double thetaL;
  double thetaR;
  double vx_;  
  double vy_ ;  
  double vth_ ;
  double theta_; 
  double linear_;
  
  ActualOdom m_actualOdom;
  tf::TransformBroadcaster odom_broadcaster;
  
};//End of class SubscribeAndPublish  


void callback(af_bringup::testConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f", 
            config.left_fix, config.right_fix);
  left_fix_ = config.left_fix;
  right_fix_ = config.right_fix;
  length_ = config.length;
  ROS_INFO("get configure: %f %f", 
            left_fix_, right_fix_);
}
  
int main(int argc, char **argv)  
{  
  //Initiate ROS  
  ros::init(argc, argv, "odometry_publisher");  
  
  //dynamic_test
  dynamic_reconfigure::Server<af_bringup::testConfig> server;
  dynamic_reconfigure::Server<af_bringup::testConfig>::CallbackType f;
  
  //Create an object of class SubscribeAndPublish that will take care of everything  
  SubscribeAndPublish SAPObject;  
  
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  
  ros::spin();  
  
  return 0;  
} 
