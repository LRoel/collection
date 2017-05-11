#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
 #include<fcntl.h> 

#include <stdio.h>
#include <iostream>

#include<iomanip>
#include<fstream>


#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <signal.h>
#include <math.h>

using namespace std;
using namespace cv;

char image_filename_left[10000];
char image_filename_right[10000];

int imgID_left = 0;
int imgID_right = 0;

IplImage left_Frame;
IplImage right_Frame;

cv::Mat leftImg,rightImg;


void saveimageleftCallback(const sensor_msgs::Image::Ptr& msg)
{
   
    sensor_msgs::Image msg_left;
    ROS_INFO("save left start");
    //  time_left
    msg_left.header.stamp = ros::Time::now();
     	
    
    cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例
 	
 	try
	{
		cv_ptr =  cv_bridge::toCvCopy(msg,  sensor_msgs::image_encodings::BGR8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
	}
	catch(cv_bridge::Exception& e)  //异常处理  
	{
		ROS_ERROR("cv_bridge exception_left: %s", e.what());	
		return;
	}
	  
    	leftImg = cv_ptr->image;
	left_Frame = IplImage( cv_ptr->image );   
//filename :
    sprintf(image_filename_left,"/home/krund/freedemo/data/stereo/left_%d.jpg",imgID_left++);

//save_left_image
    
    //save time    
   
    ofstream stamp_left("//home//krund//freedemo//data//stereo//stero_left.txt",ios_base::app);
    if (!stamp_left)
      return;
    
    //stamp_left<<"time:left::"<<imgID_left<<":"<<msg_left.header.stamp<<endl;
    stamp_left<<msg_left.header.stamp<<endl;

    stamp_left.close();    

     cvSaveImage(image_filename_left, &left_Frame);

}


void saveimagerightCallback(const sensor_msgs::Image::Ptr& msg)
{
    ROS_WARN("save right start");
        sensor_msgs::Image msg_right;
   
    //  time_right
    msg_right.header.stamp = ros::Time::now();
     	
    
    cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例
 	
 	try
	{
		cv_ptr =  cv_bridge::toCvCopy(msg,  sensor_msgs::image_encodings::BGR8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
	}
	catch(cv_bridge::Exception& e)  //异常处理  
	{
		ROS_ERROR("cv_bridge exception_right: %s", e.what());	
		return;
	}
	  
    	rightImg = cv_ptr->image;  //cv::mat
	right_Frame = IplImage( cv_ptr->image );    //save
//filename :
    sprintf(image_filename_right,"/home/krund/freedemo/data/stereo/right_%d.jpg",imgID_right++);

//save_right_image
          //save time    
   
    ofstream stamp_right("//home//krund//freedemo//data//stereo//stero_right.txt",ios_base::app);
    if (!stamp_right)
      return;
    
    //stamp_right<<"time:right::"<<imgID_right<<":"<<msg_right.header.stamp<<endl;
    stamp_right<<msg_right.header.stamp<<endl;

    stamp_right.close();

     cvSaveImage(image_filename_right, &right_Frame);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "save_image_stereo");
    ros::NodeHandle n1,n2;
    //ros::Rate sleep_save_rate(1000);
    ros::Subscriber sub_left = n1.subscribe("/camera/left/image_raw",1000,saveimageleftCallback);

    ros::Subscriber sub_right = n2.subscribe("/camera/right/image_raw",1000,saveimagerightCallback);

     //sleep_save_rate.sleep();
     ros::spin();

	return 0;
}

