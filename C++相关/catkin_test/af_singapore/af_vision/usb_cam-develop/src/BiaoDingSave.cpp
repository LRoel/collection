#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
 #include<fcntl.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <iomanip>
#include <fstream>


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

#define KEYCODE_S 0x73

using namespace std;
using namespace cv;

int flag_left = 1;
int flag_right = 1;

char image_filename_left[10000];
char image_filename_right[10000];


       /*** channge ID  ***/
/***  for example 0 1 2 3 4 5 ***/
int imgID_left;
int imgID_right;

IplImage left_Frame;
IplImage right_Frame;

cv::Mat leftImg,rightImg;

char key1;


void saveimageleftCallback(const sensor_msgs::Image::Ptr& msg)
{

	if(flag_left == 1){

 	ROS_INFO("left_start");
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
    sprintf(image_filename_left,"/home/krund/freedemo/data/stereo_biaoding/left_%d.jpg",imgID_left++);

//save_left_image

     cvSaveImage(image_filename_left, &left_Frame);
	 flag_left = 0;

	}


}


void saveimagerightCallback(const sensor_msgs::Image::Ptr& msg)
{

	if(flag_right == 1){

   	ROS_WARN("right_start");
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
    sprintf(image_filename_right,"/home/krund/freedemo/data/stereo_biaoding/right_%d.jpg",imgID_right++);

//save_right_image

     cvSaveImage(image_filename_right, &right_Frame);

	 flag_right = 0;

	}

}






int main(int argc, char **argv)
{

 	ros::init(argc, argv, "biaoding_image_stereo");

	ros::NodeHandle n1,n2;

	ros::Subscriber sub_left;
	ros::Subscriber sub_right;

	//SaveImageOnce saveimageonce;

	char key;
        int biao = 0; //change num to txt name
        imgID_left = biao;
        imgID_right = biao;


	while(ros::ok())
	{

		cin>>key;

		if(key == 's')
		{

		//saveimageleftCallback(sensor_msgs::Image::Ptr& msg);
		sub_left = n1.subscribe("/camera/left/image_raw",1000,saveimageleftCallback);

    	sub_right = n2.subscribe("/camera/right/image_raw",1000,saveimagerightCallback);

		ros::spin();
		}

		else
			cout<<"not done"<<endl;

		flag_left == 1;
		flag_right == 1;


	}

	return 0;
}
