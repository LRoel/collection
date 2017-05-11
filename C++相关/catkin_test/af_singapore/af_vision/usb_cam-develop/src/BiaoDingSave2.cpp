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


using namespace std;
using namespace cv;

int imgID_left;
int imgID_right;

class BiaodingStereoSaveImage
{
  public:
    BiaodingStereoSaveImage()
    {
      ROS_INFO("BiaoTest--1");
      sub_left_ = n_left_.subscribe("/camera/left/image_raw", 1, &BiaodingStereoSaveImage::saveimageleftCallback, this);

      sub_right_ = n_right_.subscribe("/camera/right/image_raw", 1, &BiaodingStereoSaveImage::saveimagerightCallback, this);

    }
    void saveimageleftCallback(const sensor_msgs::Image::Ptr& msg);
    void saveimagerightCallback(const sensor_msgs::Image::Ptr& msg);
    void savestereoimage();
  private:
    ros::NodeHandle n_left_, n_right_;
    ros::Subscriber sub_left_, sub_right_;


    char image_filename_left[100];
    char image_filename_right[100];

    IplImage left_Frame;
    IplImage right_Frame;

    Mat leftImg,rightImg;
};

void BiaodingStereoSaveImage::saveimageleftCallback(const sensor_msgs::Image::Ptr& msg)
{
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
    
    ROS_INFO("Save stereo left begin!");
//filename :
    sprintf(image_filename_left,"/home/krund/freedemo/data/stereo/left_%d.jpg",imgID_left);
//save_left_image
    ROS_INFO("1");
    cvSaveImage(image_filename_left, &left_Frame);
    
    
  ROS_INFO("left_end");
}

void BiaodingStereoSaveImage::saveimagerightCallback(const sensor_msgs::Image::Ptr& msg)
{
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

    ROS_INFO("Save stereo right begin!");
    sprintf(image_filename_right,"/home/krund/freedemo/data/stereo/right_%d.jpg",imgID_right);
//save_right_image
    cvSaveImage(image_filename_right, &right_Frame);
  ROS_WARN("right_end");
}

void BiaodingStereoSaveImage::savestereoimage()
{
    ROS_INFO("Save stereo begin!");
//filename :
    sprintf(image_filename_left,"/home/krund/freedemo/data/stereo/left_%d.jpg",imgID_left++);
//save_left_image
    ROS_INFO("1");
    cvSaveImage(image_filename_left, &left_Frame);
    ROS_INFO("1-1");
//filename :
    sprintf(image_filename_right,"/home/krund/freedemo/data/stereo/right_%d.jpg",imgID_right++);
//save_right_image
    cvSaveImage(image_filename_right, &right_Frame);
    ROS_INFO("Save stereo OK!");
}

int main(int argc, char** argv)
{
  char waitKey;
  ros::init(argc, argv, "biaoding_image_stereo2");
  BiaodingStereoSaveImage BSSIObject;
  ROS_INFO("please put down the key--");

  ros::Rate r(100);
  ROS_INFO("test-1");
  while(ros::ok())
  {
    cin.get(waitKey);
//    ROS_INFO("test-2");

    if(waitKey == 's')
    {
        ROS_INFO("test----------32");
//        BSSIObject.savestereoimage();
        imgID_left++;
        imgID_right++;
        ROS_INFO("putting");

    }
    else if(waitKey == 'q')
    {
        ROS_INFO("the system will be out");
        break;
    }
//    ROS_INFO("test---3");
    ros::spinOnce();
    r.sleep();

  }

    return 0;
}
