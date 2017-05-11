
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ros/console.h"

#include "UartDriver.h"

#include "af_bringup/Robot_encode.h"
#include "af_bringup/af_navMsg.h"

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

//#include "tf/LinearMath/Transform.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
//%SOCKET

#include <unistd.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <stdio.h>

#define   PI 3.14159265358979





/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%

unsigned short check_summ(unsigned char * buffer, int start, int end)
{
    unsigned short sum = 0;    //赋初值

    for(int i=start; i<end+1; i++)
        {
            sum += buffer[i];
        }

    return sum;
}


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {

   fd_Encoder = UART0_Open(fd_Encoder,"/dev/ttyS0");//打开串口，返回文件描述符
                ROS_INFO("usart ok\n");

    //Topic you want to publish
    pub_ = n_.advertise<af_bringup::Robot_encode>("robot_encode_val", 10);

    //Topic you want to subscribe
    sub_ = n_.subscribe("cmd_vel", 1, &SubscribeAndPublish::velCallback, this);
  }

  void serial_Loop()
    {
    err = UART0_Init(fd_Encoder,115200,0,8,1,'N');
   if(TRUE == err)
    {
        ROS_INFO("Set Port Exactly!");
        ROS_INFO("fd_Encoder = %d.err = %d.",fd_Encoder,err);
    }
   else
    {
        ROS_INFO("init Port fault!");
    }
        ROS_INFO("ros ready");
        while (ros::ok())
        {
        static   int num = 0;
        unsigned char recv_len = 0;
        unsigned char temp[1] = {0x00};
        unsigned char RS485_RX_BUF[9] = {0x00};
        unsigned short csvalue = 0;
        int length = 0;
        int uart_status = 0;

    while(length<9)
        {
            recv_len = read(fd_Encoder, temp , 1);
            if(recv_len == 1)
            {
                switch (uart_status)
                {
                    case 0 :                                       //寻找帧头第一个0x58
                    {
                                                if (temp[0] == 0x86)
                        {
                            RS485_RX_BUF[0] = temp[0];
                            uart_status = 1;
                            length = 1;
                        }
                        else
                        {
                            uart_status = 0;
                            length = 0;
                        }
                    }
                    break;

                    case 1 :                                        //寻找帧头第二个0x58
                    {
            if (temp[0] == 0x08)
                        {
                            RS485_RX_BUF[1] = temp[0];
                            uart_status = 2;
                            length = 2;
                        }
                        else
                        {
                            uart_status= 0;
                            length = 0;
                        }
                    }
                    break;

                    case 2 :                    //找到帧头后,接收后面数据
                    {
                        RS485_RX_BUF[length++] = temp[0];
                    }
                    break;

                    default:                    //默认为寻找帧头
                    {
                        uart_status = 0;
                        length = 0;
                    }
                    break;

                }
              }
        }

      num++;

    csvalue = check_summ(RS485_RX_BUF,2,7)&0xFF; //计算crc校验值
        
    if(csvalue == (RS485_RX_BUF[8]))
      {
               // ROS_INFO("\r\n");

        af_bringup::Robot_encode output;
        output.left_encode = -1 * ((RS485_RX_BUF[2]<<8) | (RS485_RX_BUF[3]));  //获取左轮编码值
        output.right_encode = -1 * ((RS485_RX_BUF[4]<<8) | (RS485_RX_BUF[5]));  //获取右轮编码值
        //output.left_encode = (RS485_RX_BUF[2]<<8) | (RS485_RX_BUF[3]);  //获取左轮编码值
        //output.right_encode =  (RS485_RX_BUF[4]<<8) | (RS485_RX_BUF[5]);  //获取右轮编码值
              pub_.publish(output);
                
        }
        else //crc出错
        {
            ROS_INFO("UART0_Recv crc_chk false!");

        }


           usleep(1000);

     }
     close( fd_Encoder );
}




   // void amclCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr & msg)
 //void velCallback(const af_bringup::af_navMsg::ConstPtr & navMsg)
void velCallback(const geometry_msgs::Twist & msg)
{

    float linear;
  float angular;
  short tmp_linear;
  short tmp_angular;

    unsigned char *pchar = NULL;
    pchar = (unsigned char*)malloc(12 * sizeof(unsigned char));
        int send_length=0;

    linear = msg.linear.x;
    tmp_linear = short(linear * 100);
    //ROS_INFO("%d  ",tmp_linear);
    angular = msg.angular.z;
    tmp_angular = short(angular * 1800 / PI);
    //ROS_INFO("%d  \n",tmp_angular);

        //ROS_INFO("Received  %f  %f ", robvel[0], robvel[1]);
        pchar[0] = 0x56;                    //帧头

        pchar[2] = tmp_linear & 0xff;
        pchar[1] = (tmp_linear >> 8) & 0xff;

        pchar[4] = tmp_angular & 0xff;
        pchar[3] = (tmp_angular >> 8) & 0xff;

        unsigned short sum_crc = check_summ(pchar, 1,6);   //CRC校验

        pchar[7] = sum_crc & 0xff;

        for(int j=0; j<8; j++)
        {
            printf("%02x ", pchar[j]);
        }
        ROS_INFO("%f ",tmp_angular/10.0);
        send_length = write(fd_Encoder, pchar,8); //串口发送两轮的速度给stm32

        free(pchar);
}




private:
//
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  int fd_Encoder;                   //文件描述符
  int err;                          //返回调用函数的状态

};//End of class SubscribeAndPublish






// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
   //startNetServiceFork(HELLO_WORLD_SERVER_PORT, NULL);    //开启传输栅格地图的网络套接字

   ros::init(argc, argv, "serial");

   SubscribeAndPublish SAPObject;

   boost::thread t = boost::thread(boost::bind(&SubscribeAndPublish::serial_Loop, &SAPObject));

   //boost::thread t(serial_Loop);
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
// %Tag(SUBSCRIBER)%


   ros::spin();

   t.interrupt();
   t.join();

// %EndTag(SUBSCRIBER)%

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
// %Tag(SPIN)%

// %EndTag(SPIN)%

  return 0;
}
// %EndTag(FULLTEXT)%
