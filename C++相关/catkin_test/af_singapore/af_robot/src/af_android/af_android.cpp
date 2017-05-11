/*
 * af_android.cpp
 *
 *  Created on: 2016年8月20日
 *      Author: shengsong
 */
#include "af_android/af_android.h"

void AFAndroid::writeCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
    if ((msg->data[0] != '\x56') && (msg->data[0] != '\x55'))
        return;
    if (msg->data.size() != 10)
        std::cout << "发送数据长度小于10.." << std::endl;
    for (int i=0; i<10; i++)
        write(fd_, &msg->data[i], 1);
    std::cout << "命令发送给安卓端" << std::endl;
}

AFAndroid::AFAndroid(std::string port, int baud_rate, int data_bits, int stop_bits, int parity)
: fd_(0)
{
    fd_ = UART0_Open(fd_, port.c_str());
    int err = UART0_Init(fd_, baud_rate, 0, data_bits, stop_bits, parity);
    if (err == 0)
    {
        std::cout << "初始化串口成功" << std::endl;
    }
    else
    {
        std::cout << "初始化串口失败" << std::endl;
        exit(EXIT_FAILURE);
    }

    android_pub_ = nh_.advertise< std_msgs::Int8MultiArray >("/android", 10);
    android_sub_ = nh_.subscribe("/android", 10, &AFAndroid::writeCallback, this);

    read_thread = boost::thread(&AFAndroid::readThread, this);

}

AFAndroid::~AFAndroid()
{
//    read_thread.join();
}

void AFAndroid::readThread()
{
    char recv_c = '\0';
    std_msgs::Int8MultiArray msg;
    msg.data.resize(10);
    int index = 0;
    while (ros::ok())
    {
        boost::mutex::scoped_lock lock(io_mutex);
        if (read(fd_, &recv_c, 1) != 1)
            continue;
        std::cout <<index << " 读取字符: " << std::hex << int(recv_c) <<std::endl;
        msg.data[index] = recv_c;
        if (index == 0)
        {
            if (recv_c == '\x55')
            {
                index = 1;
            }
        }
        else if (index == 9)
        {
            int cs = 0;
            for (int i=1; i<9; i++)
                cs += msg.data[i];
//            if ((cs & 0xFF) != (0xFF & msg.data[9]))
            if(0)
            {
                std::cout << "校验和不匹配" << std::endl;
                for(int i=0; i<10; i++)
                    std::cout << std::hex << int(msg.data[i])<< ' ';
                std::cout << std::endl;
                index = 0;
            }
            else
            {
                std::cout << "校验和匹配" << std::endl;
                android_pub_.publish(msg);
                index = 0;
            }
        }
        else if ((index > 0) && (index < 9))
        {
            index ++;
        }
    }
}
