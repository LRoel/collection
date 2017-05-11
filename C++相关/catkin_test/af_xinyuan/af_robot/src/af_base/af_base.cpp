/*
 * af_base.cpp
 *
 *  Created on: 2016年11月24日
 *      Author: shengsong
 */

#include "af_base/af_base.h"
unsigned char check_sum(unsigned char *buffer, int start, int end)
{
    unsigned char sum = 0;

    for (int i=start; i<end+1; i++)
    {
        sum += buffer[i];
    }

    return sum;
}

void AFBase::writeCmd(short linear, short alpha)
{
    unsigned char buffer[8] = {};
    
    if (alpha > 19)
        alpha = 19;
    else if (alpha < -19)
        alpha = -19;

    buffer[0] = 0x56;
    buffer[2] = linear & 0xFF;
    buffer[1] = (linear >> 8) & 0xFF;
    buffer[3] = alpha & 0xFF;
    buffer[7] = check_sum(buffer, 1, 6) & 0xFF;

    write(fd_, buffer, 8);
    std::cout << "命令发送到串口 angle:" << alpha << std::endl;
    ROS_INFO("angle:%d", alpha);
}
void AFBase::writeCallbackAlpha(const geometry_msgs::Twist & msg)
{
    writeCmd((short)(msg.linear.x * 100), (short)msg.angular.z);
}
void AFBase::writeCallback(const geometry_msgs::Twist & msg)
{
    short linear = (short)(msg.linear.x * 100); // 一厘米为单位
    short alpha = (short)(atan (0.581 * msg.angular.z / msg.linear.x) * 57.2958) % 180; // 180/pi

    writeCmd(linear, alpha);

//    for (int i=0; i<msg->data.size(); i++)
//        write(fd_, &msg->data[i], 1);

}

AFBase::AFBase(std::string port, int baud_rate, int data_bits, int stop_bits, int parity)
: fd_(0)
{

    fd_ = UART0_Open(fd_, port.c_str());
    int err = UART0_Init(fd_, baud_rate, 0, data_bits, stop_bits, parity);
    if (err == 0)
    {
        std::cout << "初始化成功" << std::endl;
    }
    else
    {
        std::cout << "初始化失败" << std::endl;
        exit(EXIT_FAILURE);
    }

    base_pub_ = nh_.advertise< af_bringup::Robot_encode >("robot_encode_val", 100);
    base_sub_ = nh_.subscribe("cmd_vel", 100, &AFBase::writeCallback, this);

    alpha_sub_ = nh_.subscribe("cmd_alpha", 100, &AFBase::writeCallbackAlpha, this);

    read_thread = boost::thread(&AFBase::readThread, this);
}

AFBase::~AFBase()
{
    close(fd_);
}

void AFBase::readThread()
{
    const int bit_len = 10;
    char recv_c = '\0';
    std_msgs::Int8MultiArray msg;
    msg.data.resize(bit_len);
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
            if (recv_c == '\x86')
            {
                index = 1;
            }
        }
        else if (index == bit_len-1)
        {
            int cs = 0;
            for (int i=1; i<bit_len-1; i++)
                cs += msg.data[i];
            if ((cs & 0xFF) != (msg.data[bit_len-1] & 0xFF))
            {
                std::cout << "校验和不匹配, "<< (cs & 0xFF) << std::endl;
                for(int i=0; i<bit_len; i++)
                    std::cout << std::hex << (0xFF & (msg.data[i]))<< ' ';
                std::cout << std::endl;
                index = 0;
            }
            else
            {
                std::cout << "校验和匹配" << std::endl;
		for (int i=0; i<bit_len; i++)
			std::cout << std::hex << (0xFF & msg.data[i]) << ' ';
		std::cout << std::endl;
                af_bringup::Robot_encode encode_msg;
                encode_msg.left_encode = (msg.data[2] << 8) | (msg.data[3] & 0xFF);
                encode_msg.right_encode = (msg.data[4] << 8) | (msg.data[5] & 0xFF);
                encode_msg.theta = (char)(msg.data[6] & 0xFF);
                base_pub_.publish(encode_msg);
                index = 0;
            }
        }
        else if ((index > 0) && (index < bit_len-1))
        {
            index ++;
        }
    }
}
