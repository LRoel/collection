#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <string>

#include "std_msgs/Int8MultiArray.h"
#include "UartDriver.h"


class AndroidSerial
{
private:
    ros::NodeHandle nh_;
    ros::Publisher serial_pub_;
    ros::Subscriber serial_sub_;

    int fd_;

    boost::thread read_thread;

    void writeCallback(const std_msgs::Int8MultiArray& msg);
    // TODO 记录const作用
public:
    AndroidSerial(std::string port="/dev/pts/23", int baud_rate = 115200, int flow_ctrl = 0, int data_bits = 8, int stop_bits = 1, int parity = 'N');
    ~AndroidSerial();
    /**
     * \brief 读串口数据
     */
    void readSerial();
    /**
     * \brief read_thread等线程join方法
     */
};

void AndroidSerial::writeCallback(const std_msgs::Int8MultiArray& msg)
{
    if ((msg.data[0] != '\x56') && (msg.data[0] != '\x55'))
    {
        return;
    }

    for (int i=0; i<10; i++)
    {
        write(fd_, &msg.data[i], 1);
    }
    std::cout << "Command sent to Android" <<std::endl;

}

AndroidSerial::AndroidSerial(std::string port, int baud_rate,int flow_ctrl,int data_bits,int stop_bits,int parity)
:fd_(0)
{
    // 打开串口 返回描述符 TODO 串口类
    fd_ = UART0_Open(fd_, port.c_str());
    int err = UART0_Init(fd_, baud_rate, flow_ctrl, data_bits, stop_bits, parity);
    if(TRUE == err)
    {
        printf("串口初始化成功\n");
        printf("fd_Encoder = %d.err = %d.\n",fd_,err);
    }
    else
    {
        printf("串口初始化失败!\n");
    }

    // 接收和发布协议字符串
    serial_pub_ = nh_.advertise <std_msgs::Int8MultiArray> ("/android_string",1);
    serial_sub_ = nh_.subscribe ("/android_string", 10, &AndroidSerial::writeCallback, this);

    read_thread = boost::thread(boost::bind(&AndroidSerial::readSerial, this));

}

AndroidSerial::~AndroidSerial()
{
    read_thread.join();
}

void AndroidSerial::readSerial()
{
    unsigned char recv_c = '\0';
    std_msgs::Int8MultiArray msg;
    msg.data.resize(10);
    int index = 0;
    while (ros::ok())
    {
        if (read(fd_, &recv_c, 1) != 1)
            continue;
        // TODO WARN std::cout is not protected by mutex
        std::cout << "Char read: "<< std::hex<<int(recv_c) << " len="<< index+1 <<std::endl;

        if (recv_c == '\x55' && index == 0)
        {
            msg.data[0] = recv_c;
            index = 1;
            continue;
        }

        if (index == 0)
            continue;
        msg.data[index] = recv_c;
        index ++;
        if (index == 10)
        {
            int cs = 0;
            for (int i=1; i<9; i++)
                cs += msg.data[i];
            if ((cs & 0xFF) != (unsigned char)(msg.data[9]))
            {
                // checksum 不匹配
                std::cout << "校验和不匹配" << std::endl;
                index = 0;
                continue;
            }
            else
            {
                // checksum 匹配
                std::cout << "校验和匹配" << std::endl;
                serial_pub_.publish(msg);
                index = 0;
            }
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "android_serial_node");
    ros::NodeHandle nh("~");
    std::string port;
    nh.param <std::string> ("port", port, "/dev/pts/23");
    std::cout << "串口："<< port << std::endl;
    AndroidSerial ser(port);
    ros::spin();
  return 0;
}
