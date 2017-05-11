/*
 * af_robot.cpp
 *
 *  Created on: 2016年8月6日
 *      Author: shengsong
 */
#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Twist.h"
//#include "af_move/map_db.h"

#define MAX_LINEAR_SPEED 1.0
#define MAX_ANGULAR_SPEED 1.0


//TODO af_path af_robot 公用 ros内部

#define PLANNING 0x00
#define PATROLLING 0x01
#define NAVIGATING 0x02
#define IDLING 0x03
#define CONTROLLING 0x04 //TODO redundant to IDLING
#define CHARGING 0x05 //TODO redundant to NAVIGATING
#define MALFUNC 0x06 //TODO redundant for whole robot system

// ros 接收状态表
#define PATROLLING_R 0x01
#define NAVIGATING_R 0x02
#define IDLING_R 0x03 //< 遇到行人停止
#define CONTROLLING_R 0x04
#define STRING_LIGHT_R 0x05
#define PHOTO_TAKEN_R 0x06

// ros 发送状态表、android 接收状态表
#define PATROLLING_A 0x01
#define NAVIGATING_A 0x02
#define CONTROLLING_A 0x03
#define TOCHARGE_A 0x04 //< 去充电
#define CHARGING_A 0x05 //< 充电中
#define CHARGED_A 0x06 //< 充电完成
#define IDLING_A 0x07 //< 停止、待命
#define MALFUNC_A 0x08 //< 故障

int prev_state = IDLING;
int current_state = IDLING;
int current_id = 0;

void androidStringCallback(const std_msgs::Int8MultiArray::ConstPtr& msg);
void cmdFdbkCallback(const std_msgs::Int8::ConstPtr& msg);

ros::Subscriber android_sub;
ros::Publisher android_pub;

ros::Publisher cmd_pub;
ros::Subscriber cmd_fdbk_sub;

ros::Publisher vel_pub;

ros::Publisher disable_pub;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "af_robot");
    ros::NodeHandle nh;

    android_sub = nh.subscribe("android_string", 100, &androidStringCallback);
    android_pub = nh.advertise< std_msgs::Int8MultiArray >("android_string", 100);

    cmd_pub = nh.advertise< std_msgs::Int8MultiArray > ("cmd", 10);
    cmd_fdbk_sub = nh.subscribe("cmd_fdbk", 100, &cmdFdbkCallback);

    vel_pub = nh.advertise< geometry_msgs::Twist >("cmd_vel", 100);

    disable_pub = nh.advertise< std_msgs::Int8 >("disable_move", 10);

    ros::Rate r(10);
    while (ros::ok())
    {
        // 发送实时数据 x,y 电量 温度
                //充电完成状态判断
        // 地点\路口\方向
        // 触发命令数据 拍照\对讲\报警
        //TODO 人/车检测数据

        // 实时数据 当前x,y 电量 温度
        // 充电状态
        // 巡逻时 拍照
        // 导航时 地点 路口 方向
        //           对讲 报警
        ros::spinOnce();
        r.sleep();
    }
}

// 从/android_string 接收android端下发协议
void androidStringCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
    std::cout << "收到安卓指令 " << std::endl;
    if (msg->data[0] != '\x55')
    {
        std::cout << "错误帧头 " << std::endl;
        return;
    }
    if (current_state == TOCHARGE_A && msg->data[1] != CONTROLLING_R )
    {
        std::cout << "当期 充电状态 且 收到的不是中控室控制命令 " << std::endl;
        return;
    }
    std_msgs::Int8MultiArray cmd_msg;
    cmd_msg.data.clear();
    cmd_msg.data.resize(8);

//    current_id = 0;

    switch (msg->data[1])
    {// 已验证 发布新指令会停下move_base
    case PATROLLING_R: // 巡航
        current_state = PATROLLING_A;
        current_id = msg->data[2];
        cmd_msg.data[0] = PATROLLING;
        cmd_msg.data[1] = current_id;
        ///TODO !!!使用move_base规划至巡逻路线第一个节点,必须保证move_base分层地图闭合且正常运行在闭合路线中!!!
        cmd_pub.publish(cmd_msg);
        break;
    case NAVIGATING_R: // 导航
        current_state = NAVIGATING_A;
        current_id = msg->data[2];
        cmd_msg.data[0] = NAVIGATING;
        cmd_msg.data[1] = current_id;
        cmd_pub.publish(cmd_msg);
        break;

    case IDLING_R: // 行人
        current_state = IDLING_A;//msg->data[1];
        cmd_msg.data[0] = IDLING;
        cmd_pub.publish(cmd_msg);
        break;

    case CONTROLLING_R: // 中控
    {
        std::cout<< "收到中控室控制命令 "<<std::endl;
        current_state = CONTROLLING_A;//msg->data[1]
        cmd_msg.data[0] = IDLING;
        cmd_pub.publish(cmd_msg);
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = MAX_LINEAR_SPEED * msg->data[3] / 100.0;
        vel_msg.angular.z = MAX_ANGULAR_SPEED * msg->data[2] / 100.0;
        if (vel_msg.linear.x > MAX_LINEAR_SPEED)
            vel_msg.linear.x = MAX_LINEAR_SPEED;
        else if (vel_msg.linear.x < -MAX_LINEAR_SPEED)
            vel_msg.linear.x = -MAX_LINEAR_SPEED;
        if (vel_msg.angular.z > MAX_ANGULAR_SPEED)
            vel_msg.angular.z = MAX_ANGULAR_SPEED;
        else if (vel_msg.angular.z < -MAX_ANGULAR_SPEED)
            vel_msg.angular.z = -MAX_ANGULAR_SPEED;

        vel_pub.publish(vel_msg);
    }
        break;

    case STRING_LIGHT_R: // 灯带
        break;

    case PHOTO_TAKEN_R: // 拍照完成
        if (current_state == PATROLLING_A)
        {
            std_msgs::Int8 disable_msg;
            disable_msg.data = 0;
            disable_pub.publish(disable_msg);
        }
        break;
    }
}

void cmdFdbkCallback(const std_msgs::Int8::ConstPtr& msg)
{
    std_msgs::Int8MultiArray msg_string;
    msg_string.data.clear();
    msg_string.data.resize(10);

    switch (current_state)
    {
    case PATROLLING_A:
        if (msg->data != PATROLLING)
            break;
        msg_string.data[0] = '\x56';
        msg_string.data[1] = (unsigned char)(PATROLLING_A);
        msg_string.data[2] = (unsigned char)(current_id);
        msg_string.data[3] = '\x01'; //finished flag
        msg_string.data[9] = 0xff & (msg_string.data[1] + msg_string.data[2] + msg_string.data[3]);
        android_pub.publish(msg_string);
        break;
        break;
    case NAVIGATING_A:
        if (msg->data != NAVIGATING)
            return;
        msg_string.data[0] = '\x56';
        msg_string.data[1] = (unsigned char)(PATROLLING_A);
        msg_string.data[2] = (unsigned char)(current_id);
        msg_string.data[3] = '\x01'; // 完成标志
        msg_string.data[9] = 0xff & (msg_string.data[1] + msg_string.data[2] + msg_string.data[3]);
        android_pub.publish(msg_string);
        break;
    case IDLING_A:
        break;
    case CONTROLLING_A:
        break;
    case TOCHARGE_A:
        break;
    case CHARGING_A:
        break;
    case CHARGED_A:
        break;
    case MALFUNC_A:
        break;
    default:
        break;
    }
}
