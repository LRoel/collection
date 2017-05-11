/*
 * af_robot.cpp
 *
 *  Created on: 2016年8月18日
 *      Author: shengsong
 */
#include "af_robot/af_robot.h"

AFRobot::AFRobot()
:current_state_(IDLING_A), current_id_(0),
 x_(0), y_(0),
 battery_percentage_(100), inner_temperature_(0),
 malfunc_flag_(false), lifting_flag_(false), emergency_stop_flag_(false),
 call_security_flag_(false), front_collision_flag_(false), deviated_flag_(false),
 photo_flag_(false)
{
    android_sub_ = nh_.subscribe("android", 100, &AFRobot::androidCallback, this);
    android_pub_ = nh_.advertise< std_msgs::Int8MultiArray >("android", 100);

    cmd_pub_ = nh_.advertise< std_msgs::Int8MultiArray > ("cmd", 10);
//    cmd_fdbk_sub_ = nh_.subscribe("cmd_fdbk", 100, &AFRobot::cmdFdbkCallback, this);
    cmd_fdbk_sub_ = nh_.subscribe("path_fdbk", 100, &AFRobot::cmdFdbkCallback, this);

    vel_pub_ = nh_.advertise< geometry_msgs::Twist >("cmd_vel", 100);
    alpha_pub_ = nh_.advertise< geometry_msgs::Twist >("cmd_alpha", 100);


//    disable_pub_ = nh_.advertise< std_msgs::Int8 >("disable_move", 10);
    current_goal_sub_ = nh_.subscribe< af_msgs::Vertex >("current_goal", 10, &AFRobot::currentGoalCallback, this);
    current_result_sub_ = nh_.subscribe< af_msgs::Vertex >("current_result", 10, &AFRobot::currentResultCallback, this);

    state_sub_ = nh_.subscribe< af_msgs::RobotState >("robot_state", 10, &AFRobot::stateCallback, this);
    init_state_sub_ = nh_.subscribe< std_msgs::Bool >("/cmd_vel/state", 10, &AFRobot::initStateCallback, this);

    camera_pub_ = nh_.advertise< std_msgs::String >("camera", 10);
    camera_sub_ = nh_.subscribe< std_msgs::String >("camera", 10, &AFRobot::cameraCallback, this);

    pause_pub_ = nh_.advertise< std_msgs::Int8 >("pause", 10);
    continue_pub_ = nh_.advertise< std_msgs::Int8 >("continue", 10);
//    cancel_pub_ = nh_.advertise< actionlib_msgs::GoalID >("move_base/cancel", 10);
//    costmap_sub_ = nh_.subscribe< nav_msgs::OccupancyGrid >("/move_base/local_costmap/costmap", 1, &AFRobot::costmapCallback, this);

    marker_pub_ = nh_.advertise< visualization_msgs::Marker >("camera_marker", 10);
    text_pub_ = nh_.advertise< std_msgs::String >("text", 10);

    try
    {
        listener_.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0));
    }
    catch (...)
    {
    }
    report_thread_ = boost::thread(&AFRobot::reportThread, this);
    last_patrol_id = 0;
    current_result_ = 0;
}
void AFRobot::initStateCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if (msg->data == false)
        return;
    static bool reported_once = false;
    if (reported_once == true)
        return;
    reported_once = true;
    std_msgs::Int8MultiArray string_msg;
    string_msg.data.clear();
    string_msg.data.resize(10);

    string_msg.data[0] = 0x56;
    string_msg.data[1] = 0x03; // 触发命令 临时
    string_msg.data[2] = 0x0A; // 初始化成功
    string_msg.data[9] = checksum(string_msg);
    android_pub_.publish(string_msg);

}

void AFRobot::androidCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(report_m);
    std::cout << "androidCallback" << std::endl;
    if (msg->data[0] != '\x55')
    {
        if (msg->data[0] != '\x56')
            std::cout << "错误帧头!!!" << std::endl;
        return;
    }


    std::cout << "收到安卓命令..." << std::endl;
    if (current_state_ == TOCHARGE_A && msg->data[1] != CONTROLLING_R)
    {
        std::cout << "前往充电桩中, 只执行中控室命令..." << std::endl;
        return;
    }

    if (current_state_ == CHARGING_A)
    {
        std::cout << "充电中, 不执行命令..." << std::endl;
        return;
    }

    if (msg->data.size() != 10)
    {
        std::cout << "错误帧长度" << std::endl;
    }

    std_msgs::Int8MultiArray cmd_msg;
    cmd_msg.data.clear();
    cmd_msg.data.resize(8);


    switch (msg->data[1])
    {
    case PATROLLING_R: // 巡航
        if (msg->data[2] == 0)
        {
            std::cout << "不完整的数据!!!" << std::endl;
            return;
        }
        std::cout << "last_patrol_id "<<last_patrol_id << std::endl;
        std::cout << "current_state_ "<<current_state_ << std::endl;

        if (last_patrol_id == msg->data[2])
        {
            current_state_= PATROLLING_A;
//            std_msgs::Int8 continue_msg;
//            continue_pub_.publish(continue_msg); // TODO
            pauseMove(PEDESTRIAN_PAUSE, false);
        }
        else
        {
//            std_msgs::Int8 pause_msg;       //取消move_base之前运行数据 改用cancel
//            pause_pub_.publish(pause_msg); // TODO
            pauseMove(CANCEL);
//            pauseMove(CANCEL, false);
            last_patrol_id = msg->data[2];

            current_state_= PATROLLING_A;
            current_id_ = msg->data[2];
            cmd_msg.data[0] = PATROLLING;
            cmd_msg.data[1] = msg->data[2];
            cmd_pub_.publish(cmd_msg);

        }
        break;
    case NAVIGATING_R: // 导航
        if (msg->data[2] == '\x0')
        {
            std::cout << "不完整的数据!!!" << std::endl;
            return;
        }

    {
        std_msgs::Int8 pause_msg;
        pause_pub_.publish(pause_msg); // TODO
        pauseMove(CANCEL);
    }
        current_state_= NAVIGATING_A;
        current_id_ = msg->data[2];
        cmd_msg.data[0] = NAVIGATING;
        cmd_msg.data[1] = msg->data[2];
        cmd_pub_.publish(cmd_msg);
        break;

    case IDLING_R: // 行人
    if (current_state_ == PATROLLING_A)
    {
        current_state_= IDLING_A;
//        std_msgs::Int8 pause_msg;
//        pause_pub_.publish(pause_msg); // TODO
        pauseMove(PEDESTRIAN_PAUSE, true);
    }
    break;

    case CONTROLLING_R: // 中控
    {
//        ROS_DEBUG("收到中控室控制命令 ");
        current_state_= CONTROLLING_A;
//        cmd_msg.data[0] = IDLING;
//        cmd_pub_.publish(cmd_msg);
//        std_msgs::Int8 pause_msg;
//        pause_pub_.publish(pause_msg);
        pauseMove(CANCEL);

#ifdef DIFF
        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = MAX_LINEAR_SPEED * msg->data[3] / 100.0;
        vel_msg.angular.z = - MAX_ANGULAR_SPEED * msg->data[2] / 100.0;

        // 最大速度限制
        if (vel_msg.linear.x > MAX_LINEAR_SPEED)
            vel_msg.linear.x = MAX_LINEAR_SPEED;
        else if (vel_msg.linear.x < -MAX_LINEAR_SPEED)
            vel_msg.linear.x = -MAX_LINEAR_SPEED;

        if (vel_msg.angular.z > MAX_ANGULAR_SPEED)
            vel_msg.angular.z = MAX_ANGULAR_SPEED;
        else if (vel_msg.angular.z < -MAX_ANGULAR_SPEED)
            vel_msg.angular.z = -MAX_ANGULAR_SPEED;

        // 忽略+-20% 以内的速度
        if (vel_msg.linear.x >= -0.20 && vel_msg.linear.x <= 0.20)
            vel_msg.linear.x = 0;

        if (vel_msg.linear.z >= -0.20 && vel_msg.linear.z <= 0.20)
            vel_msg.linear.z = 0;

        geometry_msgs::Twist vel_msg;
        vel_msg.linear.x = vel;
        vel_msg.angular.z = tan(angle / 180.0 * 3.1415926) * vel / 0.581;
        vel_pub_.publish(vel_msg);

#endif
#define ACK
#ifdef ACK
        double vel, angle;


        vel = MAX_LINEAR_SPEED * msg->data[3] / 100.0;
        angle = - MAX_STEERING_ANGLE * msg->data[2] / 100.0;

	if (vel > MAX_LINEAR_SPEED)
		vel = MAX_LINEAR_SPEED;
	else if (vel < -MAX_LINEAR_SPEED)
		vel = -MAX_LINEAR_SPEED;

	if (angle > MAX_STEERING_ANGLE)
		angle = MAX_STEERING_ANGLE;
	else if (angle < -MAX_STEERING_ANGLE)
		angle = -MAX_STEERING_ANGLE;

	if (vel >= -0.01 && vel <= 0.01)
		vel = 0;

        geometry_msgs::Twist alpha_msg;
        alpha_msg.linear.x = vel;
        alpha_msg.angular.z = angle;
        alpha_pub_.publish(alpha_msg);
#endif


    }
        break;

    case STRING_LIGHT_R: // 灯带

        break;

    case PHOTO_TAKEN_R: // 拍照完成
        if (current_state_== PATROLLING_A)
        {
//            std_msgs::Int8 continue_msg;
//            continue_pub_.publish(continue_msg);

            pauseMove(PHOTO_PAUSE, false);
        }
        break;
    }

    if ((msg->data[1] == NAVIGATING_R) || (msg->data[1] == CONTROLLING_R))
            last_patrol_id = 0;

    std::cout << "androidCallback" << std::endl;

}

void AFRobot::cmdFdbkCallback(const std_msgs::Int8::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(report_m);
    std_msgs::Int8MultiArray string_msg;
    string_msg.data.clear();
    string_msg.data.resize(10);

    //TODO 回馈状态筛查
    if (msg->data == 4)
    {
        malfunc_flag_ = true;
        return;
    }
    switch (current_state_)
    {
    case PATROLLING_A:

        if (msg->data != 3)
            break;
        string_msg.data[0] = 0x56;
        string_msg.data[1] = (unsigned char)(PATROLLING_A);
        string_msg.data[2] = (unsigned char)(current_id_);
        string_msg.data[3] = 0x01; // 完成标志
        string_msg.data[9] = checksum(string_msg);
        android_pub_.publish(string_msg);
        current_state_ = IDLING_A;
        last_patrol_id = 0;
        break;
    case NAVIGATING_A:
        if (msg->data != 3)
            break;
        string_msg.data[0] = 0x56;
        string_msg.data[1] = (unsigned char)(NAVIGATING_A);
        string_msg.data[2] = (unsigned char)(current_id_);
        string_msg.data[6] = 0x01; // 完成标志
        string_msg.data[9] = checksum(string_msg);
        android_pub_.publish(string_msg);
        current_state_ = IDLING_A;
        break;
    case IDLING_A:
        // NOTHING TO DO
        break;
    case CONTROLLING_A:
        // NOTHING TO DO
        break;
    case TOCHARGE_A:
        if (msg->data != 3)
            break;
        ROS_INFO("到达充电桩");
        current_state_ = CHARGING_A;
        break;
    case CHARGING_A:
        // NOTHING TO DO
        // SHOULD NOT REACH HERE
        break;
    case CHARGED_A:
        //NOTHING TO DO
        // SHOULD NOT REACH HERE
        break;
    case MALFUNC_A:
        //NOTHING TO DO ???
        break;
    default:
        break;
    }
}

void AFRobot::currentResultCallback(const af_msgs::Vertex::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(report_m);
    current_result_ = msg->id;
    std_msgs::Int8MultiArray string_msg;
    string_msg.data.clear();
    string_msg.data.resize(10);
    switch (current_state_)
    {
    case PATROLLING_A:
    {
        int intro_id = 0;
        if (isIntroSite(msg->id, &intro_id))
        {
            string_msg.data[0] = 0x56;
            string_msg.data[1] = 0x02; // 导航状态 //TODO 临时
            string_msg.data[2] = current_id_;
            string_msg.data[3] = intro_id; // 地点介绍id
            string_msg.data[9] = checksum(string_msg);
            android_pub_.publish(string_msg);
        }

        int camera_angle = 0;
        if (isPhotoSite(msg->id, &camera_angle))
        {
            pauseMove(PHOTO_PAUSE, true);

            int heading;
            try
            {

                listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_);
                heading = int(tf::getYaw(transform_.getRotation()) / std::asin(1) * 90);
            }
            catch (...)
            {
                heading = 0;
            }
            std_msgs::String camera_msg;
            std::stringstream ss;
            camera_angle -= heading;

            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "my_namespace";
            marker.id = 2;
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x_;
            marker.pose.position.y = y_;
            marker.pose.position.z = 0.0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = std::sin((camera_angle + heading) *std::asin(1) / 180);
            marker.pose.orientation.w = std::cos((camera_angle + heading) *std::asin(1) / 180);
            marker.scale.x = 1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.a = 1.0; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker_pub_.publish(marker);

            ss << 1 << ' ' << camera_angle;
            camera_msg.data = ss.str();
            camera_pub_.publish(camera_msg);
            photo_flag_ = true;

            ss.clear();
            ss.str("");
            ss << camera_angle + heading << ' '<< heading;
            std_msgs::String text_msg;
            text_msg.data = ss.str();
            text_pub_.publish(text_msg);
//
//            ros::Rate r(1);
//            r.sleep();
////TODO 等待十秒
//            string_msg.data.clear();
//            string_msg.data.resize(10);
//            string_msg.data[0] = 0x56;
//            string_msg.data[1] = 0x03; // 触发命令
//            string_msg.data[2] = 0x01; // 拍照
//            string_msg.data[9] = checksum(string_msg);
//            android_pub_.publish(string_msg);
        }
    }
        break;

    case NAVIGATING_A:
    {
        int intro_id = 0;
        if (isIntroSite(msg->id, &intro_id))
        {
            string_msg.data[0] = 0x56;
            string_msg.data[1] = 0x02; // 导航状态
            string_msg.data[2] = current_id_;
            string_msg.data[3] = intro_id; // 地点介绍id
            string_msg.data[9] = checksum(string_msg);
            android_pub_.publish(string_msg);
        }
    }
        break;
    default:
        break;
    }
}

void AFRobot::currentGoalCallback(const af_msgs::Vertex::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(report_m);
    std_msgs::Int8MultiArray string_msg;
    string_msg.data.clear();
    string_msg.data.resize(10);
    switch (current_state_)
    {
    case PATROLLING_A:
//        if (isPhotoSite(msg->id))
//        {
//            std_msgs::Int8 disable_msg;
//            disable_msg.data = 1;
//            disable_pub_.publish(disable_msg);
//        }
        break;

    case NAVIGATING_A:
//        if (isCrossing(msg->id) && (msg->id != vertex_[n_path_-1].id))
//        {
            // 如果当前目标点为终点,不需上报路口信息
//            for (int i=0; i<n_path_; i++)
//            {
//                if (vertex_[i].id == msg->id)
//                    break; /* for loop */
//            }
//            int num = 0;
//            for (int i=0; i<getVertexNum(); i++)
//            {
////                if (getLength())
//            }
//            string_msg.data[0] = 0x56;
//            string_msg.data[1] = 0x02; // 导航状态
//            string_msg.data[2] = current_id_;
//            string_msg.data[4] = 0;
//            string_msg.data[5] = 0;
//            string_msg.data[9] = checksum(string_msg);
//            android_pub_.publish(string_msg);
//        }
        break;

    default:
        break;
    }
}

void AFRobot::reportThread()
{
    std_msgs::Int8MultiArray string_msg;
    string_msg.data.clear();
    string_msg.data.resize(10);

    ros::Rate r(REPORT_RATE);

    while (ros::ok())
    {
        boost::mutex::scoped_lock lock(report_m);
        try
        {
            listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_);
            x_ = transform_.getOrigin().x();
            y_ = transform_.getOrigin().y();
        }
        catch (...)
        {
            ROS_WARN_ONCE("Transform from /map to /base_footprint is not running...");
            x_ = 0;
            y_ = 0;
        }

        string_msg.data[0] = 0x56;
        string_msg.data[1] = 0x05; // 实时状态上报
        string_msg.data[2] = current_state_; // 当前状态
        string_msg.data[3] = (int)(x_) >> 8; // x高8位    // TODO 像素点映射
        string_msg.data[4] = (int)(x_) & 0xFF; // x低8位
        string_msg.data[5] = (int)(y_) >> 8;
        string_msg.data[6] = (int)(y_) & 0xFF;
        string_msg.data[7] = battery_percentage_;
        string_msg.data[8] = inner_temperature_;
        string_msg.data[9] = checksum(string_msg);
//        android_pub_.publish(string_msg);

        static double start_sec = ros::Time::now().toSec();
        if (photo_flag_)
        {
            std::cout << start_sec << std::endl;
            std::cout << ros::Time::now().toSec() << std::endl;
            std::cout << ros::Time::now().toSec() - start_sec << std::endl;
            std::cout << "===============" << std::endl;
            if (ros::Time::now().toSec() - start_sec >= 10)
            {
                std_msgs::Int8MultiArray string_msg;
                string_msg.data.clear();
                string_msg.data.resize(10);
                string_msg.data[0] = 0x56;
                string_msg.data[1] = 0x03; // 触发命令
                string_msg.data[2] = 0x01; // 拍照
                string_msg.data[9] = checksum(string_msg);
                android_pub_.publish(string_msg);
                photo_flag_ = false;
            }
        }
        else
        {
            start_sec = ros::Time::now().toSec();
        }


        lock.unlock();

        r.sleep();
    }
}

void AFRobot::stateCallback(const af_msgs::RobotState::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(report_m);
    // 电源状态
    battery_percentage_ = msg->electric_quantity;
    // 内部温度
    inner_temperature_;
    // 故障状态
    malfunc_flag_ = msg->error;

    lifting_flag_ = msg->back_infrared || msg->front_infrared;

    emergency_stop_flag_ = msg->emergency_stop;
    call_security_flag_ = msg->call_security;
    front_collision_flag_ = msg->front_collision;

    deviated_flag_ = msg->deviation_path;

    do
    {
        static bool low_battery_reported = false;
        if (battery_percentage_ < LOW_BATTERY)
        {
            if (low_battery_reported == true)
                break;
            low_battery_reported = true;
            std_msgs::Int8MultiArray string_msg;
            string_msg.data.clear();
            string_msg.data.resize(10);
            string_msg.data[0] = 0x56;
            string_msg.data[1] = 0x03; // 触发状态
            string_msg.data[2] = 06; // 没电
            string_msg.data[9] = checksum(string_msg);
            android_pub_.publish(string_msg);

            current_state_ = TOCHARGE_A;

            std_msgs::Int8MultiArray cmd_msg;
            cmd_msg.data.clear();
            cmd_msg.data.resize(8);
            cmd_msg.data[0] = NAVIGATING;
            Vertex station_v;
            selectStation(station_v, 1);
            cmd_msg.data[1] = station_v.id;
            cmd_pub_.publish(cmd_msg);
        }
        else
        {
            low_battery_reported = false;
        }
    }
    while (0);

    do
    {
        static bool full_battery_reported = false;
        if (battery_percentage_ == 100)
        {
            if (full_battery_reported == true)
                break;
            full_battery_reported = true;

            current_state_ = CHARGED_A;
        }
        else
        {
            full_battery_reported = false;
        }
    }
    while (0);

    do
    {
        static bool malfunc_reported = false;
        if (malfunc_flag_ == true)
        {
            if (malfunc_reported == true)
                break;
            malfunc_reported = true;
            std_msgs::Int8MultiArray string_msg;
            string_msg.data.clear();
            string_msg.data.resize(10);
            string_msg.data[0] = 0x56;
            string_msg.data[1] = 0x03; // 触发状态
            string_msg.data[2] = 0x08; // 故障
            string_msg.data[9] = checksum(string_msg);
            android_pub_.publish(string_msg);
            current_state_ = MALFUNC_A;

            std_msgs::Int8MultiArray cmd_msg;
            cmd_msg.data.clear();
            cmd_msg.data.resize(8);
            cmd_msg.data[0] = IDLING;
            cmd_pub_.publish(cmd_msg);
        }
        else
        {
            malfunc_reported = false;
        }
    }
    while (0);

    do
    {
        static bool lifting_reported = false;
        if (lifting_flag_ == true)
        {
            if (lifting_reported == true)
                break;
            lifting_reported = true;
            std_msgs::Int8MultiArray string_msg;
            string_msg.data.clear();
            string_msg.data.resize(10);
            string_msg.data[0] = 0x56;
            string_msg.data[1] = 0x03; // 触发状态
            string_msg.data[2] = 0x07; // 搬离
            string_msg.data[9] = checksum(string_msg);
            android_pub_.publish(string_msg);
        }
        else
        {
            lifting_reported = false;
        }
    }
    while (0);

    do
    {
        static bool call_reported = false, call_on_off = false;
        if (call_security_flag_ == true)
        {
            if (call_reported == true)
                break;
            call_reported = true;
            call_on_off = !call_on_off;

            if (call_on_off == true)
            {
//                std_msgs::Int8 pause_msg;
//                pause_pub_.publish(pause_msg);
                pauseMove(INTERCOM_PAUSE, true);
            }
            else
            {
//                std_msgs::Int8 continue_msg;
//                continue_pub_.publish(continue_msg);
                pauseMove(INTERCOM_PAUSE, false);
            }
            std_msgs::Int8MultiArray string_msg;
            string_msg.data.clear();
            string_msg.data.resize(10);
            string_msg.data[0] = 0x56;
            string_msg.data[1] = 0x03; // 触发状态
            string_msg.data[2] = (call_on_off == true) ? 0x02 : 0x09; // 0x02对讲开始, 0x09对讲结束
            string_msg.data[9] = checksum(string_msg);
            android_pub_.publish(string_msg);
        }
        else
        {
            call_reported = false;
        }
    }
    while (0);

    do
    {
        static bool deviation_reported = false;
        if (deviated_flag_ == true)
        {
            if (deviation_reported == true)
                break;
            deviation_reported = true;
            std_msgs::Int8MultiArray string_msg;
            string_msg.data.clear();
            string_msg.data.resize(10);
            string_msg.data[0] = 0x56;
            string_msg.data[1] = 0x03; // 触发状态
            string_msg.data[2] = 0x05; // 偏离航线
            string_msg.data[9] = checksum(string_msg);
            android_pub_.publish(string_msg);
        }
        else
        {
            deviation_reported = false;
        }
//        std::cout << "lifting_flag_=" << lifting_flag_ << std::endl;
//        std::cout << "lifting_reported=" << lifting_reported << std::endl;

    }
    while (0);

}

void AFRobot::cameraCallback(const std_msgs::String::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(report_m);
    std::stringstream ss(msg->data.c_str());
    int cmd, angle;
    ss >> cmd >> angle;
//    std::cout << cmd << ' ' << angle << std::endl;
//    if (cmd != 0) // 麦克风阵列
        return; // TODO 目前唤醒词被屏蔽
    std_msgs::Int8MultiArray string_msg;
    string_msg.data.clear();
    string_msg.data.resize(10);
    string_msg.data[0] = 0x56;
    string_msg.data[1] = 0x03; // 触发状态
    string_msg.data[2] = 04; // 唤醒词
    string_msg.data[9] = checksum(string_msg);
    android_pub_.publish(string_msg);

}

int AFRobot::checksum(std_msgs::Int8MultiArray msg)
{
    int checksum = 0;
    for (int i=1; i<msg.data.size()-1; i++)
        checksum += msg.data[i];
    return checksum & 0xff;
}

int AFRobot::pauseMove(int event_index, bool pause_flag)
{
    static bool event[4] = {};
    if (event_index == CANCEL)
    {
        for (int i=0; i<4; i++)
            event[i] = false;
        std_msgs::Int8 pause_msg;
        pause_pub_.publish(pause_msg);
        return 0;
    }

    if (pause_flag == true)
    {
        std_msgs::Int8 pause_msg;
        pause_pub_.publish(pause_msg);
        event[event_index] = true;
    }
    else
    {
        event[event_index] = false;
        int i;
        for (i=0; i<4; i++)
            if (event[i] == true)
                break;
        switch (i)
        {
        case 0: // cancel
            break;
        case 1: // intercom
            break;
        case 2: // pedestrian
            break;
        case 3: // photo
        {
            int camera_angle = 0;
            if (isPhotoSite(current_result_, &camera_angle))
            {
                int heading;
                try
                {
                    listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_);
                    heading = int(tf::getYaw(transform_.getRotation()) / std::asin(1) * 90);
                }
                catch (...)
                {
                    heading = 0;
                }

                std_msgs::String camera_msg;
                std::stringstream ss;
                camera_angle -= heading;
                ss << 1 << ' ' << camera_angle;
                camera_msg.data = ss.str();
                camera_pub_.publish(camera_msg);
                photo_flag_ = true;
            }
        }
            break;
        default:
            std_msgs::Int8 continue_msg;
            continue_pub_.publish(continue_msg);
            break;
        }

    }

    return 0;
}
