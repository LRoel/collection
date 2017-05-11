/*
 * af_mapping.cpp
 *
 *  Created on: 2016年8月23日
 *      Author: shengsong
 */

#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread.hpp>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

#include "visualization_msgs/Marker.h"

#include "af_msgs/Vertex.h"
#include "map_db.h"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71
#define KEYCODE_SPACE 0x20
#define KEYCODE_ENTER 0x0A


class AFMapping : public Map
{
    Vertex prev_v_, current_pos_;
    ros::NodeHandle nh_;
    double linear_, angular_, l_scale_, a_scale_;
    ros::Publisher twist_pub_;
    ros::Publisher marker_pub_;
    boost::thread plot_thrd;

    ros::Publisher pose_array_pub_;

    ros::Subscriber point_sub_;

    tf::TransformListener listener_;
    tf::StampedTransform transform_;

    boost::mutex db_m_;
    void plotThread();

public:
    AFMapping();
    void keyLoop();
    void clickPoint(const geometry_msgs::PoseStamped::ConstPtr& msg);

};

AFMapping::AFMapping()
: linear_(0), angular_(0),
  l_scale_(1), a_scale_(1)
{
    nh_.param("scale_angular", a_scale_, a_scale_);
    nh_.param("scale_linear", l_scale_, l_scale_);

    twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    pose_array_pub_ = nh_.advertise< geometry_msgs::PoseArray >("pose_array", 10);

    marker_pub_ = nh_.advertise< visualization_msgs::Marker >("visualization_marker", 10);

    point_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &AFMapping::clickPoint, this);

    try
    {
        listener_.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0));
        listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_);
        current_pos_.x = transform_.getOrigin().x();
        current_pos_.y = transform_.getOrigin().y();
    }
    catch (...)
    {
        current_pos_.x = transform_.getOrigin().x();
        current_pos_.y = transform_.getOrigin().y();
    }
    plot_thrd = boost::thread(&AFMapping::plotThread, this);
    prev_v_.id = 0;
}

void AFMapping::clickPoint(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    boost::mutex::scoped_lock lock(db_m_);
    Vertex current_v;
    current_v.x = msg->pose.position.x;
    current_v.y = msg->pose.position.y;
    current_v.id = addVertex(current_v, false);

    std::cout << "Currently at Vertex"<< current_v.id<< std::endl;
    if (prev_v_.id != 0)
        addEdge(distance(prev_v_.id, current_v.id), prev_v_.id, current_v.id, false);

    prev_v_ = current_v;
    std::cout << "&&&&" << getVertexNum() << std::endl;
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    (void)sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

void AFMapping::keyLoop()
{
    char c;
    bool dirty=false;


    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the robot.");

    for(;;)
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        linear_=angular_=0;
        ROS_INFO("value: 0x%02X\n", c);

        boost::mutex::scoped_lock lock(db_m_);

        switch(c)
        {
        case KEYCODE_Q:
            static int cnt = 0;
            cnt ++;

            ROS_DEBUG("ESCAPE");
            if (cnt == 3)
            {
                delData();
                cnt = 0;
            }
            break;
        case KEYCODE_L:
            ROS_DEBUG("LEFT");
            angular_ = 1.0;
            dirty = true;
            break;
        case KEYCODE_R:
            ROS_DEBUG("RIGHT");
            angular_ = -1.0;
            dirty = true;
            break;
        case KEYCODE_U:
            ROS_DEBUG("UP");
            linear_ = 1.0;
            dirty = true;
            break;
        case KEYCODE_D:
            ROS_DEBUG("DOWN");
            linear_ = -1.0;
            dirty = true;
            break;
        case KEYCODE_SPACE:
            ROS_DEBUG("ENTER");
            angular_ = 0.0;
            linear_ = 0.0;
            dirty = true;
            break;
        case KEYCODE_ENTER:
            {
                Vertex current_v = current_pos_;
                current_v.id = addVertex(current_pos_, false);
                selectVertex(current_v, current_v.id);

                std::cout << "Currently at Vertex"<< current_v.id<< std::endl;
                if (prev_v_.id != 0)
                    addEdge(distance(prev_v_.id, current_v.id), prev_v_.id, current_v.id, false);

                prev_v_ = current_v;
            }
            break;
        }

        std::cout << "&&&&" << getVertexNum() << std::endl;
        geometry_msgs::Twist twist;
        twist.angular.z = a_scale_*angular_;
        twist.linear.x = l_scale_*linear_;
        if(dirty ==true)
        {
            twist_pub_.publish(twist);
            dirty=false;
        }
    //        vertex_ = boost::shared_ptr< Vertex[] >(new Vertex[getVertexNum()]);
    }
    return;
}

void AFMapping::plotThread()
{
    ros::Rate r(10);

    while (ros::ok())
    {
        ros::spinOnce();
        boost::mutex::scoped_lock lock(db_m_);
        try
        {
            listener_.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0));
            listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_);
            current_pos_.x = transform_.getOrigin().x();
            current_pos_.y = transform_.getOrigin().y();
        }
        catch (...)
        {
            current_pos_.x = transform_.getOrigin().x();
            current_pos_.y = transform_.getOrigin().y();
        }
        visualization_msgs::Marker points, line_list;
        points.header.frame_id = line_list.header.frame_id = "map";
        points.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_list.ns = "map_database";
        points.action = line_list.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
        points.id = 0;
        line_list.id = 1;
        points.type = visualization_msgs::Marker::POINTS;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        points.scale.x = 0.2;
        points.scale.y = 0.2;
        line_list.scale.x = 0.1;

        points.color.g =1.0f;
        points.color.a =1.0;
        line_list.color.r =1.0f;
        line_list.color.a =1.0;

        for (int i=0; i<getVertexNum(); i++)
        {
            Vertex v;
            selectVertex(v, i+1);
            geometry_msgs::Point p;
            p.x = v.x;
            p.y = v.y;
            p.z = 0;

            points.points.push_back(p);
        }

        for (int i=0; i<getEdgeNum(); i++)
        {
            Edge e;
            Vertex v1, v2;
            selectEdge(e, i+1);
            selectVertex(v1, e.v1);
            selectVertex(v2, e.v2);
            geometry_msgs::Point p1, p2;
            p1.x = v1.x;
            p1.y = v1.y;
            p1.z = 0;
            p2.x = v2.x;
            p2.y = v2.y;
            p2.z = 0;

            line_list.points.push_back(p1);
            line_list.points.push_back(p2);
        }

        marker_pub_.publish(line_list);
        marker_pub_.publish(points);
        lock.unlock();
        r.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "af_mapping");
    AFMapping af_mapping;
//    ros::spin();
    signal(SIGINT, quit);

    af_mapping.keyLoop();
    return 0;
}



