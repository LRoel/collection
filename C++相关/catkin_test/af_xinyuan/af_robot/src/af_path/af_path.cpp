/*
 * af_path.cpp
 *
 *  Created on: 2016年8月16日
 *      Author: shengsong
 */

#include "af_path/af_path.h"

int AFPath::clearPath()
{
    size_ = 0;
    vertex_.reset();
    return 0;
}

int AFPath::getPatrolPath(int id)
{
    delInterim();
    std::string patrol_path_s(selectPatrolPath(id));
    std::stringstream patrol_path(patrol_path_s);
    if (patrol_path.rdbuf()->in_avail() == 0)
    {
        std::cout << "AFPath::getPatrolPath(int) 失败，巡逻路径(" << id << ")不存在。    " << std::endl;
        return -1;
    }
    // 取巡逻路径
    int vid;
    size_ = 0;
    while (patrol_path >> vid)
    {
        //TODO 能否一次完成而不重置stringstream?
        size_ ++;
    }
    size_ ++;
    vertex_ = boost::shared_ptr< Vertex[] >(new Vertex[size_]);

    try
    {
        listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_);
        x_ = transform_.getOrigin().x();
        y_ = transform_.getOrigin().y();
    }
    catch (...)
    {
        x_ = 0;
        y_ = 0;
    }
    vertex_[0].x = x_;
    vertex_[0].y = y_;
    int sid = addVertex(x_, y_, true);
    vertex_[0].id = sid;
    Vertex v;
    selectVertex(v, getVertexNum());
    if (v.interim)
    {
        double r = 0;
        Edge e;
        while (selectEdge(e, locateEdge(x_, y_, r)) != 0) // TODO 必须保证线路密度足够大 否则会绕至远点再到终点
        {
            std::cout << x_ << ' ' << y_ << '*' << r<<std::endl;
            r += 0.1;
        }
        Map::addEdge(distance(sid, e.v1), sid, e.v1, true);
        Map::addEdge(distance(sid, e.v2), sid, e.v2, true);
    }

    patrol_path.clear();
    patrol_path.str(patrol_path_s); // 重置stringstream 必须使用clear()及str()方法
    for (int i=1; i<size_; i++)
    {
        patrol_path >> vertex_[i].id;
        selectVertex(vertex_[i], vertex_[i].id);
    }
    return 0;
}

int AFPath::getNavPath(int id)
{
    delInterim();
    try
    {
        listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform_);
        x_ = transform_.getOrigin().x();
        y_ = transform_.getOrigin().y();
    }
    catch (...)
    {
        x_ = 0;
        y_ = 0;
    }
    int sid = addVertex(x_, y_, true);
//    double r = WIDTH / 2;
    Vertex v;
    selectVertex(v, getVertexNum());
    if (v.interim)
    {
        double r = 0;
        Edge e;
        while (selectEdge(e, locateEdge(x_, y_, r)) != 0) // TODO 必须保证线路密度足够大 否则会绕至远点再到终点
        {
            std::cout << x_ << ' ' << y_ << '*' << r<<std::endl;
            r += 0.1;
        }
        Map::addEdge(distance(sid, e.v1), sid, e.v1, true);
        Map::addEdge(distance(sid, e.v2), sid, e.v2, true);
    }
    setPath(sid, id);
    searchPath();
    size_ = getNumPath(); //含起始点
    vertex_ = boost::shared_ptr< Vertex[] >(new Vertex[size_]);
    for (int i=0; i<size_; i++)
        getPathVertex(vertex_[i], i);
    return 0;
}

int AFPath::pubPath()
{
    af_msgs::Path path_msg;
    path_msg.size = size_;
    path_msg.vertex.clear();
    path_msg.vertex.resize(path_msg.size);
    for (int i=0; i<path_msg.size; i++)
    {
        path_msg.vertex[i].id = vertex_[i].id;
        path_msg.vertex[i].x = vertex_[i].x;
        path_msg.vertex[i].y = vertex_[i].y;
        std::cout << path_msg.vertex[i].id << " ";
    }
    std::cout << std::endl;
    path_pub_.publish(path_msg);
    return 0;
}

void AFPath::cmdCallback(const std_msgs::Int8MultiArray::ConstPtr& msg)
{
    std::cout<<"SELECT * FROM patrol_route WHERE id=" << (unsigned char)(msg->data[1]) <<std::endl;
    clearPath();
    switch (msg->data[0])
    {
    case PATROLLING:
        if (msg->data[1] == '\x0')
        {
            std::cout << "不完整的数据!!!" << std::endl;
            return;
        }
        std::cout<<"SELECT * FROM patrol_route WHERE id=" << (unsigned char)(msg->data[1]) <<std::endl;
        getPatrolPath((unsigned char)(msg->data[1]));
        pubPath();
        break;
    case NAVIGATING:
        if (msg->data[1] == '\x0')
        {
            std::cout << "不完整的数据!!!" << std::endl;
            return;
        }
        getNavPath((unsigned char)(msg->data[1]));
        pubPath();
        break;
    case IDLING:
        pubPath();
        break;
    default:
        std::cout << "AFPath::cmdCallback(const std_msgs::IntMultiArray::ConstPtr&) : 接收到由af_robot发送的错误指令 " << std::endl;
        break;
    }
}

AFPath::AFPath(const std::string host, const std::string user, const std::string pass, std::string database)
:AFPathPlanner(host, user, pass, database), size_(0), current_state_(IDLING),
 x_(0), y_(0)
{
    cmd_sub_ = nh_.subscribe< std_msgs::Int8MultiArray >("cmd", 10, &AFPath::cmdCallback, this);
//    path_fdbk_sub_ = nh_.subscribe< std_msgs::Int8 >("path_fdbk", 10, &AFPath::pathFdbkCallback, this);

    path_pub_ = nh_.advertise< af_msgs::Path >("path", 10);
//    cmd_fdbk_pub_ = nh_.advertise< std_msgs::Int8 >("cmd_fdbk", 10);

    try
    {
        listener_.waitForTransform("/map", "/base_footprint", ros::Time(0), ros::Duration(10.0));
    }
    catch (...)
    {
    }
    vertex_ = boost::shared_ptr< Vertex[] >(new Vertex[0]);
}


