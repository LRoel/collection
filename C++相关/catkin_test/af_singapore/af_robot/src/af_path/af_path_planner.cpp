/*
 * af_path_planner.cpp
 *
 *  Created on: 2016年8月4日
 *      Author: shengsong
 */
#include "af_path/af_path_planner.h"

AFPathPlanner::AFPathPlanner(const std::string host, const std::string user, const std::string pass, const std::string database)
:Map(host, user, pass, database), source_(0), destination_(0)
{
    dist_ = (boost::shared_ptr< double[] >) new double[getVertexNum()];
    path_prev_ = (boost::shared_ptr< int[] >) new int[getVertexNum()];
    num_path_ = (boost::shared_ptr< int[] >) new int[getVertexNum()];
}

int AFPathPlanner::getPrev(int id)
{
    return path_prev_[id-1];
}

int AFPathPlanner::getPathVertex(Vertex& v, int index)
{
    v = path_v_[index];
    return 0;
}

int AFPathPlanner::getPathEdge(Edge& e, int index)
{
    e = path_e_[index];
    return 0;
}

int AFPathPlanner::setPath(int s, int d)
{
    if (s <= 0 || s > getVertexNum() || d <= 0 || d > getVertexNum())
    {
        std::cout << "Planner::search(int s, int d) failed: Invalid input argument(s) " <<s <<" "<<d<<std::endl;
        return -1;
    }
    source_ = s;
    destination_ = d;
    return 0;
}

int AFPathPlanner::getSource()
{
    return source_;
}

int AFPathPlanner::getDestination()
{
    return destination_;
}

int AFPathPlanner::searchPath()
{
    if (source_ == 0 || destination_ == 0)
    {
        std::cout << "AFPathPlanner::searchPath(int, int) failed: Path arguments not initialised. " << std::endl;
        return -1;
    }
    if (source_ == destination_)
    {
        std::cout << "Staying around ..." <<std::endl;
        path_prev_[destination_-1] = source_;
        num_path_[destination_-1] = 2;

        path_v_ = boost::shared_ptr< Vertex[] >(new Vertex[num_path_[destination_-1]]);
        path_e_ = boost::shared_ptr< Edge[] >(new Edge[num_path_[destination_-1]]);
        Vertex v = {};
        Edge e = {};
        selectVertex(v, source_);
        path_v_[0] = path_v_[1] = v;
        path_e_[0] = path_e_[1] = e;

        return 0;
    }

    int num_v = getVertexNum();

    boost::shared_ptr< bool[] > visited(new bool[num_v]);//u,t,source & destination 从1开始,数组和内容从0开始 //注意bool[]

    for (int i=1; i<=num_v; i++)
    {
        dist_[i-1] = getLength(source_,i); // 刷新源点直接联通的节点距离
        std::cout << "i=" << i <<" " << dist_[i-1] << std::endl;
        visited[i-1] = false;
        if (dist_[i-1] > 0) // 与源点直接联通 未联通距离为0
        {
            path_prev_[i-1] = source_; // 前驱节点
            std::cout << path_prev_[i-1] << "*" << source_ <<std::endl;
            num_path_[i-1] = 2; // 与源点直接联通的节点处的路径点数为2
        }
        else
        {
            path_prev_[i-1] = -1;
            num_path_[i-1] = -1; // 当前节点不在路径上
        }
    }
    visited[source_-1] = true; // 源点直接联通的节点都已访问
    num_path_[source_-1] = 1; // 源点处的路径点数为1

    for (int i=0; i<num_v; i++)
        std::cout << path_prev_[i] << ' ';
    std::cout << std::endl;

    for (int i=2; i<=num_v; i++) // 访问其他num_v-1个节点
    {
        double tmp = 1000000; // 最短距离临时变量
        int u = source_;
        for (int j=1; j<=num_v; j++) // 取当前阶段扩展的路径点数最少的未扩展的节点
        {
            if(!visited[j-1] && dist_[j-1] < tmp && dist_[j-1] > 0)
            {
                u = j;
                tmp = dist_[j-1];
            }
        }
        if (u == source_)
            continue;
        visited[u-1] = true;
        if (u == destination_)
        {
            std::cout << "Shortest path found." << std::endl;

            Vertex v = {};
            Edge e = {};
            path_v_ = boost::shared_ptr< Vertex[] >(new Vertex[num_path_[destination_-1]]);
            path_e_ = boost::shared_ptr< Edge[] >(new Edge[num_path_[destination_-1]]);

            int t = destination_;
            while (t != source_)
            {
                Map::selectEdge(e, t, getPrev(t));
                Map::selectVertex(v, t);
                path_v_[num_path_[t-1]-1] = v; //path_v_:0~num_path_[destination_-1]
                path_e_[num_path_[t-1]-1 -1] = e; //path_e_:0~num_path_[destination_-1]-1
                // 从源点开始 源点为0 第一条线路0, 最后一点为num_path_[d_-1], 最后一条路为空(ID=-1)
                std::cout << t <<"("<< num_path_[t-1]-1<< ")*";
                t = getPrev(t);
            }

            std::cout << t << "("<< num_path_[t-1]-1<< ")" <<std::endl;
            Map::selectVertex(v, source_);
            path_v_[0] = v;
            path_e_[num_path_[destination_-1] -1].id = 0;

            return 0;
        }

        for (int j=1; j<=num_v; j++)
        {
            double length = getLength(u,j);
            std::cout << "length("<<u<<','<<j<<")="<<length << '\t';
            std::cout << visited[j-1] <<" " << dist_[j-1]<< std::endl;
            if (!visited[j-1] && length > 0)
            {
                int t_dist = dist_[u-1] + length;
                if ((t_dist < dist_[j-1] && dist_[j-1] > 0) || dist_[j-1] == 0) // 计算最短距离
                {
                    dist_[j-1] = t_dist; // 节点j目前最短距离
                    path_prev_[j-1] = u;    // 前驱节点
                    num_path_[j-1] = num_path_[u-1]+1; //   路径上节点数目(自source至j)
                }
            }
            std::cout << "length("<<u<<','<<j<<")="<<length << '\t';
                std::cout << visited[j-1] <<" " << dist_[j-1]<< std::endl;

        }
    }
    std::cout << "AFPathPlanner::searchPath() failed: No available path found. " << std::endl;
    return -1;
}

int AFPathPlanner::locatePathVertex(Vertex v, int id, double r)
{
    if (id <= 0 || id > getVertexNum())
    {
        std::cout << "AFPathPlanner::locatePathVertex(Vertex, int, double) failed: Invalid argument(s) " << id << std::endl;
        return -1;
    }
    int t_id = getVertexIndex(id);
    if (t_id == -1)
    {
        std::cout << "Vertex(" << id << ") is not on the path. " << std::endl;
        return -1;
    }
    Vertex v0 = path_v_[getVertexIndex(id)-1];

    if ((v.x - v0.x)*(v.x - v0.x) + (v.y - v0.y)*(v.y - v0.y) > r*r)
    {
        std::cout << "Vertex(" << v.x << ',' << v.y << ") is not a vertex of the path. " << std::endl;
        return -1;
    }
    return 0;
}

int AFPathPlanner::locatePathEdge(Vertex v, int id, double r)
{
    if (id <= 0 || id > getEdgeNum())
    {
        std::cout << "AFPathPlanner::locatePathEdge(Edge, int, double) failed: Invalid argument(s) " << id << std::endl;
        return -1;
    }
    int t_id = getEdgeIndex(id);
    if (t_id == -1)
    {
        std::cout << "Edge(" << id << ") is not on the path. " << std::endl;
        return -1;
    }
    Edge e = path_e_[t_id-1];
    Vertex v1 = path_v_[getVertexIndex(e.v1)-1], v2 = path_v_[getVertexIndex(e.v2)-1];
    if ((v1.y - v2.y) * v.x - (v2.x - v1.x) * v.y + v1.x * v2.y - v2.x * v1.y > r * sqrt((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y)))
    {
        std::cout << "Vertex(" << v.x << ',' << v.y << ") is not on Edge(" << e.id <<"). " << std::endl;
        return -1;
    }
    return 0;
}

int AFPathPlanner::getEdgeIndex(int id)
{
    if (id <= 0 || id > getEdgeNum())
    {
        std::cout << "AFPathPlanner::getVertexIndex(int) failed: Invalid argument(s) " << id << std::endl;
        return -1;
    }
    for (int i=1; i<=getNumPath(); i++)
    {
        if (path_e_[i-1].id == id)//path_e_[]从0索引 id从1索引
            return i;
    }
    return 0;
}

int AFPathPlanner::getVertexIndex(int id)
{
    if (id <= 0 || id > getVertexNum())
    {
        std::cout << "AFPathPlanner::getVertexIndex(int) failed: Invalid argument(s) " << id << std::endl;
        return -1;
    }
    for (int i=1; i<=getNumPath(); i++)
    {
        if (path_v_[i-1].id == id)//path_v_[]从0索引 id从1索引
            return i;
    }
    return 0;
}

int AFPathPlanner::getNumPath()
{
    return num_path_[destination_-1];
}
