/*
 * af_path_planner.h
 *
 *  Created on: 2016年8月5日
 *      Author: shengsong
 */

#ifndef INCLUDE_AF_PATH_PLANNER_H_
#define INCLUDE_AF_PATH_PLANNER_H_

#include "map_db.h"
#include <boost/smart_ptr/shared_ptr.hpp>

class AFPathPlanner : public Map
{
    boost::shared_ptr< double[] > dist_; // 线路长度
    boost::shared_ptr< int[] > path_prev_; // 前驱节点
    boost::shared_ptr< int[] > num_path_; // 路线上节点数

    boost::shared_ptr< Vertex[] > path_v_; // 最优路线上的节点
    boost::shared_ptr< Edge[] > path_e_; // 最优路线上的边

    int source_, destination_;
public:
    AFPathPlanner(const std::string host = HOST, const std::string user = USER, const std::string pass = PASS, const std::string database = DB);
    int getPrev(int id);
    int getPathVertex(Vertex& v, int index);
    int getPathEdge(Edge& e, int index);
    int setPath(int s, int d);
    int getSource();
    int getDestination();
    int searchPath();
    int locatePathVertex(Vertex v, int id, double r = ERR_DIST);
    int locatePathEdge(Vertex v, int id, double r = ERR_DIST);
    int getEdgeIndex(int id);
    int getVertexIndex(int id);
    int getNumPath();

};

#endif /* INCLUDE_AF_PATH_PLANNER_H_ */
