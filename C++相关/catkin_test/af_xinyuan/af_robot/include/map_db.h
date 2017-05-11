/*
 * map_db.h
 *
 *  Created on: 2016年8月4日
 *      Author: shengsong
 */

#ifndef INCLUDE_MAP_DB_H_
#define INCLUDE_MAP_DB_H_

#include <mysql_connection.h>

#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

#include <cmath>
#include <sstream>

#include "map_struct.h"

#define HOST "localhost"
#define USER "root"
#define PASS "ros"
#define DB "af_map"

#define ERR_DIST 1.0 //TODO should be changed to ros param
#define WIDTH 3.0


class Map
{
    sql::Driver* driver_;
    std::auto_ptr< sql::Connection > con_;
    int n_vertex_; ///< Brief The number of vertices
    int n_edge_; ///< Brief The number of edges

public:
    Map(const std::string host = HOST, const std::string user = USER, const std::string pass = PASS, const std::string database = DB);

    int getVertexNum(); ///< Brief 获取节点数
    int addVertex(double x, double y, bool interim = true);
    int addVertex(Vertex v, bool interim = true);
    int delVertex(int id);
    int selectVertex(Vertex& v, int id);
    int selectVertex(Vertex& v, double x, double y, double r = ERR_DIST);
    int locateVertex(double x, double y, double r = ERR_DIST);
    int locateVertex(Vertex v, double r = ERR_DIST);

    int getEdgeNum(); ///< Brief 获取道路数
    int addEdge(double length, int v1, int v2, bool interim = true);
    int addEdge(Edge e, bool interim = true);
    int delEdge(int id);
    int selectEdge(Edge& e, int id);
    int selectEdge(Edge& e, int v1, int v2);
    int locateEdge(double x, double y, double r = WIDTH / 2);
    int locateEdge(Vertex v, double r = WIDTH / 2);
    double getLength(int id);
    double getLength(int v1, int v2);

    int delInterim(); ///< Brief 删除所有临时节点和道路
    int delData();

    double distance(Vertex v1, Vertex v2);
    double distance(int v1, int v2);

    bool isPhotoSite(int id, int* angle = NULL);
    bool isIntroSite(int id, int* intro_id = NULL);
    bool isCrossing(int id);

    std::string selectPatrolPath(int id);
    int selectStation(Vertex& v, int id);

};

#endif /* INCLUDE_MAP_DB_H_ */
