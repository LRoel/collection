/*
 * map_struct.h
 *
 *  Created on: 2016年8月11日
 *      Author: shengsong
 */

#ifndef INCLUDE_MAP_STRUCT_H_
#define INCLUDE_MAP_STRUCT_H_

struct Vertex
{
    int id;
    double x, y;
    bool interim;
};

struct Edge
{
    int id;
    double length;
    int v1, v2;
    bool interim;
};



#endif /* INCLUDE_MAP_STRUCT_H_ */
