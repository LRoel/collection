#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import time

from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid


def get_map(self):
    rospy.wait_for_message('/map', OccupancyGrid)
    print "ok"
    resp = GetMapResponse()
    resp.map = static_map
    return resp


def save_map(msg):
    global static_map
    static_map = msg


if __name__ == '__main__':
    rospy.init_node('static_map_test')

    static_map = OccupancyGrid()
    static_map.header.stamp = rospy.Time.now()
    static_map.header.frame_id = 'map'
    static_map.info.resolution = 0.1

    s = rospy.Service('/static_map', GetMap, get_map)
    rospy.Subscriber('/map', OccupancyGrid, save_map)

    rospy.spin()
