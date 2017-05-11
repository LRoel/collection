#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
import thread
import time

import tf2_ros
import tf
import geometry_msgs.msg
import numpy as np

from numpy import linalg as la

from tf.transformations import quaternion_from_matrix
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PointStamped


def odom_cb(odommsg):
    # odommsg = Odometry()
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = odommsg.pose.pose.position.x
    t.transform.translation.y = odommsg.pose.pose.position.y
    t.transform.translation.z = odommsg.pose.pose.position.z

    t.transform.rotation.x = odommsg.pose.pose.orientation.x
    t.transform.rotation.y = odommsg.pose.pose.orientation.y
    t.transform.rotation.z = odommsg.pose.pose.orientation.z
    t.transform.rotation.w = odommsg.pose.pose.orientation.w
    # print "pub ok!!"
    br.sendTransform(t)
    print "pub odom->base_footprint ok"

    # br2 = tf2_ros.TransformBroadcaster()
    # t2 = geometry_msgs.msg.TransformStamped()
    # t2.header.stamp = rospy.Time.now()
    # t2.header.frame_id = "map"
    # t2.child_frame_id = "odom"
    # t2.transform.translation.x = 0
    # t2.transform.translation.y = 0
    # t2.transform.translation.z = 0
    #
    # t2.transform.rotation.x = 0
    # t2.transform.rotation.y = 0
    # t2.transform.rotation.z = 0
    # t2.transform.rotation.w = 1
    # # print "pub ok!!"
    # br2.sendTransform(t2)
    # print "pub map->odom ok"

    # br3 = tf2_ros.TransformBroadcaster()
    # t3 = geometry_msgs.msg.TransformStamped()
    # t3.header.stamp = rospy.Time.now()
    # t3.header.frame_id = "laser"
    # t3.child_frame_id = "base_footprint"
    # t3.transform.translation.x = 0
    # t3.transform.translation.y = 0
    # t3.transform.translation.z = 0
    #
    # t3.transform.rotation.x = 0
    # t3.transform.rotation.y = 0
    # t3.transform.rotation.z = 0
    # t3.transform.rotation.w = 1
    # # print "pub ok!!"
    # br3.sendTransform(t3)
    # print "pub laser->base_footprint ok"




if __name__ == '__main__':
    rospy.init_node('odom_tf', anonymous=True)

    rospy.Subscriber('/odom', Odometry, odom_cb)

    rospy.spin()