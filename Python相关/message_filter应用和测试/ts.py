#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import my_message_filter
from af_msgs.msg import comb
from af_msgs.msg import Robot_encode


def callback(msg1, msg2):
    # msg1 = GPS_Odom()
    # msg2 = LaserScan()

    print "msg1", msg1.header.stamp, "msg2", msg2.header.stamp


if __name__ == '__main__':
    rospy.init_node('filter_test')
    print "init ok"
    msg1_sub = my_message_filter.Subscriber('gps', comb)
    msg2_sub = my_message_filter.Subscriber('robot_encode_val', Robot_encode)
    ts = my_message_filter.ApproximateTimeSynchronizer([msg1_sub, msg2_sub], 100, 0.01)
    ts.registerCallback(callback)
    print "register ok"
    rospy.spin()
