#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import my_message_filter
from af_msgs.msg import comb
from af_msgs.msg import Robot_encode


class GPS_Encode:
    def __init__(self):
        rospy.init_node('filter_test')
        print "init ok"

        print "register ok"


def callback(msg1, msg2):
    # msg1 = GPS_Odom()
    # msg2 = LaserScan()

    print "msg1", msg1.header.stamp, "msg2", msg2.header.stamp


if __name__ == '__main__':

    rospy.spin()
