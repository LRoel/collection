#!/usr/bin/env python
import rospy
import tf
import threading
import numpy as np
import math
import datetime
import tf2_ros
import tf
import tf.transformations as tft
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def callback(msg):
    ranges = np.asarray(msg.ranges)
    ranges[ranges > 65] = 0.0
    print ranges.argmax(), ranges.max()


def listener():
    rospy.init_node('scan_test', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
