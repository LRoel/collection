#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import threading
import numpy as np
import numpy.linalg as la
import math
import time
import tf2_ros

from std_srvs.srv import Empty
from af_msgs.srv import SetPose

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TransformStamped
from af_msgs.msg import Gnss_Odom_lyz
from af_msgs.msg import Fuse_tf
from af_bringup.msg import Robot_encode
from af_msgs.msg import SetPose_resp

from tf import transformations
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import tf.transformations as tft


def amcl_initial_callback(pose):
    """AMCL 初始化回调函数

    Args:
        msg: ros回调数据

    """
    rospy.wait_for_service('initialpose')

    try:
        set_initial_pos = rospy.ServiceProxy('initialpose', SetPose)
        resp1 = set_initial_pos(pose)
        print resp1.responce.success
        print resp1.responce.covariance
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == '__main__':
    rospy.init_node('amcl_initial_callback')

    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, amcl_initial_callback)

    rospy.spin()
