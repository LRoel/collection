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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TransformStamped
from af_msgs.msg import Gnss_Odom_lyz
from af_msgs.msg import Fuse_tf
from af_bringup.msg import Robot_encode
from std_msgs.msg import Bool

from tf import transformations
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import tf.transformations as tft


odom_x = 2.0
odom_y = 2.0
odom_theta = 1.0
map_x = 0.0
map_y = 0.0
map_theta = 0.0
mat44_map_laser = tft.euler_matrix(0, 0, map_theta)
mat44_map_laser[0][3] = map_x
mat44_map_laser[1][3] = map_y
mat44_laser_map = la.inv(mat44_map_laser)
mat44_odom_laser = tft.euler_matrix(0, 0, odom_theta)
mat44_odom_laser[0][3] = odom_x
mat44_odom_laser[1][3] = odom_y
mat44_map_odom = la.inv(np.dot(mat44_odom_laser, mat44_laser_map))
theta = tft.euler_from_matrix(mat44_map_odom)[2]
x = mat44_map_odom[0][3]
y = mat44_map_odom[1][3]
print x, y, theta

