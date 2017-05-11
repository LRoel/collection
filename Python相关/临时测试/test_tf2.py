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
from geometry_msgs.msg import TransformStamped

rospy.init_node('tf2_test')

tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(0.1)


def tran_mat44(trans):
    # trans = TransformStamped()
    trans_xyz = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
    trans_quat = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
    mat44 = np.dot(tft.translation_matrix(trans_xyz), tft.quaternion_matrix(trans_quat))
    return mat44

while not rospy.is_shutdown():
    try:
        base_odom = tfBuffer.lookup_transform('odom', 'base_footprint', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rate.sleep()
        continue

    base_odom_mat44 = tran_mat44(base_odom)

    print base_odom_mat44
    print base_odom


    rate.sleep()


