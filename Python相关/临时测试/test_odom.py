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
from geometry_msgs.msg import TransformStamped


def callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    # msg = Odometry()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w

    br.sendTransform(t)


def listener():
    rospy.init_node('odom_pub', anonymous=True)

    rospy.Subscriber("odom", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()