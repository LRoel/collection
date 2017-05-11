#!/usr/bin/env python

import rospy

import tf2_ros
import math

from af_msgs.msg import Robot_encode
sum_theta = 0


n = 0

def do_it(msg):
    global sum_theta,n
    d_theta = msg.theta
    sum_theta += d_theta
    if n % 10 == 0:
        print sum_theta * 180 / math.pi
    n += 1


if __name__ == '__main__':
    try:
        rospy.init_node('yaw_test')

        rospy.Subscriber('/imu_encode', Robot_encode, do_it)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
