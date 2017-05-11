#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import message_filters
from sensor_msgs.msg import Imu
from af_msgs.msg import Robot_encode

n = 0
offset = 0
offset_data = []

def callback(msg1, msg2):
    global n, offset
    #print "msg1", msg1.header.seq, "msg2", msg2.header.seq
    out = Robot_encode()
    out.header.stamp = msg2.header.stamp
    out.header.seq = msg2.header.seq
    out.left_encode = msg2.left_encode
    out.right_encode = msg2.right_encode
    out.theta = msg1.angular_velocity.z * 0.01
    if n <= 3000:
        print "initial``````"
        offset_data.append(msg1.angular_velocity.z)
        if n == 3000:
            offset = sum(offset_data) / len(offset_data)
            print "initial ok"
            print offset
        n += 1
    else:
        out.theta = (msg1.angular_velocity.z - offset) * 0.01
        pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('filter_test')
    print "init ok"
    msg1_sub = message_filters.Subscriber('/imu/data_raw', Imu)
    msg2_sub = message_filters.Subscriber('robot_encode_val', Robot_encode)
    pub = rospy.Publisher('/imu_encode', Robot_encode, queue_size=100)
    ts = message_filters.ApproximateTimeSynchronizer([msg1_sub, msg2_sub], 100, 0.01)
    ts.registerCallback(callback)
    print "register ok"
    rospy.spin()
