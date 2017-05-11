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


class Imu_data:
    def __init__(self):
        self.data = 0

    def update(self, msg):
        self.data = msg.angular_velocity.z


def callback(msg):
    global n, offset
    #print "msg1", msg1.header.seq, "msg2", msg2.header.seq
    out = Robot_encode()
    out.header.stamp = msg.header.stamp
    out.header.seq = msg.header.seq
    out.left_encode = msg.left_encode
    out.right_encode = msg.right_encode
    if n <= 3000:
        print "initial``````"
        offset_data.append(imu_sub.data)
        if n == 3000:
            offset = sum(offset_data) / len(offset_data)
            print "initial ok"
            print offset
        n += 1
    else:
        if msg.left_encode == 0 and msg.right_encode == 0:
            out.theta = 0.0
        else:
            out.theta = (imu_sub.data - offset) * 0.01
        pub.publish(out)


if __name__ == '__main__':
    rospy.init_node('filter_fix_test')
    print "init ok"
    imu_sub = Imu_data()
    rospy.Subscriber('/imu/data_raw', Imu, imu_sub.update)
    rospy.Subscriber('/robot_encode_val', Robot_encode, callback)
    pub = rospy.Publisher('/imu_encode', Robot_encode, queue_size=100)
    print "register ok"
    rospy.spin()
