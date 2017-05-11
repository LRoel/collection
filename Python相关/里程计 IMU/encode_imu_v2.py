#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import time
import message_filters
from sensor_msgs.msg import Imu
from af_msgs.msg import Robot_encode
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse


class IMU_Fix(Imu):
    def __init__(self, *args, **kwds):
        super(IMU_Fix, self).__init__(*args, **kwds)
        self.zero_offset = 0

    def fix(self):
        self.angular_velocity.z -= self.zero_offset
        return self

    def set_zero(self):
        self.angular_velocity.z = 0
        return self


class Filter:
    def __init__(self):
        rospy.init_node('filter_test')
        self.out = IMU_Fix()
        print "init ok"
        msg1_sub = message_filters.Subscriber('/imu/data_raw', IMU_Fix)
        msg2_sub = message_filters.Subscriber('robot_encode_val', Robot_encode)
        self._offset_data = []
        self._flag_zero_offset = False
        s = rospy.Service('/imu/zero_offset', Trigger, self.zero_offset)
        self.pub = rospy.Publisher('/imu_encode', Robot_encode, queue_size=100)
        ts = message_filters.ApproximateTimeSynchronizer([msg1_sub, msg2_sub], 100, 0.01)
        ts.registerCallback(self.callback)
        print "register ok"

    def callback(self, msg1, msg2):
        self.out.header.stamp = msg2.header.stamp
        self.out.header.seq = msg2.header.seq
        self.out.left_encode = msg2.left_encode
        self.out.right_encode = msg2.right_encode
        if msg2.left_encode == 0 and msg1.right_encode == 0:
            self.out.theta = msg1.zero().angular_velocity.z * 0.01
        else:
            self.out.theta = msg1.fix().angular_velocity.z * 0.01
        if self._flag_zero_offset:
            self._offset_data.append(msg1.angular_velocity.z)
        print self.out.fix()
        self.pub.publish(self.out.fix())

    def zero_offset(self):
        resp = TriggerResponse()
        self._flag_zero_offset = True
        time.sleep(30)
        self._flag_zero_offset = False
        if len(self._offset_data) >= 2500:
            IMU_Fix.zero_offset = sum(self._offset_data) / len(self._offset_data)
            resp.success = True
            resp.message = '零偏初始化成功'
        else:
            resp.success = False
            resp.message = '零偏初始化失败'


def filter_imu_encode():
    t = Filter()
    rospy.spin()


if __name__ == '__main__':
    try:
        filter_imu_encode()
    except rospy.ROSInterruptException:
        pass


# t = IMU_Fix()
#
# n = 0
# offset = 0
# offset_data = []
#
# def callback(msg1, msg2):
#     global n, offset
#     #print "msg1", msg1.header.seq, "msg2", msg2.header.seq
#     out = Robot_encode()
#     out.header.stamp = msg2.header.stamp
#     out.header.seq = msg2.header.seq
#     out.left_encode = msg2.left_encode
#     out.right_encode = msg2.right_encode
#     out.theta = msg1.angular_velocity.z * 0.01
#     if n <= 3000:
#         # print "initial``````"
#         offset_data.append(msg1.angular_velocity.z)
#         if n == 3000:
#             offset = sum(offset_data) / len(offset_data)
#             print "initial ok"
#             print offset
#         n += 1
#     else:
#         out.theta = (msg1.angular_velocity.z - offset) * 0.01
#         pub.publish(out)
#
#
# if __name__ == '__main__':
#     rospy.init_node('filter_test')
#     print "init ok"
#     msg1_sub = message_filters.Subscriber('/imu/data_raw', Imu)
#     msg2_sub = message_filters.Subscriber('robot_encode_val', Robot_encode)
#     s = rospy.Service('/initialpose', SetPose, t2.amcl_set_pose)
#     pub = rospy.Publisher('/imu_encode', Robot_encode, queue_size=100)
#     ts = message_filters.ApproximateTimeSynchronizer([msg1_sub, msg2_sub], 100, 0.01)
#     ts.registerCallback(callback)
#     print "register ok"
#     rospy.spin()
