#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import message_filters
from sensor_msgs.msg import Imu
from af_msgs.msg import Robot_encode
from std_srvs.srv import Empty


class IMU_Fix(Imu):
    def __init__(self, *args, **kwds):
        super(IMU_Fix, self).__init__(*args, **kwds)
        self.zero_offset = 100

    def fix(self):
        self.angular_velocity.z -= self.zero_offset
        return self


class Test:
    def __init__(self):
        rospy.init_node('test_node')
        self.out = IMU_Fix()
        print "init ok"
        rospy.Subscriber('/imu', IMU_Fix, self.callback)
        print "register ok"

    def callback(self, msg):
        print msg
        print msg.fix()

    def zero_offset(self, req):
        pass


def test():
    t = Test()
    rospy.spin()


if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException:
        pass