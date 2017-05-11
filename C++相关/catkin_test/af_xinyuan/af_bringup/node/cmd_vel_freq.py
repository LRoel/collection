#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
from geometry_msgs.msg import Twist


class FilterCmd:
    def __init__(self):
        self.x_data = [0] * 4
        self.y_data = [0] * 4
        self.a = [1.0, -1.76, 1.1829, -0.2781]
        self.b = [1.0, 3.0, 3.0, 1.0]

    def filter(self, x):
        self.x_data.append(x)
        self.x_data.pop(0)
        y = self.b[3] * self.x_data[0] + self.b[2] * self.x_data[1] + \
            self.b[1] * self.x_data[2] + self.b[0] * self.x_data[3] - \
            self.a[3] * self.y_data[0] - self.a[2] * self.y_data[1] - \
            self.a[1] * self.y_data[2]
        self.y_data.append(y)
        self.y_data.pop(0)
        print "x_data", self.x_data
        print "y_data", self.y_data
        return y

    def init(self):
        self.x_data = [0] * 4
        self.y_data = [0] * 4


class FilterCmd2:
    def __init__(self):
        self.data = []
        # self.weight = [0.071004774439889686843407901051250519231 * 1.2444006859614807,
        #                0.060993473006745221876023776985675795004 * 1.2444006859614807,
        #                0.078509113286873544179655937114148400724 * 1.2444006859614807,
        #                0.09199840278315227215522043024975573644 * 1.2444006859614807,
        #                0.099294079283712782801885055050661321729 * 1.2444006859614807,
        #                0.099294079283712782801885055050661321729 * 1.2444006859614807,
        #                0.09199840278315227215522043024975573644 * 1.2444006859614807,
        #                0.078509113286873544179655937114148400724 * 1.2444006859614807,
        #                0.060993473006745221876023776985675795004 * 1.2444006859614807,
        #                0.071004774439889686843407901051250519231 * 1.2444006859614807]

        self.weight = [0.018329393628309424701949836844505625777,
                       0.019007513944708710751374525216306210496,
                       0.027284393967880214010301997973328980152,
                       0.036511378741936453828920150499470764771,
                       0.046167834731909287993012469542009057477,
                       0.055627605559800397116987369372509419918,
                       0.064247722909847732042365464621980208904,
                       0.071392357595016511795371627613349119201,
                       0.076487062045247619646026748796430183575,
                       0.079145801116610861591915693225018912926,
                       0.079145801116610861591915693225018912926,
                       0.076487062045247619646026748796430183575,
                       0.071392357595016511795371627613349119201,
                       0.064247722909847732042365464621980208904,
                       0.055627605559800397116987369372509419918,
                       0.046167834731909287993012469542009057477,
                       0.036511378741936453828920150499470764771,
                       0.027284393967880214010301997973328980152,
                       0.019007513944708710751374525216306210496,
                       0.018329393628309424701949836844505625777]

    def filter(self, x):
        self.data.append(x)
        if len(self.data) > 20:
            self.data.pop(0)
        print "data", self.data
        if len(self.data) < 20:
            data_out = sum(self.data) * 1.0 / len(self.data)
        else:
            data_out = self.weight[0] * self.data[0] + self.weight[1] * self.data[1] +\
                    self.weight[2] * self.data[2] + self.weight[3] * self.data[3] +\
                    self.weight[4] * self.data[4] + self.weight[5] * self.data[5] +\
                    self.weight[6] * self.data[6] + self.weight[7] * self.data[7] +\
                    self.weight[8] * self.data[8] + self.weight[9] * self.data[9] +\
                    self.weight[10] * self.data[10] + self.weight[11] * self.data[11] +\
                    self.weight[12] * self.data[12] + self.weight[13] * self.data[13] +\
                    self.weight[14] * self.data[14] + self.weight[15] * self.data[15] +\
                    self.weight[16] * self.data[16] + self.weight[17] * self.data[17] +\
                    self.weight[18] * self.data[18] + self.weight[19] * self.data[19]
        return data_out

    def init(self):
        self.data = []


class FreqCmd:
    def __init__(self):
        rospy.init_node('cmd_vel_freq')
        self.cmd_vel = Twist()
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb)
        self.pub = rospy.Publisher('/cmd_vel_freq', Twist, queue_size=100)
        self.pub_time = rospy.Time.now()
        self.cmd_time = rospy.Time.now() - rospy.Duration(10)
        self.filter = FilterCmd2()
        self.stop_flag = False

        t1 = threading.Thread(target=self.pub_cmd)
        t1.start()

        rospy.spin()

    def stop_mv(self):
        rate = rospy.Rate(20)
        cmd_vel_out = Twist()
        self.stop_flag = True
        for i in range(1, 20):
            self.pub_time = rospy.Time.now()
            cmd_vel_out.angular.z = 0.0
            cmd_vel_out.linear.x = self.filter.filter(0.0)
            self.pub.publish(cmd_vel_out)
            rate.sleep()
        self.filter.init()

    def cmd_cb(self, msg):
        self.cmd_vel.linear.x = msg.linear.x
        self.cmd_vel.angular.z = msg.angular.z
        self.cmd_time = rospy.Time.now()

    def pub_cmd(self):
        rate = rospy.Rate(20)
        cmd_vel_out = Twist()
        while not rospy.is_shutdown():
            self.pub_time = rospy.Time.now()
            if self.pub_time - self.cmd_time >= rospy.Duration(1):
                if not self.stop_flag:
                    self.stop_mv()
                continue
            else:
                self.stop_flag = False
                cmd_vel_out.angular.z = self.cmd_vel.angular.z
                cmd_vel_out.linear.x = self.filter.filter(self.cmd_vel.linear.x)
                # cmd_vel_out.linear.x = self.cmd_vel.linear.x
                self.pub.publish(cmd_vel_out)
            rate.sleep()


def freq_cmd():
    FreqCmd()


if __name__ == '__main__':
    freq_cmd()
