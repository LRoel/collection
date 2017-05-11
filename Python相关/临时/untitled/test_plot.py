#!/usr/bin/env python
import rospy
import math
import matplotlib.pyplot as plt

from nav_msgs.msg import Odometry

a_x = []
a_y = []
a_a = []
a_w = []
a_t = []

show_flag = 1

def odom_callback(odommsg):

    global show_flag
    a_t.append(show_flag)
    a_x.append(odommsg.pose.pose.position.x)
    #print a_x
    a_y.append(odommsg.pose.pose.position.y)
    a_a.append(odommsg.twist.twist.linear.x)
    a_w.append(odommsg.twist.twist.angular.z)
    if show_flag % 10400 == 0:
        plt.figure(0)
        plt.scatter(a_x, a_y)
        plt.figure(1)
        plt.plot(a_a)
        plt.figure(2)
        plt.plot(a_w)
        plt.show()
    print show_flag
    show_flag = show_flag + 1

def get_yaw():
    # print "nav filter"
    global tf_listener, pub

    rospy.init_node('nav_filter_node', anonymous=True)

    rospy.Subscriber('/odom', Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    get_yaw()