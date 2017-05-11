#!/usr/bin/env python
import rospy
import math

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def odom_callback(odommsg):

    orientation = [odommsg.pose.pose.orientation.x, odommsg.pose.pose.orientation.y, odommsg.pose.pose.orientation.z,
                   odommsg.pose.pose.orientation.w]
    (odom_roll, odom_pitch, odom_yaw) = euler_from_quaternion(orientation)

    odom_yaw_angle = odom_yaw * 180 / math.pi

    print ('yaw_angle = {0:.10f}  :  yaw_radius = {1:.10f}' .format(odom_yaw_angle, odom_yaw))



def get_yaw():
    # print "nav filter"
    global tf_listener, pub

    rospy.init_node('nav_filter_node', anonymous=True)

    rospy.Subscriber('/odom', Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    get_yaw()