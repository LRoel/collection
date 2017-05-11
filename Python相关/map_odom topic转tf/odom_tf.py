#!/usr/bin/env python

import rospy

import tf2_ros

from geometry_msgs.msg import TransformStamped


def do_it(tf_msg):
    print "ok"
    br = tf2_ros.TransformBroadcaster()
    t = tf_msg

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"

    br.sendTransform(t)


if __name__ == '__main__':
    try:
        rospy.init_node('combined_test')

        rospy.Subscriber('/map_base', TransformStamped, do_it)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
