#!/usr/bin/env python
import rospy

import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('shenjun')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('/map_base', Odometry, queue_size=100)

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        # msg = TransformStamped()
        # trans = TransformStamped()
        #
        # msg  = trans.transform.translation
        # msg.rotation = trans.transform.rotation
        msg = Odometry()
        msg.header = trans.header
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = trans.transform.translation.x
        msg.pose.pose.position.y = trans.transform.translation.y
        msg.pose.pose.orientation.x = trans.transform.rotation.x
        msg.pose.pose.orientation.y = trans.transform.rotation.y
        msg.pose.pose.orientation.z = trans.transform.rotation.z
        msg.pose.pose.orientation.w = trans.transform.rotation.w

        pub.publish(msg)

        rate.sleep()
