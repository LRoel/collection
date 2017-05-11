#!/usr/bin/env python
import rospy

import tf2_ros
from geometry_msgs.msg import TransformStamped

if __name__ == '__main__':
    rospy.init_node('amcl_pun')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    pub = rospy.Publisher('/map_base', TransformStamped, queue_size=100)

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
        pub.publish(trans)

        rate.sleep()
