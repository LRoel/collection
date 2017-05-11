#!/usr/bin/env python
#!/usr/bin/env python
import rospy
import thread

import math
import tf2_ros
import tf
import geometry_msgs.msg
import turtlesim.srv
import numpy as np
from tf.transformations import quaternion_from_matrix

from geometry_msgs.msg import PointStamped
from tf.transformations import quaternion_matrix
import time


class RobotLoc:
    def __init__(self):
        self.tl = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tl)
        self.br = tf2_ros.TransformBroadcaster()
        self.R_map_odom = np.eye(2)
        self.T_map_odom = np.zeros((2, 1))
        self.icp_fix()

    def where(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                trans = self.tl.lookup_transform('base_footprint', 'map', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            q = [0] * 4
            q[0] = trans.transform.rotation.x
            q[1] = trans.transform.rotation.y
            q[2] = trans.transform.rotation.z
            q[3] = trans.transform.rotation.w
            R = quaternion_matrix(q)[:2, :2]
            T = np.array([[trans.transform.translation.x], [trans.transform.translation.y]])
            return R, T

    def icp_fix(self):
        # print "fixed"
        fix_R = self.R_map_odom
        fix_T = self.T_map_odom
        t = geometry_msgs.msg.TransformStamped()
        q = [0] * 4
        if fix_R[1, 0] > fix_R[0, 1]:
            q[2] = 1 / 2.0 * math.sqrt(2 - fix_R[0, 0] - fix_R[1, 1])
        else:
            q[2] = -1 / 2.0 * math.sqrt(2 - fix_R[0, 0] - fix_R[1, 1])
        q[3] = 1 / 2.0 * math.sqrt(2 + fix_R[0, 0] + fix_R[1, 1])
        q_sum = math.sqrt(q[2] ** 2 + q[3] ** 2)
        q[2] /= q_sum
        q[3] /= q_sum

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "odom"
        t.transform.translation.x = fix_T[0, 0]
        t.transform.translation.y = fix_T[1, 0]
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # print "pub ok!!"
        self.br.sendTransform(t)


def icp_map_process():
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        robot_loc.icp_fix()
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('tf_test')
    global robot_loc
    robot_loc = RobotLoc()
    try:
        thread.start_new_thread(icp_map_process, ())
    except:
        print "Error: unable to start thread"

    print robot_loc.where()
    print robot_loc.T_map_odom
    robot_loc.R_map_odom = np.eye(2)
    robot_loc.T_map_odom = np.array([[1], [1]])
    robot_loc.icp_fix()
    print robot_loc.where()
    print robot_loc.T_map_odom
    print robot_loc.where()
    print robot_loc.where()
    rospy.spin()
