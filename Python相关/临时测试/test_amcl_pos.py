#!/usr/bin/env python

import rospy
import tf
import threading
import numpy as np
import numpy.linalg as la
import math
import datetime
import tf2_ros

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TransformStamped
from af_msgs.msg import Gnss_Odom_lyz
from af_msgs.msg import Fuse_tf
from visualization_msgs.msg import Marker
from af_bringup.msg import Robot_encode
from std_msgs.msg import Bool

from tf import transformations
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import tf.transformations as tft


MAX_STD = 999999999


def matrix_from_theta(theta):
    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


def theta_from_matrix(R):
    _R_12 = R[0, 1]
    _R_22 = R[1, 1]
    if -0.001 < _R_22 < 0.001 and _R_12 > 0:
        theta = math.pi / 2
    elif -0.001 < _R_22 < 0.001 and _R_12 < 0:
        theta = math.pi / 2
    elif 0.001 <= _R_22 and _R_12 > 0:
        theta = math.atan(_R_12 / _R_22)
    elif 0.001 <= _R_22 and _R_12 < 0:
        theta = math.atan(_R_12 / _R_22)
    elif _R_22 <= -0.001 and _R_12 > 0:
        theta = math.atan(_R_12 / _R_22) + math.pi
    elif _R_22 <= -0.001 and _R_12 < 0:
        theta = math.atan(_R_12 / _R_22) - math.pi
    return theta


def amcl_update(input_x, input_y, input_yaw):
    global pub_amcl
    amcl_init_pose.header.stamp = rospy.Time.now()
    amcl_init_pose.header.frame_id = "map"

    amcl_init_pose.pose.pose.position.x = input_x
    amcl_init_pose.pose.pose.position.y = input_y
    amcl_init_pose.pose.pose.position.z = 0

    q = quaternion_from_euler(0, 0, input_yaw)
    amcl_init_pose.pose.pose.orientation.x = q[0]
    amcl_init_pose.pose.pose.orientation.y = q[1]
    amcl_init_pose.pose.pose.orientation.z = q[2]
    amcl_init_pose.pose.pose.orientation.w = q[3]

    amcl_init_pose.pose.covariance = [0.0] * 36
    amcl_init_pose.pose.covariance[0] = 0.25
    amcl_init_pose.pose.covariance[7] = 0.25
    amcl_init_pose.pose.covariance[35] = 0.14

    pub_amcl.publish(amcl_init_pose)
    print "pub amcl ok" + str(input_yaw)


class AMCL_unit:
    def __init__(self):
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD
        self._theta_base = 0
        self._x_base = 0
        self._y_base = 0

    def update(self, odom_msg):
        _pos = np.array([[odom_msg.pose.pose.position.x], [odom_msg.pose.pose.position.y]])
        odom_pose.x = _pos[0, 0]
        odom_pose.y = _pos[1, 0]
        _pos_base = np.array([[self._x_base], [self._y_base]], dtype=float)
        _quat = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z,
                 odom_msg.pose.pose.orientation.w]
        _theta = tft.euler_from_quaternion(_quat)[2]
        _pos_updated = np.dot(matrix_from_theta(self._theta_base), _pos) + _pos_base
        odom_pose.theta = _theta
        self.x = _pos_updated[0, 0]
        self.y = _pos_updated[1, 0]
        self.theta = _theta + self._theta_base
        out_pose.set_odom(odom_pose.x, odom_pose.y, odom_pose.theta)
        # print "updated"
        # self.output()

    def fix(self, odom_map_msg):
        # print "fixed"
        _quat = [odom_map_msg.pose.pose.orientation.x, odom_map_msg.pose.pose.orientation.y, odom_map_msg.pose.pose.orientation.z,
                 odom_map_msg.pose.pose.orientation.w]
        self._theta_base = tft.euler_from_quaternion(_quat)[2]
        self._x_base = odom_map_msg.pose.pose.position.x
        self._y_base = odom_map_msg.pose.pose.position.y
        self.x_std = math.sqrt(odom_map_msg.pose.covariance[0])
        self.y_std = math.sqrt(odom_map_msg.pose.covariance[7])
        self.theta_std = math.sqrt(odom_map_msg.pose.covariance[35])

    def lock(self):
        self.x_std = MAX_STD
        self.y_std = MAX_STD
        self.theta_std = MAX_STD

    def output(self):
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


class Odom_unit:
    def __init__(self):
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD

    def output(self):
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


class GPS_unit:
    def __init__(self):
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD

    def update(self, gps_msg):
        self.x = gps_msg.X
        self.x_std = gps_msg.X_std
        self.y = gps_msg.Y
        self.y_std = gps_msg.Y_std
        self.theta = gps_msg.Hdg
        self.theta_std = gps_msg.Hdg_std

    def lock(self):
        self.x_std = MAX_STD
        self.y_std = MAX_STD
        self.theta_std = MAX_STD

    def output(self):
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


class SELECTED():
    def __init__(self):
        self.choice = {0: "GPS", 1: "AMCL", 2: "NONE"}
        self.x = 0
        self.x_choice = self.choice[2]
        self.y = 0
        self.y_choice = self.choice[2]
        self.theta = 0
        self.theta_choice = self.choice[2]
        self.fuse_tf = Fuse_tf()
        self.fuse_tf.map_x = 0
        self.fuse_tf.map_y = 0
        self.fuse_tf.map_theta = 0
        self.fuse_tf.odom_x = 0
        self.fuse_tf.odom_y = 0
        self.fuse_tf.odom_theta = 0

    def set_map(self, x, y, theta):
        self.fuse_tf.map_x = x
        self.fuse_tf.map_y = y
        self.fuse_tf.map_theta = theta

    def set_odom(self,x, y, theta):
        self.fuse_tf.odom_x = x
        self.fuse_tf.odom_y = y
        self.fuse_tf.odom_theta = theta

    def output(self):
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_choice: ", self.x_choice, "| y_choice: ", self.y_choice, "| theta_choice: ", self.theta_choice


def sendTransform(out_pose):
    pub.publish(out_pose.fuse_tf)


def sendRVIZ():
    _amcl_rviz = Marker()
    _gps_rviz = Marker()
    _amcl_rviz.header.frame_id = "map"
    _amcl_rviz.header.stamp = rospy.Time.now()
    _amcl_rviz.type = _amcl_rviz.ARROW
    _amcl_rviz.action = _amcl_rviz.ADD
    _amcl_rviz.ns = "my_namespace"
    _amcl_rviz.id = 0
    _amcl_rviz.pose.position.x = amcl_pose.x
    _amcl_rviz.pose.position.y = amcl_pose.y
    _amcl_rviz.pose.position.z = 0
    _amcl_quat = tft.quaternion_from_euler(0, 0, amcl_pose.theta)
    _amcl_rviz.pose.orientation.x = _amcl_quat[0]
    _amcl_rviz.pose.orientation.y = _amcl_quat[1]
    _amcl_rviz.pose.orientation.z = _amcl_quat[2]
    _amcl_rviz.pose.orientation.w = _amcl_quat[3]
    _gps_rviz.header.frame_id = "map"
    _gps_rviz.header.stamp = rospy.Time.now()
    _gps_rviz.type = _amcl_rviz.ARROW
    _gps_rviz.action = _amcl_rviz.ADD
    _gps_rviz.ns = "my_namespace"
    _gps_rviz.id = 1
    _gps_rviz.pose.position.x = gps_pose.x
    _gps_rviz.pose.position.y = gps_pose.y
    _gps_rviz.pose.position.z = 0
    _gps_quat = tft.quaternion_from_euler(0, 0, gps_pose.theta)
    _gps_rviz.pose.orientation.x = _gps_quat[0]
    _gps_rviz.pose.orientation.y = _gps_quat[1]
    _gps_rviz.pose.orientation.z = _gps_quat[2]
    _gps_rviz.pose.orientation.w = _gps_quat[3]
    pub_rviz_amcl.publish(_amcl_rviz)
    pub_rviz_gps.publish(_gps_rviz)


def tran_mat44(trans):
    trans_xyz = [trans.transform.translation.x, trans.transform.translation.y, 0]
    trans_quat = [0, 0, trans.transform.rotation.z,
                  trans.transform.rotation.w]
    mat44 = np.dot(tft.translation_matrix(trans_xyz), tft.quaternion_matrix(trans_quat))
    return mat44


def tran_theta_T(trans):
    _quat = [trans.transform.rotation.x, trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w]
    _theta = tft.euler_from_quaternion(_quat)[2]
    _T = np.array([[trans.transform.translation.x], [trans.transform.translation.y]])
    return _theta, _T


def normalization(a, b):
    sum = a + b
    return a/sum, b/sum


def make_choice():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        _std_x1 = amcl_pose.x_std
        _std_y1 = amcl_pose.y_std
        if _std_x1 < 0.5 and _std_y1 < 0.5:
            out_pose.x = amcl_pose.x
            out_pose.y = amcl_pose.y
            out_pose.theta = amcl_pose.theta
        else:
            continue
        out_pose.set_map(out_pose.x, out_pose.y, out_pose.theta)
        # sendRVIZ()
        sendTransform(out_pose)
        amcl_pose.lock()
        gps_pose.lock()

        rate.sleep()


if __name__ == '__main__':
    try:
        rospy.init_node('combined_test')
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        amcl_pose = AMCL_unit()
        amcl_init_pose = PoseWithCovarianceStamped()
        gps_pose = GPS_unit()
        odom_pose = Odom_unit()
        out_pose = SELECTED()
        rospy.Subscriber('/fuse/map_odom', PoseWithCovarianceStamped, amcl_pose.fix)
        rospy.Subscriber('/Gnss_Odom_res', Gnss_Odom_lyz, gps_pose.update)
        rospy.Subscriber('/odom', Odometry, amcl_pose.update)
        pub = rospy.Publisher('/fuse/tf', Fuse_tf, queue_size=100)
        pub_amcl_pos = rospy.Publisher('/fuse/amcl', PoseWithCovarianceStamped, queue_size=100)
        pub_amcl = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        pub_rviz_amcl = rospy.Publisher('/rviz/amcl', Marker, queue_size=100)
        pub_rviz_gps = rospy.Publisher('/rviz/gps', Marker, queue_size=100)
        # while rospy.is_shutdown():
        #     amcl_pose.output()
        #     gps_pose.output()

        t1 = threading.Thread(target=make_choice)
        t1.start()
        t1.join()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
