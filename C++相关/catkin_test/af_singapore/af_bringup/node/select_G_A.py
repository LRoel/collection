#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import threading
import numpy as np
import numpy.linalg as la
import math
import time
import tf2_ros

from std_srvs.srv import Empty
from af_msgs.srv import SetPose

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan
from af_msgs.msg import Gnss_Odom_lyz
from af_msgs.msg import Fuse_tf
from af_msgs.msg import SetPose_resp
# from dynamic_reconfigure.server import Server
# from af_bringup.cfg import fuseConfig

import tf.transformations as tft

### 最大的标准差
MAX_STD = 999999999
amcl_initial_pub_flag = True
lock = threading.Lock()


def normalize(z):
    """归一化

    Args:
        z: 角度输入

    Returns:
        角度输出(0~pi)

    """

    return math.atan2(math.sin(z), math.cos(z))


def angle_diff(a, b):
    """角度差值

    Args:
        a: 输入角度
        b: 输入角度

    Returns:
        角度差值

    """

    a = normalize(a)
    b = normalize(b)
    d1 = a - b
    d2 = 2 * math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2


def matrix_from_theta(theta):
    """角度生成旋转矩阵(2x2)

    Args:
        theta: 输入角度

    Returns:
        输出旋转矩阵

    """

    return np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])


def theta_from_matrix(R):
    """旋转矩阵(2x2)转成角度

    Args:
        R: 输入旋转矩阵

    Returns:
        输出角度值

    """

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


class AMCL_unit:
    """AMCL存储类

    Attributes:
        x:  x
        x_std:  x标准差
        y:  y
        y_std:  y标准差
        theta:  角度
        theta_std:  角度标准差
        _theta_base:    累计角度
        _x_base:    累计x
        _y_base:    累计y

s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    """

    def __init__(self):
        """构造函数"""
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD
        self._theta_base = 0
        self._x_base = 0
        self._y_base = 0

    def update(self):
        """里程计更新

        Args:
            odom_msg:   ros里程计输入

        See Also:
            _pos:   位置矩阵
            _pose_base: 位置基准矩阵
            _pose_update:   坐标系变换
            _quat:  四元数

        """

        _pos = np.array([[odom_pose.x], [odom_pose.y]])
        _pos_base = np.array([[self._x_base], [self._y_base]], dtype=float)
        _theta = odom_pose.theta
        _pos_updated = np.dot(matrix_from_theta(self._theta_base), _pos) + _pos_base
        self.x = _pos_updated[0, 0]
        self.y = _pos_updated[1, 0]
        self.theta = _theta + self._theta_base

    def fix(self, odom_map_msg):
        """AMCL修正输入

        将AMCL修正输入的map->odom和里程计输入融合生成实时的AMCL输出值(机器人在地图上的激光定位)

        Args:
            odom_map_msg: AMCL的输入的map->odom

        """

        # print "fixed"
        _quat = [odom_map_msg.pose.pose.orientation.x, odom_map_msg.pose.pose.orientation.y,
                 odom_map_msg.pose.pose.orientation.z,
                 odom_map_msg.pose.pose.orientation.w]
        self._theta_base = tft.euler_from_quaternion(_quat)[2]
        self._x_base = odom_map_msg.pose.pose.position.x
        self._y_base = odom_map_msg.pose.pose.position.y
        self.x_std = math.sqrt(odom_map_msg.pose.covariance[0])
        self.y_std = math.sqrt(odom_map_msg.pose.covariance[7])
        self.theta_std = math.sqrt(odom_map_msg.pose.covariance[35])

    def lock(self):
        """将AMCL输出结果置为不可信"""
        self.x_std = MAX_STD
        self.y_std = MAX_STD
        self.theta_std = MAX_STD

    def output(self):
        """输出"""
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


class Odom_unit:
    """里程计数据存储类

    Attributes:
        x:  x
        x_std:  x标准差
        y:  y
        y_std:  y标准差
        theta:  角度
        theta_std:  角度标准差

    """

    def __init__(self):
        """构造函数"""
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD
        self.flag_odom_recovery = 0
        ### 机器人是否运动标志
        self.flag_robot_move = 0
        ### 判断机器人是否运动的速度限制
        self.STATIC_LIMIT_LINEAR = 0.05
        ### 判断机器人是否运动的角度限制
        self.STATIC_LIMIT_ANGULAR = 0.05

    def update(self, odom_msg):
        """里程计更新

        Args:
            odom_msg:   ros里程计输入

        See Also:
            _pos:   位置矩阵
            _pose_base: 位置基准矩阵
            _pose_update:   坐标系变换
            _quat:  四元数
            flag_robot_move:    机器人运动标志

        """

        _quat = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z,
                 odom_msg.pose.pose.orientation.w]
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        """
        增加按照一定比例
        """
        if self.x_std < MAX_STD:
            self.x_std += math.fabs(0.01 * odom_msg.twist.twist.linear.x) * 0.5
        if self.y_std < MAX_STD:
            self.y_std += math.fabs(0.01 * odom_msg.twist.twist.linear.x) * 0.5
        self.theta = tft.euler_from_quaternion(_quat)[2]

        if self.odom_recovery_judge() and not self.flag_odom_recovery:
            # self.flag_robot_move = 0
            self.odom_recovery()

        if odom_msg.twist.twist.linear.x ** 2 + odom_msg.twist.twist.linear.y ** 2 <= self.STATIC_LIMIT_LINEAR ** 2 \
                and math.fabs(odom_msg.twist.twist.angular.z) <= self.STATIC_LIMIT_ANGULAR:
            self.flag_robot_move = 0
        else:
            self.flag_robot_move = 1
            # """
            # 动就增加,或者增加按照一定比例
            # """
            # if self.x_std < MAX_STD:
            #     self.x_std += 0.01
            # if self.y_std < MAX_STD:
            #     self.y_std += 0.01
            # print "updated"
            # self.output()
        out_pose.set_odom(odom_pose.x, odom_pose.y, odom_pose.theta)

        amcl_pose.update()

    def odom_recovery_judge(self):
        # print 'flag_odom_recovery', self.flag_odom_recovery, 'x_std', self.x_std, out_pose.x_std
        if self.x_std >= 3 or self.y_std >= 3:
            return True
        else:
            return False

    def odom_recovery(self):
        self.flag_odom_recovery = 1
        print "Odom is in recovery state, waiting for GPS and AMCL good pos"

    def output(self):
        """输出"""
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


class GPS_unit:
    """GPS数据存储类

    Attributes:
        x:  x
        x_std:  x标准差
        y:  y
        y_std:  y标准差
        theta:  角度
        theta_std:  角度标准差

    """

    def __init__(self):
        """构造函数"""
        self.x = 0
        self.x_std = MAX_STD
        self.y = 0
        self.y_std = MAX_STD
        self.theta = 0
        self.theta_std = MAX_STD
        self.a = 1
        self.b = 0

    # def fuse_callback(self, config, level):
    #     rospy.loginfo("""Reconfigure Request: {a}, {b},""".format(**config))
    #     self.a = config.a
    #     self.b = config.b
    #     return config

    def update(self, gps_msg):
        """GPS数据更新函数

        Args:
            gps_msg: GPS数据输入

        """
        self.x = gps_msg.X
        self.x_std = self.a * gps_msg.X_std + self.b
        self.y = gps_msg.Y
        self.y_std = self.a * gps_msg.Y_std + self.b
        self.theta = gps_msg.Hdg
        self.theta_std = self.a * gps_msg.Hdg_std + self.b

    def lock(self):
        """将GPS输出结果置为不可信"""
        self.x_std = MAX_STD
        self.y_std = MAX_STD
        self.theta_std = MAX_STD

    def output(self):
        """输出"""
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


class SELECTED:
    """筛选结果类

    Attributes:
        x:  x
        x_std:  x标准差
        y:  y
        y_std:  y标准差
        theta:  角度
        theta_std:  角度标准差
        choice:
            {0: "GPS", 1: "AMCL", 2: "NONE"}
        fuse_tf:    ros输出格式

    """

    def __init__(self):
        """构造函数"""
        self.choice = {0: "GPS", 1: "AMCL", 2: "NONE"}
        self.x = 0
        self.x_std = MAX_STD
        self.x_choice = self.choice[2]
        self.y = 0
        self.y_std = MAX_STD
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
        """将选则的最终结果数据放在输出中"""
        self.fuse_tf.map_x = x
        self.fuse_tf.map_y = y
        self.fuse_tf.map_theta = theta

    def set_odom(self, x, y, theta):
        """将里程计数据放在输出中"""
        self.fuse_tf.odom_x = x
        self.fuse_tf.odom_y = y
        self.fuse_tf.odom_theta = theta

    def output(self):
        """输出"""
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_choice: ", self.x_choice, "| y_choice: ", self.y_choice, "| theta_choice: ", self.theta_choice


class MyTrnasform:
    def __init__(self):
        self.br = tf2_ros.TransformBroadcaster()
        self.mat44_map_odom = tft.identity_matrix()
        t_ = threading.Thread(target=self.tf_publish)
        t_.start()

    def sendTransform(self, out_pose):
        """发送TF(topic)

        Args:
            out_pose: ros输出格式数据

        """

        pub.publish(out_pose.fuse_tf)
        mat44_map_laser = tft.euler_matrix(0, 0, out_pose.fuse_tf.map_theta)
        mat44_map_laser[0][3] = out_pose.fuse_tf.map_x
        mat44_map_laser[1][3] = out_pose.fuse_tf.map_y
        mat44_laser_map = la.inv(mat44_map_laser)
        mat44_odom_laser = tft.euler_matrix(0, 0, out_pose.fuse_tf.odom_theta)
        mat44_odom_laser[0][3] = out_pose.fuse_tf.odom_x
        mat44_odom_laser[1][3] = out_pose.fuse_tf.odom_y
        self.mat44_map_odom = la.inv(np.dot(mat44_odom_laser, mat44_laser_map))

    def tf_publish(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map"
            t.child_frame_id = "odom"
            t.transform.translation.x = self.mat44_map_odom[0][3]
            t.transform.translation.y = self.mat44_map_odom[1][3]
            t.transform.translation.z = 0.0
            q = tft.quaternion_from_matrix(self.mat44_map_odom)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.br.sendTransform(t)
            rate.sleep()


def tran_mat44(trans):
    """将ros格式中的Transform数据转为矩阵(4X4)

    Args:
        trans: ros格式Transform数据

    """

    trans_xyz = [trans.transform.translation.x, trans.transform.translation.y, 0]
    trans_quat = [0, 0, trans.transform.rotation.z,
                  trans.transform.rotation.w]
    mat44 = np.dot(tft.translation_matrix(trans_xyz), tft.quaternion_matrix(trans_quat))
    return mat44


def tran_theta_T(trans):
    """将ros格式中的Transform数据转为矩阵(2X2)

    Args:
        trans: ros格式Transform数据

    """

    _quat = [trans.transform.rotation.x, trans.transform.rotation.y,
             trans.transform.rotation.z,
             trans.transform.rotation.w]
    _theta = tft.euler_from_quaternion(_quat)[2]
    _T = np.array([[trans.transform.translation.x], [trans.transform.translation.y]])
    return _theta, _T


def judge(a, b):
    if a < b:
        return True
    else:
        return False


def max_judge(a, b):
    if a == MAX_STD and b == MAX_STD:
        return True
    else:
        return False


def make_choice():
    """筛选函数"""
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if max_judge(amcl_pose.x_std, gps_pose.x_std) or max_judge(amcl_pose.y_std, gps_pose.y_std) or max_judge(
                amcl_pose.theta_std, gps_pose.theta_std):
            continue
        else:
            if (amcl_pose.x_std > 0.5 and gps_pose.x_std > 0.5) or (amcl_pose.y_std > 0.5 and gps_pose.y_std > 0.5):
                continue
            else:
                if judge(amcl_pose.x_std, gps_pose.x_std):
                    out_pose.x = amcl_output.x
                    out_pose.x_std = amcl_pose.x_std
                else:
                    out_pose.x = gps_pose.x
                    out_pose.x_std = gps_pose.x_std

                if judge(amcl_pose.y_std, gps_pose.y_std):
                    out_pose.y = amcl_output.y
                    out_pose.y_std = amcl_pose.y_std
                else:
                    out_pose.y = gps_pose.y
                    out_pose.y_std = gps_pose.y_std

                out_pose.theta = amcl_output.theta

                out_pose.set_map(out_pose.x, out_pose.y, out_pose.theta)
                odom_pose.x_std = 0
                odom_pose.y_std = 0
                # print "!!!", amcl_pose.x_std, gps_pose.x_std
                odom_pose.flag_odom_recovery = 0
                # sendRVIZ()
                my_tf.sendTransform(out_pose)
        # amcl_pose.lock()
        # gps_pose.lock()

        rate.sleep()


class Job_AMCL(threading.Thread):
    def __init__(self, *args, **kwargs):
        super(Job_AMCL, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()  # 用于暂停线程的标识
        self.__flag.set()  # 设置为True
        self.__running = threading.Event()  # 用于停止线程的标识
        self.__running.set()  # 将running设置为True
        self._android_flag = True

    def run(self):
        self.amcl_process()

    def pause(self):
        self.__flag.clear()  # 设置为False, 让线程阻塞

    def resume(self):
        self.__flag.set()  # 设置为True, 让线程停止阻塞

    def stop(self):
        self.__flag.set()  # 将线程从暂停状态恢复, 如何已经暂停的话
        self.__running.clear()  # 设置为False

    def android_enable(self, msg):
        global odom_pose
        self._android_flag = msg.data
        if self._android_flag:
            odom_pose.x_std = 0
            odom_pose.y_std = 0
            odom_pose.flag_odom_recovery = 0
            print "/android_enable is True"
            self.stop()
        else:
            print "/android_enable is False"
            self.run()

    def amcl_initial_update(self, x_std=1, y_std=1, theta_std=1):
        """ AMCL初始化函数

        Args:
            x_std: x方向方差
            y_std: y方向方差
            theta_std: 角度方差

        """

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue

            global pub_amcl
            amcl_init_pose.header.stamp = rospy.Time.now()
            amcl_init_pose.header.frame_id = "map"
            amcl_init_pose.pose.pose.position.x = trans.transform.translation.x
            amcl_init_pose.pose.pose.position.y = trans.transform.translation.y
            amcl_init_pose.pose.pose.position.z = trans.transform.translation.z
            amcl_init_pose.pose.pose.orientation.x = trans.transform.rotation.x
            amcl_init_pose.pose.pose.orientation.y = trans.transform.rotation.y
            amcl_init_pose.pose.pose.orientation.z = trans.transform.rotation.z
            amcl_init_pose.pose.pose.orientation.w = trans.transform.rotation.w
            amcl_init_pose.pose.covariance = [0.0] * 36
            amcl_init_pose.pose.covariance[0] = x_std ** 2
            amcl_init_pose.pose.covariance[7] = y_std ** 2
            amcl_init_pose.pose.covariance[35] = theta_std ** 2
            if amcl_initial_pub_flag:
                pub_amcl.publish(amcl_init_pose)
            break
            # print "pub amcl ok" + str(input_yaw)

    def amcl_initial_callback(self, msg):
        """AMCL 初始化回调函数

        Args:
            msg: ros回调数据

        """

        rospy.wait_for_service('initialpose')

        try:
            set_initial_pos = rospy.ServiceProxy('initialpose', SetPose)
            resp1 = set_initial_pos(msg)
            print resp1.responce.success
            print resp1.responce.covariance
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def amcl_set_pose(self, req):
        """
        安卓控制的位置纠正服务服务端
        :param req: initialpose中Header和方差均完备的PoseStamp格式输入
        :return: resp:  在5s内的定位情况,定位方差
        """

        global amcl_output
        amcl_output.x = req.initial_pose.pose.pose.position.x
        amcl_output.y = req.initial_pose.pose.pose.position.y
        _quat = [req.initial_pose.pose.pose.orientation.x, req.initial_pose.pose.pose.orientation.y,
                 req.initial_pose.pose.pose.orientation.z,
                 req.initial_pose.pose.pose.orientation.w]
        amcl_output.theta = tft.euler_from_quaternion(_quat)[2]
        out_pose.set_map(amcl_output.x, amcl_output.y, amcl_output.theta)
        my_tf.sendTransform(out_pose)
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.stamp = rospy.Time.now()
        init_pose.header.frame_id = "map"
        init_pose.pose.pose.position.x = req.initial_pose.pose.pose.position.x
        init_pose.pose.pose.position.y = req.initial_pose.pose.pose.position.y
        init_pose.pose.pose.position.z = req.initial_pose.pose.pose.position.z
        init_pose.pose.pose.orientation.x = req.initial_pose.pose.pose.orientation.x
        init_pose.pose.pose.orientation.y = req.initial_pose.pose.pose.orientation.y
        init_pose.pose.pose.orientation.z = req.initial_pose.pose.pose.orientation.z
        init_pose.pose.pose.orientation.w = req.initial_pose.pose.pose.orientation.w
        init_pose.pose.covariance = [0.0] * 36
        init_pose.pose.covariance[0] = 0.4
        init_pose.pose.covariance[7] = 0.4
        init_pose.pose.covariance[35] = 0.07
        pub_amcl.publish(init_pose)
        t = 0
        resp = SetPose_resp()
        self.amcl_nomotion_update()
        time.sleep(0.25)
        self.amcl_nomotion_update()
        time.sleep(0.25)
        while t < 10 and not rospy.is_shutdown():
            self.amcl_nomotion_update()
            time.sleep(0.25)
            if amcl_pose.x_std > 0.5 or amcl_pose.y_std > 0.5:
                t += 1
            else:
                amcl_output.x = amcl_pose.x
                amcl_output.y = amcl_pose.y
                amcl_output.theta = amcl_pose.theta
                out_pose.set_map(amcl_output.x, amcl_output.y, amcl_output.theta)
                my_tf.sendTransform(out_pose)
                resp.success = True
                resp.covariance = [amcl_pose.x_std, amcl_pose.y_std, amcl_pose.theta_std]
                return resp
        resp.success = False
        resp.covariance = [amcl_pose.x_std, amcl_pose.y_std, amcl_pose.theta_std]
        return resp

    def amcl_global_update(self):
        """AMCL全局撒粒子"""
        rospy.wait_for_service('global_localization')
        try:
            global_update = rospy.ServiceProxy('global_localization', Empty)
            global_update()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def amcl_nomotion_update(self):
        """AMCL未运动状态下粒子更新"""
        rospy.wait_for_service('request_nomotion_update')
        try:
            nomotion_update = rospy.ServiceProxy('request_nomotion_update', Empty)
            nomotion_update()
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def amcl_check(self):
        if amcl_pose.x_std <= 0.5 and amcl_pose.y_std <= 0.5:
            return True
        else:
            return False

    def amcl_recovery(self):
        """AMCL恢复状态过程"""
        global amcl_pose
        if odom_pose.flag_robot_move:
            t = 0
            while t < 15 and self.__running.isSet():
                self.__flag.wait()
                time.sleep(1)
                # print amcl_pose.x_std, amcl_pose.y_std
                if amcl_pose.x_std > 0.5 or amcl_pose.y_std > 0.5:
                    t += 1
                else:
                    return True
            return False
        else:
            t = 0
            while t < 15 and self.__running.isSet():
                self.__flag.wait()
                self.amcl_nomotion_update()
                time.sleep(0.25)
                self.amcl_nomotion_update()
                time.sleep(0.25)
                self.amcl_nomotion_update()
                time.sleep(0.25)
                self.amcl_nomotion_update()
                time.sleep(0.25)
                self.amcl_nomotion_update()
                # print amcl_pose.x_std, amcl_pose.y_std
                if amcl_pose.x_std > 0.5 or amcl_pose.y_std > 0.5:
                    t += 1
                else:
                    return True
            return False

    def amcl_process(self):
        """AMCL线程

        包括AMCL位置纠正,AMCL位置正确度判断和AMCL自恢复

        """

        global amcl_output
        while not rospy.is_shutdown() and self.__running.isSet():
            self.__flag.wait()  # 为True时立即返回, 为False时阻塞直到内部的标识位为True后返回
            if amcl_pose.x_std > 0.5 or amcl_pose.y_std > 0.5:
                rospy.wait_for_message('/scan', LaserScan)
                n = 0
                while not rospy.is_shutdown and self.__running.isSet()():
                    self.__flag.wait()
                    if n < 7:
                        self.amcl_initial_update()
                        print "amcl_initial"
                    elif n % 7 == 0:
                        self.amcl_global_update()
                        print "amcl_global"
                    n += 1
                    if self.amcl_recovery():
                        print "recovery ok"
                        amcl_output.x = amcl_pose.x
                        amcl_output.y = amcl_pose.y
                        amcl_output.theta = amcl_pose.theta
                        break
                        # print '!!!!!!!!!!!!!!!!!!!!'
            else:
                amcl_output.x = amcl_pose.x
                amcl_output.y = amcl_pose.y
                amcl_output.theta = amcl_pose.theta
                # print amcl_pose.x_std, amcl_pose.y_std
                # print "amcl_ok"


def monitor():
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_message('/scan', LaserScan, 1)
        except rospy.exceptions.ROSException:
            amcl_pose.lock()

    while not rospy.is_shutdown():
        try:
            rospy.wait_for_message('/Gnss_Odom_res', Gnss_Odom_lyz, 1)
        except rospy.exceptions.ROSException:
            gps_pose.lock()


if __name__ == '__main__':
    time.sleep(1)
    rospy.init_node('combined_test')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    amcl_pose = AMCL_unit()
    amcl_output = AMCL_unit()
    amcl_init_pose = PoseWithCovarianceStamped()
    gps_pose = GPS_unit()
    odom_pose = Odom_unit()
    out_pose = SELECTED()
    my_tf = MyTrnasform()

    t2 = Job_AMCL()

    rospy.Subscriber('/fuse/map_odom', PoseWithCovarianceStamped, amcl_pose.fix)
    rospy.Subscriber('/Gnss_Odom_res', Gnss_Odom_lyz, gps_pose.update)
    rospy.Subscriber('/odom', Odometry, odom_pose.update)
    rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, t2.amcl_initial_callback)
    s = rospy.Service('/initialpose', SetPose, t2.amcl_set_pose)
    # srv = Server(fuseConfig, gps_pose.fuse_callback)
    rospy.Subscriber('/android_enable', Bool, t2.android_enable)
    pub = rospy.Publisher('/fuse/tf', Fuse_tf, queue_size=100)
    pub_amcl = rospy.Publisher('/auto_initialpose', PoseWithCovarianceStamped, queue_size=1)

    t1 = threading.Thread(target=make_choice)
    t3 = threading.Thread(target=monitor)
    t1.start()
    t3.start()

    rospy.spin()
