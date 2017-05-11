#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import random
import numpy as np
import thread
import time

import tf2_ros
import tf
import geometry_msgs.msg
import numpy as np

from numpy import linalg as la

from tf.transformations import quaternion_from_matrix
from tf.transformations import quaternion_matrix
from tf.transformations import euler_from_matrix
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PointStamped
import pylab as pl

# import wx
# from matplotlib.figure import Figure
# from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas

DEBUG_MODE = "map"

step = 3
# TIMER_ID = wx.NewId()


class KD_node:

    def __init__(self, point=None, split=None, LL = None, RR = None):
        """
        point:数据点
        split:划分域
        LL, RR:节点的左儿子跟右儿子
        """
        self.point = point
        self.split = split
        self.left = LL
        self.right = RR


def createKDTree(root, data_list):
    """
    root:当前树的根节点
    data_list:数据点的集合(无序)
    return:构造的KDTree的树根
    """
    LEN = len(data_list)
    if LEN == 0:
        return
        # 数据点的维度
    dimension = len(data_list[0])
    # 方差
    max_var = 0
    # 最后选择的划分域
    split = 0
    for i in range(dimension):
        ll = []
        for t in data_list:
            ll.append(t[i])
        var = computeVariance(ll)
        if var > max_var:
            max_var = var
            split = i
            # 根据划分域的数据对数据点进行排序
    data_list.sort(key=lambda x: x[split])
    # 选择下标为len / 2的点作为分割点
    point = data_list[LEN / 2]
    root = KD_node(point, split)
    root.left = createKDTree(root.left, data_list[0:(LEN / 2)])
    root.right = createKDTree(root.right, data_list[(LEN / 2 + 1):LEN])
    return root


def computeVariance(arrayList):
    """
    arrayList:存放的数据点
    return:返回数据点的方差
    """
    for ele in arrayList:
        ele = float(ele)
    LEN = len(arrayList)
    array = np.array(arrayList)
    sum1 = array.sum()
    array2 = array * array
    sum2 = array2.sum()
    mean = sum1 / LEN
    # D[X] = E[x^2] - (E[x])^2
    variance = sum2 / LEN - mean ** 2
    return variance


def findNN(root, query):
    """
    root:KDTree的树根
    query:查询点
    return:返回距离data最近的点NN，同时返回最短距离min_dist
    """
    # 初始化为root的节点
    NN = root.point
    min_dist = computeDist(query, NN)
    nodeList = []
    temp_root = root
    ##二分查找建立路径
    while temp_root:
        nodeList.append(temp_root)
        dd = computeDist(query, temp_root.point)
        if min_dist > dd:
            NN = temp_root.point
            min_dist = dd
            # 当前节点的划分域
        ss = temp_root.split
        if query[ss] <= temp_root.point[ss]:
            temp_root = temp_root.left
        else:
            temp_root = temp_root.right
            ##回溯查找
    while nodeList:
        # 使用list模拟栈，后进先出
        back_point = nodeList.pop()
        ss = back_point.split
        # print "back.point = ", back_point.point
        ##判断是否需要进入父亲节点的子空间进行搜索
        if abs(query[ss] - back_point.point[ss]) < min_dist:
            if query[ss] <= back_point.point[ss]:
                temp_root = back_point.right
            else:
                temp_root = back_point.left

            if temp_root:
                nodeList.append(temp_root)
                curDist = computeDist(query, temp_root.point)
                if min_dist > curDist:
                    min_dist = curDist
                    NN = temp_root.point
    return NN, min_dist


def computeDist(pt1, pt2):
    """
    计算两个数据点的距离
    return:pt1和pt2之间的距离
    """
    sum = 0.0
    for i in range(len(pt1)):
        sum = sum + (pt1[i] - pt2[i]) * (pt1[i] - pt2[i])
    return math.sqrt(sum)


class ICPUnit:
    def __init__(self):
        self.laser_point = np.zeros((2, 1))
        self.map_point = np.zeros((2, 1))

    def ros_init(self):
        self.stamp = rospy.Time.now()
        self.seq = 0

    def fill_in(self, _laser, _map):
        self.laser_point = _laser
        self.map_point = _map
        self.stamp = rospy.Time.now()
        self.seq += 1

    def pick_up(self):
        return self.laser_point, self.map_point


class RobotLoc:
    def __init__(self):
        self.R_map_odom = np.eye(2)
        self.T_map_odom = np.zeros((2, 1))
        # self.T_map_odom = np.array([[-3.0], [-3.0]])

    def set_R_T(self, R, T):
        # self.R_map_odom = np.dot(self.R_map_odom, R)
        # self.T_map_odom += T
        # _R = np.eye(4)
        # _R[:2, :2] = R
        # ax, ay, az = euler_from_matrix(_R)
        # jump_theta = np.abs(az)
        # if jump_theta > 15 * math.pi / 180:
        #     R = np.eye(2)
        # jump_T = np.sqrt(T[0, 0]**2 and T[1, 0]**2)
        # if jump_T > 0.3:
        #     T = np.zeros((2, 1))
        print T[0, 0], T[1, 0]
        if DEBUG_MODE == 'odom':
            self.R_map_odom = R
            self.T_map_odom = T
        elif DEBUG_MODE == 'map':
            self.R_map_odom = np.dot(self.R_map_odom, R)
            self.T_map_odom += T

    def ros_init(self):
        self.tl = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tl)
        self.br = tf2_ros.TransformBroadcaster()
        self.icp_fix()

    def map(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                trans = self.tl.lookup_transform('map', 'base_footprint', rospy.Time())
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

    def odom(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                trans = self.tl.lookup_transform('odom', 'base_footprint', rospy.Time())
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
        _R = np.eye(4)
        _R[:2, :2] = fix_R
        q = quaternion_from_matrix(_R)
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


class MapData:

    def __init__(self, info, data):
        self.map_load_time = info.map_load_time
        self.resolution = info.resolution
        self.width = info.width
        self.height = info.height
        self.origin = info.origin
        self.grid_data = data
        tmp = np.asarray(data).reshape(self.height, self.width)
        tmp_y, tmp_x = np.where(tmp == 100)
        self.data = np.array([tmp_x, tmp_y]) * self.resolution
        alpha = np.linspace(-np.pi / 2, np.pi / 2, 10)
        x = 8 * np.cos(alpha)
        y = 8 * np.sin(alpha)
        _shape = np.array([x, y])
        np_zero = np.zeros((2, 1))
        _shape = np.c_[np_zero, _shape]
        # _shape = np.c_[_shape, np_zero]
        self.shape = _shape

    def get_map(self, R, T):
        T_fro = - np.array([[self.origin.position.x], [self.origin.position.y]])
        R_fro = la.inv(quaternion_matrix((self.origin.orientation.x, self.origin.orientation.y, self.origin.orientation.z,
                               self.origin.orientation.w))[0:2, 0:2])
        T_bak = np.array([[self.origin.position.x], [self.origin.position.y]])
        R_bak = quaternion_matrix((self.origin.orientation.x, self.origin.orientation.y, self.origin.orientation.z,
                               self.origin.orientation.w))[0:2, 0:2]
        T = T + T_fro
        R = np.dot(R, R_fro)
        _shape = np.dot(R, self.shape) + np.dot(T, np.ones((1, self.shape.shape[1])))
        _shape_max = np.floor(_shape.max(axis=1)) / 0.05 + 1 / 0.05
        _shape_min = np.floor(_shape.min(axis=1)) / 0.05 - 1 / 0.05
        # _shape_zero = np.floor(_shape[:, 0])
        tmp = np.asarray(self.grid_data).reshape(self.height, self.width)
        _shape_global_bool = np.zeros_like(tmp, dtype=bool)
        _shape_global_bool[int(_shape_min[1]):int(_shape_max[1]), int(_shape_min[0]):int(_shape_max[0])] = True
        _global_bool = np.logical_and(tmp == 100, _shape_global_bool)
        tmp_y, tmp_x = np.where(_global_bool)
        data = np.array([tmp_x, tmp_y]) * self.resolution
        # tmp_y2, tmp_x2 = np.where(tmp == 100)
        # data2 = np.array([tmp_x2, tmp_y2]) * self.resolution
        # tmp_y3, tmp_x3 = np.where(_shape_global_bool)
        # data3 = np.array([tmp_x3, tmp_y3]) * self.resolution
        data = np.dot(R_bak, data) + np.dot(T_bak, np.ones((1, data.shape[1])))
        # data2 = np.dot(R_bak, data2) + np.dot(T_bak, np.ones((1, data2.shape[1])))
        # data3 = np.dot(R_bak, data3) + np.dot(T_bak, np.ones((1, data3.shape[1])))
        return data

    def init_pose(self):
        T = np.array([[self.origin.position.x], [self.origin.position.y]])
        R = quaternion_matrix((self.origin.orientation.x, self.origin.orientation.y, self.origin.orientation.z,
                              self.origin.orientation.w))[0:2, 0:2]
        self.data = np.dot(R, self.data) + np.dot(T, np.ones((1, self.data.shape[1])))


class PointScan:

    def __init__(self, seq=0, point=np.array([0, 0])):
        self.seq = seq
        self.point = point

    def align(self, R, T):
        self.point = np.dot(R, self.point) + np.dot(T, np.ones((1, self.point.shape[1])))

robot_loc = RobotLoc()
scan_point = PointScan()
X = PointScan()
Y = PointScan()
icp_unit = ICPUnit()
handle_scan_flag = 0

# plt_data_a_x = []
# plt_data_a_y = []
# plt_data_b_x = []
# plt_data_b_y = []
# plt_data_c_x = []
# plt_data_c_y = []
# plt_data_d_x = []
# plt_data_d_y = []


# class PlotFigure(wx.Frame):
#     global plt_data_a_x, plt_data_a_y, plt_data_b_x, plt_data_b_y, plt_data_c_x, plt_data_c_y
#
#     def __init__(self):
#
#         wx.Frame.__init__(self, None, wx.ID_ANY, title="!!!", size=(1000, 1000))
#         self.fig = Figure((10, 10), 100)
#         self.canvas = FigureCanvas(self, wx.ID_ANY, self.fig)
#         self.ax = self.fig.add_subplot(111)
#         self.ax.set_ylim([-10, 10])
#         self.ax.set_xlim([-5, 15])
#         self.ax.set_autoscale_on(False)
#         self.ax.grid(True)
#         self.user, = self.ax.plot(plt_data_a_x, plt_data_a_y, 'o', label='map')
#         self.ax.legend(loc='upper center')
#         self.canvas.draw()
#         self.bg = self.canvas.copy_from_bbox(self.ax.bbox)
#         wx.EVT_TIMER(self, TIMER_ID, self.onTimer)
#
#     def onTimer(self, evt):
#         """callback function for timer events"""
#
#         self.canvas.restore_region(self.bg)
#         a, = self.ax.plot(plt_data_b_x, plt_data_b_y, 'or')
#         self.ax.draw_artist(a)
#         b, = self.ax.plot(plt_data_c_x, plt_data_c_y, 'og')
#         self.ax.draw_artist(b)
#         c, = self.ax.plot(plt_data_d_x, plt_data_d_y, 'ok')
#         self.ax.draw_artist(c)
#         self.canvas.blit(self.ax.bbox)


def scan2point(laser_msg):
    global step
    index = np.arange(laser_msg.angle_min,laser_msg.angle_max,laser_msg.angle_increment).tolist()
    point_data = np.empty((2, 1))
    for i in range(0, len(index), step):
        if laser_msg.range_min <= laser_msg.ranges[i] <= 7:
            point_data = np.insert(point_data, -1, np.array([laser_msg.ranges[i]*math.cos(index[i]), laser_msg.ranges[i]*math.sin(index[i])]), axis=1)
    point_data = np.delete(point_data, -1, axis=1)
    return point_data


def MAP_GXWX(_map, x):
    return int(np.floor((x - _map.origin.position.x) / _map.resolution + 0.5))


def MAP_GYWY(_map, y):
    return int(np.floor((y - _map.origin.position.y) / _map.resolution + 0.5))


def MAP_VALID(_map, i, j):
    return (i >= 0) and (i < _map.width) and (j >= 0) and (j < _map.height)


def MAP_INDEX(_map, i, j):
    return i + j * _map.width


def map_calc_range(_map, ox, oy, oa, max_range=8):
    # Bresenham raytracing

    x0 = MAP_GXWX(_map, ox)
    y0 = MAP_GYWY(_map, oy)

    x1 = MAP_GXWX(_map, ox + max_range * np.cos(oa))
    y1 = MAP_GYWY(_map, oy + max_range * np.sin(oa))

    if abs(y1 - y0) > abs(x1 - x0):
        steep = 1
    else:
        steep = 0

    if steep:
        tmp = x0
        x0 = y0
        y0 = tmp

        tmp = x1
        x1 = y1
        y1 = tmp

    deltax = abs(x1 - x0)
    deltay = abs(y1 - y0)
    error = 0
    deltaerr = deltay

    x = x0
    y = y0

    if x0 < x1:
        xstep = 1
    else:
        xstep = -1
    if y0 < y1:
        ystep = 1
    else:
        ystep = -1

    '''
    _map = MapData()
    -01 空白区域
    000 未知区域
    100 障碍区域
    '''
    if steep:
        if not MAP_VALID(_map, y, x) or _map.grid_data[MAP_INDEX(_map, y, x)] > 0:
            return np.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * _map.resolution
    else:
        if not MAP_VALID(_map, x, y) or _map.grid_data[MAP_INDEX(_map, x, y)] > 0:
            return np.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * _map.resolution

    while x != (x1 + xstep * 1):
        x += xstep
        error += deltaerr
        if 2 * error >= deltax:
            y += ystep
            error -= deltax

        if steep:
            if not MAP_VALID(_map, y, x) or _map.grid_data[MAP_INDEX(_map, y, x)] > 0:
                return np.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * _map.resolution
        else:
            if not MAP_VALID(_map, x, y) or _map.grid_data[MAP_INDEX(_map, x, y)] > 0:
                return np.sqrt((x - x0) * (x - x0) + (y - y0) * (y - y0)) * _map.resolution

    return max_range


'''

def handle_scan():
    print "ok!"
    global handle_scan_flag
    handle_scan_flag = 0
'''


def handle_scan(laser_msg):
    _R, _T = robot_loc.map()
    _R_ori, _T_ori = robot_loc.odom()

    # print _R, _T, _R_ori, _T_ori
    dynamic_map = static_map.get_map(_R_ori, _T_ori)

    scan_point = PointScan(laser_msg.header.seq, scan2point(laser_msg))
    if DEBUG_MODE == 'odom':
        scan_point.align(_R_ori, _T_ori)
    elif DEBUG_MODE == 'map':
        scan_point.align(_R, _T)


    # icp_unit.fill_in(scan_point.point, static_map.data)
    icp_unit.fill_in(scan_point.point, dynamic_map)
    global handle_scan_flag
    handle_scan_flag = 0


def scan_cb(laser_msg):
    robot_loc.icp_fix()
    global handle_scan_flag
    handle_scan_flag += 1

    if handle_scan_flag == 4:
        try:
            thread.start_new_thread(handle_scan, (laser_msg,))
            # print "OK: handle scan process"
        except:
            print "Error: unable to start handle scan process"


def static_map_get():
    rospy.wait_for_service('/static_map')
    try:
        get_map = rospy.ServiceProxy('/static_map', GetMap)
        map_data = get_map().map
        global static_map
        static_map = MapData(map_data.info, map_data.data)
        static_map.init_pose()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def icp(X, P):
    num_x = X.shape[1]
    num_p = P.shape[1]
    mean_x = X.mean(axis=1).reshape((2, 1))
    mean_p = P.mean(axis=1).reshape((2, 1))
    X_m = X - np.dot(mean_x, np.ones((1, num_x)))
    P_m = P - np.dot(mean_p, np.ones((1, num_p)))
    A = np.dot(X_m, P_m.transpose())
    U, tmp, V = la.svd(A)
    num_a = A.shape[1]
    S = np.identity(num_a)
    if la.det(A) < 0:
        S[-1, -1] = -1
    R = np.dot(np.dot(U, S), V.transpose())
    T = mean_x - np.dot(R, mean_p)
    return R, T


def closest_point_kd(kdtree, P, P_ori):
    num_p = P.shape[1]
    Y = np.empty(np.shape(P))
    dist_collect = []
    Y_collect = []
    P_collect = []
    P_ori_collect = []
    Y_out = []
    P_out = []
    for i in range(num_p):
        # index, min_dist = findNN(kdtree, np.array(P[i,:]).reshape(-1,).tolist())
        index, min_dist = findNN(kdtree, P[:, i].tolist())
        Y[:, i] = index
        dist_collect.append(min_dist)
        Y_collect.append(Y[:, i])
        P_collect.append(P[:, i])
        P_ori_collect.append(P_ori[:, i])
    disk_np = np.asarray(dist_collect)
    disk_std = np.std(disk_np)
    disk_aver = np.average(disk_np)
    ppp_dist = disk_std * 2.5
    for i in range(len(dist_collect)):
        if abs(dist_collect[i] - disk_aver) <= ppp_dist:
        # if 1:
            Y_out.append(Y_collect[i])
            P_out.append(P_ori_collect[i])
    return np.asarray(Y_out).transpose(), np.asarray(P_out).transpose()
    # return Y, P


def icp_map_process():
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        # print "123"
        P_p_scan, dynamic_map = icp_unit.pick_up()
        # global plt_data_d_x, plt_data_d_y
        # plt_data_d_x = P_p_scan[0, :]
        # plt_data_d_y = P_p_scan[1, :]
        if not len(P_p_scan):
            continue
        n = 0
        d_k_1 = 65535
        root = KD_node()
        kdtree = createKDTree(root, dynamic_map.transpose().tolist())

        # global plt_data_b_x, plt_data_b_y
        # plt_data_b_x = dynamic_map[0, :]
        # plt_data_b_y = dynamic_map[1, :]
        P_p_loop = P_p_scan
        while not rospy.is_shutdown():

            Y_p, P_p_ori = closest_point_kd(kdtree, P_p_loop, P_p_scan)

            try:
                R, T = icp(Y_p, P_p_ori)
            except:
                # print Y_p, P_p_o, P_p
                # print kdtree
                print "!!!!!!!!!"
                break
            num_p_o = P_p_ori.shape[1]
            P_out = np.dot(R, P_p_ori) + np.dot(T, np.ones((1, num_p_o)))
            num_p = P_p_scan.shape[1]
            P_p_loop = np.dot(R, P_p_scan) + np.dot(T, np.ones((1, num_p)))
            d_k = la.norm(Y_p - P_out) / num_p_o
            if abs(d_k_1 - d_k) <= 10 ** -6 or n >= 50:
                P_p_out = P_p_loop
                break

            n += 1
            d_k_1 = d_k

        # print n
        # if d_k_1 <= 0.10:
        #     robot_loc.R_map_odom = R
        #     robot_loc.T_map_odom = T
        # print R, T
        # print T[0, 0]
        # R_233 = np.array([[0.99998905,0.00468033],[-0.00468033,0.99998905]])
        # T_233 = np.array([[-0.36718446],[0.02488377]])
        if 1 <= n <= 49:
            robot_loc.set_R_T(R, T)
        # tf_pub(R, T)
        # global plt_data_c_x, plt_data_c_y
        # plt_data_c_x = P_p_out[0, :]
        # plt_data_c_y = P_p_out[1, :]
        rate.sleep()
        # time.sleep(2)


# def tf_publish():
#     rate = rospy.Rate(0.5)
#     while not rospy.is_shutdown():
#         # print "456"
#         robot_loc.icp_fix()
#         print robot_loc.T_map_odom
#
#         rate.sleep()


if __name__ == '__main__':
    rospy.init_node('icp_ros_node', anonymous=True)
    robot_loc.ros_init()
    icp_unit.ros_init()
    # rospy.Subscriber('/odom', Odometry, odom_cb)

    static_map_get()
    time.sleep(1)
    rospy.Subscriber('/scan', LaserScan, scan_cb)

    a = np.linspace(-1.5,1.5,730)
    x = []
    y = []
    for i in a:
        c = map_calc_range(static_map, 0, 0, i)
        x.append(c*np.cos(i))
        y.append(c*np.sin(i))

    pl.figure()

    pl.plot(static_map.data[0,:], static_map.data[1,:], 'o')
    pl.plot(x, y, "o")

    pl.show()

    # rospy.Subscriber('/odom', Odometry, robot_loc.set_r_t)


    # while icp_unit.seq == 0:
    #     time.sleep(1)
    #
    # try:
    #     thread.start_new_thread(icp_map_process, ())
    # except:
    #     print "Error: unable to start thread1"

    # try:
    #     thread.start_new_thread(tf_publish, ())
    # except:
    #     print "Error: unable to start thread2"

    # print robot_loc.where()

    rospy.spin()
    # app = wx.PySimpleApp()
    # frame = PlotFigure()
    # t = wx.Timer(frame, TIMER_ID)
    # t.Start(50)
    # frame.Show()
    # app.MainLoop()
