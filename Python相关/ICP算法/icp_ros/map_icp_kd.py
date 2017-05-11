#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
import math
import numpy as np
import thread
import time

import wx
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas

from tf.transformations import quaternion_matrix
from tf import TransformListener

from numpy import linalg as la
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap

step = 3

TIMER_ID = wx.NewId()


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


class RobotLoc:
    def __init__(self):
        self.R = np.eye(2)
        self.T = np.zeros((2, 1))
        self.fro_R = np.eye(2)
        self.fro_T = np.zeros((2, 1))

    def set_r(self, q):
        self.R = np.dot(self.fro_R, quaternion_matrix((q.x, q.y, q.z, q.w))[0:2, 0:2])

    def set_t(self, p):
        self.T = self.fro_T + np.array([[p.x], [p.y]])

    def icp_fix(self, fix_R, fix_T):
        self.fro_R = fix_R
        self.fro_T = fix_T


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

    def init_pose(self):
        T = np.array([[self.origin.position.x], [self.origin.position.y]])
        R = quaternion_matrix((self.origin.orientation.x, self.origin.orientation.y, self.origin.orientation.z,
                              self.origin.orientation.w))[0:2, 0:2]
        self.data = np.dot(R, self.data) + np.dot(T, np.ones((1,self.data.shape[1])))


class PointScan:

    def __init__(self, seq=0, point=np.array([0, 0])):
        self.seq = seq
        self.point = point

    def align(self, R, T):
        self.point = np.dot(R, self.point) + np.dot(T, np.ones((1, self.point.shape[1])))


scan_point = PointScan()
X = Y = PointScan()
grid_point = PointScan()
robot_loc = RobotLoc()

plt_data_a_x = []
plt_data_a_y = []
plt_data_b_x = []
plt_data_b_y = []
plt_data_c_x = []
plt_data_c_y = []
plt_data_d_x = []
plt_data_d_y = []


class PlotFigure(wx.Frame):
    global plt_data_a_x, plt_data_a_y, plt_data_b_x, plt_data_b_y, plt_data_c_x, plt_data_c_y

    def __init__(self):

        wx.Frame.__init__(self, None, wx.ID_ANY, title="!!!", size=(1000, 1000))
        self.fig = Figure((10, 10), 100)
        self.canvas = FigureCanvas(self, wx.ID_ANY, self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_ylim([-10, 10])
        self.ax.set_xlim([-5, 15])
        self.ax.set_autoscale_on(False)
        self.ax.grid(True)
        self.user, = self.ax.plot(plt_data_a_x, plt_data_a_y, 'o', label='map')
        self.ax.legend(loc='upper center')
        self.canvas.draw()
        self.bg = self.canvas.copy_from_bbox(self.ax.bbox)
        wx.EVT_TIMER(self, TIMER_ID, self.onTimer)

    def onTimer(self, evt):
        """callback function for timer events"""

        self.canvas.restore_region(self.bg)
        a, = self.ax.plot(plt_data_b_x, plt_data_b_y, 'k')
        self.ax.draw_artist(a)
        b, = self.ax.plot(plt_data_c_x, plt_data_c_y, 'og')
        self.ax.draw_artist(b)
        c, = self.ax.plot(plt_data_d_x, plt_data_d_y, 'or')
        self.ax.draw_artist(c)
        theta = math.atan2(robot_loc.R[0, 1], robot_loc.R[1, 1]) + np.pi

        plt_data_e_x = (robot_loc.T[0, 0]-8, robot_loc.T[0, 0]+8, robot_loc.T[0, 0]+8, robot_loc.T[0, 0]-8, robot_loc.T[0, 0]-8)
        plt_data_e_y = (robot_loc.T[1, 0]+8, robot_loc.T[1, 0]+8, robot_loc.T[1, 0]-8, robot_loc.T[1, 0]-8, robot_loc.T[1, 0]+8)
        e, = self.ax.plot(plt_data_e_x, plt_data_e_y, 'k')
        self.ax.draw_artist(e)
        # f, = self.ax.plot(plt_data_f_x, plt_data_f_y, 'g')
        # self.ax.draw_artist(f)
        # g, = self.ax.plot(plt_data_g_x, plt_data_g_y, 'or')
        # self.ax.draw_artist(g)
        self.canvas.blit(self.ax.bbox)


def scan2point(laser_msg):
    global step
    index = np.arange(laser_msg.angle_min,laser_msg.angle_max,laser_msg.angle_increment).tolist()
    point_data = np.empty((2, 1))
    for i in range(0, len(index), step):
        if laser_msg.range_min <= laser_msg.ranges[i] <= 7:
            point_data = np.insert(point_data, -1, np.array([laser_msg.ranges[i]*math.cos(index[i]), laser_msg.ranges[i]*math.sin(index[i])]), axis=1)
    point_data = np.delete(point_data, -1, axis=1)
    return point_data


def scan_cb(laser_msg):
    global scan_point, grid_point
    scan_point = PointScan(laser_msg.header.seq, scan2point(laser_msg))
    scan_point.align(robot_loc.R, robot_loc.T)


def odom_cb(odom_msg):
    robot_loc.set_r(odom_msg.pose.pose.orientation)
    robot_loc.set_t(odom_msg.pose.pose.position)


def static_map_get():
    global plt_data_a_x, plt_data_a_y
    rospy.wait_for_service('/static_map')
    try:
        get_map = rospy.ServiceProxy('/static_map', GetMap)
        map_data = get_map().map
        global static_map
        static_map = MapData(map_data.info, map_data.data)
        static_map.init_pose()
        plt_data_a_x = static_map.data[0, :].tolist()
        plt_data_a_y = static_map.data[1, :].tolist()
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


def closest_point_kd(kdtree, P):
    num_p = P.shape[1]
    Y = np.empty(np.shape(P))
    n = 0
    dist_collect = []
    Y_collect = []
    P_collect = []
    Y_out = []
    P_out = []
    for i in range(num_p):
        index, min_dist = findNN(kdtree, P[:, i].tolist())
        Y[:, i] = index
        dist_collect.append(min_dist)
        Y_collect.append(Y[:, i])
        P_collect.append(P[:, i])
    average_dist = sum(dist_collect) / len(dist_collect)
    for i in range(len(dist_collect)):
        if abs(dist_collect[i] - average_dist) <= 0.1:
            Y_out.append(Y_collect[i])
            P_out.append(P_collect[i])
    return np.asarray(Y_out).transpose(), np.asarray(P_out).transpose()


def get_scan_data():
    global scan_point
    return scan_point.point


def icp_map_process():
    while not rospy.is_shutdown():
        P_p = get_scan_data()
        n = 0
        d_k_1 = 65535
        root = KD_node()
        kdtree = createKDTree(root, static_map.data.transpose().tolist())
        while not rospy.is_shutdown():
            Y_p, P_p_o = closest_point_kd(kdtree, P_p)
            R, T = icp(Y_p, P_p_o)
            num_p_o = P_p_o.shape[1]
            P_out = np.dot(R, P_p_o) + np.dot(T, np.ones((1, num_p_o)))
            num_p = P_p.shape[1]
            P_p = np.dot(R, P_p) + np.dot(T, np.ones((1, num_p)))
            d_k = la.norm(Y_p - P_out) / num_p_o
            if abs(d_k_1 - d_k) <= 10**-6 or n >= 50:
                break
            n += 1
            d_k_1 = d_k

        print n
        if d_k <= 0.010:
            robot_loc.icp_fix(R, T)
            global plt_data_d_x, plt_data_d_y
            plt_data_d_x = P_p[0, :]
            plt_data_d_y = P_p[1, :]
        global plt_data_b_x, plt_data_b_y
        plt_data_b_x.append(robot_loc.T[0][0])
        plt_data_b_y.append(robot_loc.T[1][0])
        time.sleep(0.1)


def icp_ros():
    rospy.init_node('icp_ros_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_cb)
    rospy.Subscriber('/odom', Odometry, odom_cb)

    static_map_get()

    while scan_point.seq == 0:
        time.sleep(0.01)

    try:
        thread.start_new_thread(icp_map_process, ())
    except:
        print "Error: unable to start thread"

    app = wx.PySimpleApp()
    frame = PlotFigure()
    t = wx.Timer(frame, TIMER_ID)
    t.Start(50)
    frame.Show()
    app.MainLoop()


if __name__ == '__main__':
    try:
        icp_ros()
    except rospy.ROSInterruptException:
        pass