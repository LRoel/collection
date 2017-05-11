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
import matplotlib.font_manager as font_manager
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas

from tf.transformations import quaternion_matrix
from tf import TransformListener

from numpy import linalg as la
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap

step = 5

TIMER_ID = wx.NewId()


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
        a, = self.ax.plot(plt_data_b_x, plt_data_b_y, 'or')
        self.ax.draw_artist(a)
        b, = self.ax.plot(plt_data_c_x, plt_data_c_y, 'og')
        self.ax.draw_artist(b)
        c, = self.ax.plot(plt_data_d_x, plt_data_d_y, 'ok')
        self.ax.draw_artist(c)
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
    global plt_data_b_x, plt_data_b_y
    plt_data_b_x = scan_point.point[0, :].tolist()
    plt_data_b_y = scan_point.point[1, :].tolist()


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


def closest_point_grid(map, P):
    # TODO 栅格地图寻找最近点
    # map = MapData()
    Y_list = []
    P_list = []
    for laser_data in P.transpose():
        tmp_t = [map.origin.position.x, map.origin.position.y]
        tmp_laser = laser_data - tmp_t
        grid_data = tmp_laser // map.resolution
        if map.grid_data[int(grid_data[1] * map.width + grid_data[0])] == 100:
            Y_list.append(grid_data * static_map.resolution)
            P_list.append(laser_data)

    if len(Y_list):
        grid_out = np.asarray(Y_list).transpose()
        Y_p = grid_out + np.dot(np.asarray(tmp_t).reshape(-1, 1), np.ones((1, grid_out.shape[1])))
        P_p = np.asarray(P_list).transpose()
        return Y_p, P_p
    else:
        return P, P


def get_scan_data():
    global scan_point
    return scan_point.point


def icp_map_process():
    while not rospy.is_shutdown():
        P_p_o = get_scan_data()
        n = 0
        d_k_1 = 65535
        P_p_tmp = P_p_o
        while not rospy.is_shutdown():
            Y_p, P_p = closest_point_grid(static_map, P_p_tmp)
            if Y_p.any():
                R, T = icp(Y_p, P_p)
                num_p = P_p.shape[1]
                P_p = np.dot(R, P_p) + np.dot(T, np.ones((1, num_p)))
                num_p_o = P_p_o.shape[1]
                P_out = np.dot(R, P_p_o) + np.dot(T, np.ones((1, num_p_o)))
                P_p_tmp = P_out
                d_k = la.norm(Y_p - P_p) / num_p

            if abs(d_k_1 - d_k) <= 0.00001 or n >= 50:
                # P_out = P_p_o
                break
            n += 1
            d_k_1 = d_k
        global plt_data_d_x, plt_data_d_y
        plt_data_d_x = P_p_o[0, :]
        plt_data_d_y = P_p_o[1, :]
        print n
        robot_loc.icp_fix(R, T)
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