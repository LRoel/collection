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
import pylab as pl

step = 3

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
        alpha = np.linspace(-np.pi / 2, np.pi / 2, 10)
        x = 8 * np.cos(alpha)
        y = 8 * np.sin(alpha)
        _shape = np.array([x, y])
        np_zero = np.zeros((2, 1))
        _shape = np.c_[np_zero, _shape]
        # _shape = np.c_[_shape, np_zero]
        self.shape = _shape

    def get_map(self, T, R):
        T_fro = - np.array([[self.origin.position.x], [self.origin.position.y]])
        R_fro = la.inv(quaternion_matrix((self.origin.orientation.x, self.origin.orientation.y, self.origin.orientation.z,
                               self.origin.orientation.w))[0:2, 0:2])
        T_bak = np.array([[self.origin.position.x], [self.origin.position.y]])
        R_bak = quaternion_matrix((self.origin.orientation.x, self.origin.orientation.y, self.origin.orientation.z,
                               self.origin.orientation.w))[0:2, 0:2]
        T = T + T_fro
        R = np.dot(R, R_fro)
        _shape = np.dot(R, self.shape) + np.dot(T, np.ones((1, self.shape.shape[1])))
        _shape_max = np.floor(_shape.max(axis=1)) / 0.05
        _shape_min = np.floor(_shape.min(axis=1)) / 0.05
        # _shape_zero = np.floor(_shape[:, 0])
        tmp = np.asarray(self.grid_data).reshape(self.height, self.width)
        _shape_global_bool = np.zeros_like(tmp, dtype=bool)
        _shape_global_bool[int(_shape_min[1]):int(_shape_max[1]), int(_shape_min[0]):int(_shape_max[0])] = True
        _global_bool = np.logical_and(tmp == 100, _shape_global_bool)
        tmp_y, tmp_x = np.where(_global_bool)
        data = np.array([tmp_x, tmp_y]) * self.resolution
        tmp_y2, tmp_x2 = np.where(tmp == 100)
        data2 = np.array([tmp_x2, tmp_y2]) * self.resolution
        tmp_y3, tmp_x3 = np.where(_shape_global_bool)
        data3 = np.array([tmp_x3, tmp_y3]) * self.resolution
        data = np.dot(R_bak, data) + np.dot(T_bak, np.ones((1, data.shape[1])))
        data2 = np.dot(R_bak, data2) + np.dot(T_bak, np.ones((1, data2.shape[1])))
        data3 = np.dot(R_bak, data3) + np.dot(T_bak, np.ones((1, data3.shape[1])))
        return data, data2, data3

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
#         # print "!!!"
#         """callback function for timer events"""
#
#         self.canvas.restore_region(self.bg)
#         a, = self.ax.plot(plt_data_b_x, plt_data_b_y, 'k')
#         self.ax.draw_artist(a)
#         b, = self.ax.plot(plt_data_c_x, plt_data_c_y, 'og')
#         self.ax.draw_artist(b)
#         c, = self.ax.plot(plt_data_d_x, plt_data_d_y, 'or')
#         self.ax.draw_artist(c)
#         # theta = math.atan2(robot_loc.R[0, 1], robot_loc.R[1, 1]) + np.pi
#         #
#         # plt_data_e_x = (robot_loc.T[0, 0]-8, robot_loc.T[0, 0]+8, robot_loc.T[0, 0]+8, robot_loc.T[0, 0]-8, robot_loc.T[0, 0]-8)
#         # plt_data_e_y = (robot_loc.T[1, 0]+8, robot_loc.T[1, 0]+8, robot_loc.T[1, 0]-8, robot_loc.T[1, 0]-8, robot_loc.T[1, 0]+8)
#         # e, = self.ax.plot(plt_data_e_x, plt_data_e_y, 'k')
#         # self.ax.draw_artist(e)
#         # f, = self.ax.plot(plt_data_f_x, plt_data_f_y, 'g')
#         # self.ax.draw_artist(f)
#         # g, = self.ax.plot(plt_data_g_x, plt_data_g_y, 'or')
#         # self.ax.draw_artist(g)
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
        _data, _data2, _data3 = static_map.get_map(robot_loc.T, robot_loc.R)
        plt_data_a_x = _data2[0, :].tolist()
        plt_data_a_y = _data2[1, :].tolist()
        pl.figure()
        pl.plot(plt_data_a_x, plt_data_a_y, 'o')
        plt_data_a_x = _data[0, :].tolist()
        plt_data_a_y = _data[1, :].tolist()
        pl.plot(plt_data_a_x, plt_data_a_y, 'o')
        # plt_data_a_x = _data3[0, :].tolist()
        # plt_data_a_y = _data3[1, :].tolist()
        # pl.plot(plt_data_a_x, plt_data_a_y, 'o')
        pl.show()
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def icp_ros():
    rospy.init_node('icp_ros_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_cb)
    rospy.Subscriber('/odom', Odometry, odom_cb)

    static_map_get()

    # app = wx.PySimpleApp()
    # frame = PlotFigure()
    # t = wx.Timer(frame, TIMER_ID)
    # t.Start(50)
    # frame.Show()
    # app.MainLoop()


if __name__ == '__main__':
    try:
        icp_ros()
    except rospy.ROSInterruptException:
        pass