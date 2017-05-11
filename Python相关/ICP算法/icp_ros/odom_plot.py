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
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PointStamped

import wx
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import FigureCanvasWxAgg as FigureCanvas

TIMER_ID = wx.NewId()

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
        self.ax.set_ylim([10, 20])
        self.ax.set_xlim([5, 15])
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
        a, = self.ax.plot(plt_data_b_x, plt_data_b_y, 'r')
        self.ax.draw_artist(a)
        b, = self.ax.plot(plt_data_c_x, plt_data_c_y, 'og')
        self.ax.draw_artist(b)
        c, = self.ax.plot(plt_data_d_x, plt_data_d_y, 'ok')
        self.ax.draw_artist(c)
        self.canvas.blit(self.ax.bbox)


def odom_cb(odommsg):
    # odommsg = Odometry()
    print "++++++++++++++++++++++++++"
    global plt_data_b_x, plt_data_b_y
    plt_data_b_x.append(odommsg.pose.pose.position.x)
    plt_data_b_y.append(odommsg.pose.pose.position.y)
    print odommsg.pose.pose.position.x, odommsg.pose.pose.position.y

if __name__ == '__main__':
    rospy.init_node('plot_node', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_cb)

    app = wx.PySimpleApp()
    frame = PlotFigure()
    t = wx.Timer(frame, TIMER_ID)
    t.Start(50)
    frame.Show()
    app.MainLoop()
