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

plt_data_dk = []
plt_data_seq = []
plt_data_R = []
plt_data_T = []
plt_data_tmp = []
plt_data_odom_x = []
plt_data_odom_y = []

step = 5

odom_msg = Odometry()

# wxWidgets object ID for the timer
TIMER_ID = wx.NewId()
# number of data points

# tf_listener = TransformListener()
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
        # tmp_y = self.height - tmp_y
        self.data = np.array([tmp_x, tmp_y]) * self.resolution

    # def display_map(self):
    #     pl.plot(self.data[0, :], self.data[1, :], 'o', label="map")

    def init_pose(self):
        T = np.array([[self.origin.position.x], [self.origin.position.y]])
        # T = -T
        R = quaternion_matrix((self.origin.orientation.x, self.origin.orientation.y, self.origin.orientation.z,
                              self.origin.orientation.w))[0:2, 0:2]
        self.data = np.dot(R, self.data) + np.dot(T, np.ones((1,self.data.shape[1])))
        # self.data = np.dot(R, self.data)


class PointScan:

    def __init__(self, seq=0, point=np.array([0, 0])):
        self.seq = seq
        self.point = point


scan_point = PointScan()
fix_point = PointScan()
X = Y = PointScan()
grid_point = PointScan()


class PlotFigure(wx.Frame):
    """Matplotlib wxFrame with animation effect"""
    global t
    global static_map, fix_point, grid_point

    def __init__(self):

        wx.Frame.__init__(self, None, wx.ID_ANY, title="!!!", size=(1000, 1000))
        # Matplotlib Figure
        self.fig = Figure((10, 10), 100)
        # bind the Figure to the backend specific canvas
        self.canvas = FigureCanvas(self, wx.ID_ANY, self.fig)
        # add a subplot
        self.ax = self.fig.add_subplot(111)
        # limit the X and Y axes dimensions
        self.ax.set_ylim([-10, 10])
        self.ax.set_xlim([-5, 15])

        self.ax.set_autoscale_on(False)
        # self.ax.set_xticks([])
        # # we want a tick every 10 point on Y (101 is to have 10
        # self.ax.set_yticks(range(0, 101, 10))
        # # disable autoscale, since we don't want the Axes to ad
        # # draw a grid (it will be only for Y)
        self.ax.grid(True)
        # generates first "empty" plots
        # self.user = [None] * POINTS
        self.user, = self.ax.plot(static_map.data[0, :], static_map.data[1, :], 'o', label='map')

        # add the legend
        self.ax.legend(loc='upper center',
                       ncol=4,
                       prop=font_manager.FontProperties(size=10))
        # force a draw on the canvas()
        # trick to show the grid and the legend
        self.canvas.draw()
        # save the clean background - everything but the line
        # is drawn and saved in the pixel buffer background
        self.bg = self.canvas.copy_from_bbox(self.ax.bbox)
        # bind events coming from timer with id = TIMER_ID
        # to the onTimer callback function
        wx.EVT_TIMER(self, TIMER_ID, self.onTimer)

    def onTimer(self, evt):
        """callback function for timer events"""
        # restore the clean background, saved at the beginning
        self.canvas.restore_region(self.bg)
        # update the data
        # temp = np.random.randint(10, 80)
        # self.user = self.user[1:] + [temp]
        # update the plot
        # self.l_user.set_ydata(self.user)
        a, = self.ax.plot(fix_point.point[0, :], fix_point.point[1, :], 'or')
        # just draw the "animated" objects
        self.ax.draw_artist(
            a)
        # It is used to efficiently update Axes data (axis ticks, labels, etc are not updated)
        b, = self.ax.plot(grid_point.point[0, :], grid_point.point[1, :], 'og', label='2')
        # just draw the "animated" objects
        self.ax.draw_artist(
            b)
        # It is used to efficiently update Axes data (axis ticks, labels, etc are not updated)
        self.canvas.blit(self.ax.bbox)

    def __del__(self):
        t.Stop()



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


def icp(X,P):
    # get the num of pixel
    num_x = X.shape[1]
    num_p = P.shape[1]
    # get the mean of scan_data
    mean_x = X.mean(axis=1).reshape((2, 1))
    mean_p = P.mean(axis=1).reshape((2, 1))
    # subtract the means
    X_m = X - np.dot(mean_x, np.ones((1, num_x)))
    P_m = P - np.dot(mean_p, np.ones((1, num_p)))
    # compute A
    A = np.dot(X_m, P_m.transpose())
    # compute R
    U, tmp, V = la.svd(A)
    num_a = A.shape[1]
    S = np.identity(num_a)
    if la.det(A) < 0:
        S[-1, -1] = -1
    R = np.dot(np.dot(U, S), V.transpose())
    # compute T
    T = mean_x - np.dot(R, mean_p)
    return R, T


def center(In1, In2):
    num_1 = In1.shape[1]
    # compute the means
    mean_1 = In1.mean(axis=1).reshape((2, 1))
    mean_2 = In2.mean(axis=1).reshape((2, 1))

    # subtract the means
    Out1 = In1 + np.dot((mean_2 - mean_1), np.ones((1, num_1)))
    return Out1


def closest_point_KD(X, P):
    num_p = P.shape[1]
    Y = np.empty(np.shape(P))

    root = KD_node()
    kdtree = createKDTree(root, X.transpose().tolist())

    for i in range(num_p):
        # index, min_dist = findNN(kdtree, np.array(P[i,:]).reshape(-1,).tolist())
        index, min_dist = findNN(kdtree, P[:, i].tolist())
        Y[:, i] = index
    return Y


def scan2point(lasermsg):
    global step
    index = np.arange(lasermsg.angle_min,lasermsg.angle_max,lasermsg.angle_increment).tolist()
    point_data = np.empty((2, 1))
    for i in range(0, len(index), step):
        if lasermsg.range_min <= lasermsg.ranges[i] <= 7:
            point_data = np.insert(point_data, -1, np.array([lasermsg.ranges[i]*math.cos(index[i]), lasermsg.ranges[i]*math.sin(index[i])]), axis=1)
    point_data = np.delete(point_data, -1, axis=1)
    return point_data


def scan_cb(lasermsg):
    global scan_point, fix_point, odom_msg, grid_point
    scan_point = PointScan(lasermsg.header.seq, scan2point(lasermsg))
    # pl.figure()
    # if tf_listener.frameExists("/base_link") and tf_listener.frameExists("/map"):
    #     t = tf_listener.getLatestCommonTime("/base_link", "/map")
    #     position, quaternion = tf_listener.lookupTransform("/base_link", "/map", t)
        # print position, quaternion
    position = odom_msg.pose.pose.position
    quaternion = odom_msg.pose.pose.orientation
    # print position, quaternion
    T = np.array([[position.x], [position.y]])
    R = quaternion_matrix((quaternion.x, quaternion.y, quaternion.z, quaternion.w))[0:2, 0:2]
    fix_point.point = np.dot(R, scan_point.point) + np.dot(T, np.ones((1, scan_point.point.shape[1])))
    tmp = grid(fix_point, static_map)
    if tmp.any():
        grid_point.point = tmp


def grid(fix_point, static_map):
    # fix_point = PointScan()
    # static_map = MapData()
    grid_show = []
    for laser_data in fix_point.point.transpose():
        tmp_t = [static_map.origin.position.x, static_map.origin.position.y]
        tmp_laser = laser_data - tmp_t
        grid_data = tmp_laser//static_map.resolution
        if static_map.grid_data[int(grid_data[1]*static_map.width+grid_data[0])] == 100:
            grid_show.append(grid_data * static_map.resolution)

    grid_out = np.asarray(grid_show).transpose()
    if grid_out.any():
        return grid_out + np.dot(np.asarray(tmp_t).reshape(-1,1), np.ones((1, grid_out.shape[1])))
    else:
        return grid_out



def odom_cb(odommsg):
    global odom_msg
    odom_msg = odommsg
    plt_data_odom_x.append(odommsg.pose.pose.position.x)
    plt_data_odom_y.append(odommsg.pose.pose.position.y)


def icp_process(init_X):
    X = init_X
    R_fro = np.eye(2)
    W = np.zeros(2).reshape(2, 1)
    while not rospy.is_shutdown():
        tmp = scan_point
        if tmp.seq > X.seq + 20:
            P = tmp
        else:
            continue
        P_p_o = P.point
        X_p = X.point
        P_p = center(P_p_o, X_p)

        n = 0
        d_k_1 = 0
        while not rospy.is_shutdown():
            Y_p = closest_point_KD(X_p, P_p)
            R, T = icp(Y_p, P_p_o)
            num_p = P_p.shape[1]
            P_p = np.dot(R, P_p_o) + np.dot(T, np.ones((1, num_p)))
            d_k = la.norm(Y_p - P_p) / num_p

            if (abs(d_k_1 - d_k) <= 0.0001 or n >= 50):
                break
            n = n + 1
            d_k_1 = d_k

        plt_data_dk.append(d_k)
        plt_data_seq.append(P.seq)
        plt_data_R.append(R)

        W_cur = T
        R_cur = R
        dW = np.dot(R_fro, W_cur)
        W = W + dW
        R_fro = R_fro * R_cur
        plt_data_tmp.append(math.acos(R_fro[0][0]))
        plt_data_T.append(W)
        if P.seq >= 3200:
            # data = np.array(plt_data_T).reshape(-1, 2)
            # pl.figure()
            # pl.plot(plt_data_seq, plt_data_tmp, 'o')
            # # pl.savefig('/home/exbot/icpjpg/' + 'plt_data_seq' + '.jpg')
            # pl.figure()
            # pl.plot(data[:, 0], data[:, 1])
            # # pl.plot(plt_data_odom_x, plt_data_odom_y)
            # # pl.plot(data[:, 0], 'o')
            # # pl.plot(data[:, 1], 'o')
            # # pl.savefig('/home/exbot/icpjpg/' + 'dW' + '.jpg')
            # # pl.show()

            while 1:
                print "ok!!!!!!!!!!"

        O = PointScan()
        O.point = P_p
        # pl.figure()
        # X.display_point()
        # P.display_point()
        # O.display_point()
        # pl.xlim(-1, 5)
        # pl.ylim(-3, 3)
        # pl.legend()
        # pl.savefig('/home/exbot/icpjpg/' + str(P.seq) + '_' + str(dW.reshape(1, -1).tolist()) + '.jpg')

        # if d_k > 0.01:
        #     O = PointScan()
        #     O.point = P_p
        #     pl.figure()
        #     X.display_point()
        #     P.display_point()
        #     O.display_point()
        #     pl.savefig('/home/exbot/icpjpg/'+ str(step)+'_'+str(P.seq)+'_'+str(d_k)+'.jpg')
        X = P
        # print X.seq


def icp_map_process():
    # if static_map
    # X = static_map
    R_fro = np.eye(2)
    W = np.zeros(2).reshape(2, 1)
    while not rospy.is_shutdown():
        tmp = scan_point
        if tmp.seq > X.seq + 20:
            P = tmp
        else:
            continue
        P_p_o = P.point
        X_p = X.point
        P_p = center(P_p_o, X_p)

        n = 0
        d_k_1 = 0
        while not rospy.is_shutdown():
            Y_p = closest_point_KD(X_p, P_p)
            R, T = icp(Y_p, P_p_o)
            num_p = P_p.shape[1]
            P_p = np.dot(R, P_p_o) + np.dot(T, np.ones((1, num_p)))
            d_k = la.norm(Y_p - P_p) / num_p

            if (abs(d_k_1 - d_k) <= 0.0001 or n >= 50):
                break
            n = n + 1
            d_k_1 = d_k

        plt_data_dk.append(d_k)
        plt_data_seq.append(P.seq)
        plt_data_R.append(R)

        W_cur = T
        R_cur = R
        dW = np.dot(R_fro, W_cur)
        W = W + dW
        R_fro = R_fro * R_cur
        plt_data_tmp.append(math.acos(R_fro[0][0]))
        plt_data_T.append(W)
        if P.seq >= 3200:
            # data = np.array(plt_data_T).reshape(-1, 2)
            # pl.figure()
            # pl.plot(plt_data_seq, plt_data_tmp, 'o')
            # # pl.savefig('/home/exbot/icpjpg/' + 'plt_data_seq' + '.jpg')
            # pl.figure()
            # pl.plot(data[:, 0], data[:, 1])
            # # pl.plot(plt_data_odom_x, plt_data_odom_y)
            # # pl.plot(data[:, 0], 'o')
            # # pl.plot(data[:, 1], 'o')
            # # pl.savefig('/home/exbot/icpjpg/' + 'dW' + '.jpg')
            # # pl.show()

            while 1:
                print "ok!!!!!!!!!!"

        O = PointScan()
        O.point = P_p
        # pl.figure()
        # X.display_point()
        # P.display_point()
        # O.display_point()
        # pl.xlim(-1, 5)
        # pl.ylim(-3, 3)
        # pl.legend()
        # pl.savefig('/home/exbot/icpjpg/' + str(P.seq) + '_' + str(dW.reshape(1, -1).tolist()) + '.jpg')

        # if d_k > 0.01:
        #     O = PointScan()
        #     O.point = P_p
        #     pl.figure()
        #     X.display_point()
        #     P.display_point()
        #     O.display_point()
        #     pl.savefig('/home/exbot/icpjpg/'+ str(step)+'_'+str(P.seq)+'_'+str(d_k)+'.jpg')
        X = P
        # print X.seq


def static_map_get():
    global static_map
    rospy.wait_for_service('/static_map')
    try:
        get_map = rospy.ServiceProxy('/static_map', GetMap)
        map_data = get_map().map
        static_map = MapData(map_data.info, map_data.data)
        static_map.init_pose()
        # static_map.display_map()
        # scan_point.display_point()
        # pl.legend()
        # pl.show()

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def icp_ros():

    rospy.init_node('icp_ros_node', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, scan_cb)
    rospy.Subscriber('/odom', Odometry, odom_cb)
    # rospy.Subscriber('/map', OccupancyGrid, map_get)
    static_map_get()
    # global tf_listener
    # tf_listener = TransformListener()
    # spin() simply keeps python from exiting until this node is stopped
    # print "!!!!!!!!!!!!!!!"
    # while scan_point.seq == 0:
    #     time.sleep(0.01)
    # X = scan_point
    # print X.seq
    # try:
    #     thread.start_new_thread(icp_process, (X,))
    # except:
    #     print "Error: unable to start thread"
    app = wx.PySimpleApp()
    frame = PlotFigure()
    global t
    t = wx.Timer(frame, TIMER_ID)
    t.Start(50)
    frame.Show()
    app.MainLoop()



if __name__ == '__main__':
    try:
        icp_ros()
    except rospy.ROSInterruptException:
        pass

# X = np.array([[1,2,3,4,5],[1,1,1,1,1]])
# P = np.array([[2,3,4,5],[2,2,2,2]])
# Y = closest_point_KD(X, P)
# icp(Y,P)
