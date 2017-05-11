#!/usr/bin/env python

import numpy as np
import numpy.linalg as la
import math
import datetime


MAX_STD = 999999


class AMCL_unit:
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

    def output(self):
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_std: ", self.x_std, "| y_std: ", self.y_std, "| theta_std: ", self.theta_std


class SELECTED():
    def __init__(self):
        self.choice = {0: "GPS", 1: "AMCL", 2: "NONE"}
        self.x = 0
        self.x_std = MAX_STD
        self.x_choice = self.choice[2]
        self.y = 0
        self.y_std = MAX_STD
        self.y_choice = self.choice[2]
        self.theta = 0
        self.theta_std = MAX_STD
        self.theta_choice = self.choice[2]

    def output(self):
        print "x: ", self.x, "| y: ", self.y, "| theta: ", self.theta
        print "x_choice: ", self.x_choice, "| y_choice: ", self.y_choice, "| theta_choice: ", self.theta_choice


def normalization(a, b):
    sum = a + b
    return a/sum, b/sum


def test():
    _x1 = amcl_pose.x
    _x2 = gps_pose.x
    _std_x1 = amcl_pose.x_std
    _std_x2 = gps_pose.x_std
    if _std_x1 == MAX_STD and _std_x2 != MAX_STD:
        _w_x1 = 0
        _w_x2 = 1
    elif _std_x1 != MAX_STD and _std_x2 == MAX_STD:
        _w_x1 = 1
        _w_x2 = 0
    elif _std_x1 == MAX_STD and _std_x2 == MAX_STD:
        return -1
    else:
        _w_x1 = _std_x2 / (_std_x1 + _std_x2)
        _w_x2 = _std_x1 / (_std_x1 + _std_x2)
    if _w_x1 > _w_x2:
        out_pose.x_choice = out_pose.choice[1]
    else:
        out_pose.x_choice = out_pose.choice[0]
    _w_x1, _w_x2 = normalization(_w_x1, _w_x2)
    out_pose.x = _w_x1 * _x1 + _w_x2 * _x2
    _y1 = amcl_pose.y
    _y2 = gps_pose.y
    _std_y1 = amcl_pose.y_std
    _std_y2 = gps_pose.y_std
    if _std_y1 == MAX_STD and _std_y2 != MAX_STD:
        _w_y1 = 0
        _w_y2 = 1
    elif _std_y1 != MAX_STD and _std_y2 == MAX_STD:
        _w_y1 = 1
        _w_y2 = 0
    elif _std_y1 == MAX_STD and _std_y2 == MAX_STD:
        return -1
    else:
        _w_y1 = _std_y2 / (_std_y1 + _std_y2)
        _w_y2 = _std_y1 / (_std_y1 + _std_y2)
    if _w_y1 > _w_y2:
        out_pose.y_choice = out_pose.choice[1]
    else:
        out_pose.y_choice = out_pose.choice[0]
    _w_y1, _w_y2 = normalization(_w_y1, _w_y2)
    out_pose.y = _w_y1 * _y1 + _w_y2 * _y2
    _theta1 = amcl_pose.theta
    _theta2 = gps_pose.theta
    _std_theta1 = amcl_pose.theta_std
    _std_theta2 = gps_pose.theta_std
    if _std_theta1 == MAX_STD and _std_theta2 != MAX_STD:
        _w_theta1 = 0
        _w_theta2 = 1
    elif _std_theta1 != MAX_STD and _std_theta2 == MAX_STD:
        _w_theta1 = 1
        _w_theta2 = 0
    elif _std_theta1 == MAX_STD and _std_theta2 == MAX_STD:
        return -1
    else:
        _w_theta1 = _std_theta2 / (_std_theta1 + _std_theta2)
        _w_theta2 = _std_theta1 / (_std_theta1 + _std_theta2)
    if _w_theta1 > _w_theta2:
        out_pose.theta_choice = out_pose.choice[1]
    else:
        out_pose.theta_choice = out_pose.choice[0]
    _w_theta1, _w_theta2 = normalization(_w_theta1, _w_theta2)
    out_pose.theta = _w_theta1 * _theta1 + _w_theta2 * _theta2
    out_pose.output()


if __name__ == '__main__':
    try:
        amcl_pose = AMCL_unit()
        gps_pose = GPS_unit()
        out_pose = SELECTED()

        amcl_pose.x = 1
        amcl_pose.x_std = 0.00001
        amcl_pose.y = 10
        amcl_pose.y_std = 0.00001
        amcl_pose.theta = math.pi/2
        amcl_pose.theta_std = 0.00001

        test()
    except:
        pass