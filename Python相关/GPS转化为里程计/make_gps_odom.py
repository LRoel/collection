#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import math
from af_msgs.msg import Fuse_G_O, GPS_Odom


last_gps_time = 0


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


class GPSOdom:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 999999
        self.x_std = 999999
        self.y_std = 999999
        self.theta_std = 999999
        self.__ros_init()

    def __ros_init(self):
        rospy.init_node('make_gps_odom')
        rospy.Subscriber('/fuse', Fuse_G_O, self.update)
        self.pub = rospy.Publisher('/pose', GPS_Odom, queue_size=100)
        rospy.spin()

    def update(self, fuse_data):
        # fuse_data = Fuse_G_O()
        global last_gps_time
        if fuse_data.gps_time != last_gps_time:
            out_data = GPS_Odom()
            longitude_0 = 116.294938621 * math.pi / 180
            latitude_0 = 39.9593658664 * math.pi / 180
            yaw_0 = 163.021179199 * math.pi / 180
            latitude = fuse_data.lat * math.pi / 180
            longitude = fuse_data.lon * math.pi / 180
            yaw = fuse_data.heading * math.pi / 180

            e = 1.0 / 298.3
            R_e = 6378254l
            Rm = R_e * (1 - e * e) / (1 - e * e * math.sin(latitude_0) * math.sin(latitude_0)) ** (3.0 / 2)
            Rn = R_e / (1 - e * e * math.sin(latitude_0) * math.sin(latitude_0)) ** (1.0 / 2)

            lon_1_0 = longitude - longitude_0
            lat_1_0 = latitude - latitude_0

            out_data.header.stamp = rospy.Time.now()
            out_data.header.frame_id = 'map'
            out_data.X = Rn * math.cos(latitude_0) * lon_1_0
            out_data.X_std = fuse_data.lon_std
            out_data.Y = Rm * lat_1_0
            out_data.Y_std = fuse_data.lat_std
            out_data.Theta = angle_diff(yaw, yaw_0)
            out_data.Theta_std = fuse_data.hdg_std
            self.pub.publish(out_data)
        last_gps_time = fuse_data.gps_time


def make_gps_odom():
    GPSOdom()


if __name__ == '__main__':
    make_gps_odom()

