#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

from sensor_msgs.msg import LaserScan


class Filter_Laser:
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

    """

    def __init__(self):
        """构造函数"""
        rospy.init_node('scan_filter')
        self.scan_msg = LaserScan()
        rospy.Subscriber('/scan/origin', LaserScan, self.update)
        self.pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    def update(self, msg):
        """里程计更新

        Args:
            odom_msg:   ros里程计输入

        See Also:
            _pos:   位置矩阵
            _pose_base: 位置基准矩阵
            _pose_update:   坐标系变换
            _quat:  四元数

        """

        scan_filter = LaserScan()
        scan_filter.header = msg.header
        scan_filter.angle_increment = msg.angle_increment
        scan_filter.angle_max = msg.angle_max
        scan_filter.angle_min = msg.angle_min
        scan_filter.intensities = msg.intensities
        scan_filter.scan_time = msg.scan_time
        scan_filter.range_max = msg.range_max
        scan_filter.range_min = msg.range_min
        scan_filter.time_increment = msg.time_increment
        scan_range = []
        for i in msg.ranges:
            if i >= 64:
                scan_range.append(float("inf"))
            else:
                i_fix = i - 0.1
                if i_fix < scan_filter.range_min:
                    i_fix = scan_filter.range_min
                scan_range.append(i_fix)
        scan_filter.ranges = scan_range
        self.scan_msg = scan_filter
        self.publish()

    def publish(self):
        """AMCL修正输入

        将AMCL修正输入的map->odom和里程计输入融合生成实时的AMCL输出值(机器人在地图上的激光定位)

        Args:
            odom_map_msg: AMCL的输入的map->odom

        """

        self.pub.publish(self.scan_msg)


def main():
    s = Filter_Laser()
    rospy.spin()


if __name__ == '__main__':
    main()