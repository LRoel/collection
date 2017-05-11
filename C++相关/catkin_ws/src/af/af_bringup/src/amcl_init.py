#!/usr/bin/env python

import rospy
import math
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped
from path_planning.msg import comb

from tf.transformations import quaternion_from_euler

amcl_init_pose = PoseWithCovarianceStamped()

longitude_0 = 113.6938667 * math.pi / 180
latitude_0 = 34.7989201 * math.pi / 180

gps_recovery_x = []
gps_recovery_y = []

gps_recovery_flag = 1

def gps_cb(gpsMsg):

    global gps_recovery_flag, latitude_0, longitude_0, heading_0, pub, amcl_init_pose

    latitude_tmp = gpsMsg.Latitude
    longitude_tmp = gpsMsg.Longitude

    gps_recovery_x.append(latitude_tmp)
    gps_recovery_y.append(longitude_tmp)

    if gps_recovery_flag:
        N = len(gps_recovery_x)
        print "X"*(N//5)+"%d%%"%N
        if N >= 100:
            narray_x = np.array(gps_recovery_x)
            narray_y = np.array(gps_recovery_y)
            sum1_x = narray_x.sum()
            sum1_y = narray_y.sum()
            narray2_x = narray_x * narray_x
            narray2_y = narray_y * narray_y
            sum2_x = narray2_x.sum()
            sum2_y = narray2_y.sum()
            mean_x = sum1_x / N
            mean_y = sum1_y / N
            gps_var_x = sum2_x / N - mean_x ** 2
            gps_var_y = sum2_y / N - mean_y ** 2
            if len(gps_recovery_x) > 100:
                gps_recovery_x.pop(0)
                gps_recovery_y.pop(0)
            print (gps_var_x + gps_var_y) / 2
            if (gps_var_x + gps_var_y) / 2 <= 0.000005 ** 2:
                gps_recovery_flag = 0
                longitude_1 = mean_x * math.pi / 180
                latitude_1 = mean_y * math.pi / 180
                print "113.6938667", mean_x
                heading_1 = gpsMsg.Heading * math.pi / 180
                yaw = heading_1 + math.pi / 2
                # yaw = heading_1
                q = quaternion_from_euler(0, 0, yaw)
                e = 1.0 / 298.3
                R_e = 6378254l
                Rm = R_e * (1 - e * e) / (1 - e * e * math.sin(latitude_0) * math.sin(latitude_0)) ** (3.0 / 2)
                Rn = R_e / (1 - e * e * math.sin(latitude_0) * math.sin(latitude_0)) ** (1.0 / 2)
                lon_1_0 = latitude_1 - longitude_0
                lat_1_0 = longitude_1 - latitude_0
                robot_x = Rn * math.cos(latitude_0) * lon_1_0
                robot_y = Rm * lat_1_0
                amcl_init_pose.header.stamp = rospy.Time.now()
                amcl_init_pose.header.frame_id = "map"

                amcl_init_pose.pose.pose.position.x = robot_x
                amcl_init_pose.pose.pose.position.y = robot_y
                amcl_init_pose.pose.pose.position.z = 0
                amcl_init_pose.pose.pose.orientation.x = q[0]
                amcl_init_pose.pose.pose.orientation.y = q[1]
                amcl_init_pose.pose.pose.orientation.z = q[2]
                amcl_init_pose.pose.pose.orientation.w = q[3]
                
                print robot_x, robot_y
                pub.publish(amcl_init_pose)


def amcl_init():
    global pub

    rospy.init_node('amcl_init_node', anonymous=True)

    rospy.Subscriber('gps', comb, gps_cb)

    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    amcl_init()

