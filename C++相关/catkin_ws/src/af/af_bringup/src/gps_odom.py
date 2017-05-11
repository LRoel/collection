#!/usr/bin/env python

import rospy
import math
import tf
import numpy as np

from nav_msgs.msg import Odometry
from path_planning.msg import comb
from af_bringup.msg import Robot_encode
from tf import transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
# from tf.msg import tfMessage

robot_odom = Odometry()

La = 0
odom_limit = 1.0
init_flag = 0
gps_flag = 0
nav_err = 0
constraint_r = 0.1
gps_limit = 0.05
odom_msg = [0.0] * 3

# longitude_0 = 113.6944569 * math.pi / 180
# latitude_0 = 34.7994373 * math.pi / 180

longitude_0 = 113.6938607 * math.pi / 180
latitude_0 = 34.7989156 * math.pi / 180
gps_get_flag = 0
count = 0
heading_tmp_old = 0
odom_yaw_old = 0
heading_gps_old = 0
heading_gps_recovery = 0

gps_recovery_x = []
gps_recovery_y = []
success_flag = []

odom_yaw_angle = []

gps_recovery_flag = 1
gps_var = None

seq = 0
yaw = 0
yaw_sum = 0
robot_x = 0
robot_y = 0
last_time = 0
odom_w = 0

X_SET = 0
Y_SET = 0

f1 = open('/home/ros/gps_odom_debug.txt','a')
# n = 0
#
#
# def af_init():
#     global , init_flag
#     if gps_flag > 1000
#     init_flag = 1

## Status GPS Lidar Return  ###############
#           0   0      0
#           0   1      1
#           1   0      2
#           1   1      3
############################################

def align(a):

    if a < 0:
        b = 360 + a        #-180~+180 -> 0~360
        #b = math.pi + a
    else:
        b = a
    return b
    
def delta_range(sum_tmp):
    #print "sum_tmp",sum_tmp
    if sum_tmp > 180:
        sum_angle = sum_tmp - 360
    elif sum_tmp < -180:
        sum_angle = sum_tmp + 360
    else:
        sum_angle = sum_tmp
    #print "sum_angle",sum_angle
    return sum_angle

def odom_cb(odommsg):
    # print "odom cb"
    global odom_msg, odom_yaw_angle, odom_yaw_old, odom_w

    odom_msg[0] = odommsg.twist.twist.angular.z
    odom_msg[1] = odommsg.twist.twist.linear.x
    orientation = [odommsg.pose.pose.orientation.x, odommsg.pose.pose.orientation.y, odommsg.pose.pose.orientation.z,
                   odommsg.pose.pose.orientation.w]
    (odom_roll, odom_pitch, odom_yaw) = euler_from_quaternion(orientation)
    odom_yaw = align(odom_yaw)
    odom_delta_angle = delta_range((odom_yaw-odom_yaw_old) * 180 / math.pi)
    odom_yaw_angle.append(odom_delta_angle)
    odom_yaw_old = odom_yaw
    odom_w = odommsg.twist.twist.angular.z * 180 / math.pi

def gps_cb(gpsMsg):

    global odom_msg,robot_odom,gps_recovery_flag,latitude_0,longitude_0,seq,X_SET,Y_SET, odom_w
    global yaw,gps_get_flag,robot_x,robot_y,last_time, heading_tmp_old, heading_gps_old, odom_yaw_angle, heading_gps_recovery, success_flag

    seq += 1
    current_time = rospy.get_time()
    delta_time = current_time - last_time
    print delta_time
    latitude_tmp = gpsMsg.Latitude
    longitude_tmp = gpsMsg.Longitude

    gps_recovery_x.append(latitude_tmp)
    gps_recovery_y.append(longitude_tmp)

    if gps_recovery_flag:
        N = len(gps_recovery_x)
        print N
        last_time = current_time
        if N > 10:
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
            if len(gps_recovery_x) > 10:
                gps_recovery_x.pop(0)
                gps_recovery_y.pop(0)
            print (gps_var_x + gps_var_y)/2
            if (gps_var_x + gps_var_y)/2 <= 0.000005 ** 2:
                gps_recovery_flag = 0
                #latitude_0 = mean_x * math.pi / 180
                #longitude_0 = mean_y * math.pi / 180
                #s = "!!!!!! latitude_0 : "+str(mean_x)+"  longitude_0 : "+str(mean_y) + '!!!!!!!\n'
                #f1.write(s)
                heading_tmp_old = gpsMsg.Heading
                odom_yaw_angle = []
                heading_gps_recovery = gpsMsg.Heading
                
            else:
                return 0
        else:
            return 0

    gps_get_flag = 1

    heading_gps = gpsMsg.Heading
    heading_odom = heading_gps_recovery + sum(odom_yaw_angle)

    print heading_gps - heading_gps_recovery
    sum_odom_angle = sum(odom_yaw_angle)
    sum_gps_angle = delta_range(heading_gps - heading_gps_recovery)

    print sum_odom_angle, sum_gps_angle
    print sum_gps_angle/delta_time, odom_w
    #if (abs(sum_gps_angle) <= abs(sum_odom_angle) + 5) and gpsMsg.NSV1 > 4 and abs(sum_gps_angle/delta_time - odom_w) < 20:
    # print "--------"+str(odom_w)+"   "+str(abs(sum_gps_angle/delta_time - odom_w))
    # if gpsMsg.NSV1 > 4 and abs(sum_gps_angle/delta_time - odom_w) < 20:
    if gpsMsg.NSV1 > 4 and gpsMsg.Pitch != 0:
        heading_tmp = heading_gps
        # heading_tmp = heading_gps
        heading_gps_recovery = heading_gps
        odom_yaw_angle = []
        last_time = current_time
        print "gps"
    else:
        success_flag = []
        heading_tmp = heading_odom
        print "odom"

    heading_tmp_old = heading_tmp
    heading_gps_old = heading_gps
    print heading_tmp, heading_odom

    ve_tmp = gpsMsg.Ve
    vn_tmp = gpsMsg.Vn

    if vn_tmp > 0:
        sign = 1
    else:
        sign = 0
    robot_v = sign * math.sqrt(ve_tmp ** 2 + vn_tmp ** 2)

    latitude = latitude_tmp * math.pi / 180
    longitude = longitude_tmp * math.pi / 180
    yaw = heading_tmp * math.pi / 180 + math.pi / 2
    q = quaternion_from_euler(0, 0, yaw)

    e = 1.0 / 298.3
    R_e = 6378254l
    Rm = R_e * (1 - e * e) / (1 - e * e * math.sin(latitude_0) * math.sin(latitude_0)) ** (3.0 / 2)
    Rn = R_e / (1 - e * e * math.sin(latitude_0) * math.sin(latitude_0)) ** (1.0 / 2)

    lon_1_0 = longitude - longitude_0
    lat_1_0 = latitude - latitude_0

    robot_x = Rn * math.cos(latitude_0) * lon_1_0
    robot_y = Rm * lat_1_0

    robot_x = robot_x + X_SET
    robot_y = robot_y + Y_SET

    robot_odom.header.seq = seq
    robot_odom.header.stamp = rospy.Time.now()
    robot_odom.header.frame_id = "odom"
    robot_odom.child_frame_id = "base_footprint"

    robot_odom.pose.pose.position.x = robot_x
    robot_odom.pose.pose.position.y = robot_y
    robot_odom.pose.pose.position.z = 0
    robot_odom.pose.pose.orientation.x = q[0]
    robot_odom.pose.pose.orientation.y = q[1]
    robot_odom.pose.pose.orientation.z = q[2]
    robot_odom.pose.pose.orientation.w = q[3]

    robot_odom.twist.twist.linear.x = gpsMsg.Ve
    robot_odom.twist.twist.linear.y = gpsMsg.Vn
    if gpsMsg.Status == '0B':
        robot_odom.twist.twist.linear.z = 1
    elif gpsMsg.Status == '04':
        robot_odom.twist.twist.linear.z = 2
    elif gpsMsg.Status == '05':
        robot_odom.twist.twist.linear.z = 3
    else:
        robot_odom.twist.twist.linear.z = 0

    robot_odom.twist.twist.angular.x = 0
    robot_odom.twist.twist.angular.y = 0
    robot_odom.twist.twist.angular.z = odom_msg[0]

    pub.publish(robot_odom)

    # print robot_odom
    # robot_trans = tf.TransformBroadcaster()
    # print robot_x + m_tranX
    # robot_trans.sendTransform((robot_x, robot_y, 0),
    #                 quaternion_from_euler(0, 0, yaw),
    #                 rospy.Time.now(),
    #                 "base_footprint",
    #                 "odom")

    # br = tf.TransformBroadcaster()
    # # print robot_x + m_tranX
    # br.sendTransform((0, 0, 0),
    #                  quaternion_from_euler(0, 0, 0),
    #                  rospy.Time.now(),
    #                  "odom",
    #                  "map")
    

def gps_odom():
    global tf_listener, pub

    rospy.init_node('gps_odom_node', anonymous=True)

    rospy.Subscriber('/gps', comb, gps_cb)
    rospy.Subscriber('/odom', Odometry, odom_cb)

    pub = rospy.Publisher('/odom/gps', Odometry, queue_size=10)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    gps_odom()


