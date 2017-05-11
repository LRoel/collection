#!/usr/bin/env python

import rospy
import tf
import numpy as np
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from af_bringup.msg import Robot_encode

from tf import transformations
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

import matplotlib.pyplot as plt

plt_data = []
plt_data_x = []
plt_data_y = []
plt_data1 = []
plt_data1_x = []
plt_data1_y = []
plt_data2 = []
plt_data2_x = []
plt_data2_y = []
plt_data3 = []
plt_data3_x = []
plt_data3_y = []
plt_data4 = []
plt_data5 = []
plt_data6 = []
plt_data7 = []
plt_data8 = []
plt_data9 = []


gps_msg = [0.0] * 3
odom_msg = [0.0] * 3
amcl_msg = [0.0] * 3
out_msg = [0.0] * 3
init_msg = [0.0] * 3
gps_msg_old = [0.0] * 3
odom_msg_old = [0.0] * 3
amcl_msg_old = [0.0] * 3
out_msg_old = [0.0] * 3

gps_msg_delta = [0.0] * 3
odom_msg_delta = [0.0] * 3
amcl_msg_delta = [0.0] * 3

encode_delta_x = []
encode_delta_y = []
encode_delta_yaw = []
encode_delta_x_amcl = []
encode_delta_y_amcl = []
encode_delta_yaw_amcl = []


#odom_gps_delta_x = []
#odom_gps_delta_y = []
#odom_gps_delta_yaw = []
#odom_amcl_delta_x = []
#odom_amcl_delta_y = []
#odom_amcl_delta_yaw = []

odom_gps_x = []
odom_gps_y = []
odom_gps_yaw = []
odom_amcl_x = []
odom_amcl_y = []
odom_amcl_yaw = []
odom_out_x = []
odom_out_y = []
odom_out_yaw = []

gps_init_x = []
gps_init_y = []

constraint_r = 0.10

gps_flag = 0
amcl_flag = 0
init_flag = 0


m_tranX = m_tranY = m_rotation = 0
m_sumQ = 0

odom_filter = Odometry()
odom_fix = Odometry()

n = 0
gps_yaw_filter = 0

print "init start"

## Status GPS Lidar Return  ###############
#           0   0      0
#           0   1      1
#           1   0      2
#           1   1      3
############################################

def delta_range(sum_tmp_rad):

    sum_tmp_deg = sum_tmp_rad * 180 / math.pi
    if sum_tmp_deg > 180:
        sum_angle = sum_tmp_deg - 360
    elif sum_tmp_deg < -180:
        sum_angle = sum_tmp_deg + 360
    else:
        sum_angle = sum_tmp_deg
    return sum_angle * math.pi / 180

def align(a):

    if a < 0:
        b = 2 * math.pi + a        #-180~+180 -> 0~360
        #b = math.pi + a
    else:
        b = a
    return b

def do_yaw(a):
     b = a % (2 * math.pi)
     return b

def gps_callback(gpsmsg):

    global gps_msg,gps_msg_old,gps_msg_delta,gps_flag
    global encode_delta_x,encode_delta_y,encode_delta_yaw
    global init_flag, init_msg, amcl_flag, gps_yaw_flag, m_sumQ
    global out_msg

    gps_msg[0] = gpsmsg.pose.pose.position.x
    gps_msg[1] = gpsmsg.pose.pose.position.y
    gps_orientation = [gpsmsg.pose.pose.orientation.x, gpsmsg.pose.pose.orientation.y, gpsmsg.pose.pose.orientation.z,
                   gpsmsg.pose.pose.orientation.w]
    (gps_roll, gps_pitch, gps_yaw) = euler_from_quaternion(gps_orientation)

    gps_msg[2] = align(gps_yaw)
    plt_data1.append(gps_msg[2])
    plt_data1_x.append(gps_msg[0])
    plt_data1_y.append(gps_msg[1])
    plt_data9.append(rospy.get_time())

    gps_msg_delta[0] = gps_msg[0] - gps_msg_old[0]
    gps_msg_delta[1] = gps_msg[1] - gps_msg_old[1]
    gps_msg_delta[2] = do_yaw(gps_msg[2] - gps_msg_old[2])

    gps_init_x.append(gps_msg[0])
    gps_init_y.append(gps_msg[1])

    if not init_flag:
        N = len(gps_init_x)
        print N
        if N >= 10:
            narray_x = np.array(gps_init_x)
            narray_y = np.array(gps_init_y)
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
            if len(gps_init_x) > 100:
                gps_init_x.pop(0)
                gps_init_y.pop(0)
            #print (gps_var_x + gps_var_y) / 2
            if (gps_var_x + gps_var_y) / 2 <= 0.05 ** 2:
                init_flag = 1
                init_msg[0] = mean_x
                init_msg[1] = mean_y
                init_msg[2] = gps_yaw
                out_msg = list(init_msg)
                m_sumQ = init_msg[2]
                gps_msg_old = list(gps_msg)
                print "init ok!"
                print init_msg

    if gps_flag == 1:
        encode_delta_x = []
        encode_delta_y = []
        encode_delta_yaw = []
        gps_msg_old = list(gps_msg)

def amcl_callback(amclmsg):

    global amcl_msg, amcl_msg_old, amcl_msg_delta, amcl_flag
    global encode_delta_x_amcl,encode_delta_y_amcl,encode_delta_yaw_amcl

    if init_flag:
        amcl_msg[0] = amclmsg.pose.pose.position.x
        amcl_msg[1] = amclmsg.pose.pose.position.y
        orientation = [amclmsg.pose.pose.orientation.x, amclmsg.pose.pose.orientation.y, amclmsg.pose.pose.orientation.z,
                       amclmsg.pose.pose.orientation.w]
        (amcl_roll, amcl_pitch, amcl_yaw) = euler_from_quaternion(orientation)
        amcl_msg[2] = align(amcl_yaw)

        amcl_msg_delta[0] = amcl_msg[0] - amcl_msg_old[0]
        amcl_msg_delta[1] = amcl_msg[1] - amcl_msg_old[1]
        amcl_msg_delta[2] = do_yaw(amcl_msg[2] - amcl_msg_old[2])

        if amcl_flag == 1:
            encode_delta_x_amcl = []
            encode_delta_y_amcl = []
            encode_delta_yaw_amcl = []
            amcl_msg_old = list(amcl_msg)

def encode_cb(encodemsg):

    global m_sumQ, m_sumX, m_sumY, m_tranX, m_tranY, m_rotation
    global out_msg, encode_delta_x, encode_delta_y, encode_delta_yaw, encode_delta_x_amcl, encode_delta_y_amcl, encode_delta_yaw_amcl

    if init_flag:

        m_length = 0.553
        PulseToDistance = math.pi * 0.10185916357881302 / 2000

        thetaL = encodemsg.left_encode
        thetaR = encodemsg.right_encode

        m_tranX = (thetaL + thetaR) * PulseToDistance * math.cos(+m_sumQ) / 2.0
        m_tranY = (thetaL + thetaR) * PulseToDistance * math.sin(+m_sumQ) / 2.0

        m_rotation = (thetaR - thetaL) * PulseToDistance / m_length

        encode_delta_x.append(m_tranX)
        encode_delta_y.append(m_tranY)
        encode_delta_yaw.append(m_rotation)

        encode_delta_x_amcl.append(m_tranX)
        encode_delta_y_amcl.append(m_tranY)
        encode_delta_yaw_amcl.append(m_rotation)

        # (m_tranX, m_tranY, m_rotation) = cal_motion(thetaL, thetaR)
        # sumTranslation(m_tranX, m_tranY, m_rotation)

        m_sumQ = out_msg[2] + m_rotation
        m_sumQ = do_yaw(m_sumQ)

def odom_callback(odommsg):

    global odom_msg, odom_msg_old, odom_msg_delta, out_msg, out_msg_old
    global encode_delta_x,encode_delta_y,encode_delta_yaw,encode_delta_x_amcl,encode_delta_y_amcl,encode_delta_yaw_amcl
    global odom_amcl_x,odom_amcl_y,odom_amcl_yaw,odom_gps_x,odom_gps_y,odom_gps_yaw,odom_out_x,odom_out_y,odom_out_yaw
    global constraint_r, amcl_flag, gps_flag, odom_fix ,odom_filter
    global n, m_tranX, m_tranY, m_rotation

    if init_flag:

        tmp_msg_x = odommsg.pose.pose.position.x
        tmp_msg_y = odommsg.pose.pose.position.y
        tmp_orientation = [odommsg.pose.pose.orientation.x, odommsg.pose.pose.orientation.y, odommsg.pose.pose.orientation.z,
                       odommsg.pose.pose.orientation.w]
        (tmp_msg_roll, tmp_msg_pitch, tmp_msg_yaw) = euler_from_quaternion(tmp_orientation)

        tmp_base_mat44 = np.dot(transformations.translation_matrix((tmp_msg_x,tmp_msg_y,0)),
                                transformations.quaternion_matrix(quaternion_from_euler(0, 0, tmp_msg_yaw)))

        odom_tmp_mat44 = np.dot(transformations.translation_matrix((init_msg[0],init_msg[1],0)),
                                 transformations.quaternion_matrix(quaternion_from_euler(0, 0, init_msg[2])))
        # filter_base_mat44 is the new pose in target_frame as a 4x4
        odom_base_mat44 = np.dot(odom_tmp_mat44, tmp_base_mat44)

        odom_xyz = tuple(transformations.translation_from_matrix(odom_base_mat44))[:3]
        odom_quat = tuple(transformations.quaternion_from_matrix(odom_base_mat44))

        odom_fix.header.stamp = rospy.Time.now()
        odom_fix.header.frame_id = "odom"
        odom_fix.child_frame_id = "base_footprint"

        # odom_fix.pose.pose.position.x = odom_xyz[0]
        # odom_fix.pose.pose.position.y = odom_xyz[1]
        # odom_fix.pose.pose.position.z = 0
        # odom_fix.pose.pose.orientation.x = odom_quat[0]
        # odom_fix.pose.pose.orientation.y = odom_quat[1]
        # odom_fix.pose.pose.orientation.z = odom_quat[2]
        # odom_fix.pose.pose.orientation.w = odom_quat[3]
        #
        # pub_fix.publish(odom_fix)
        #
        # odom_msg[0] = odom_xyz[0]
        # odom_msg[1] = odom_xyz[1]
        # odom_orientation = odom_quat
        # (odom_roll, odom_pitch, odom_yaw) = euler_from_quaternion(odom_orientation)
        # odom_msg[2] = odom_yaw

        odom_fix.pose.pose.position.x = odom_xyz[0]
        odom_fix.pose.pose.position.y = odom_xyz[1]
        odom_fix.pose.pose.position.z = 0
        odom_fix.pose.pose.orientation.x = odom_quat[0]
        odom_fix.pose.pose.orientation.y = odom_quat[1]
        odom_fix.pose.pose.orientation.z = odom_quat[2]
        odom_fix.pose.pose.orientation.w = odom_quat[3]

        pub_fix.publish(odom_fix)

        odom_msg[0] = odom_xyz[0]
        odom_msg[1] = odom_xyz[1]
        odom_orientation = odom_quat
        (odom_roll, odom_pitch, odom_yaw) = euler_from_quaternion(odom_orientation)
        odom_msg[2] = align(odom_yaw)
        print odom_msg[2]
        plt_data3.append(odom_msg[2])
        plt_data3_x.append(odom_msg[0])
        plt_data3_y.append(odom_msg[1])
        plt_data8.append(rospy.get_time())

        #odom_gps_delta_x.append(odom_msg[0] - odom_msg_old[0])
        #odom_gps_delta_y.append(odom_msg[1] - odom_msg_old[1])
        #odom_gps_delta_yaw.append(delta_range(odom_msg[2] - odom_msg_old[2]))
        # print odom_gps_delta_yaw
        odom_gps_x = gps_msg[0] + sum(encode_delta_x)
        odom_gps_y = gps_msg[1] + sum(encode_delta_y)
        odom_gps_yaw = do_yaw(gps_msg[2] + sum(encode_delta_yaw))
        print odom_gps_yaw
        plt_data2.append(odom_gps_yaw)
        plt_data2_x.append(odom_gps_x)
        plt_data2_y.append(odom_gps_y)

        # odom_amcl_delta_x.append(odom_msg[0]-odom_msg_old[0])
        # odom_amcl_delta_y.append(odom_msg[1] - odom_msg_old[1])
        # odom_amcl_delta_yaw.append(delta_range(odom_msg[2] - odom_msg_old[2]))
        odom_amcl_x = amcl_msg[0] + sum(encode_delta_x_amcl)
        odom_amcl_y = amcl_msg[1] + sum(encode_delta_y_amcl)
        odom_amcl_yaw = do_yaw(amcl_msg[2] + sum(encode_delta_yaw_amcl))

        odom_out_x = out_msg[0] + m_tranX
        odom_out_y = out_msg[1] + m_tranY
        odom_out_yaw = do_yaw(out_msg[2] + m_rotation)
        print odom_out_yaw
        #print odom_out_yaw,delta_range(odom_msg[2] - odom_msg_old[2]),out_msg[0]

        ## Status GPS Lidar Return  ###############
        #           0   0      0
        #           0   1      1
        #           1   0      2
        #           1   1      3
        ############################################

        if (gps_msg_delta[0] - sum(encode_delta_x))**2 + (gps_msg_delta[1]-sum(encode_delta_y))**2 <= constraint_r ** 2:
            if (amcl_msg_delta[0] - sum(encode_delta_x))**2 + (amcl_msg_delta[1]-sum(encode_delta_y))**2 <= constraint_r ** 2:
                nav_flag = 3
            else:
                nav_flag = 2
        else:
            if (amcl_msg_delta[0] - sum(encode_delta_x))**2 + (amcl_msg_delta[1]-sum(encode_delta_y))**2 <= constraint_r ** 2:
                nav_flag = 1
            else:
                nav_flag = 0

        #print gps_msg_delta[0],sum(odom_gps_delta_x),gps_msg_delta[1],sum(odom_gps_delta_y)
        #print amcl_msg_delta[0],sum(odom_amcl_delta_x),amcl_msg_delta[1],sum(odom_amcl_delta_y)
        #print nav_flag,(gps_msg_delta[0] - sum(odom_gps_delta_x))**2 + (gps_msg_delta[1]-sum(odom_gps_delta_y))**2,(amcl_msg_delta[0] - sum(odom_amcl_delta_x))**2 + (amcl_msg_delta[1]-sum(odom_amcl_delta_y))**2

        #nav_flag = 3
        #print nav_flag

        if nav_flag == 3:
            out_msg[0] = odom_gps_x
            out_msg[1] = odom_gps_y
            out_msg[2] = odom_gps_yaw
            gps_flag = 1
            amcl_flag = 1
            print "!!!!!!!!!! GPS + AMCL !!!!!!!!!!!"

        elif nav_flag == 2:
            out_msg[0] = odom_gps_x
            out_msg[1] = odom_gps_y
            out_msg[2] = odom_gps_yaw
            gps_flag = 1
            amcl_flag = 0
            print "!!!!!!!!!! GPS !!!!!!!!!!!!!!!!!!"

        elif nav_flag == 1:
            # out_msg[0] = odom_amcl_x
            # out_msg[1] = odom_amcl_y
            # out_msg[2] = odom_amcl_yaw
            out_msg[0] = odom_out_x
            out_msg[1] = odom_out_y
            out_msg[2] = odom_out_yaw
            gps_flag = 0
            amcl_flag = 1
            print "!!!!!!!!!!!!!!!! AMCL !!!!!!!!!!!"

        else:
            out_msg[0] = odom_out_x
            out_msg[1] = odom_out_y
            out_msg[2] = odom_out_yaw
            gps_flag = 0
            amcl_flag = 0
            print "!!! ODOM !!!!!!!!!!!!!!!!!!!!!!!!"

        plt_data.append(out_msg[2])
        plt_data_x.append(out_msg[0])
        plt_data_y.append(out_msg[1])
        n += 1
        if n > 100000000:
            fig = plt.figure(1)
            ax = fig.add_subplot(111)
            plot1 = ax.plot(plt_data8, plt_data, label='filter')
            plot2 = ax.plot(plt_data9, plt_data1, label='gps')
            plot3 = ax.plot(plt_data8, plt_data2, label='gps_odom')
            plot4 = ax.plot(plt_data8, plt_data3, label='odom')
            plt.legend(loc='lower right')
            plt.title('yaw')

            fig2 = plt.figure(2)
            ax2 = fig2.add_subplot(111)
            plot1 = ax2.plot(plt_data8, plt_data_x, label='filter')
            plot2 = ax2.plot(plt_data9, plt_data1_x, label='gps')
            plot3 = ax2.plot(plt_data8, plt_data2_x, label='gps_odom')
            plot4 = ax2.plot(plt_data8, plt_data3_x, label='odom')
            plt.legend(loc='lower left')
            plt.title('x')

            fig3 = plt.figure(3)
            ax3 = fig3.add_subplot(111)
            plot1 = ax3.plot(plt_data8, plt_data_y, label='filter')
            plot2 = ax3.plot(plt_data9, plt_data1_y, label='gps')
            plot3 = ax3.plot(plt_data8, plt_data2_y, label='gps_odom')
            plot4 = ax3.plot(plt_data8, plt_data3_y, label='odom')
            plt.legend(loc='lower left')
            plt.title('y')

            plt.show()
            n = 0
        print n

        #print out_msg
        odom_filter.header.stamp = rospy.Time.now()
        odom_filter.header.frame_id = "odom"
        odom_filter.child_frame_id = "base_footprint"

        quat = quaternion_from_euler(0, 0, out_msg[2])
        odom_filter.pose.pose.position.x = out_msg[0]
        odom_filter.pose.pose.position.y = out_msg[1]
        odom_filter.pose.pose.position.z = 0
        odom_filter.pose.pose.orientation.x = quat[0]
        odom_filter.pose.pose.orientation.y = quat[1]
        odom_filter.pose.pose.orientation.z = quat[2]
        odom_filter.pose.pose.orientation.w = quat[3]

        pub_filter.publish(odom_filter)

        #DEBUG
        gps_flag = 1
        amcl_flag = 1

        odom_msg_old = list(odom_msg)

        # map_base_trans = (out_msg[0], out_msg[1], 0)
        # map_base_quat = quaternion_from_euler(0, 0, out_msg[2])
        #
        # # try:
        # #     (amcl_trans, amcl_rot) = tf_listener.lookupTransform('/amcl', '/base_footprint', rospy.Time(0))
        # # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        # #     return 1
        # # # /laser_map-/base_footprint
        # # amcl_af_trans = amcl_trans
        # # amcl_af_quat = amcl_rot
        #
        # # /base_footprint - /odom
        # try:
        #     (base_odom_trans, base_odom_quat) = tf_listener.lookupTransform('/base_footprint', '/odom', rospy.Time(0))
        # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #     return 1
        #
        # map_base_mat44 = np.dot(transformations.translation_matrix(map_base_trans),
        #                         transformations.quaternion_matrix(map_base_quat))
        #
        # base_odom_mat44 = np.dot(transformations.translation_matrix(base_odom_trans),
        #                          transformations.quaternion_matrix(base_odom_quat))
        # # txpose is the new pose in target_frame as a 4x4
        # txpose = np.dot(map_base_mat44, base_odom_mat44)
        #
        # xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
        # quat = tuple(transformations.quaternion_from_matrix(txpose))
        #
        # br = tf.TransformBroadcaster()
        # br.sendTransform(xyz, quat, rospy.Time.now(), "odom", "map")


        # /map-/base_footprint
        # print robot_odom
        robot_trans = tf.TransformBroadcaster()
        # print robot_x + m_tranX
        robot_trans.sendTransform((out_msg[0], out_msg[1], 0),
                                  quaternion_from_euler(0, 0, out_msg[2]),
                                  rospy.Time.now(),
                                  "base_footprint",
                                  "odom")

        br = tf.TransformBroadcaster()
        # print robot_x + m_tranX
        br.sendTransform((0, 0, 0),
                         quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "odom",
                         "map")
        # last_time = current_time


def ago_go():
    # print "nav filter"
    global tf_listener, pub_filter, pub_fix
    rospy.init_node('ago_go_node', anonymous=True)
    tf_listener = tf.TransformListener()
    rospy.Subscriber('/odom/gps', Odometry, gps_callback)
    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amcl_callback)
    rospy.Subscriber('robot_encode_val', Robot_encode, encode_cb)
    pub_filter = rospy.Publisher('/odom/filter', Odometry, queue_size=10)
    pub_fix = rospy.Publisher('/odom/fix', Odometry, queue_size=10)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    ago_go()

