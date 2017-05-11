#!/usr/bin/env python

import rospy

from sensor_msgs.msg import Imu

show_flag = 1

f1 = open('/home/exbot/comb2.txt','a')

def imu_callback(imuMsg):

    global show_flag

    imuMsg.linear_acceleration.x = -float(words[3]) * accel_factor
    imuMsg.linear_acceleration.y = float(words[4]) * accel_factor
    imuMsg.linear_acceleration.z = float(words[5]) * accel_factor

    imuMsg.linear_acceleration_covariance[0] = 0.0001
    imuMsg.linear_acceleration_covariance[4] = 0.0001
    imuMsg.linear_acceleration_covariance[8] = 0.0001


    imuMsg.angular_velocity.x = float(words[6])
# in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
    imuMsg.angular_velocity.y = -float(words[7])
# in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
    imuMsg.angular_velocity.z = -float(words[8])

    imuMsg.angular_velocity_covariance[0] = 0.0001
    imuMsg.angular_velocity_covariance[4] = 0.0001
    imuMsg.angular_velocity_covariance[8] = 0.0001
    s = str(imuMsg.linear_acceleration.x) + ',' + str(imuMsg.linear_acceleration.y) + ',' + str(
        imuMsg.linear_acceleration.z) + ',' + str(imuMsg.angular_velocity.x) + ',' + str(
        imuMsg.angular_velocity.y) + ',' + str(imuMsg.angular_velocity.z) + '\n'
    f1.write(s)

    rospy.loginfo(show_flag)

    show_flag += 1


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('zhengzhou_node', anonymous=True)

    rospy.Subscriber('/imu/data_raw', Imu, imu_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
while not rospy.is_shutdown():
    listener()

