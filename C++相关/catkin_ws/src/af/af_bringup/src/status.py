#!/usr/bin/env python


import rospy
import serial
import sys
import struct
import string
import tf
import Image

# cs = 0
# css = 0
seq = 0

# from std_msgs.msg import UInt8MultiArray
from af_msgs.msg import RobotState
from std_msgs.msg import Bool

# def checksum(nmea_str0):
#     nmea_str = nmea_str0[0:-5]
#     return reduce(operator.xor, map(ord, nmea_str), 0)

abnormal_detection_flag = 0
init_flag = 0

def init_cb(init_state):
    global init_flag
    init_flag = init_state.data

def abnormal_detection_cb(robot_state):
    global abnormal_detection_flag
    abnormal_detection_flag = robot_state.abnormal_detection

def other_cb(robot_state):
    global abnormal_detection_flag,seq,init_flag
    
    state.sonar = robot_state.sonar
    state.voltage = robot_state.voltage
    state.electric_quantity = robot_state.electric_quantity
    state.charge_current = robot_state.charge_current
    state.discharge_current = robot_state.discharge_current
    state.error_status = robot_state.error_status
    state.error = robot_state.error
    state.vibration_detection = robot_state.vibration_detection
    state.control_lost = robot_state.control_lost
    state.emergency_stop = robot_state.emergency_stop
    state.call_security = robot_state.call_security
    state.sonar_errors = robot_state.sonar_errors
    state.front_collision = robot_state.front_collision
    # state.back_infrared = robot_state.back_infrared
    # state.front_infrared = robot_state.front_infrared
    state.back_infrared = False
    state.front_infrared = False
    state.motor_4_lost = robot_state.motor_4_lost
    state.motor_3_lost = robot_state.motor_3_lost
    state.motor_2_lost = robot_state.motor_2_lost
    state.motor_1_lost = robot_state.motor_1_lost
    state.ros_lost = robot_state.ros_lost
    state.remote_lost = robot_state.remote_lost
    state.deviation_path = robot_state.deviation_path
    state.abnormal_detection = abnormal_detection_flag
    
    state.header.stamp = rospy.Time.now()
    state.header.frame_id = ''
    state.header.seq = seq
    seq += 1
    if init_flag:
    # if 1:
        pub.publish(state)

def state_pub():
    global state,pub
    rospy.init_node('robot_state_node', anonymous=True)
    pub = rospy.Publisher('robot_state', RobotState, queue_size=1)
    state = RobotState()
    rospy.Subscriber('robot_state/abnormal_detection', RobotState, abnormal_detection_cb)
    rospy.Subscriber('robot_state/other', RobotState, other_cb)
    rospy.Subscriber('/cmd_vel/state', Bool, init_cb)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    
if __name__ == '__main__':
    try:
        state_pub()
    except rospy.ROSInterruptException:
        pass


