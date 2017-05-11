#!/usr/bin/env python

import rospy
import serial
import sys
import string

import binascii

from struct import * 
from std_msgs.msg import String

port = '/dev/ttyUSB0'

try:
    ser = serial.Serial(port=port, baudrate=2400, timeout=1)
except serial.serialutil.SerialException:
    print("Camera not found at port " + port + ". Did you specify the correct port in the launch file?")
    # exit
    sys.exit(0)


def str2num(string):
    try:
        return int(string)
    except:
        return None


def cameraCallback(camera_msg):
    set = camera_msg.data.split(' ')
    # s[1] taken into pre-set positions as num
    # Composing protocal command
    # Writing num to camera port
    print set[1]
    num = (str2num(set[1]) % 360) // 10 + 1
    if num == 37:
        num = 1

    # flip left-right
#    num = 38 - num

    if num == 33:
        num = 40
    buffer = '\xff\x01\x00\x07\x00'
    buffer += chr(num)
    cs = (0x01 + 0x07 + num) & 0xff
    print chr(cs)
    buffer += chr(cs)
    print binascii.b2a_hex(buffer)
    for i in range(10):
        ser.write(buffer)
    
    

def main():

#    ser.flush()
#    ser.timeout = 0.01
    
    rospy.init_node('camera', anonymous=True)
    camera_sub = rospy.Subscriber('/camera', String, cameraCallback)


    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        rospy.spin()
        rate.sleep()

if __name__ == '__main__':
    main()
