#!/usr/bin/env python

import rospy
import serial
import sys
import string

from struct import *
from std_msgs.msg import String

port = '/dev/ttyUSB0'

try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    print("Mic array not found at port " + port + ". Did you specify the correct port in the launch file?")
    # exit
    sys.exit(0)

def str2num(string):
    try:
        return int(string)
    except:
        return None


def main():
    ser.flush()
    ser.timeout = 0.01
    # Initialising micArray
    # wake up
    buffer = "HMA\taec\tget\t\t"
    ser.write(buffer) 
    buffer = "HMA\twake_chk\tset\t2\t\t"
    ser.write(buffer)
    rospy.init_node('mic_array', anonymous=True)
    camera_pub = rospy.Publisher('/path_planning/camera', String, queue_size=10)
    processing_time_pub = rospy.Publisher('/path_planning/processing_time', String, queue_size=10)

    #rate = rospy.Rate(1000)
    s = 'H'
    t_start = rospy.get_time()
    t_stop = t_start
    while not rospy.is_shutdown():
        # Reading one byte
        ch = ser.read()
        # Determinging head and tail
        if ch == 'H':
            s = 'H'
            t_start = rospy.get_time()
        elif ch == '\t' and s[-1] == '\t':
            set = s.split('\t')
            print set
            print 'wake_dir' in set
            if 'wake_dir' not in set:
                continue
            t_stop = rospy.get_time()
            print t_stop - t_start
            processing_time_pub.publish(str(t_stop - t_start))
            for ss in set:
                num = str2num(ss)
                if num != None:
                    break
            # Publishing the angle(num) by camera_pub
            ts = "0 " + str(num)
            print ts;
            camera_pub.publish(ts)
        else:
            s += ch
     #   rate.sleep();

if __name__ == '__main__':
    main()
