#!/usr/bin/env python


import rospy
import serial
import sys
import struct
import string


# cs = 0
# css = 0

# from std_msgs.msg import UInt8MultiArray
from af_msgs.msg import AfStatus

# def checksum(nmea_str0):
#     nmea_str = nmea_str0[0:-5]
#     return reduce(operator.xor, map(ord, nmea_str), 0)

port = '/dev/ttyS0'

try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    print("IMU not found at port " + port + ". Did you specify the correct port in the launch file?")
    # exit
    sys.exit(0)


def talker():

    pub = rospy.Publisher('Af_status', AfStatus, queue_size=1)
    rospy.init_node('Af_status_node', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    status = AfStatus()

    seq = 0

    while not rospy.is_shutdown():
        ser_data = ser.read(1)
        status.header.stamp = rospy.Time.now()
        status.header.frame_id = ''
        status.header.seq = seq
        pub.publish(status)

        if ser_data == '\x85':
            # print "85"
            if ser.read(1) == '\x16':
                s = ser.read(21)
                if len(s) == 21:
                    data = struct.unpack(">10HB", s)
                    sonar = data[0:10]
                    cs = data[-1]

                    css_tmp = struct.unpack(">21B", s)
                    css = sum(css_tmp[0:-1]) & 0xff

                    if cs == css:
                        status.sonar = sonar
                        print "sonar",sonar

            else:
                continue

        elif ser_data == '\x87':
            # print "87"
            if ser.read(1) == '\x09':
                s = ser.read(8)
                # print s
                if (len(s) == 8):
                    data = struct.unpack("<HBHHB", s)
                    dianya = data[0]
                    dianliang = data[1]
                    chongdianliu = data[2]
                    fangdianliu = data[3]
                    cs = data[-1]

                    css_tmp = struct.unpack(">8B", s)
                    css = sum(css_tmp[0:-1]) & 0xff

                    if (cs == css):

                        status.voltage = dianya
                        status.electric_quantity = dianliang
                        status.charge_current = chongdianliu
                        status.discharge_current = fangdianliu

                        print ("dianya = %d, dianliang = %d, chongdianliu = %d,fangdianliu = %d" % (dianya, dianliang, chongdianliu, fangdianliu))

            else:
                continue

        elif ser_data == '\x88':
            # print "88"
            if ser.read(1) == '\x06':
                s = ser.read(5)
                # print s
                if (len(s) == 5):
                    data = struct.unpack("<IB", s)
                    # print data
                    if error_tmp != 0:
                        error = 1
                    else:
                        error = 0
                    cs = data[-1]

                    css_tmp = struct.unpack(">5B", s)
                    css = sum(css_tmp[0:-1]) & 0xff

                    if (cs == css):
                        error_status = "%8x"%data[0]
                        status.error_status = error_status
                        status.error = error
                        print "%8x"%data[0]
            else:
                continue

        else:
            continue

        seq = seq + 1

        #print array
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
