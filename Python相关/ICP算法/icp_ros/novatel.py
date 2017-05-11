# !/usr/bin/env python

import rospy
import serial
import sys
import string
import operator
import math
import threading
import struct

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix
from af_msgs.msg import Novatel


novatel_msg = Novatel()


class Port:
    """ Common base class for DataPort and ControlPort. Provides functionality to
      recv/send novatel-formatted packets from the socket. Could in future
      support LoggingPort and DisplayPort."""
    checksum_struct = struct.Struct("<hh")

    def __init__(self, port, baudrate):
        try:
            self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)
            self.pub = rospy.Publisher('/novatel', Novatel, queue_size=100)
        except serial.serialutil.SerialException:
            print("IMU not found at port " + port + ". Did you specify the correct port in the launch file?")
            # exit
            sys.exit(0)

    def recv(self):
        """ Receive a packet from the port's socket.
        Returns (header, pkt_str)
        Returns None, None when no data. """
        global header_id

        novatel_msg.Heading = 0
        novatel_msg.Heading_sd = 0

        try:
            while not rospy.is_shutdown():
                if self.ser.read() != '#':
                    continue
                line0 = self.ser.readline()
                print line0
                cs = line0[-8:]
                print cs
                # try:
                #     cs1 = int(cs, 16)
                # except:
                #     continue
                # cs2 = checksum(line0)
                # print("cs1,cs2", cs1, cs2)
                # if cs1 != cs2:
                #     continue
                if line0[:9] == "BESTPOSA,":
                    try:
                        header_frame, pose_frame = string.split(line0, ";")
                        header_frame = header_frame.replace("BESTPOSA,", "")
                        header_frame = header_frame.replace("\r\n", "")
                        header_words = string.split(header_frame, ",")

                        novatel_msg.GPSWeek = string.atoi(header_words[4])
                        novatel_msg.GPSTime = string.atof(header_words[5])

                        pose_frame = pose_frame.replace("\r\n", "")
                        pose_frame = pose_frame.replace("*", ",")
                        pose_words = string.split(pose_frame, ",")

                        novatel_msg.Altitude = string.atof(pose_words[4])
                        novatel_msg.Altitude_sd = string.atof(pose_words[9])
                        novatel_msg.Latitude = string.atof(pose_words[2])
                        novatel_msg.Latitude_sd = string.atof(pose_words[7])
                        novatel_msg.Longitude = string.atof(pose_words[3])
                        novatel_msg.Longitude_sd = string.atof(pose_words[8])
                        self.pub.publish(novatel_msg)

                    except:
                        print "bestpos error"

                elif line0[:9] == "HEADINGA,":
                    try:
                        header_frame, pose_frame = string.split(line0, ";")
                        # header_frame = header_frame.replace("HEADINGA,", "")
                        # header_frame = header_frame.replace("\r\n", "")
                        # header_words = string.split(header_frame, ",")
                        pose_frame = pose_frame.replace("\r\n", "")
                        pose_frame = pose_frame.replace("*", ",")
                        pose_words = string.split(pose_frame, ",")

                        novatel_msg.Heading = string.atof(pose_words[3])
                        novatel_msg.Heading_sd = string.atof(pose_words[6])

                    except:
                        print "heading error"
        except:
            print "big error"



    @classmethod
    def _checksum(cls, buff):
        """ Compute novatel checksum. Expects a StringIO with a
      size that is a multiple of four bytes. """
        checksum = 0

        while True:
            data = buff.read(cls.checksum_struct.size)

            if len(data) == 0:
                break
            if len(data) < 4:
                pad_count = len(data) % 4
                data = data + "\x00" * pad_count
                raise ValueError("Checksum data length is not a multiple of 4. %d" % len(data))
            print(data)
            c1, c2 = cls.checksum_struct.unpack(data)
            checksum += c1 + c2
        print(checksum, checksum % 65536)  # novatel 32 bit crc
        return checksum % 65536


# def checksum(nmea_str0):
#     nmea_str = nmea_str0[0:-5]
#     print nmea_str
#     return reduce(operator.xor, map(ord, nmea_str), 0)


def talker():
    rospy.init_node('novatel_node', anonymous=True)
    port = Port('/dev/ttyUSB0', 115200)
    port.recv()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
