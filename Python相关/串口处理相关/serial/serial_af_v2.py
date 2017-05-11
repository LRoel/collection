#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import struct
import string
import re
import crcmod


crc_novetal = crcmod.mkCrcFun(0x104C11DB7, 0, True, 0)

from af_msgs.msg import Fuse_G_O

mutex = threading.Lock()


class Fuse_GPS_Odom:
    def __init__(self):
        self.fuse_gps = GPS()
        self.fuse_encoder = Encoder()
        self.fuse_gps_odom_msg = Fuse_G_O()
        self.fill()

    def fill(self):
        self.fuse_gps_odom_msg.gps_time = self.fuse_gps.gps_time
        self.fuse_gps_odom_msg.longitude = self.fuse_gps.longitude
        self.fuse_gps_odom_msg.latitude = self.fuse_gps.latitude
        self.fuse_gps_odom_msg.longitude_sd = self.fuse_gps.longitude_sd
        self.fuse_gps_odom_msg.latitude_sd = self.fuse_gps.latitude_sd
        self.fuse_gps_odom_msg.height_sd = self.fuse_gps.height_sd
        self.fuse_gps_odom_msg.heading_time = self.fuse_gps.heading_time
        self.fuse_gps_odom_msg.heading = self.fuse_gps.heading
        self.fuse_gps_odom_msg.heading_sd = self.fuse_gps.heading_sd
        self.fuse_gps_odom_msg.gps_cnt = self.fuse_gps.gps_cnt
        self.fuse_gps_odom_msg.left_encoder_val = self.fuse_encoder.left_encoder_val
        self.fuse_gps_odom_msg.right_encoder_val = self.fuse_encoder.right_encoder_val
        self.fuse_gps_odom_msg.encoder_cnt = self.fuse_encoder.encoder_cnt


class GPS:
    def __init__(self):
        self.gps_time = 0
        self.longitude = 0
        self.latitude = 0
        self.longitude_sd = 0
        self.latitude_sd = 0
        self.height_sd = 0
        self.heading_time = 0
        self.heading = 0
        self.heading_sd = 0
        self.gps_cnt = 0

    def plot(self):
        print "gps_time", str(self.gps_time)
        print "longitude", str(self.longitude)
        print "longitude_sd", str(self.longitude_sd)
        print "latitude", str(self.latitude)
        print "latitude_sd", str(self.latitude_sd)
        print "height_sd", str(self.height_sd)
        print "heading_time", str(self.heading_time)
        print "heading", str(self.heading)
        print "heading_sd", str(self.heading_sd)
        print "gps_cnt", str(self.gps_cnt)


class Encoder:
    def __init__(self):
        self.left_encoder_val = 0
        self.right_encoder_val = 0
        self.encoder_cnt = 0

    def plot(self):
        print "left_encoder_val", str(self.left_encoder_val)
        print "right_encoder_val", str(self.right_encoder_val)
        print "encoder_cnt", str(self.encoder_cnt)


fuse_gps_odom = Fuse_GPS_Odom()


'''
Field#	Field name	Field type	Data description	Bin bytes	Bin Offest
1	sync	char	0xaa	1	0
2	sync	char	0x44	1	1
3	sync	char	0x12	1	2
4	Sol stat	Enum	Bestpos	4	3
5	Pos type	Enum	Bestpos	4	7
6	Lat	Double	Bestpos	8	11
7	Lon	Double	Bestpos	8	19
8	Hgt	Double	Bestpos	8	27
9	Lat σ	Float	Bestpos	4	35
10	Lonσ	Float	Bestpos	4	39
11	Hgt σ	    # n = 0
Float	Bestpos	4	43
12	#SVs	Uchar	Bestpos	1	47
13	#solnSVs	Uchar	Bestpos	1	48
14	Heading	Float	Heading	4	49
15	Hdg std dev	Float	Heading	4	53
16	Hor spd	Double	Bestvel	8	57
17	Trk gnd	Double	Bestvel	8	65
18	Vert spd	Double	Bestvel	8	73
19	Time	Uint	Time stamps	4	81
20	Crc	Uint	Crc	4	85
'''


def gps_process(port='/dev/ttyUSB1', baudrate=460800, timeout=1):
    gps_ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    gps_ser.flushOutput()
    gps_ser.flushInput()
    gps = GPS()
    # gps_pat = re.compile(r'[0-9.]+')
    print "in gps process"

    while not rospy.is_shutdown():
        if gps_ser.read(1) == '\xaa':
            if gps_ser.read(1) == '\x44':
                if gps_ser.read(1) == '\x12':
                    gps_a = gps_ser.read(86)
                else:
                    continue
            else:
                continue
        else:
            continue
        if len(gps_a) == 86:
            gps_b = struct.unpack('>2I3d3f2B2f3dI4B', gps_a)
            crc = (gps_b[19]<<24)|(gps_b[18]<<16)|(gps_b[17]<<8)|(gps_b[16])
            crc_split = '\xaa'+'\x44'+'\x12'+gps_a[0:82]
            gps_crc = crc_novetal(crc_split)
            if gps_crc == crc:
                try:
                    gps.gps_time = 0
                    gps.latitude = gps_b[2]
                    gps.longitude = gps_b[3]
                    gps.latitude_sd = gps_b[5]
                    gps.longitude_sd = gps_b[6]
                    gps.height_sd = gps_b[7]
                    gps.heading = gps_b[10]
                    gps.heading_sd = gps_b[11]
                    gps.gps_cnt = gps_b[15]
                    fuse_gps_odom.fuse_gps = gps
                    fuse_gps_odom.fill()
                    pub.publish(fuse_gps_odom.fuse_gps_odom_msg)
                    print "gps filled"
                except:
                    print "gps serial error"
            else:
                continue
        else:
            continue


'''
Field#	Field name	Field type	Data description	Bin bytes	Bin Offest
1	Sync	Ucahr	0x86	1	0
2	Data_lenth	uchar	0x0a	1	1
3	Encoder_left	short	Encoder	2	2
4	Encoder_right	Short	Encoder	2	4
5	Time	Uint	Time stamps	4	6
6	Check_sum	Uchar	Checksum	1	10
'''


def encoder_process(port='/dev/ttyUSB0', baudrate=460800, timeout=1):
    encoder_ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    encoder_ser.flushOutput()
    encoder_ser.flushInput()
    encoder = Encoder()
    print "in encoder process"

    while not rospy.is_shutdown():
        if encoder_ser.read(1) == '\x86':
            if encoder_ser.read(1) == '\x0a':
                encoder_a = encoder_ser.read(9)
                if len(encoder_a) == 9:
                    encoder_b = struct.unpack('>2hIB', encoder_a)
                    crc = encoder_b[3]
                    crc_split = struct.unpack("<10B", '\x86'+'\x0a'+encoder_a[0:-1])
                    encoder_crc = sum(crc_split) & 0xff
                    if encoder_crc == crc:
                        try:
                            encoder.left_encoder_val = encoder_b[0]
                            encoder.right_encoder_val = encoder_b[1]
                            encoder.encoder_cnt = encoder_b[2]
                            fuse_gps_odom.fuse_encoder = encoder
                            print "encoder_filled"
                            fuse_gps_odom.fill()
                            pub.publish(fuse_gps_odom.fuse_gps_odom_msg)
                            print "pub ok"
                        except:
                            print "encoder serial error"
                    else:
                        continue
                else:
                    continue
            else:
                continue
        else:
            continue


if __name__ == '__main__':
    try:
        rospy.init_node('serial_fuse', anonymous=True)
        pub = rospy.Publisher('/fuse', Fuse_G_O, queue_size=100)
        #t1 = threading.Thread(target=encoder_process)
        t2 = threading.Thread(target=gps_process)
        #t1.start()
        t2.start()
        #t1.join()
        t2.join()
        # TODO 线程读取串口
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
