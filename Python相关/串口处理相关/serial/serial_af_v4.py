#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import struct
import math
import numpy as np
import string
import re
import crcmod
from sensor_msgs.msg import LaserScan


crc_novetal = crcmod.mkCrcFun(0x104C11DB7, 0, True, 0)

from af_msgs.msg import Fuse_G_O

mutex = threading.Lock()

# solution_status = {
#             0: "SOLUTION_STATUS_SOLUTION_COMPUTED",
#             1: Fuse_G_O.SOLUTION_STATUS_INSUFFICIENT_OBSERVATIONS,
#             2: Fuse_G_O.SOLUTION_STATUS_NO_CONVERGENCE,
#             3: Fuse_G_O.SOLUTION_STATUS_SINGULARITY_AT_PARAMETERS_MATRIX,
#             4: Fuse_G_O.SOLUTION_STATUS_COVARIANCE_TRACE_EXCEEDS_MAXIMUM,
#             5: Fuse_G_O.SOLUTION_STATUS_TEST_DISTANCE_EXCEEDED,
#             6: Fuse_G_O.SOLUTION_STATUS_COLD_START,
#             7: Fuse_G_O.SOLUTION_STATUS_VELOCITY_OR_HEIGHT_LIMIT_EXCEEDED,
#             8: Fuse_G_O.SOLUTION_STATUS_VARIANCE_EXCEEDS_LIMITS,
#             9: Fuse_G_O.SOLUTION_STATUS_RESIDUALS_TOO_LARGE,
#             10: Fuse_G_O.SOLUTION_STATUS_INVALID_FIX,
#             11: Fuse_G_O.SOLUTION_STATUS_UNAUTHORIZED,
#         }
# position_type_to_status = {
#             0: Fuse_G_O.POSITION_TYPE_NONE,
#             1: Fuse_G_O.POSITION_TYPE_FIXED,
#             2: Fuse_G_O.POSITION_TYPE_FIXEDHEIGHT,
#             4: Fuse_G_O.POSITION_TYPE_FLOATCONV,
#             5: Fuse_G_O.POSITION_TYPE_WIDELANE,
#             6: Fuse_G_O.POSITION_TYPE_NARROWLANE,
#             8: Fuse_G_O.POSITION_TYPE_DOPPLER_VELOCITY,
#             16: Fuse_G_O.POSITION_TYPE_SINGLE,
#             17: Fuse_G_O.POSITION_TYPE_PSRDIFF,
#             18: Fuse_G_O.POSITION_TYPE_WAAS,
#             19: Fuse_G_O.POSITION_TYPE_PROPAGATED,
#             20: Fuse_G_O.POSITION_TYPE_OMNISTAR,
#             32: Fuse_G_O.POSITION_TYPE_L1_FLOAT,
#             33: Fuse_G_O.POSITION_TYPE_IONOFREE_FLOAT,
#             34: Fuse_G_O.POSITION_TYPE_NARROW_FLOAT,
#             48: Fuse_G_O.POSITION_TYPE_L1_INT,
#             49: Fuse_G_O.POSITION_TYPE_WIDE_INT,
#             50: Fuse_G_O.POSITION_TYPE_NARROW_INT,
#             51: Fuse_G_O.POSITION_TYPE_RTK_DIRECT_INS,
#             52: Fuse_G_O.POSITION_TYPE_INS_SBAS,
#             53: Fuse_G_O.POSITION_TYPE_INS_PSRSP,
#             54: Fuse_G_O.POSITION_TYPE_INS_PSRDIFF,
#             55: Fuse_G_O.POSITION_TYPE_INS_RTKFLOAT,
#             56: Fuse_G_O.POSITION_TYPE_INS_RTKFIXED,
#             57: Fuse_G_O.POSITION_TYPE_INS_OMNISTAR,
#             58: Fuse_G_O.POSITION_TYPE_INS_OMNISTAR_HP,
#             59: Fuse_G_O.POSITION_TYPE_INS_OMNISTAR_XP,
#             64: Fuse_G_O.POSITION_TYPE_OMNISTAR_HP,
#             65: Fuse_G_O.POSITION_TYPE_OMNISTAR_XP,
#             68: Fuse_G_O.POSITION_TYPE_PPP_CONVERGING,
#             69: Fuse_G_O.POSITION_TYPE_PPP,
#             70: Fuse_G_O.POSITION_TYPE_OPERATIONAL,
#             71: Fuse_G_O.POSITION_TYPE_WARNING,
#             72: Fuse_G_O.POSITION_TYPE_OUT_OF_BOUNDS,
#             73: Fuse_G_O.POSITION_TYPE_INS_PPP_CONVERGING,
#             74: Fuse_G_O.POSITION_TYPE_INS_PPP,
#         }


class Fuse_GPS_Odom:
    def __init__(self):
        self.fuse_gps = GPS()
        self.fuse_encoder = Encoder()
        self.fuse_gps_odom_msg = Fuse_G_O()
        self.fill()

    def fill(self):
        self.fuse_gps_odom_msg.Slo_stat = self.fuse_gps.Slo_stat
        self.fuse_gps_odom_msg.Pos_type = self.fuse_gps.Pos_type
        self.fuse_gps_odom_msg.lat = self.fuse_gps.lat
        self.fuse_gps_odom_msg.lat_std = self.fuse_gps.lat_std
        self.fuse_gps_odom_msg.lon = self.fuse_gps.lon
        self.fuse_gps_odom_msg.lon_std = self.fuse_gps.lon_std
        self.fuse_gps_odom_msg.hgt = self.fuse_gps.hgt
        self.fuse_gps_odom_msg.hgt_std = self.fuse_gps.hgt_std
        self.fuse_gps_odom_msg.SVs = self.fuse_gps.SVs
        self.fuse_gps_odom_msg.solnSVs = self.fuse_gps.solnSVs
        self.fuse_gps_odom_msg.heading = self.fuse_gps.heading
        self.fuse_gps_odom_msg.hdg_std = self.fuse_gps.hdg_std
        self.fuse_gps_odom_msg.hor_spd = self.fuse_gps.hor_spd
        self.fuse_gps_odom_msg.Trk_gnd = self.fuse_gps.Trk_gnd
        self.fuse_gps_odom_msg.Vert_spd = self.fuse_gps.Vert_spd
        self.fuse_gps_odom_msg.gps_time = self.fuse_gps.gps_time

        self.fuse_gps_odom_msg.left_encoder_val = self.fuse_encoder.left_encoder_val
        self.fuse_gps_odom_msg.right_encoder_val = self.fuse_encoder.right_encoder_val
        self.fuse_gps_odom_msg.encoder_time = self.fuse_encoder.encoder_time


class GPS:
    def __init__(self):
        self.Slo_stat = 0
        self.Pos_type = 0
        self.lat = 0
        self.lat_std = 0
        self.lon = 0
        self.lon_std = 0
        self.hgt = 0
        self.hgt_std = 0
        self.SVs = 0
        self.solnSVs = 0
        self.heading = 0
        self.hdg_std = 0
        self.hor_spd = 0
        self.Trk_gnd = 0
        self.Vert_spd = 0
        self.gps_time = 0

    def plot(self):
        print "Slo_stat" + str(self.Slo_stat)
        print "Pos_type" + str(self.Pos_type)
        print "lat" + str(self.lat)
        print "lat_std"+str(self.lat_std)
        print "lon"+str(self.lon)
        print "lon_std"+str(self.lon_std)
        print "hgt"+str(self.hgt)
        print "hgt_std"+str(self.hgt_std)
        print "SVs"+str(self.SVs)
        print "solnSVs"+str(self.solnSVs)
        print "heading"+str(self.heading)
        print "hdg_std"+str(self.hdg_std)
        print "hor_spd"+str(self.hor_spd)
        print "Trk_gnd"+str(self.Trk_gnd)
        print "Vert_spd"+str(self.Vert_spd)
        print "gps_time"+str(self.gps_time)


class Encoder:
    def __init__(self):
        self.left_encoder_val = 0
        self.right_encoder_val = 0
        self.encoder_time = 0

    def plot(self):
        print "left_encoder_val", str(self.left_encoder_val)
        print "right_encoder_val", str(self.right_encoder_val)
        print "encoder_time", str(self.encoder_time)


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

'''
Field#	Field name	Field type	Data description	Bin bytes	Bin Offest
1	Sync	Ucahr	0x86	1	0
2	Data_lenth	uchar	0x0a	1	1
3	Encoder_left	short	Encoder	2	2
4	Encoder_right	Short	Encoder	2	4
5	Time	Uint	Time stamps	4	6
6	Check_sum	Uchar	Checksum	1	10
'''

sound_range = LaserScan()
sound_seq = 0
sound_range.range_max = 3.50
sound_range.range_min = 0.290
sound_range.scan_time = 1.0
sound_range.time_increment = 0.0
sound_range.angle_increment = 0.05
sound_range.angle_min = - 30 * math.pi / 180
sound_range.angle_max = 30 * math.pi / 180
m_ = np.arange(sound_range.angle_min, sound_range.angle_max, sound_range.angle_increment)
size_ = len(m_)


def range_pub(range_msg):
    global sound_seq
    sound_range.header.stamp = rospy.Time.now()
    sound_range.header.seq = sound_seq
    sound_range.header.frame_id = 'range1'
    sound_range.ranges = size_ * [range_msg[0] / 1000.0]
    pub_range_1.publish(sound_range)
    sound_range.header.frame_id = 'range2'
    sound_range.ranges = size_ * [range_msg[1] / 1000.0]
    pub_range_2.publish(sound_range)
    sound_range.header.frame_id = 'range3'
    sound_range.ranges = size_ * [range_msg[2] / 1000.0]
    pub_range_3.publish(sound_range)
    sound_range.header.frame_id = 'range4'
    sound_range.ranges = size_ * [range_msg[3] / 1000.0]
    pub_range_4.publish(sound_range)
    sound_range.header.frame_id = 'range5'
    sound_range.ranges = size_ * [range_msg[4] / 1000.0]
    pub_range_5.publish(sound_range)
    sound_range.header.frame_id = 'range6'
    sound_range.ranges = size_ * [range_msg[5] / 1000.0]
    pub_range_6.publish(sound_range)
    sound_range.header.frame_id = 'range7'
    sound_range.ranges = size_ * [range_msg[6] / 1000.0]
    pub_range_7.publish(sound_range)
    sound_range.header.frame_id = 'range8'
    sound_range.ranges = size_ * [range_msg[7] / 1000.0]
    pub_range_8.publish(sound_range)
    sound_seq += 1


def serial_process(port='/dev/ttyUSB0', baudrate=460800, timeout=1):
    serial_ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    serial_ser.flushOutput()
    serial_ser.flushInput()
    encoder = Encoder()
    gps = GPS()
    print "in serial process"

    while not rospy.is_shutdown():
        ser_1 = serial_ser.read(1)
        if ser_1 == '\x86':
            if serial_ser.read(1) == '\x0a':
                encoder_a = serial_ser.read(9)
                # print '\x86\x0a' + encoder_a
                if len(encoder_a) == 9:
                    encoder_b = struct.unpack('<2hIB', encoder_a)
                    crc = encoder_b[3]
                    crc_split = struct.unpack("<9B", '\x0a' + encoder_a[0:-1])
                    encoder_crc = sum(crc_split) & 0xff
                    if encoder_crc == crc:
                        try:
                            encoder.left_encoder_val = encoder_b[0]
                            encoder.right_encoder_val = encoder_b[1]
                            encoder.encoder_time = encoder_b[2]
                            fuse_gps_odom.fuse_encoder = encoder
                            # print "encoder_filled"
                            fuse_gps_odom.fill()
                            pub.publish(fuse_gps_odom.fuse_gps_odom_msg)
                            # print "pub ok"
                        except:
                            print "encoder serial error"
                    else:
                        continue
                else:
                    continue
            else:
                continue
        elif ser_1 == '\xaa':
            # print 'aa'
            if serial_ser.read(1) == '\x44':
                # print '44'
                if serial_ser.read(1) == '\x12':
                    # print '12'
                    gps_a = serial_ser.read(86)
                    if len(gps_a) == 86:
                        gps_b = struct.unpack('<2I3d3f2B2f3dI4B', gps_a)
                        crc = (gps_b[19] << 24) | (gps_b[18] << 16) | (gps_b[17] << 8) | (gps_b[16])
                        crc_split = '\xaa' + '\x44' + '\x12' + gps_a[0:82]
                        gps_crc = crc_novetal(crc_split)
                        if gps_crc == crc:
                            try:
                                gps.Slo_stat = gps_b[0]
                                gps.Pos_type = gps_b[1]
                                gps.lat = gps_b[2]
                                gps.lon = gps_b[3]
                                gps.hgt = gps_b[4]
                                gps.lat_std = gps_b[5]
                                gps.lon_std = gps_b[6]
                                gps.hgt_std = gps_b[7]
                                gps.SVs = gps_b[8]
                                gps.solnSVs = gps_b[9]
                                gps.heading = gps_b[10]
                                gps.hdg_std = gps_b[11]
                                gps.hor_spd = gps_b[12]
                                gps.Trk_gnd = gps_b[13]
                                gps.Vert_spd = gps_b[14]
                                gps.gps_time = gps_b[15]
                                fuse_gps_odom.fuse_gps = gps
                                # fuse_gps_odom.fill()
                                # pub.publish(fuse_gps_odom.fuse_gps_odom_msg)
                                # print "gps filled"
                            except:
                                print "gps serial error"
                        else:
                            continue
                    else:
                        continue
                else:
                    continue
            else:
                continue
        elif ser_1 == '\x88':
            if serial_ser.read(1) == '\x13':
                range_a = serial_ser.read(17)
                if len(range_a) == 17:
                    range_b = struct.unpack('<8HB', range_a)
                    crc = range_b[8]
                    crc_split = struct.unpack("<17B", '\x13' + range_a[0:-1])
                    range_crc = sum(crc_split) & 0xff
                    if range_crc == crc:
                        try:
                            # encoder.left_encoder_val = encoder_b[0]
                            # encoder.right_encoder_val = encoder_b[1]
                            # encoder.encoder_time = encoder_b[2]
                            # fuse_gps_odom.fuse_encoder = encoder
                            range_pub(range_b[0:8])
                            # print "range_filled"
                            # fuse_gps_odom.fill()
                            # pub.publish(fuse_gps_odom.fuse_gps_odom_msg)
                            # print "pub ok"
                        except:
                            print "encoder serial error"
                    else:
                        continue
                else:
                    continue
            else:
                continue
        else:
            print "!!!!!!!!!!!!!!!!!!!!"
            continue


if __name__ == '__main__':
    try:
        rospy.init_node('serial_fuse', anonymous=True)
        pub = rospy.Publisher('/fuse', Fuse_G_O, queue_size=100)
        pub_range_1 = rospy.Publisher('/range/1', LaserScan, queue_size=100)
        pub_range_2 = rospy.Publisher('/range/2', LaserScan, queue_size=100)
        pub_range_3 = rospy.Publisher('/range/3', LaserScan, queue_size=100)
        pub_range_4 = rospy.Publisher('/range/4', LaserScan, queue_size=100)
        pub_range_5 = rospy.Publisher('/range/5', LaserScan, queue_size=100)
        pub_range_6 = rospy.Publisher('/range/6', LaserScan, queue_size=100)
        pub_range_7 = rospy.Publisher('/range/7', LaserScan, queue_size=100)
        pub_range_8 = rospy.Publisher('/range/8', LaserScan, queue_size=100)

        serial_process()
        # TODO 线程读取串口
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
