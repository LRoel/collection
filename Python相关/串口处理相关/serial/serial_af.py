#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import struct
import string
import re
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


def gps_process(port='/dev/ttyUSB2', baudrate=115200, timeout=1):
    gps_ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    gps_ser.flushOutput()
    gps_ser.flushInput()
    gps = GPS()
    gps_pat = re.compile(r'[0-9.]+')
    print "in gps process"

    while not rospy.is_shutdown():

        gps_a = gps_ser.readline()
        
        # print len(gps_a)
        if len(gps_a) == 109:
            gps_b = struct.unpack('<2c13s14s14s8s8s8s10s13s13sHcB2c', gps_a)
            if gps_b[0] == '\x23' and gps_b[1] == '\x64':
                crc = gps_b[13]
                crc_split = struct.unpack("<106B", gps_a[0:106])
                gps_crc = sum(crc_split) & 0xff
                # print gps_b
                if gps_crc == crc:
                    # print gps_b
                    # _gps_tmp = gps_b[2]
                    # _gps_tmp.replace('\x00','')
                    # _gps_tmp.replace('\t', '')
                    # _gps_split = gps_pat.findall(_gps_tmp)
                    # _gps_split = _gps_tmp.split(",")
                    # print len(_gps_split), _gps_split
                    # if len(_gps_split) >= 9:
                    try:
                        gps.gps_time = string.atof(gps_pat.findall(gps_b[2])[0])
                        gps.latitude = string.atof(gps_pat.findall(gps_b[3])[0])
                        gps.longitude = string.atof(gps_pat.findall(gps_b[4])[0])
                        gps.latitude_sd = string.atof(gps_pat.findall(gps_b[5])[0])
                        gps.longitude_sd = string.atof(gps_pat.findall(gps_b[6])[0])
                        gps.height_sd = string.atof(gps_pat.findall(gps_b[7])[0])
                        if gps_pat.findall(gps_b[8]):
                            gps.heading_time = string.atof(gps_pat.findall(gps_b[8])[0])
                        else:
                            gps.heading_time = 0
                        if gps_pat.findall(gps_b[9]):
                            gps.heading = string.atof(gps_pat.findall(gps_b[9])[0])
                        else:
                            gps.heading = 0
                        if gps_pat.findall(gps_b[10]):
                            gps.heading_sd = string.atof(gps_pat.findall(gps_b[10])[0])
                        else:
                            gps.heading_sd = 0
                        gps.gps_cnt = gps_b[11]
                        # mutex.acquire()
                        fuse_gps_odom.fuse_gps = gps
                        print "gps filled"
                        # fuse_gps_odom.fill()
                        # pub.publish(fuse_gps_odom.fuse_gps_odom_msg)
                        # mutex.release()
                    except:
                        print "gps serial error"
                    # else:
                        # continue
                else:
                    continue
            else:
                continue
        else:
            continue


def encoder_process(port='/dev/ttyUSB0', baudrate=115200, timeout=1):
    encoder_ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    encoder_ser.flushOutput()
    encoder_ser.flushInput()
    encoder = Encoder()
    print "in encoder process"

    # n = 0
    while not rospy.is_shutdown():
        if encoder_ser.read(1) == '\x86':
            if encoder_ser.read(1) == '\x09':
                encoder_a = encoder_ser.read(7)
                # print n
                # n += 1
                # print len(encoder_a)
                if len(encoder_a) == 7:
                    encoder_b = struct.unpack('>2h3B', encoder_a)
                    crc = encoder_b[4]
                    crc_split = struct.unpack("<8B", '\x86'+'\x09'+encoder_a[0:6])
                    encoder_crc = sum(crc_split) & 0xff
                    # if encoder_crc != crc:
                        # print '\x86'+'\x09'+encoder_a
		            # print '\x86'+'\x09'+encoder_a
                    if encoder_crc == crc:
                    # if 1:
                        try:
                            encoder.left_encoder_val = encoder_b[0]
                            encoder.right_encoder_val = encoder_b[1]
                            # encoder.encoder_cnt = encoder_b[2]                           
                            encoder.encoder_cnt = (encoder_b[3]<<8)|(encoder_b[2])                        
                            # print encoder.encoder_cnt
                            fuse_gps_odom.fuse_encoder = encoder
                            # print encoder_b[2]
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
        t1 = threading.Thread(target=encoder_process)
        t2 = threading.Thread(target=gps_process)
        t1.start()
        t2.start()
        t1.join()
        t2.join()
        # TODO 线程读取串口
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
