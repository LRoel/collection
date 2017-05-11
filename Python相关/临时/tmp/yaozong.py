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
from af_msgs.msg import Fuse_G_O
from af_msgs.msg import RobotState


def serial_process():
	serial_ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1)
	serial_ser.flushOutput()
	serial_ser.flushInput()
	print "in serial process"

	while not rospy.is_shutdown():
		ser_1 = serial_ser.read(1)
		if ser_1 == '\x68':
			if serial_ser.read(1) == '\x0d':
				encoder_a = serial_ser.read(12)
				# print encoder_a

				if len(encoder_a) == 12:
					# print "i am in"
					encoder_b = struct.unpack('12B', encoder_a)
					crc = encoder_b[-1]
					crc_split = struct.unpack("<12B", '\x0d' + encoder_a[0:-1])
					encoder_crc = sum(crc_split) & 0xff
					if encoder_crc == crc:
						try:
							# a = str(hex(encoder_b[-4])).replace('0x','')
							# i_a = int(a)*100
							# b = str(hex(encoder_b[-3])).replace('0x','')
							# i_b = int(b)
							# c = str(hex(encoder_b[-2])).replace('0x','')
							# i_c = int(c) /100.0

							a = (encoder_b[-4] >> 4 & 0xf) * 10 + (encoder_b[-4] & 0xf)
							b = (encoder_b[-3] >> 4 & 0xf) * 10 + (encoder_b[-3] & 0xf)
							c = (encoder_b[-2] >> 4 & 0xf) * 10 + (encoder_b[-2] & 0xf)
							num = a * 100 + b + c/100.0
							print num

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
		serial_process()
	except rospy.ROSInterruptException:
		pass

