#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import binascii

mutex = threading.Lock()


def gps_process(port='/dev/pts/22', baudrate=115200, timeout=1):
    gps_ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    str_send = '23 64 34 36 35 37 30 38 2E 30 30 30 00 00 2C 33 39 2E 39 35 38 38 34 36 37 34 38 35 2C 31 31 36 2E 32 39 35 33 30 30 31 31 36 2C 37 2E 34 35 33 32 11 2C 32 2E 38 30 37 39 09 2C 37 2E 32 38 39 33 07 2C 34 31 38 35 35 37 2E 30 30 2C 37 35 2E 35 36 36 34 34 34 33 39 00 2C 30 2E 31 30 34 39 38 31 31 31 00 00 2C 96 3E 2C 27 0D 0A 23 64 34 36 35 37 30 38 2E 30 30 30 00 00 2C 33 39 2E 39 35 38 38 34 36 37 34 38 35 2C 31 31 36 2E 32 39 35 33 30 30 31 31 36 2C 37 2E 34 35 33 32 2C 2C 32 2E 38 30 37 39 2C 2C 37 2E 32 38 39 33 2C 2C 34 31 38 35 35 37 2E 30 30 2C 37 35 2E 35 36 36 34 34 34 33 39 37 2C 30 2E 31 30 34 39 38 31 31 31 37 00 2C 3F 2D 2C 90 0D 0A'
    buffer_send = str_send.split(' ')
    while 1:
        for i in buffer_send:
            buffer_ready = binascii.a2b_hex(i)
            gps_ser.write(buffer_ready)


def encoder_process(port='/dev/pts/28', baudrate=115200, timeout=1):
    encoder_ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
    str_send = '86 09 00 00 00 00 00 00 8F 86 09 00 00 00 00 00 01 90'
    buffer_send = str_send.split(' ')
    while 1:
        for i in buffer_send:
            buffer_ready = binascii.a2b_hex(i)
            encoder_ser.write(buffer_ready)

# encoder_process()


if __name__ == '__main__':
    try:
        # t1 = threading.Thread(target=encoder_process)
        t2 = threading.Thread(target=gps_process)
        # t1.start()
        t2.start()
        # t1.join()
        t2.join()
        encoder_process()
        # TODO 线程读取串口
        while 1:
            print "I am ok"
    except rospy.ROSInterruptException:
        pass