#!/usr/bin/env python

import rospy
import serial
import sys
import string
import operator
import math

from path_planning.msg import comb
#print("import comb")

longitude_0 = 120.0
latitude_0 = 39.0

def checksum(nmea_str0):
    nmea_str = nmea_str0[0:-5]
    return reduce(operator.xor, map(ord, nmea_str), 0)


port = '/dev/pts/8'

try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    print("IMU not found at port " + port + ". Did you specify the correct port in the launch file?")
    # exit
    sys.exit(0)


def LL2xy(longitude_0,latitude_0,longitude_1,latitude_1):

    longitude_0 = longitude_0 * math.pi / 180
    latitude_0 = latitude_0 * math.pi / 180
    longitude_1 = longitude_1 * math.pi / 180
    latitude_1 = latitude_1 * math.pi / 180
    e=1.0/298.3
    R_e=6378254l
    Rm=R_e*(1-e*e)/( 1-e*e*math.sin(latitude_0)*math.sin(latitude_0) )**(3.0/2)
    Rn=R_e/( 1-e*e*math.sin(latitude_0)*math.sin(latitude_0) )**(1.0/2)

    lon_1_0 = longitude_1 - longitude_0
    lat_1_0 = latitude_1 - latitude_0

    x_0_1 = Rn*lon_1_0
    y_0_1 = Rm*lat_1_0

    # rospy.loginfo(x_0_1)
    # rospy.loginfo(y_0_1)

    return (x_0_1,y_0_1)

def talker():

    global longitude_0, latitude_0
    ser.flush()

    pub = rospy.Publisher('comb', comb, queue_size=100)
    pub_n = rospy.Publisher('gps',comb, queue_size=100)
    rospy.init_node('gps_node', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    combMsg = comb()

    seq = 0

    while not rospy.is_shutdown():

        # seconds = rospy.get_time()
        # now = rospy.get_rostime()s
        # rospy.loginfo("Current time %i %i", now.secs, now.nsecs)
        # li = "%i.%i\n" % (now.secs, now.nsecs)
        if (ser.read() != '$'):
            continue
        line0 = ser.readline()
        cs = line0[-4:-2]
        cs1 = int(cs, 16)
        cs2 = checksum(line0)
        print("cs1,cs2", cs1, cs2)
        # if (cs1 != cs2):
        #     continue
        if (cs2 == 'FF'):
            continue
        line = line0.replace("GPFPD,", "")
        line = line.replace("\r\n", "")
        if ("\x00" in line):
            continue
        if (not string.find('*', line)):
            continue
        line = line.replace("*", ",")

        words = string.split(line, ",")

        GPSWeek = string.atoi(words[0])
        GPSTime = string.atof(words[1])
        Heading = string.atof(words[2])
        Pitch = string.atof(words[3])
        Roll = string.atof(words[4])
        Latitude = string.atof(words[5])
        Longitude = string.atof(words[6])
        Altitude = string.atof(words[7])
        Ve = string.atof(words[8])
        Vn = string.atof(words[9])
        Vu = string.atof(words[10])
        Baseline = string.atof(words[11])
        NSV1 = string.atoi(words[12])
        NSV2 = string.atoi(words[13])
        Status = words[14]

        if Status == '03':
            Vne = math.sqrt(Vn * Vn + Ve * Ve)
            if (Vne > 0.3 and Ve < 0):
                Heading = math.acos(Vn / Vne)
            else:
                Heading = 2 * 3.141592658 - math.acos(Vn / Vne)


        combMsg.GPSWeek = GPSWeek
        combMsg.GPSTime = GPSTime
        combMsg.Heading = Heading
        combMsg.Pitch = Pitch
        combMsg.Roll = Roll
        combMsg.Latitude = Latitude
        combMsg.Longitude = Longitude
        combMsg.Altitude = Altitude
        combMsg.Ve = Ve
        combMsg.Vn = Vn
        combMsg.Vu = Vu
        combMsg.Baseline = Baseline
        combMsg.NSV1 = NSV1
        combMsg.NSV2 = NSV2
        combMsg.Status = Status
        (x, y) = LL2xy(longitude_0=longitude_0, latitude_0=latitude_0, longitude_1=Longitude, latitude_1=Latitude)
        combMsg.x = x
        combMsg.y = y
        combMsg.header.stamp = rospy.Time.now()
        combMsg.header.frame_id = 'gps'
        combMsg.header.seq = seq

        pub_n.publish(combMsg)
        print("pub gps")

        if Status == '03' or Status == '04' or Status == '05' or Status == '08' or Status == '0B':
        #rospy.loginfo(hello_str)
            pub.publish(combMsg)
            print("pub comb")

        # pub.publish(combMsg)
        # print("pub comb")

        seq += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
