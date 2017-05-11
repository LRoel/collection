#!/usr/bin/env python

import rospy
import serial
import sys
import string
import operator
import math

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from path_planning.msg import comb
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix

global_g = 9.8
deg_rad = math.pi / 180
last_heading = 0
Heading_HPD = 0


def checksum(nmea_str0):
    nmea_str = nmea_str0[0:-5]
    print nmea_str
    return reduce(operator.xor, map(ord, nmea_str), 0)


port = '/dev/ttyS2'

try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    print("IMU not found at port " + port + ". Did you specify the correct port in the launch file?")
    # exit
    sys.exit(0)


def talker():
    global global_g, deg_rad, Heading_HPD

    ser.flush()

    pub_m = rospy.Publisher('/imu/data_raw', Imu, queue_size=100)
    pub = rospy.Publisher('comb', comb, queue_size=100)
    pub_n = rospy.Publisher('comb_now', comb, queue_size=100)
    pub_g = rospy.Publisher('gps', comb, queue_size=100)
    pub_o = rospy.Publisher('/gps/fix', NavSatFix, queue_size=100)

    rospy.init_node('comb_node', anonymous=True)
    rate = rospy.Rate(100)  # 10hz

    imuMsg = Imu()
    combMsg = comb()
    gpsMsg = comb()
    gpsmeanMsg = NavSatFix()

    seq = 0

    while not rospy.is_shutdown():
        # $GTIMU,GPSWeek,GPSTime,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,Tpr*cs<CR><LF>

        if ser.read() != '$':
            continue
        line0 = ser.readline()
        print line0
        cs = line0[-4:-2]
        print cs
        for s_tmp in line0[6:-8]:
            if s_tmp.isalpha():
                continue
        try:
            cs1 = int(cs, 16)
        except:
            continue
        cs2 = checksum(line0)
        print("cs1,cs2", cs1, cs2)
        if cs1 != cs2:
            continue
        if line0[0:5] == "GTIMU":
            line = line0.replace("GTIMU,", "")
            line = line.replace("\r\n", "")
            if "\x00" in line:
                continue
            if not string.find('*', line):
                continue
            line = line.replace("*", ",")

            words = string.split(line, ",")
            if len(words) != 10:
                continue

            GyroX = string.atof(words[2])
            GyroY = string.atof(words[3])
            GyroZ = string.atof(words[4])
            AccX = string.atof(words[5])
            AccY = string.atof(words[6])
            AccZ = string.atof(words[7])

            imuMsg.linear_acceleration.x = AccX * global_g
            imuMsg.linear_acceleration.y = AccY * global_g
            imuMsg.linear_acceleration.z = AccZ * global_g

            imuMsg.linear_acceleration_covariance[0] = 0.0001
            imuMsg.linear_acceleration_covariance[4] = 0.0001
            imuMsg.linear_acceleration_covariance[8] = 0.0001

            imuMsg.angular_velocity.x = GyroX * deg_rad
            imuMsg.angular_velocity.y = GyroY * deg_rad
            imuMsg.angular_velocity.z = GyroZ * deg_rad

            imuMsg.angular_velocity_covariance[0] = 0.0001
            imuMsg.angular_velocity_covariance[4] = 0.0001
            imuMsg.angular_velocity_covariance[8] = 0.0001

            q = quaternion_from_euler(0, 0, 0)
            imuMsg.orientation.x = q[0]
            imuMsg.orientation.y = q[1]
            imuMsg.orientation.z = q[2]
            imuMsg.orientation.w = q[3]
            imuMsg.header.stamp = rospy.Time.now()
            imuMsg.header.frame_id = 'imu'
            imuMsg.header.seq = seq

            seq = seq + 1
            pub_m.publish(imuMsg)
            rate.sleep()

        elif line0[0:5] == "GPFPD":
            line = line0.replace("GPFPD,", "")
            line = line.replace("\r\n", "")
            if ("\x00" in line):
                continue
            if (not string.find('*', line)):
                continue
            line = line.replace("*", ",")

            words = string.split(line, ",")
            if len(words) != 16:
                continue

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
            combMsg.header.stamp = rospy.Time.now()
            combMsg.header.frame_id = 'gps'
            combMsg.header.seq = seq

            pub_n.publish(combMsg)
            print("pub comb_now")

            if Status == '03' or Status == '04' or Status == '05' or Status == '08' or Status == '0B':
                pub.publish(combMsg)
                print("pub comb")

            seq += 1
            rate.sleep()

            # $GPHPD, GPSWeek, GPSTime, Heading, Pitch, Track, Latitude, Longitude, Altitude, Ve , Vn, Vu,Baseline, NSV1, NSV2*cs<CR><LF>
        elif line0[0:5] == "GPHPD":
            line = line0.replace("GPHPD,", "")
            line = line.replace("\r\n", "")
            if ("\x00" in line):
                continue
            if (not string.find('*', line)):
                continue
            line = line.replace("*", ",")

            words = string.split(line, ",")
            if len(words) != 16:
                continue

            Status = words[14]

            GPSWeek = string.atoi(words[0])
            GPSTime = string.atof(words[1])
            Heading_HPD = string.atof(words[2])
            Heading_HPD = 360 - Heading_HPD
            Pitch = string.atof(words[3])
            Track = string.atof(words[4])
            Latitude = string.atof(words[5])
            Longitude = string.atof(words[6])
            Altitude = string.atof(words[7])
            Ve = string.atof(words[8])
            Vn = string.atof(words[9])
            Vu = string.atof(words[10])
            Baseline = string.atof(words[11])
            NSV1 = string.atoi(words[12])
            NSV2 = string.atoi(words[13])

            gpsMsg.GPSWeek = GPSWeek
            gpsMsg.GPSTime = GPSTime
            gpsMsg.Heading = Heading_HPD
            gpsMsg.Pitch = Pitch
            gpsMsg.Roll = Track
            gpsMsg.Latitude = Latitude
            gpsMsg.Longitude = Longitude
            gpsMsg.Altitude = Altitude
            gpsMsg.Ve = Ve
            gpsMsg.Vn = Vn
            gpsMsg.Vu = Vu
            gpsMsg.Baseline = Baseline
            gpsMsg.NSV1 = NSV1
            gpsMsg.NSV2 = NSV2
            gpsMsg.Status = Status
            gpsMsg.header.stamp = rospy.Time.now()
            gpsMsg.header.frame_id = 'gps'
            gpsMsg.header.seq = seq

            pub_g.publish(gpsMsg)
            print("pub gps")

            gpsmeanMsg.header.stamp = rospy.Time.now()
            gpsmeanMsg.header.frame_id = 'gps'
            gpsmeanMsg.header.seq = seq
            if Status == '05':
                gpsmeanMsg.status.status = 2
                covariance_xyz = 0.05 ** 2
            elif Status == '03':
                gpsmeanMsg.status.status = 0
                covariance_xyz = 5 ** 2
            else:
                gpsmeanMsg.status.status = -1
                covariance_xyz = 99999

            gpsmeanMsg.status.service = 1
            gpsmeanMsg.latitude = Latitude
            gpsmeanMsg.longitude = Longitude
            gpsmeanMsg.altitude = Latitude
            gpsmeanMsg.position_covariance = [covariance_xyz, 0, 0,
                                              0, covariance_xyz, 0,
                                              0, 0, covariance_xyz]
            gpsmeanMsg.position_covariance_type = 3

            pub_o.publish(gpsmeanMsg)
            print("pub gps_mean")


            seq += 1
            rate.sleep()
        else:
            continue


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

