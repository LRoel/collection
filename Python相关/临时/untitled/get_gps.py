#!/usr/bin/env python

import rospy
import math
import numpy as np

from path_planning.msg import comb
import matplotlib.pyplot as plt

Longitude0 = 0
Lattitude0 = 0
flag = 0
show_flag = 1
n =0

Lat = []
Lon = []
Altitude = []
Heading = []
Pitch = []
Roll = []
Ve = []
Vn = []
Vu = []
Baseline = []
NSV1 = []
NSV2 = []
posx = []
posy = []

n = 1

f1 = open('/home/exbot/comb2.txt','a')

def LL2xy(longitude_0,latitude_0,longitude_1,latitude_1):

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

def comb_callback(combMsg):

    global flag , Longitude0 , Lattitude0 ,show_flag,Lat,Lon,Altitude,Heading,Pitch,Roll,Ve,Vn,Vu,Baseline,NSV1,NSV2,posx,posy,n
    # n = n + 1
    # if n % 10 != 0:
    #     return 1
    Lattitude = combMsg.Latitude * math.pi / 180
    Longitude = combMsg.Longitude * math.pi / 180
    Altitude.append(combMsg.Altitude)
    # print(Altitude)
    Heading.append(-1 * combMsg.Heading + 360)
    Pitch.append( combMsg.Pitch)
    Roll.append(combMsg.Roll)
    Ve.append(combMsg.Ve)
    Vn.append(combMsg.Vn)
    Vu.append(combMsg.Vu)
    Baseline.append(combMsg.Baseline)
    NSV1.append(combMsg.NSV1)
    NSV2.append(combMsg.NSV2)
    Lon.append(Longitude)
    Lat.append(Lattitude)

    s = str(combMsg.GPSWeek)+','+str(combMsg.GPSTime)+','+str(combMsg.Latitude)+','+str(combMsg.Longitude)+','+str(combMsg.Altitude)+'\n'
    f1.write(s)


    if flag == 0:
        Longitude0 = Longitude
        Lattitude0 = Lattitude
    flag = flag + 1

    (x_robot, y_robot) = LL2xy(Longitude0, Lattitude0, Longitude, Lattitude)
    posx.append(x_robot)
    posy.append(y_robot)

    rospy.loginfo(show_flag)

    if show_flag % 1699000 == 0:
        plt.figure()
        plt.plot(posx,posy)
        plt.figure()
        ax11 = plt.subplot(311)
        ax12 = plt.subplot(312)
        ax13 = plt.subplot(313)
        plt.figure()
        ax21 = plt.subplot(311)
        ax22 = plt.subplot(312)
        ax23 = plt.subplot(313)
        plt.figure()
        ax31 = plt.subplot(311)
        ax32 = plt.subplot(312)
        ax33 = plt.subplot(313)
        plt.figure()
        ax41 = plt.subplot(411)
        ax42 = plt.subplot(412)
        ax43 = plt.subplot(413)
        ax44 = plt.subplot(414)
        plt.sca(ax11)
        plt.plot(posx,posy)
        ax11.set_title('posx-posy')
        plt.sca(ax12)
        plt.plot(Lon,Lat)
        ax13.set_title('Lon-Lat')
        plt.sca(ax13)
        plt.plot(Altitude)
        ax13.set_title('Altitude')
        plt.sca(ax21)
        plt.plot(Heading)
        ax21.set_title('Heading')
        plt.sca(ax22)
        plt.plot(Pitch)
        ax22.set_title('Pitch')
        plt.sca(ax23)
        plt.plot(Roll)
        ax23.set_title('Roll')
        plt.sca(ax31)
        plt.plot(Ve)
        ax31.set_title('Ve')
        plt.sca(ax32)
        plt.plot(Vn)
        ax32.set_title('Vn')
        plt.sca(ax33)
        plt.plot(Vu)
        ax33.set_title('Vu')
        plt.sca(ax41)
        plt.plot(NSV1)
        ax41.set_title('NSV1')
        plt.sca(ax42)
        plt.plot(NSV2)
        ax42.set_title('NSV2')
        plt.sca(ax43)
        plt.plot(Lon)
        ax43.set_title('Lon')
        plt.sca(ax44)
        plt.plot(Lat)
        ax44.set_title('Lat')
        plt.show()
        Lat = []
        Lon = []
        Altitude = []
        Heading = []
        Pitch = []
        Roll = []
        Ve = []
        Vn = []
        Vu = []
        Baseline = []
        NSV1 = []
        NSV2 = []
        posx = []
        posy = []
    show_flag = show_flag + 1


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('zhengzhou_node', anonymous=True)

    rospy.Subscriber('/comb_now', comb, comb_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
while not rospy.is_shutdown():
    listener()

