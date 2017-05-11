#!/usr/bin/env python
import rospy
import math

from path_planning.msg import comb
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import NavSatFix

pub_o = rospy.Publisher('gps/fix', NavSatFix, queue_size=100)
gpsmeanMsg = NavSatFix()

def gps_callback(gpsMsg):
    GPSWeek = gpsMsg.GPSWeek
    GPSTime = gpsMsg.GPSTime
    Heading = gpsMsg.Heading
    Pitch = gpsMsg.Pitch
    Track = gpsMsg.Roll
    Latitude = gpsMsg.Latitude
    Longitude = gpsMsg.Longitude
    Altitude = gpsMsg.Altitude
    Ve = gpsMsg.Ve
    Vn = gpsMsg.Vn
    Vu = gpsMsg.Vu
    Baseline = gpsMsg.Baseline
    NSV1 = gpsMsg.NSV1
    NSV2 = gpsMsg.NSV2
    Status = gpsMsg.Status

    gpsmeanMsg.header.stamp = rospy.Time.now()
    gpsmeanMsg.header.frame_id = 'gps'
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


def get_navfix():
    # print "nav filter"
    rospy.init_node('nav_filter_node', anonymous=True)

    rospy.Subscriber('/gps', comb, gps_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    get_navfix()