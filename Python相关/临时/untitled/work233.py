import rospy
import math
import tf
import numpy as np

from nav_msgs.msg import Odometry
from path_planning.msg import comb
from tf import transformations
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
# from tf.msg import tfMessage

fix_pos = PoseWithCovarianceStamped()

La = 0
odom_limit = 1.0
init_flag = 0
gps_flag = 0
nav_err = 0
constraint_r = 0.1
gps_limit = 0.05
gps_msg = [0.0] * 2
tmp_msg = [0.0] * 6
gps_msg_old = [0.0] * 6
lidar_msg_old = [0.0] * 6
odom_msg_old = [0.0] * 6
# longitude_0 = 113.6944569 * math.pi / 180
# latitude_0 = 34.7994373 * math.pi / 180

longitude_0 = 113.6936479 * math.pi / 180
latitude_0 = 34.7981492 * math.pi / 180
gps_get_flag = 0
count = 0

gps_recovery = []
gps_recovery_flag = 0
gps_var = None
time_NavDown = 0

gl_GPS_offsetX = 0
gl_GPS_offsetY = 0

seq = 0

# n = 0
#
#
# def af_init():
#     global , init_flag
#     if gps_flag > 1000
#     init_flag = 1

## Status GPS Lidar Return  ###############
#           0   0      0
#           0   1      1
#           1   0      2
#           1   1      3
############################################



def gps_callback(combmsg):
    global gps_msg, longitude, latitude, longitude0, latitude0, gps_recovery_flag, gps_var, gps_get_flag

    latitude = combmsg.Latitude * math.pi / 180
    longitude = combmsg.Longitude * math.pi / 180

    gps_msg[0] = latitude
    gps_msg[1] = longitude

    e = 1.0 / 298.3
    R_e = 6378254l
    Rm = R_e * (1 - e * e) / (1 - e * e * math.sin(latitude_0) * math.sin(latitude_0)) ** (3.0 / 2)
    Rn = R_e / (1 - e * e * math.sin(latitude_0) * math.sin(latitude_0)) ** (1.0 / 2)

    lon_1_0 = longitude - longitude_0
    lat_1_0 = latitude - latitude_0

    x_0_1 = Rn * math.cos(latitude_0) * lon_1_0
    y_0_1 = Rm * lat_1_0

    gps_recovery.append(x_0_1)
    if gps_recovery_flag >= 1000:

        print gps_recovery
        N = len(gps_recovery)
        narray = np.array(gps_recovery)
        sum1 = narray.sum()
        narray2 = narray * narray
        sum2 = narray2.sum()
        mean = sum1 / N
        gps_var = sum2 / N - mean ** 2
        if len(gps_recovery) > 1000:
            gps_recovery.pop(0)
        print gps_var

    gps_recovery_flag += 1
    print gps_recovery_flag


def nav_filter():
    global tf_listener, pub

    rospy.init_node('gps2333_node', anonymous=True)

    rospy.Subscriber('/path_planning/comb', comb, gps_callback)


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()



if __name__ == '__main__':
    nav_filter()