import rospy
import math
import tf
import numpy as np

from nav_msgs.msg import Odometry
from path_planning.msg import comb
from af_bringup.msg import Robot_encode
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
constraint_r = 1
gps_limit = 0.05
gps_msg = [0.0] * 6
tmp_msg = [0.0] * 6
gps_msg_old = [0.0] * 6
lidar_msg_old = [0.0] * 6
odom_msg_old = [0.0] * 6
# longitude_0 = 113.6944569 * math.pi / 180
# latitude_0 = 34.7994373 * math.pi / 180

longitude_0 = 113.6944478 * math.pi / 180
latitude_0 = 34.7993875 * math.pi / 180
gps_get_flag = 0
count = 0

gps_recovery = []
gps_recovery_flag = 1
gps_var = None
time_NavDown = 0

gl_GPS_offsetX = 0
gl_GPS_offsetY = 0

seq = 0

alpha = 0
encode_sum = 0
encode_flag = 0

n = 0

print "init complete"


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
def stop():
    # print "stop"
    return 1


def recovery():
    # print "recovery"
    global gps_recovery_flag, time_NavDown, gps_limit
    gps_recovery_flag = 1
    if not gps_var:
        if gps_var < gps_limit:
            GPS_status_recovery = 1
            time_NavDown = 0
            status_normal = 1
            return time_NavDown
        else:
            time_NavDown += 1
            return time_NavDown


def nav_status(gps_msg, lidar_msg, odom_msg, gps_msg_old, lidar_msg_old, odom_msg_old, tmp_msg):
    # print "nav status"

    global gps_get_flag
    global constraint_r
    delta2gps = (gps_msg[0] - tmp_msg[0]) ** 2 + (gps_msg[1] - tmp_msg[1]) ** 2 + (gps_msg[2] - tmp_msg[2]) ** 2
    delta2lidar = (lidar_msg[0] - tmp_msg[0]) ** 2 + (lidar_msg[1] - tmp_msg[1]) ** 2 + (lidar_msg[2] - tmp_msg[2]) ** 2
    delta2comb = (odom_msg[0] - odom_msg_old[0]) ** 2 + (odom_msg[1] - odom_msg_old[1]) ** 2 + (odom_msg[2] -
                                                                                                odom_msg_old[2]) ** 2
    # print("gpsmsg",gps_msg,"gpsmsg_old",gps_msg_old)
    # print("delta2gps",delta2gps,"delta2lidar",delta2lidar,"delta2comb",delta2comb)
    gps_get_flag = 0
    if abs(delta2gps) <= constraint_r ** 2:
        if abs(delta2lidar) <= constraint_r ** 2:
            out_flag = 3
        else:
            out_flag = 2
    else:
        if abs(delta2lidar) <= constraint_r ** 2:
            out_flag = 1
        else:
            out_flag = 0

    return (out_flag, delta2comb)


def ll2xy(longitude_0, latitude_0, longitude_1, latitude_1, alpha):
    # print "ll2xy"

    global gl_GPS_offsetX, gl_GPS_offsetY
    # latitude_1 = latitude_0
    alpha = alpha * math.pi / 180
    mat33 = np.array([[math.cos(alpha), math.sin(alpha), 0], [-math.sin(alpha), math.cos(alpha), 0], [0, 0, 1]])
    # print mat33

    e = 1.0 / 298.3
    R_e = 6378254l
    Rm = R_e * (1 - e * e) / (1 - e * e * math.sin(latitude_0) * math.sin(latitude_0)) ** (3.0 / 2)
    Rn = R_e / (1 - e * e * math.sin(latitude_0) * math.sin(latitude_0)) ** (1.0 / 2)

    lon_1_0 = longitude_1 - longitude_0
    lat_1_0 = latitude_1 - latitude_0

    x_0_1_lon = Rn * math.cos(latitude_0) * lon_1_0
    y_0_1_lat = Rm * lat_1_0

    xy_temp = np.array([[x_0_1_lon], [y_0_1_lat], [0]])

    # print xy_temp
    xy = np.dot(mat33, xy_temp)

    x_0_1 = xy[1, 0]
    y_0_1 = xy[0, 0]

    # print ("~~~~~~~~xy~~~~~~~",xy)
    # rospy.loginfo(x_0_1)
    # rospy.loginfo(y_0_1)

    return (x_0_1, y_0_1)


def gps_callback(combmsg):
    # print "gps callback"
    global n, gps_msg, longitude, latitude, longitude0, latitude0, gps_recovery_flag, gps_var, gps_get_flag, alpha
    n = n + 1
    if n % 10 != 0:
        return 1
    latitude = combmsg.Latitude * math.pi / 180
    longitude = combmsg.Longitude * math.pi / 180
    altitude = combmsg.Altitude

    heading = combmsg.Heading
    pitch = combmsg.Pitch
    roll = combmsg.Roll

    (x_robot, y_robot) = ll2xy(longitude_0, latitude_0, longitude, latitude, 184.134)
    # print x_robot
    # print ("gl_GPS_offsetX", gl_GPS_offsetX)
    # print ("gl_GPS_offsetY", gl_GPS_offsetY)
    gps_msg[0] = x_robot
    gps_msg[1] = y_robot
    gps_msg[2] = 0
    gps_msg[3] = roll * math.pi / 180
    gps_msg[4] = pitch * math.pi / 180
    gps_msg[5] = (heading - 184.134) * math.pi / 180
    # print(gps_msg)
    gps_get_flag = 1

    if gps_recovery_flag == 1:
        gps_recovery.append(gps_msg)
        N = len(gps_recovery)
        narray = np.array(gps_recovery)
        sum1 = narray.sum()
        narray2 = narray * narray
        sum2 = narray2.sum()
        mean = sum1 / N
        gps_var = sum2 / N - mean ** 2
        if len(gps_recovery) > 1000:
            gps_recovery.pop(0)


#
# def nav_filter():
#     global n
#     rospy.init_node('nav_filter_node', anonymous=True)
#
#     rospy.Subscriber('comb', comb, gps_callback)
#     rospy.Subscriber('odom', Odometry, odom_callback)
#
#
#     rospy.loginfo(n)
#     n += 1
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

#
# while not rospy.is_shutdown():
#     nav_filter()
#
#
def encode_callback(encodemsg):
    global encode_sum, encode_flag
    encode_flag = 0
    # print encode_sum
    encode_sum += (encodemsg.left_encode + encodemsg.right_encode)
    if encode_sum >= 1000:
        encode_sum = 0
        encode_flag = 1


def odom_callback(odommsg):
    # print "odom callback"

    global nav_err, La, gps_var, seq, gps_msg_old, lidar_msg_old, odom_msg_old, tmp_msg, alpha
    global gl_GPS_offsetX, gl_GPS_offsetY
    lidar_msg = [0.0] * 6
    odom_msg = [0.0] * 6

    gps_fix_msg = list(gps_msg)
    gps_fix_msg[0] = gps_msg[0] + gl_GPS_offsetX
    gps_fix_msg[1] = gps_msg[1] + gl_GPS_offsetY

    try:
        (trans, rot) = tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        # print trans
        # print rot

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return 1

    # print("3")
    (lidar_roll, lidar_pitch, lidar_yaw) = euler_from_quaternion(rot)
    lidar_msg[0:3] = trans
    lidar_msg[3] = lidar_roll
    lidar_msg[4] = lidar_pitch
    lidar_msg[5] = lidar_yaw

    odom_msg[0] = odommsg.pose.pose.position.x
    odom_msg[1] = odommsg.pose.pose.position.y
    odom_msg[2] = odommsg.pose.pose.position.z
    orientation = [odommsg.pose.pose.orientation.x, odommsg.pose.pose.orientation.y, odommsg.pose.pose.orientation.z,
                   odommsg.pose.pose.orientation.w]
    (odom_roll, odom_pitch, odom_yaw) = euler_from_quaternion(orientation)
    odom_msg[3] = odom_roll
    odom_msg[4] = odom_pitch
    odom_msg[5] = odom_yaw

    if gps_get_flag == 1 :
        (nav_flag, delta2comb) = nav_status(gps_fix_msg, lidar_msg, odom_msg, gps_msg_old, lidar_msg_old, odom_msg_old,
                                            tmp_msg)

        # print gps_msg_old
        ## Status GPS Lidar Return  ###############
        #           0   0      0
        #           0   1      1
        #           1   0      2
        #           1   1      3
        ############################################
        print (nav_flag)
        global count
        if nav_flag == 2:
            count += 1
            # print("~~~~~~~~~~~~~~",count,"~~~~~~~~~~~~~~~")
            # while(1):
            #     a = 1
            #
        if nav_flag == 3 or nav_flag == 1:
            out_msg = list(gps_fix_msg)
            tmp_msg = list(gps_fix_msg)
            La = 0
        elif nav_flag == 2:
            out_msg = list(lidar_msg)
            tmp_msg = list(lidar_msg)
            gl_GPS_offsetX = lidar_msg[0] - gps_msg[0]
            gl_GPS_offsetY = lidar_msg[1] - gps_msg[1]
            La = 0
        else:
            nav_err += 1
            La += delta2comb
            tmp_msg[0] += (odom_msg[0] - odom_msg_old[0])
            tmp_msg[1] += (odom_msg[1] - odom_msg_old[1])
            tmp_msg[2] += (odom_msg[2] - odom_msg_old[2])
            tmp_msg[3] = odom_msg[3]
            tmp_msg[4] = odom_msg[4]
            tmp_msg[5] = odom_msg[5]

            gl_GPS_offsetX = lidar_msg[0] - gps_msg[0]
            gl_GPS_offsetY = lidar_msg[1] - gps_msg[1]

            # print ("gps_fix", gps_fix_msg)
            # print ('gps_msg', gps_msg)
            # print ('odom_msg', odom_msg)
            # print ('lidar_msg', lidar_msg)
            # print ('tmp_msg', tmp_msg)

            gps_msg_old = list(gps_msg)
            lidar_msg_old = list(lidar_msg)
            odom_msg_old = list(odom_msg)

            return 0

        fix_pos.header.frame_id = "map"
        fix_pos.pose.pose.position.x = out_msg[0]
        fix_pos.pose.pose.position.y = out_msg[1]
        fix_pos.pose.pose.position.z = out_msg[2]
        q = quaternion_from_euler(out_msg[3], out_msg[4], out_msg[5])
        fix_pos.pose.pose.orientation.x = q[0]
        fix_pos.pose.pose.orientation.y = q[1]
        fix_pos.pose.pose.orientation.z = q[2]
        fix_pos.pose.pose.orientation.w = q[3]
        fix_pos.header.seq = seq
        seq = seq + 1

        print ("gps_fix", gps_fix_msg)
        print ('gps_msg', gps_msg)
        print ('odom_msg', odom_msg)
        print ('lidar_msg', lidar_msg)
        print ('tmp_msg', tmp_msg)
        print ('out_msg', out_msg)
        pub.publish(fix_pos)

        # /map-/base_footprint
        map_af_trans = out_msg[0:3]
        map_af_quat = q

        # /laser_map-/base_footprint
        lidar_af_trans = lidar_msg[0:3]
        lidar_af_quat = rot

        # /odom-/base_footprint
        odom_af_trans = odom_msg[0:3]
        odom_af_quat = orientation

        map_af_mat44 = np.dot(transformations.translation_matrix(map_af_trans),
                              transformations.quaternion_matrix(map_af_quat))

        af_map_mat44 = np.linalg.inv(map_af_mat44)

        lidar_af_mat44 = np.dot(transformations.translation_matrix(lidar_af_trans),
                                transformations.quaternion_matrix(lidar_af_quat))
        # txpose is the new pose in target_frame as a 4x4
        txpose = np.dot(lidar_af_mat44, af_map_mat44)
        # xyz and quat are txpose's position and orientation
        txpose = np.linalg.inv(txpose)

        tran_af_mat44 = np.dot(transformations.translation_matrix([0, 0, 0]),
                               transformations.quaternion_matrix(quaternion_from_euler(0, 0, alpha)))

        txpose_fix = np.dot(tran_af_mat44, txpose)
        xyz = tuple(transformations.translation_from_matrix(txpose_fix))[:3]
        quat = tuple(transformations.quaternion_from_matrix(txpose_fix))

        br = tf.TransformBroadcaster()
        br.sendTransform(xyz, quat, rospy.Time.now(), "map", "world")

        gps_msg_old = list(gps_msg)
        lidar_msg_old = list(lidar_msg)
        odom_msg_old = list(odom_msg)


def nav_filter():
    # print "nav filter"
    global tf_listener, pub

    rospy.init_node('nav_filter_node', anonymous=True)
    tf_listener = tf.TransformListener()

    rospy.Subscriber('/path_planning/comb', comb, gps_callback)
    rospy.Subscriber('odom', Odometry, odom_callback)
    rospy.Subscriber('robot_encode_val', Robot_encode, encode_callback)
    pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    nav_filter()