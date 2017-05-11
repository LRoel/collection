#!/usr/bin/env python


import rospy
import serial
import sys
import struct
import string
import tf
import Image

# cs = 0
# css = 0

# from std_msgs.msg import UInt8MultiArray
from af_msgs.msg import RobotState

# def checksum(nmea_str0):
#     nmea_str = nmea_str0[0:-5]
#     return reduce(operator.xor, map(ord, nmea_str), 0)

call_security_flag = 0
call_security_current = 0
call_security_old = 0
abnormal_detection_flag = 0

port = '/dev/ttyS1'

try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    print("not found at port " + port + ". Did you specify the correct port in the launch file?")
    # exit
    sys.exit(0)

def abnormal_detection_cb(robot_state):

    global abnormal_detection_flag
    abnormal_detection_flag = robot_state.abnormal_detection

def talker():

    global call_security_flag, call_security_current, call_security_old
    global abnormal_detection_flag

    pub = rospy.Publisher('robot_state/other', RobotState, queue_size=1)
    rospy.init_node('robot_state_other_node', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    
    state = RobotState()
    listener = tf.TransformListener()
    rate = rospy.Rate(100)
    im = Image.open('/home/ros/catkin_ws/src/af/af_nav/maps/0904gg.pgm')
    seq = 0

    while not rospy.is_shutdown():

        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            trans=(0,0,0)

        map_origin_x = -57.124
        map_origin_y = -314.777
        map_resolution = 0.050000

        grid_x = int((trans[0] - map_origin_x) / map_resolution)
        grid_y = int((trans[1] - map_origin_y) / map_resolution)
        

        pixel = im.getpixel((grid_x, 7572 - grid_y))
        # print grid_x,grid_y,pixel
        if pixel != 255:
            state.deviation_path = True
        else:
            state.deviation_path = False

        # state.abnormal_detection = abnormal_detection_flag
        state.header.stamp = rospy.Time.now()
        state.header.frame_id = ''
        state.header.seq = seq
        pub.publish(state)

        ser_data = ser.read(1)
        if ser_data == '\x85':
            # print "85"
            if ser.read(1) == '\x16':
                s = ser.read(21)
                if len(s) == 21:
                    data = struct.unpack(">10HB", s)
                    sonar = data[0:10]
                    cs = data[-1]

                    css_tmp = struct.unpack(">21B", s)
                    css = sum(css_tmp[0:-1]) & 0xff

                    if cs == css:
                        state.sonar = sonar
                        # print "pub sonar succeed, sonar",sonar

            else:
                continue

        elif ser_data == '\x87':
            # print "87"
            if ser.read(1) == '\x09':
                s = ser.read(8)
                # print s
                if (len(s) == 8):
                    data = struct.unpack("<HBHHB", s)
                    voltage = data[0]
                    electric_quantity = data[1]
                    charge_current = data[2]
                    discharge_current = data[3]
                    cs = data[-1]

                    css_tmp = struct.unpack(">8B", s)
                    css = sum(css_tmp[0:-1]) & 0xff

                    if (cs == css):

                        state.voltage = voltage
                        state.electric_quantity = electric_quantity
                        state.charge_current = charge_current
                        state.discharge_current = discharge_current

                        # print ("pub voltage succeed, voltage = %d, electric_quantity = %d, charge_current = %d,discharge_current = %d" % (voltage, electric_quantity, charge_current, discharge_current))

            else:
                continue

        elif ser_data == '\x88':
            # print "88"
            if ser.read(1) == '\x06':
                s = ser.read(5)
                # print s
                if (len(s) == 5):
                    data = struct.unpack("<IB", s)
                    # print data
                    cs = data[-1]

                    css_tmp = struct.unpack(">5B", s)
                    css = sum(css_tmp[0:-1]) & 0xff

                    if (cs == css):
                        error_state = bin(data[0])
                        error_list = list(error_state[2:])
                        error_list.reverse()
                        error_list = map(int, error_list)
                        
                        call_security_current = error_list[10]
                        if call_security_current and not call_security_old:
                            call_security_flag = not call_security_flag

                        state.vibration_detection = error_list[13]
                        state.control_lost = error_list[12]
                        state.emergency_stop = error_list[11]
                        state.call_security = call_security_flag
                        state.sonar_errors = error_list[9]
                        state.front_collision = error_list[8]
                        state.back_infrared = error_list[7]
                        state.front_infrared = error_list[6]
                        state.motor_4_lost = error_list[5]
                        state.motor_3_lost = error_list[4]
                        state.motor_2_lost = error_list[3]
                        state.motor_1_lost = error_list[2]
                        state.ros_lost = error_list[1]
                        state.remote_lost = error_list[0]

                        if sum(error_list[2:6]) != 0:
                            state.error = True
                        else:
                            state.error = False
                        state.error_status = "%8x" % data[0]
                        # print "pub state succeed, %8x" % data[0]
                        call_security_old = call_security_current
            else:
                continue

        else:
            continue

        seq = seq + 1

        #print array
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


