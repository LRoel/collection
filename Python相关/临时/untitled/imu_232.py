    #!/usr/bin/env python

    import rospy
    import serial
    import sys
    import string
    import operator
    import math



    from sensor_msgs.msg import Imu
    from tf.transformations import quaternion_from_euler

    global_g = 9.8
    deg_rad = math.pi / 180

    def checksum(nmea_str0):
        nmea_str = nmea_str0[0:-5]
        print nmea_str
        return reduce(operator.xor, map(ord, nmea_str), 0)


    port = '/dev/ttyS1'

    try:
        ser = serial.Serial(port=port, baudrate=115200, timeout=1)
    except serial.serialutil.SerialException:
        print("IMU not found at port " + port + ". Did you specify the correct port in the launch file?")
        # exit
        sys.exit(0)


    def talker():
        global global_g, deg_rad

        ser.flush()

        pub = rospy.Publisher('imu/data_raw', Imu, queue_size=100)
        rospy.init_node('imu_node', anonymous=True)
        rate = rospy.Rate(100) # 10hz

        imuMsg = Imu()

        seq = 0

        while not rospy.is_shutdown():
            #$GTIMU,GPSWeek,GPSTime,GyroX,GyroY,GyroZ,AccX,AccY,AccZ,Tpr*cs<CR><LF>

            if ser.read() != '$':
                continue
            line0 = ser.readline()
            print line0
            cs = line0[-4:-2]
            print cs
            cs1 = int(cs, 16)
            cs2 = checksum(line0)
            print("cs1,cs2", cs1, cs2)
            if cs1 != cs2:
                continue
            if line[0:5] == "GTIMU":
                line = line0.replace("GTIMU,", "")
                line = line.replace("\r\n", "")
                if "\x00" in line:
                    continue
                if not string.find('*', line):
                    continue
                line = line.replace("*", ",")

                words = string.split(line, ",")

                GyroX = string.atof(words[2])
                GyroY = string.atof(words[3])
                GyroZ = string.atof(words[4])
                AccX = string.atof(words[5])
                AccY  = string.atof(words[6])
                AccZ = string.atof(words[7])
                Tpr = string.atof(words[8])

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
                imuMsg.header.frame_id = 'imu_node'
                imuMsg.header.seq = seq

                seq = seq + 1
                pub.publish(imuMsg)
                rate.sleep()
            else:
                continue

    if __name__ == '__main__':
        try:
            talker()
        except rospy.ROSInterruptException:
            pass