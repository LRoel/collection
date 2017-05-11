#!/usr/bin/env python

import rospy
import tf
import Image


if __name__ == '__main__':
    rospy.init_node('tf_turtle')

    listener = tf.TransformListener()

    rate = rospy.Rate(100)

    im = Image.open('/home/exbot/catkin_ws/src/af/af_nav/maps/0828g.pgm')

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #rospy.loginfo(trans)

        map_origin_x = -106.523000
        map_origin_y = -62.124000
        map_resolution = 0.050000

        grid_x = (int)((trans[0] - map_origin_x) / map_resolution)
        grid_y = (int)((trans[1] - map_origin_y) / map_resolution)

        pixel = im.getpixel((grid_x, 4252 - grid_y))

        # for i in range(0,7370):
        #     for j in range(0,4252):
        #         if im.getpixel((i,j))!=205:
        #             print i,j,im.getpixel((i,j))
        #

        # print grid_x,grid_y,pixel

        if pixel != 254:
            print "You OUT!!!!"
        else:
            print "OK !!!!"
        rate.sleep()




