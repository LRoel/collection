#!/usr/bin/env python

import rospy
import time
import sys
import string

from std_srvs.srv import Empty

def clear():
    rospy.wait_for_service('/move_base/clear_costmaps')
    try:
        get_map = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        get_map()
        print "clear ok!"

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    while not rospy.is_shutdown():
        clear()
        try:
            # print sys.argv[1]
            time.sleep(string.atof(sys.argv[1]))
        except:
            time.sleep(2)
