#!/usr/bin/env python

import rospy
import time
import sys
import string

from std_srvs.srv import Empty


def set():
    rospy.wait_for_service('request_nomotion_update')
    try:
        get_map = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        get_map()
        print "clear ok!"

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


if __name__ == "__main__":
    while not rospy.is_shutdown():
        set()
