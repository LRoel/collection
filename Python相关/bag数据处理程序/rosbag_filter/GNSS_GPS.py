#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rosbag, sys, csv
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import time
import string
import os # for file management make directory
import shutil # for file management, copy file

bagFile = '2017-03-22-13-51-34.bag'
bag = rosbag.Bag(bagFile)
bag_test = rosbag.Bag('fix.bag','a')
bagContents = bag.read_messages()
bagName = bag.filename


init = 0


for subtopic, msg, t in bag.read_messages("/Gnss_Odom_res1"):	#  for each instant in time that has data for topicName
	# parse data from this instant, which is of the form of multiple lines of "Name: value\n"
	# 	- put it in the form of a list of 2-element lists
	# msgString = str(msg)
	# print subtopic
	# print t
	odom_msg = Odometry()
	odom_msg.header.stamp = t
	odom_msg.header.frame_id = 'odom'
	odom_msg.child_frame_id = 'base_footprint'
	odom_msg.pose.pose.position.x = 0
	odom_msg.pose.pose.position.y = 0
	quat = quaternion_from_euler(0, 0, 0)
	odom_msg.pose.pose.orientation.x = quat[0]
	odom_msg.pose.pose.orientation.y = quat[1]
	odom_msg.pose.pose.orientation.z = quat[2]
	odom_msg.pose.pose.orientation.w = quat[3]
	odom_msg.pose.covariance[0] = msg.X_std ** 2
	odom_msg.pose.covariance[7] = msg.Y_std ** 2
	odom_msg.pose.covariance[35] = msg.Hdg_std ** 2
	bag_test.write('/odom', odom_msg, t)

bag.close()
bag_test.close()
print "Done reading bag files."