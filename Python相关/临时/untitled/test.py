#!/usr/bin/env python
import roslib
import rospy
import numpy as np
import tf
from tf import transformations

#/map-/base_footprint
a0 = [0,0,1]
b0 = [0.0, 0.0, 0.0, 1]

#/odom-/base_footprint
a1 = [0,0,1]
b1 = [0.0, 0.0, 0.0, 1]


c0 = np.dot(transformations.translation_matrix(a0), transformations.quaternion_matrix(b0))

c1 = np.linalg.inv(c0)

mat44 = np.dot(transformations.translation_matrix(a1), transformations.quaternion_matrix(b1))
# txpose is the new pose in target_frame as a 4x4
txpose = np.dot(mat44, c1)
# xyz and quat are txpose's position and orientation
txpose = np.linalg.inv(txpose)

xyz = tuple(transformations.translation_from_matrix(txpose))[:3]
quat = tuple(transformations.quaternion_from_matrix(txpose))

print xyz
print quat


abc = np.dot(transformations.translation_matrix([0,0,0]), transformations.quaternion_matrix(quat))
print abc

